/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : SmartHouse - ESP8266 HTTP server (GET /status, GET /last, POST /msg)
  *
  * CubeMX pins (NUCLEO-F439ZI):
  *   PA6  = TIM3_CH1   (Servo1: Door)
  *   PA7  = TIM3_CH2   (Servo2: Window)
  *   PB0  = TIM3_CH3   (Servo3: Fan sweep)
  *   PB7  = LIGHT_LED
  *   PB14 = HEATER_LED
  *   PD8  = USART3_TX  (STM32 -> ESP RX)
  *   PD9  = USART3_RX  (ESP -> STM32 RX)
  *
  * Routes:
  *   GET  /status -> "server is running"
  *   GET  /last   -> last received (normalized) body
  *   POST /msg    -> body is plain text, supports MULTIPLE commands in one body
  *
  * Examples:
  *   curl http://<ESP_IP>/status
  *   curl http://<ESP_IP>/last
  *   curl -X POST http://<ESP_IP>/msg -H "Content-Type: text/plain" --data "fan on; turn on the light"
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PD */
#define WIFI_SSID  "YOUR_WIFI_SSID"
#define WIFI_PASS  "YOUR_WIFI_PASSWORD"

#define HTTP_PORT  80

/* PWM assumption: PSC=83, ARR=19999 => 1us ticks, 20ms period */
#define SERVO_PULSE_MIN_US     1000
#define SERVO_PULSE_MID_US     1500
#define SERVO_PULSE_MAX_US     2000

/* Door/Window positions */
#define DOOR_OPEN_US           SERVO_PULSE_MAX_US
#define DOOR_CLOSE_US          SERVO_PULSE_MIN_US

#define WINDOW_OPEN_US         SERVO_PULSE_MAX_US
#define WINDOW_CLOSE_US        SERVO_PULSE_MIN_US

/* Fan sweep */
#define FAN_SWEEP_INTERVAL_MS  800

/* ESP UART ring buffer */
#define ESP_RB_SIZE            4096

/* HTTP reassembly */
#define MAX_LINKS              5
#define HTTP_BUF_SIZE          2048
#define HTTP_STUCK_TIMEOUT_MS  2500

/* Split multiple commands */
#define MAX_COMMANDS_PER_MSG   6
/* USER CODE END PD */

/* USER CODE BEGIN PV */
/* Fan state */
static uint8_t  fan_running = 0;
static uint8_t  fan_at_max  = 0;
static uint32_t fan_next_ms = 0;

/* Last received msg (debug for Thunkable) */
static char g_last_msg[220] = "NONE";

/* UART3 RX ring buffer */
static volatile uint16_t esp_rb_head = 0;
static volatile uint16_t esp_rb_tail = 0;
static uint8_t esp_rb[ESP_RB_SIZE];
static uint8_t esp_rx_byte;

/* Per-link HTTP reassembly */
typedef struct
{
  uint8_t  active;
  uint16_t len;
  uint32_t last_tick;
  char     buf[HTTP_BUF_SIZE];
} HttpConn;

static HttpConn g_http[MAX_LINKS];
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Actuators */
static void SetServoPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse);

void Light_On(void);
void Light_Off(void);
void Heater_On(void);
void Heater_Off(void);

void Door_Open(void);
void Door_Close(void);
void Door_Reverse(void);

void Window_Open(void);
void Window_Close(void);
void Window_Reverse(void);

void Fan_On(void);
void Fan_Off(void);
static void Fan_Task(void);

/* ESP UART helpers */
static void ESP_RxStartIT(void);
static uint16_t ESP_RB_Available(void);
static int ESP_RB_ReadByte(uint8_t *out);
static int ESP_RB_ReadN(uint8_t *dst, uint16_t n, uint32_t timeout_ms);
static int ESP_RB_FindSubstring(const char *needle);
static int ESP_RB_WaitFor(const char *needle, uint32_t timeout_ms);

static HAL_StatusTypeDef ESP_SendCmdWait(const char *cmd, const char *expect, uint32_t timeout_ms);
static void ESP_ConnectWiFi(void);
static void ESP_StartHttpServer(void);

/* HTTP server */
static void ESP_ServerTask(void);
static void HTTP_OnDataChunk(uint8_t link_id, const char *chunk, uint16_t chunk_len);
static void HTTP_TryProcess(uint8_t link_id);
static void HTTP_ResetConn(uint8_t link_id);
static void HTTP_SendResponse(uint8_t link_id, int status_code, const char *body);

/* Parsing & commands */
static void NormalizeText(char *s);
static void ReplaceTokensForSplit(char *s);
static int  ParseContentLength(const char *hdr);
static int  ProcessInstruction(const char *cmd);
static int  ProcessInstructionList(const char *body);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* ========================= UART3 RX IRQ ========================= */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    uint16_t next = (uint16_t)((esp_rb_head + 1) % ESP_RB_SIZE);
    if (next != esp_rb_tail)
    {
      esp_rb[esp_rb_head] = esp_rx_byte;
      esp_rb_head = next;
    }
    HAL_UART_Receive_IT(&huart3, &esp_rx_byte, 1);
  }
}

static void ESP_RxStartIT(void)
{
  HAL_UART_Receive_IT(&huart3, &esp_rx_byte, 1);
}

static uint16_t ESP_RB_Available(void)
{
  if (esp_rb_head >= esp_rb_tail) return (uint16_t)(esp_rb_head - esp_rb_tail);
  return (uint16_t)(ESP_RB_SIZE - esp_rb_tail + esp_rb_head);
}

static int ESP_RB_ReadByte(uint8_t *out)
{
  if (esp_rb_tail == esp_rb_head) return 0;
  *out = esp_rb[esp_rb_tail];
  esp_rb_tail = (uint16_t)((esp_rb_tail + 1) % ESP_RB_SIZE);
  return 1;
}

static int ESP_RB_ReadN(uint8_t *dst, uint16_t n, uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  uint16_t i = 0;
  while (i < n)
  {
    uint8_t b;
    if (ESP_RB_ReadByte(&b))
    {
      dst[i++] = b;
      continue;
    }
    if ((HAL_GetTick() - start) > timeout_ms) return 0;
  }
  return 1;
}

static int ESP_RB_FindSubstring(const char *needle)
{
  if (!needle) return 0;

  char tmp[900];
  uint16_t avail = ESP_RB_Available();
  if (avail == 0) return 0;
  if (avail > (sizeof(tmp) - 1)) avail = (uint16_t)(sizeof(tmp) - 1);

  uint16_t t = esp_rb_tail;
  for (uint16_t i = 0; i < avail; i++)
  {
    tmp[i] = (char)esp_rb[t];
    t = (uint16_t)((t + 1) % ESP_RB_SIZE);
  }
  tmp[avail] = '\0';

  return (strstr(tmp, needle) != NULL);
}

static int ESP_RB_WaitFor(const char *needle, uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < timeout_ms)
  {
    if (ESP_RB_FindSubstring(needle)) return 1;
    if (ESP_RB_FindSubstring("ERROR") || ESP_RB_FindSubstring("FAIL") || ESP_RB_FindSubstring("SEND FAIL"))
      return 0;
  }
  return 0;
}

/* ========================= Actuators ========================= */
static void SetServoPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse)
{
  __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

void Light_On(void)   { HAL_GPIO_WritePin(LIGHT_LED_GPIO_Port,  LIGHT_LED_Pin,  GPIO_PIN_SET);   }
void Light_Off(void)  { HAL_GPIO_WritePin(LIGHT_LED_GPIO_Port,  LIGHT_LED_Pin,  GPIO_PIN_RESET); }
void Heater_On(void)  { HAL_GPIO_WritePin(HEATER_LED_GPIO_Port, HEATER_LED_Pin, GPIO_PIN_SET);   }
void Heater_Off(void) { HAL_GPIO_WritePin(HEATER_LED_GPIO_Port, HEATER_LED_Pin, GPIO_PIN_RESET); }

/* Door = Servo1 (TIM3_CH1 / PA6) */
void Door_Open(void)  { SetServoPulse(&htim3, TIM_CHANNEL_1, DOOR_OPEN_US); }
void Door_Close(void) { SetServoPulse(&htim3, TIM_CHANNEL_1, DOOR_CLOSE_US); }
void Door_Reverse(void)
{
  uint16_t p = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
  if (p > SERVO_PULSE_MID_US) Door_Close();
  else                       Door_Open();
}

/* Window = Servo2 (TIM3_CH2 / PA7) */
void Window_Open(void)  { SetServoPulse(&htim3, TIM_CHANNEL_2, WINDOW_OPEN_US); }
void Window_Close(void) { SetServoPulse(&htim3, TIM_CHANNEL_2, WINDOW_CLOSE_US); }
void Window_Reverse(void)
{
  uint16_t p = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);
  if (p > SERVO_PULSE_MID_US) Window_Close();
  else                       Window_Open();
}

/* Fan = Servo3 sweep (TIM3_CH3 / PB0) */
void Fan_On(void)
{
  fan_running = 1;
  fan_at_max  = 0;
  fan_next_ms = HAL_GetTick() + FAN_SWEEP_INTERVAL_MS;
  SetServoPulse(&htim3, TIM_CHANNEL_3, SERVO_PULSE_MIN_US);
}

void Fan_Off(void)
{
  fan_running = 0;
  SetServoPulse(&htim3, TIM_CHANNEL_3, SERVO_PULSE_MID_US);
}

static void Fan_Task(void)
{
  if (!fan_running) return;

  uint32_t now = HAL_GetTick();
  if (now < fan_next_ms) return;

  fan_next_ms = now + FAN_SWEEP_INTERVAL_MS;

  if (fan_at_max)
  {
    SetServoPulse(&htim3, TIM_CHANNEL_3, SERVO_PULSE_MIN_US);
    fan_at_max = 0;
  }
  else
  {
    SetServoPulse(&htim3, TIM_CHANNEL_3, SERVO_PULSE_MAX_US);
    fan_at_max = 1;
  }
}

/* ========================= ESP AT helpers ========================= */
static HAL_StatusTypeDef ESP_SendCmdWait(const char *cmd, const char *expect, uint32_t timeout_ms)
{
  /* Flush pending noise to avoid matching old OK/SEND OK */
  esp_rb_tail = esp_rb_head;

  if (cmd && cmd[0] != '\0')
    HAL_UART_Transmit(&huart3, (uint8_t*)cmd, (uint16_t)strlen(cmd), 2000);

  if (expect == NULL) return HAL_OK;
  return ESP_RB_WaitFor(expect, timeout_ms) ? HAL_OK : HAL_TIMEOUT;
}

static void ESP_ConnectWiFi(void)
{
  char cmd[256];

  ESP_SendCmdWait("AT\r\n", "OK", 1500);
  ESP_SendCmdWait("ATE0\r\n", "OK", 1500);
  ESP_SendCmdWait("AT+CIPMODE=0\r\n", "OK", 2000);
  ESP_SendCmdWait("AT+CWMODE=1\r\n", "OK", 2000);

  snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PASS);
  ESP_SendCmdWait(cmd, "WIFI CONNECTED", 30000);
  ESP_SendCmdWait("",  "OK",            30000);

  ESP_SendCmdWait("AT+CIFSR\r\n", "OK", 3000);
}

static void ESP_StartHttpServer(void)
{
  char cmd[80];

  ESP_SendCmdWait("AT+CIPMUX=1\r\n", "OK", 3000);
  ESP_SendCmdWait("AT+CIPSERVER=0\r\n", "OK", 3000);

  snprintf(cmd, sizeof(cmd), "AT+CIPSERVER=1,%d\r\n", HTTP_PORT);
  ESP_SendCmdWait(cmd, "OK", 5000);

  ESP_SendCmdWait("AT+CIPSTO=60\r\n", "OK", 2000);
}

/* ========================= HTTP ========================= */
static void HTTP_SendResponse(uint8_t link_id, int status_code, const char *body)
{
  char resp[700];
  char cmd[80];

  const char *status_text = "OK";
  if (status_code == 400) status_text = "Bad Request";
  if (status_code == 404) status_text = "Not Found";

  int body_len = (body) ? (int)strlen(body) : 0;

  snprintf(resp, sizeof(resp),
           "HTTP/1.1 %d %s\r\n"
           "Content-Type: text/plain\r\n"
           "Content-Length: %d\r\n"
           "Connection: close\r\n"
           "\r\n"
           "%s",
           status_code, status_text, body_len, body ? body : "");

  snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d,%d\r\n", link_id, (int)strlen(resp));
  if (ESP_SendCmdWait(cmd, ">", 6000) != HAL_OK)
  {
    snprintf(cmd, sizeof(cmd), "AT+CIPCLOSE=%d\r\n", link_id);
    ESP_SendCmdWait(cmd, "OK", 2000);
    return;
  }

  HAL_UART_Transmit(&huart3, (uint8_t*)resp, (uint16_t)strlen(resp), 4000);
  ESP_RB_WaitFor("SEND OK", 8000);

  snprintf(cmd, sizeof(cmd), "AT+CIPCLOSE=%d\r\n", link_id);
  ESP_SendCmdWait(cmd, "OK", 3000);
}

static void NormalizeText(char *s)
{
  if (!s) return;

  /* trim leading */
  char *p = s;
  while (*p && isspace((unsigned char)*p)) p++;
  if (p != s) memmove(s, p, strlen(p) + 1);

  /* trim trailing */
  size_t n = strlen(s);
  while (n > 0 && isspace((unsigned char)s[n - 1]))
  {
    s[n - 1] = '\0';
    n--;
  }

  /* lowercase + collapse whitespace to single spaces */
  char out[256];
  size_t oi = 0;
  uint8_t was_space = 0;

  for (size_t i = 0; s[i] && oi < sizeof(out) - 1; i++)
  {
    unsigned char c = (unsigned char)s[i];
    if (isspace(c))
    {
      if (!was_space && oi > 0) out[oi++] = ' ';
      was_space = 1;
    }
    else
    {
      out[oi++] = (char)tolower(c);
      was_space = 0;
    }
  }
  out[oi] = '\0';
  strncpy(s, out, 255);
  s[255] = '\0';
}

/* Helps splitting multiple commands: turn " and " / " then " into ';' separators */
static void ReplaceTokensForSplit(char *s)
{
  if (!s) return;

  /* Simple in-place replacement by scanning and copying to a temp */
  char tmp[256];
  size_t ti = 0;

  for (size_t i = 0; s[i] && ti < sizeof(tmp) - 1; )
  {
    if (strncmp(&s[i], " and ", 5) == 0)
    {
      tmp[ti++] = ';';
      i += 5;
      continue;
    }
    if (strncmp(&s[i], " then ", 6) == 0)
    {
      tmp[ti++] = ';';
      i += 6;
      continue;
    }
    tmp[ti++] = s[i++];
  }
  tmp[ti] = '\0';

  strncpy(s, tmp, 255);
  s[255] = '\0';
}

static int ParseContentLength(const char *hdr)
{
  const char *p = strstr(hdr, "Content-Length:");
  if (!p) p = strstr(hdr, "content-length:");
  if (!p) return -1;

  p = strchr(p, ':');
  if (!p) return -1;
  p++;

  while (*p && isspace((unsigned char)*p)) p++;
  return atoi(p);
}

/* Single command execution */
static int ProcessInstruction(const char *cmd)
{
  if (!cmd || cmd[0] == '\0') return 0;

  /* Light */
  if (strcmp(cmd, "light on") == 0 || strcmp(cmd, "light on") == 0)   { Light_On();  return 1; }
  if (strcmp(cmd, "light off") == 0 || strcmp(cmd, "light off") == 0) { Light_Off(); return 1; }

  /* Heater */
  if (strcmp(cmd, "heater on") == 0 || strcmp(cmd, "heater on") == 0)   { Heater_On();  return 1; }
  if (strcmp(cmd, "heater off") == 0 || strcmp(cmd, "heater off") == 0) { Heater_Off(); return 1; }

  /* Door */
  if (strcmp(cmd, "open the door") == 0 || strcmp(cmd, "open door") == 0)    { Door_Open();   return 1; }
  if (strcmp(cmd, "close the door") == 0 || strcmp(cmd, "close door") == 0)  { Door_Close();  return 1; }
  if (strcmp(cmd, "door reverse") == 0 || strcmp(cmd, "door other way") == 0) { Door_Reverse(); return 1; }

  /* Window */
  if (strcmp(cmd, "open the window") == 0 || strcmp(cmd, "open window") == 0)     { Window_Open();   return 1; }
  if (strcmp(cmd, "close the window") == 0 || strcmp(cmd, "close window") == 0)   { Window_Close();  return 1; }
  if (strcmp(cmd, "window reverse") == 0 || strcmp(cmd, "window other way") == 0) { Window_Reverse(); return 1; }

  /* Fan */
  if (strcmp(cmd, "fan on") == 0 || strcmp(cmd, "turn on fan") == 0)  { Fan_On();  return 1; }
  if (strcmp(cmd, "fan off") == 0 || strcmp(cmd, "turn off fan") == 0){ Fan_Off(); return 1; }

  return 0;
}

/* Multiple commands in one body: separated by ';' or newlines, also supports "and"/"then" */
static int ProcessInstructionList(const char *body)
{
  if (!body) return 0;

  char work[256];
  memset(work, 0, sizeof(work));
  strncpy(work, body, sizeof(work) - 1);

  NormalizeText(work);
  ReplaceTokensForSplit(work);

  int ok_any = 0;
  int count = 0;

  char *saveptr = NULL;
  char *tok = strtok_r(work, ";\r\n|", &saveptr);

  while (tok && count < MAX_COMMANDS_PER_MSG)
  {
    char cmd[256];
    memset(cmd, 0, sizeof(cmd));
    strncpy(cmd, tok, sizeof(cmd) - 1);
    NormalizeText(cmd);

    if (cmd[0] != '\0')
    {
      if (ProcessInstruction(cmd)) ok_any = 1;
      count++;
    }

    tok = strtok_r(NULL, ";\r\n|", &saveptr);
  }

  return ok_any;
}

/* --- HTTP chunk reassembly --- */
static void HTTP_OnDataChunk(uint8_t link_id, const char *chunk, uint16_t chunk_len)
{
  if (link_id >= MAX_LINKS) return;

  HttpConn *c = &g_http[link_id];

  if (!c->active)
  {
    c->active = 1;
    c->len = 0;
    c->buf[0] = '\0';
  }

  c->last_tick = HAL_GetTick();

  uint16_t space = (uint16_t)(HTTP_BUF_SIZE - 1 - c->len);
  if (chunk_len > space) chunk_len = space;
  if (chunk_len == 0) return;

  memcpy(&c->buf[c->len], chunk, chunk_len);
  c->len += chunk_len;
  c->buf[c->len] = '\0';

  HTTP_TryProcess(link_id);
}

static void HTTP_ResetConn(uint8_t link_id)
{
  if (link_id >= MAX_LINKS) return;
  g_http[link_id].active = 0;
  g_http[link_id].len = 0;
  g_http[link_id].last_tick = 0;
  g_http[link_id].buf[0] = '\0';
}

static void HTTP_TryProcess(uint8_t link_id)
{
  HttpConn *c = &g_http[link_id];
  if (!c->active || c->len == 0) return;

  char *hdr_end = strstr(c->buf, "\r\n\r\n");
  if (!hdr_end) return;

  int header_len = (int)(hdr_end - c->buf) + 4;

  char method[8] = {0};
  char path[64]  = {0};
  if (sscanf(c->buf, "%7s %63s", method, path) != 2)
  {
    HTTP_SendResponse(link_id, 400, "Malformed HTTP request.\n");
    HTTP_ResetConn(link_id);
    return;
  }

  /* GET routes */
  if (strcmp(method, "GET") == 0)
  {
    if (strcmp(path, "/") == 0 || strcmp(path, "/status") == 0)
    {
      HTTP_SendResponse(link_id, 200, "server is running\n");
      HTTP_ResetConn(link_id);
      return;
    }

    if (strcmp(path, "/last") == 0)
    {
      HTTP_SendResponse(link_id, 200, g_last_msg);
      HTTP_ResetConn(link_id);
      return;
    }

    HTTP_SendResponse(link_id, 404, "Not Found\n");
    HTTP_ResetConn(link_id);
    return;
  }

  /* POST /msg */
  if (strcmp(method, "POST") == 0 && strcmp(path, "/msg") == 0)
  {
    int cl = ParseContentLength(c->buf);
    if (cl < 0)
    {
      HTTP_SendResponse(link_id, 400, "Missing Content-Length.\n");
      HTTP_ResetConn(link_id);
      return;
    }

    int total_needed = header_len + cl;
    if ((int)c->len < total_needed)
    {
      /* Wait for full body */
      return;
    }

    const char *body_ptr = c->buf + header_len;

    char msg[220];
    memset(msg, 0, sizeof(msg));
    int copy_n = cl;
    if (copy_n > (int)sizeof(msg) - 1) copy_n = (int)sizeof(msg) - 1;
    memcpy(msg, body_ptr, copy_n);
    msg[copy_n] = '\0';

    /* Save normalized full body for /last */
    NormalizeText(msg);
    strncpy(g_last_msg, msg, sizeof(g_last_msg) - 1);
    g_last_msg[sizeof(g_last_msg) - 1] = '\0';

    if (msg[0] == '\0')
    {
      HTTP_SendResponse(link_id, 400, "Empty body.\n");
      HTTP_ResetConn(link_id);
      return;
    }

    if (ProcessInstructionList(msg))
      HTTP_SendResponse(link_id, 200, "OK\n");
    else
      HTTP_SendResponse(link_id, 200, "UNKNOWN COMMAND\n");

    HTTP_ResetConn(link_id);
    return;
  }

  HTTP_SendResponse(link_id, 404, "Not Found\n");
  HTTP_ResetConn(link_id);
}

/* Parse +IPD and feed HTTP reassembly */
static void ESP_ServerTask(void)
{
  /* Reset stuck partial requests */
  uint32_t now = HAL_GetTick();
  for (uint8_t i = 0; i < MAX_LINKS; i++)
  {
    if (g_http[i].active && (now - g_http[i].last_tick) > HTTP_STUCK_TIMEOUT_MS)
      HTTP_ResetConn(i);
  }

  if (!ESP_RB_FindSubstring("+IPD,")) return;

  /* Consume until "+IPD," */
  const char *pat = "+IPD,";
  uint8_t match = 0;
  uint8_t b;

  while (ESP_RB_ReadByte(&b))
  {
    if ((char)b == pat[match]) match++;
    else match = ((char)b == pat[0]) ? 1 : 0;

    if (match == 5) break;
  }

  /* Read header until ':' */
  char hdr[96];
  uint16_t hi = 0;
  memset(hdr, 0, sizeof(hdr));

  while (hi < (sizeof(hdr) - 1))
  {
    if (!ESP_RB_ReadN((uint8_t*)&b, 1, 2000)) return;
    hdr[hi++] = (char)b;
    if (b == (uint8_t)':') break;
  }

  /* hdr like "<id>,<len>:" or "<id>,<len>,ip,port:" */
  char *comma = strchr(hdr, ',');
  if (!comma) return;

  *comma = '\0';
  uint8_t link_id = (uint8_t)atoi(hdr);
  uint16_t payload_len = (uint16_t)atoi(comma + 1);

  if (payload_len == 0 || payload_len > 1800) return;

  static char payload[1900];
  memset(payload, 0, sizeof(payload));

  if (!ESP_RB_ReadN((uint8_t*)payload, payload_len, 4000)) return;
  payload[payload_len] = '\0';

  HTTP_OnDataChunk(link_id, payload, payload_len);
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();

  /* Start UART3 RX interrupts */
  ESP_RxStartIT();

  /* Start PWM for 3 servos */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // PA6
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // PA7
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // PB0

  /* Safe initial states */
  Light_Off();
  Heater_Off();
  Fan_Off();

  Door_Close();
  Window_Close();
  SetServoPulse(&htim3, TIM_CHANNEL_3, SERVO_PULSE_MID_US);

  /* Reset HTTP reassembly states */
  for (uint8_t i = 0; i < MAX_LINKS; i++)
    HTTP_ResetConn(i);

  /* ESP setup */
  ESP_ConnectWiFi();
  ESP_StartHttpServer();

  while (1)
  {
    Fan_Task();
    ESP_ServerTask();
  }
}

/* ================= CLOCK CONFIG (168 MHz, HSE=8MHz) ================= */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;

  /* FIX for your build error: PLL fields are inside RCC_OscInitStruct.PLL.* */
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    Error_Handler();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
