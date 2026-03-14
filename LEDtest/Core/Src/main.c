/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>

#include "sonar_app.h"
#include "sonar_protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_HEARTBEAT_MS 500U
#define ZEROING_DURATION_MS 800U
#define SCAN_POINT_INTERVAL_MS 60U
#define DEVICE_INFO_BROADCAST_MS 10000U
#define UART_STREAM_BUFFER_SIZE 512U
#define UART_TX_FRAME_BUFFER_SIZE 1040U
#define WAVE_SAMPLE_BYTE_MAX 8192U
#define WAVE_CHUNK_DATA_SIZE 240U
#define DEVICE_SERIAL_NUMBER "123456789"
#define APP_PI_F 3.14159265358979323846f
#define APP_CAPABILITY_SCAN (1UL << 0)
#define APP_CAPABILITY_SINGLE_DEBUG (1UL << 1)
#define APP_CAPABILITY_SWEEP_DEBUG (1UL << 2)
#define APP_CAPABILITY_RAW_WAVE (1UL << 3)
#define APP_CAPABILITY_ENVELOPE (1UL << 4)
#define APP_CAPABILITY_RECORD (1UL << 5)
#define APP_CAPABILITIES (APP_CAPABILITY_SCAN | APP_CAPABILITY_SINGLE_DEBUG | APP_CAPABILITY_SWEEP_DEBUG | APP_CAPABILITY_RAW_WAVE | APP_CAPABILITY_ENVELOPE | APP_CAPABILITY_RECORD)
#define EVENT_ZEROING_START 0x01U
#define EVENT_ZEROING_DONE 0x02U
#define EVENT_MOVE_START 0x03U
#define EVENT_MOVE_ARRIVED 0x04U
#define EVENT_MEASURE_START 0x05U
#define EVENT_MEASURE_DONE 0x06U
#define EVENT_UPLOAD_START 0x07U
#define EVENT_UPLOAD_DONE 0x08U
#define CAPTURE_TYPE_RAW_WAVE 0x00U
#define CAPTURE_TYPE_ENVELOPE 0x01U
#define WAVE_DATA_TYPE_U8_RAW 0x00U
#define WAVE_DATA_TYPE_U16_RAW 0x01U
#define WAVE_DATA_TYPE_U16_ENVELOPE 0x02U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t led_last_toggle_ms = 0U;
static uint8_t uart_rx_byte = 0U;
static uint8_t uart_stream_buffer[UART_STREAM_BUFFER_SIZE];
static volatile uint16_t uart_stream_length = 0U;
static uint8_t frame_payload_buffer[SONAR_PROTOCOL_MAX_PAYLOAD];
static uint8_t uart_tx_frame_buffer[UART_TX_FRAME_BUFFER_SIZE];
static uint8_t g_wave_bytes[WAVE_SAMPLE_BYTE_MAX];

static SonarAppContext g_sonar_app;
static uint32_t g_state_entered_ms = 0U;
static uint32_t g_scan_last_emit_ms = 0U;
static uint16_t g_scan_point_index = 0U;
static uint16_t g_scan_point_count = 0U;
static uint16_t g_scan_cycle = 0U;
static uint16_t g_active_scan_sequence = 0U;
static uint16_t g_pending_zero_sequence = 0U;
static uint8_t g_pending_zero_command = 0U;
static uint8_t g_pending_zero_valid = 0U;
static uint32_t g_last_device_info_broadcast_ms = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void App_StartUartReception(void);
static void App_ProcessHeartbeat(void);
static void App_ProcessProtocol(void);
static void App_ProcessStateMachine(void);
static void App_ProcessDeviceInfoBroadcast(void);
static void App_DispatchFrame(uint8_t command, uint16_t sequence, const uint8_t *payload, uint16_t payload_length);
static void App_HandleGetDeviceInfo(uint16_t sequence);
static void App_HandleZeroing(uint16_t sequence);
static void App_HandleConfigScan(uint16_t sequence, const uint8_t *payload, uint16_t payload_length);
static void App_HandleConfigDebug(uint16_t sequence, const uint8_t *payload, uint16_t payload_length);
static void App_HandleStartScan(uint16_t sequence, const uint8_t *payload, uint16_t payload_length);
static void App_HandleMoveToAngle(uint16_t sequence, const uint8_t *payload, uint16_t payload_length);
static void App_HandleStartSingleMeasure(uint16_t sequence);
static void App_HandleStartDebugSweep(uint16_t sequence);
static void App_HandleStop(uint16_t sequence);
static void App_StartZeroing(uint16_t sequence, uint8_t ref_command);
static void App_FinishZeroing(void);
static void App_FinishScan(uint8_t end_reason);
static void App_SendFrame(uint8_t command, uint16_t sequence, const uint8_t *payload, uint16_t payload_length);
static void App_SendAck(uint16_t sequence, uint8_t ref_command, const uint8_t *detail, uint8_t detail_length);
static void App_SendNack(uint16_t sequence, uint8_t ref_command, SonarResultCode result);
static void App_SendStatusEvent(uint8_t event, uint16_t detail);
static void App_SendScanPoint(uint16_t point_index);
static void App_SendDistanceResult(uint16_t sequence, uint16_t angle, uint32_t distance, uint16_t quality, uint8_t valid);
static void App_SendDebugSweepPoint(uint16_t sequence, uint16_t point_index, uint16_t angle, uint32_t distance, uint16_t quality, uint8_t has_wave);
static void App_SendWaveData(uint16_t sequence, uint16_t angle);
static uint16_t App_GetScanPointCount(uint16_t scan_step);
static uint32_t App_GenerateDistance(uint16_t angle);
static uint32_t App_GenerateDistanceForAngle(uint16_t angle, uint32_t base, uint32_t span, float phase_shift);
static uint16_t App_BuildWavePayload(uint16_t angle, uint8_t capture_type, uint16_t sample_count, uint8_t *data_type);
static uint8_t App_ComputeMoveDirection(uint16_t current_angle, uint16_t target_angle);
static void App_WriteU16(uint8_t *target, uint16_t value);
static void App_WriteU32(uint8_t *target, uint32_t value);
static uint16_t App_ReadU16(const uint8_t *source);
static int32_t App_FindSof(const uint8_t *buffer, uint16_t length);
static uint16_t App_StringLength(const char *text);
static uint8_t App_BuildDeviceInfoDetail(uint8_t *detail_buffer, uint16_t buffer_size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void App_StartUartReception(void)
{
  if (HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }
}

static void App_ProcessHeartbeat(void)
{
  uint32_t now = HAL_GetTick();

  if ((now - led_last_toggle_ms) >= LED_HEARTBEAT_MS)
  {
    led_last_toggle_ms = now;
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
  }
}

static void App_ProcessProtocol(void)
{
  uint8_t version = 0U;
  uint8_t command = 0U;
  uint16_t sequence = 0U;
  uint16_t payload_length = 0U;
  uint16_t total_frame_length = 0U;

  while (1)
  {
    __disable_irq();

    if (uart_stream_length < 10U)
    {
      __enable_irq();
      return;
    }

    int32_t sof_index = App_FindSof(uart_stream_buffer, uart_stream_length);
    if (sof_index < 0)
    {
      uart_stream_length = 0U;
      __enable_irq();
      return;
    }

    if (sof_index > 0)
    {
      memmove(uart_stream_buffer, &uart_stream_buffer[sof_index], uart_stream_length - (uint16_t)sof_index);
      uart_stream_length = (uint16_t)(uart_stream_length - (uint16_t)sof_index);
    }

    if (uart_stream_length < 10U)
    {
      __enable_irq();
      return;
    }

    version = uart_stream_buffer[2];
    command = uart_stream_buffer[3];
    sequence = App_ReadU16(&uart_stream_buffer[4]);
    payload_length = App_ReadU16(&uart_stream_buffer[6]);

    if ((version != SONAR_PROTOCOL_VERSION) || (payload_length > SONAR_PROTOCOL_MAX_PAYLOAD))
    {
      memmove(uart_stream_buffer, &uart_stream_buffer[1], uart_stream_length - 1U);
      uart_stream_length--;
      __enable_irq();
      continue;
    }

    total_frame_length = (uint16_t)(2U + 6U + payload_length + 2U);
    if (uart_stream_length < total_frame_length)
    {
      __enable_irq();
      return;
    }

    uint16_t expected_crc = App_ReadU16(&uart_stream_buffer[8U + payload_length]);
    uint16_t actual_crc = SonarProtocol_Crc16(&uart_stream_buffer[2], (uint16_t)(6U + payload_length));
    if (expected_crc != actual_crc)
    {
      memmove(uart_stream_buffer, &uart_stream_buffer[1], uart_stream_length - 1U);
      uart_stream_length--;
      __enable_irq();
      continue;
    }

    if (payload_length > 0U)
    {
      memcpy(frame_payload_buffer, &uart_stream_buffer[8], payload_length);
    }

    memmove(uart_stream_buffer, &uart_stream_buffer[total_frame_length], uart_stream_length - total_frame_length);
    uart_stream_length = (uint16_t)(uart_stream_length - total_frame_length);
    __enable_irq();

    App_DispatchFrame(command, sequence, frame_payload_buffer, payload_length);
  }
}

static void App_ProcessStateMachine(void)
{
  uint32_t now = HAL_GetTick();

  if ((g_sonar_app.status == SONAR_STATUS_ZEROING) && ((now - g_state_entered_ms) >= ZEROING_DURATION_MS))
  {
    App_FinishZeroing();
  }

  if ((g_sonar_app.status == SONAR_STATUS_SCANNING) && ((now - g_scan_last_emit_ms) >= SCAN_POINT_INTERVAL_MS))
  {
    g_scan_last_emit_ms = now;
    App_SendScanPoint(g_scan_point_index);
    g_scan_point_index++;

    if (g_scan_point_index >= g_scan_point_count)
    {
      App_FinishScan(0x00U);
    }
  }

  SonarApp_Process(&g_sonar_app, now);
}

static void App_ProcessDeviceInfoBroadcast(void)
{
  uint32_t now = HAL_GetTick();

  if ((now - g_last_device_info_broadcast_ms) >= DEVICE_INFO_BROADCAST_MS)
  {
    uint8_t detail[32];
    uint8_t detail_length = App_BuildDeviceInfoDetail(detail, sizeof(detail));
    g_last_device_info_broadcast_ms = now;
    App_SendAck(0U, SONAR_CMD_GET_DEVICE_INFO, detail, detail_length);
  }
}

static void App_DispatchFrame(uint8_t command, uint16_t sequence, const uint8_t *payload, uint16_t payload_length)
{
  switch (command)
  {
    case SONAR_CMD_GET_DEVICE_INFO:
      App_HandleGetDeviceInfo(sequence);
      break;

    case SONAR_CMD_ZEROING:
      App_HandleZeroing(sequence);
      break;

    case SONAR_CMD_CONFIG_SCAN:
      App_HandleConfigScan(sequence, payload, payload_length);
      break;

    case SONAR_CMD_START_SCAN:
      App_HandleStartScan(sequence, payload, payload_length);
      break;

    case SONAR_CMD_CONFIG_DEBUG:
      App_HandleConfigDebug(sequence, payload, payload_length);
      break;

    case SONAR_CMD_MOVE_TO_ANGLE:
      App_HandleMoveToAngle(sequence, payload, payload_length);
      break;

    case SONAR_CMD_START_SINGLE_MEASURE:
      App_HandleStartSingleMeasure(sequence);
      break;

    case SONAR_CMD_START_DEBUG_SWEEP:
      App_HandleStartDebugSweep(sequence);
      break;

    case SONAR_CMD_STOP:
      App_HandleStop(sequence);
      break;

    default:
      App_SendNack(sequence, command, SONAR_RESULT_UNSUPPORTED);
      break;
  }
}

static void App_HandleGetDeviceInfo(uint16_t sequence)
{
  uint8_t detail[32];
  uint8_t detail_length = App_BuildDeviceInfoDetail(detail, sizeof(detail));
  App_SendAck(sequence, SONAR_CMD_GET_DEVICE_INFO, detail, detail_length);
}

static void App_HandleZeroing(uint16_t sequence)
{
  if ((g_sonar_app.status != SONAR_STATUS_IDLE) && (g_sonar_app.status != SONAR_STATUS_READY) && (g_sonar_app.status != SONAR_STATUS_ERROR))
  {
    App_SendNack(sequence, SONAR_CMD_ZEROING, SONAR_RESULT_INVALID_STATE);
    return;
  }

  App_StartZeroing(sequence, SONAR_CMD_ZEROING);
}

static void App_HandleConfigScan(uint16_t sequence, const uint8_t *payload, uint16_t payload_length)
{
  if (payload_length != 6U)
  {
    App_SendNack(sequence, SONAR_CMD_CONFIG_SCAN, SONAR_RESULT_INVALID_PARAM);
    return;
  }

  if ((g_sonar_app.status != SONAR_STATUS_IDLE) && (g_sonar_app.status != SONAR_STATUS_READY))
  {
    App_SendNack(sequence, SONAR_CMD_CONFIG_SCAN, SONAR_RESULT_INVALID_STATE);
    return;
  }

  uint16_t scan_step = App_ReadU16(&payload[0]);
  uint16_t sound_speed = App_ReadU16(&payload[2]);
  uint8_t medium = payload[4];
  uint8_t flags = payload[5];

  if ((scan_step == 0U) || (scan_step > 36000U) || (sound_speed < 100U) || (sound_speed > 3000U) || (medium > 3U))
  {
    App_SendNack(sequence, SONAR_CMD_CONFIG_SCAN, SONAR_RESULT_INVALID_PARAM);
    return;
  }

  g_sonar_app.scan_config.scan_step = scan_step;
  g_sonar_app.scan_config.sound_speed = sound_speed;
  g_sonar_app.scan_config.medium = medium;
  g_sonar_app.scan_config.flags = flags;
  g_sonar_app.mode = SONAR_APP_MODE_SCAN;

  App_StartZeroing(sequence, SONAR_CMD_CONFIG_SCAN);
}

static void App_HandleConfigDebug(uint16_t sequence, const uint8_t *payload, uint16_t payload_length)
{
  uint8_t detail[2] = {1U, 1U};

  if (payload_length != 9U)
  {
    App_SendNack(sequence, SONAR_CMD_CONFIG_DEBUG, SONAR_RESULT_INVALID_PARAM);
    return;
  }

  if ((g_sonar_app.status != SONAR_STATUS_IDLE) && (g_sonar_app.status != SONAR_STATUS_READY))
  {
    App_SendNack(sequence, SONAR_CMD_CONFIG_DEBUG, SONAR_RESULT_INVALID_STATE);
    return;
  }

  g_sonar_app.debug_config.start_angle = App_ReadU16(&payload[0]);
  g_sonar_app.debug_config.end_angle = App_ReadU16(&payload[2]);
  g_sonar_app.debug_config.step_angle = App_ReadU16(&payload[4]);
  g_sonar_app.debug_config.capture_type = payload[6];
  g_sonar_app.debug_config.sample_count = App_ReadU16(&payload[7]);
  g_sonar_app.debug_config.flags = payload[8];

  if ((g_sonar_app.debug_config.capture_type > CAPTURE_TYPE_ENVELOPE) ||
      (g_sonar_app.debug_config.sample_count < 32U) ||
      (g_sonar_app.debug_config.sample_count > 4096U))
  {
    App_SendNack(sequence, SONAR_CMD_CONFIG_DEBUG, SONAR_RESULT_INVALID_PARAM);
    return;
  }

  if (g_sonar_app.debug_config.step_angle == 0U)
  {
    g_sonar_app.debug_config.step_angle = 100U;
  }

  g_sonar_app.mode = SONAR_APP_MODE_DEBUG_SINGLE;
  g_sonar_app.status = SONAR_STATUS_READY;
  App_SendAck(sequence, SONAR_CMD_CONFIG_DEBUG, detail, sizeof(detail));
}

static void App_HandleStartScan(uint16_t sequence, const uint8_t *payload, uint16_t payload_length)
{
  uint8_t detail[1] = {1U};

  if ((payload_length != 1U) || (payload[0] != 0x01U))
  {
    App_SendNack(sequence, SONAR_CMD_START_SCAN, SONAR_RESULT_INVALID_PARAM);
    return;
  }

  if (g_sonar_app.status != SONAR_STATUS_READY)
  {
    App_SendNack(sequence, SONAR_CMD_START_SCAN, SONAR_RESULT_INVALID_STATE);
    return;
  }

  g_active_scan_sequence = sequence;
  g_scan_point_index = 0U;
  g_scan_point_count = App_GetScanPointCount(g_sonar_app.scan_config.scan_step);
  g_scan_cycle++;
  g_sonar_app.status = SONAR_STATUS_SCANNING;
  g_sonar_app.mode = SONAR_APP_MODE_SCAN;
  g_scan_last_emit_ms = HAL_GetTick();
  g_state_entered_ms = g_scan_last_emit_ms;

  App_SendAck(sequence, SONAR_CMD_START_SCAN, detail, sizeof(detail));
  App_SendStatusEvent(EVENT_MOVE_START, g_scan_point_count);
}

static void App_HandleMoveToAngle(uint16_t sequence, const uint8_t *payload, uint16_t payload_length)
{
  uint8_t detail[6];
  uint16_t target_angle = 0U;
  uint8_t move_dir = 0U;

  if (payload_length != 2U)
  {
    App_SendNack(sequence, SONAR_CMD_MOVE_TO_ANGLE, SONAR_RESULT_INVALID_PARAM);
    return;
  }

  if (g_sonar_app.status != SONAR_STATUS_READY)
  {
    App_SendNack(sequence, SONAR_CMD_MOVE_TO_ANGLE, SONAR_RESULT_INVALID_STATE);
    return;
  }

  target_angle = App_ReadU16(payload);
  move_dir = App_ComputeMoveDirection(g_sonar_app.current_angle, target_angle);

  g_sonar_app.status = SONAR_STATUS_MOVING;
  g_sonar_app.target_angle = target_angle;
  App_SendStatusEvent(EVENT_MOVE_START, target_angle);

  SonarApp_OnMoveComplete(&g_sonar_app, target_angle);
  App_SendStatusEvent(EVENT_MOVE_ARRIVED, target_angle);

  App_WriteU16(&detail[0], g_sonar_app.current_angle);
  App_WriteU16(&detail[2], target_angle);
  detail[4] = move_dir;
  detail[5] = 1U;
  App_SendAck(sequence, SONAR_CMD_MOVE_TO_ANGLE, detail, sizeof(detail));
}

static void App_HandleStartSingleMeasure(uint16_t sequence)
{
  uint8_t detail[1] = {1U};
  uint32_t distance = 0U;

  if (g_sonar_app.status != SONAR_STATUS_READY)
  {
    App_SendNack(sequence, SONAR_CMD_START_SINGLE_MEASURE, SONAR_RESULT_INVALID_STATE);
    return;
  }

  g_sonar_app.status = SONAR_STATUS_MEASURING;
  g_sonar_app.mode = SONAR_APP_MODE_DEBUG_SINGLE;
  App_SendAck(sequence, SONAR_CMD_START_SINGLE_MEASURE, detail, sizeof(detail));
  App_SendStatusEvent(EVENT_MEASURE_START, 0U);

  if ((g_sonar_app.debug_config.flags & 0x08U) != 0U)
  {
    App_SendWaveData(sequence, g_sonar_app.current_angle);
  }

  if ((g_sonar_app.debug_config.flags & 0x04U) != 0U)
  {
    distance = App_GenerateDistanceForAngle(g_sonar_app.current_angle, 950UL, 500UL, 0.0f);
    App_SendDistanceResult(sequence, g_sonar_app.current_angle, distance, 96U, 1U);
  }

  SonarApp_OnMeasureComplete(&g_sonar_app);
  App_SendStatusEvent(EVENT_MEASURE_DONE, 0U);
}

static void App_HandleStartDebugSweep(uint16_t sequence)
{
  uint8_t detail[1] = {1U};
  uint16_t point_index = 0U;
  uint16_t angle = g_sonar_app.debug_config.start_angle;
  uint16_t step = g_sonar_app.debug_config.step_angle;
  uint8_t descending = 0U;
  uint8_t has_wave = ((g_sonar_app.debug_config.flags & 0x08U) != 0U) ? 1U : 0U;

  if (g_sonar_app.status != SONAR_STATUS_READY)
  {
    App_SendNack(sequence, SONAR_CMD_START_DEBUG_SWEEP, SONAR_RESULT_INVALID_STATE);
    return;
  }

  if (step == 0U)
  {
    step = 100U;
  }

  descending = (g_sonar_app.debug_config.end_angle < g_sonar_app.debug_config.start_angle) ? 1U : 0U;
  g_sonar_app.status = SONAR_STATUS_DEBUG_SCANNING;
  g_sonar_app.mode = SONAR_APP_MODE_DEBUG_SWEEP;
  App_SendAck(sequence, SONAR_CMD_START_DEBUG_SWEEP, detail, sizeof(detail));
  App_SendStatusEvent(EVENT_MOVE_START, g_sonar_app.debug_config.start_angle);

  while (1)
  {
    uint32_t distance = App_GenerateDistanceForAngle(angle, 780UL, 720UL, 0.0f);
    uint16_t quality = (uint16_t)(70U + (point_index % 25U));

    g_sonar_app.current_angle = angle;
    App_SendDebugSweepPoint(sequence, point_index, angle, distance, quality, has_wave);
    point_index++;

    if (angle == g_sonar_app.debug_config.end_angle)
    {
      break;
    }

    if (descending != 0U)
    {
      if (angle <= g_sonar_app.debug_config.end_angle + step)
      {
        angle = g_sonar_app.debug_config.end_angle;
      }
      else
      {
        angle = (uint16_t)(angle - step);
      }
    }
    else
    {
      if ((uint32_t)angle + step >= g_sonar_app.debug_config.end_angle)
      {
        angle = g_sonar_app.debug_config.end_angle;
      }
      else
      {
        angle = (uint16_t)(angle + step);
      }
    }
  }

  {
    uint8_t finish_payload[3];
    App_WriteU16(&finish_payload[0], point_index);
    finish_payload[2] = 0U;
    App_SendFrame(SONAR_CMD_DEBUG_SWEEP_FINISH, sequence, finish_payload, sizeof(finish_payload));
  }

  g_sonar_app.status = SONAR_STATUS_READY;
  App_SendStatusEvent(EVENT_MOVE_ARRIVED, g_sonar_app.current_angle);
}

static void App_HandleStop(uint16_t sequence)
{
  uint8_t detail[1] = {1U};

  if ((g_sonar_app.status == SONAR_STATUS_IDLE) || (g_sonar_app.status == SONAR_STATUS_READY))
  {
    App_SendNack(sequence, SONAR_CMD_STOP, SONAR_RESULT_INVALID_STATE);
    return;
  }

  if (g_sonar_app.status == SONAR_STATUS_ZEROING)
  {
    g_pending_zero_valid = 0U;
    SonarApp_OnZeroingComplete(&g_sonar_app);
    App_SendAck(sequence, SONAR_CMD_STOP, detail, sizeof(detail));
    App_SendStatusEvent(EVENT_MOVE_ARRIVED, 0U);
    return;
  }

  if (g_sonar_app.status != SONAR_STATUS_SCANNING)
  {
    g_sonar_app.status = SONAR_STATUS_READY;
    App_SendAck(sequence, SONAR_CMD_STOP, detail, sizeof(detail));
    App_SendStatusEvent(EVENT_MEASURE_DONE, 0U);
    return;
  }

  App_SendAck(sequence, SONAR_CMD_STOP, detail, sizeof(detail));
  App_FinishScan(0x01U);
}

static void App_StartZeroing(uint16_t sequence, uint8_t ref_command)
{
  g_pending_zero_sequence = sequence;
  g_pending_zero_command = ref_command;
  g_pending_zero_valid = 1U;
  g_state_entered_ms = HAL_GetTick();
  g_sonar_app.status = SONAR_STATUS_ZEROING;
  g_sonar_app.current_angle = 0U;
  g_sonar_app.target_angle = 0U;
  App_SendStatusEvent(EVENT_ZEROING_START, 0U);
}

static void App_FinishZeroing(void)
{
  uint8_t detail[5];

  SonarApp_OnZeroingComplete(&g_sonar_app);
  App_SendStatusEvent(EVENT_ZEROING_DONE, 0U);

  if (g_pending_zero_valid == 0U)
  {
    return;
  }

  if (g_pending_zero_command == SONAR_CMD_CONFIG_SCAN)
  {
    detail[0] = 1U;
    App_WriteU16(&detail[1], g_sonar_app.current_angle);
    App_WriteU16(&detail[3], App_GetScanPointCount(g_sonar_app.scan_config.scan_step));
    App_SendAck(g_pending_zero_sequence, SONAR_CMD_CONFIG_SCAN, detail, sizeof(detail));
  }
  else if (g_pending_zero_command == SONAR_CMD_ZEROING)
  {
    detail[0] = 1U;
    App_WriteU16(&detail[1], g_sonar_app.current_angle);
    App_SendAck(g_pending_zero_sequence, SONAR_CMD_ZEROING, detail, 3U);
  }

  g_pending_zero_valid = 0U;
}

static void App_FinishScan(uint8_t end_reason)
{
  uint8_t payload[5];

  payload[0] = (uint8_t)(g_scan_point_index & 0xFFU);
  payload[1] = (uint8_t)((g_scan_point_index >> 8U) & 0xFFU);
  payload[2] = end_reason;
  payload[3] = 0U;
  payload[4] = 0U;

  App_SendFrame(SONAR_CMD_SCAN_FINISH, g_active_scan_sequence, payload, sizeof(payload));
  g_sonar_app.status = SONAR_STATUS_READY;
  App_SendStatusEvent(EVENT_MEASURE_DONE, end_reason);
}

static void App_SendFrame(uint8_t command, uint16_t sequence, const uint8_t *payload, uint16_t payload_length)
{
  if (payload_length > SONAR_PROTOCOL_MAX_PAYLOAD)
  {
    return;
  }

  uart_tx_frame_buffer[0] = SONAR_PROTOCOL_SOF_HIGH;
  uart_tx_frame_buffer[1] = SONAR_PROTOCOL_SOF_LOW;
  uart_tx_frame_buffer[2] = SONAR_PROTOCOL_VERSION;
  uart_tx_frame_buffer[3] = command;
  App_WriteU16(&uart_tx_frame_buffer[4], sequence);
  App_WriteU16(&uart_tx_frame_buffer[6], payload_length);

  if ((payload != NULL) && (payload_length > 0U))
  {
    memcpy(&uart_tx_frame_buffer[8], payload, payload_length);
  }

  uint16_t crc = SonarProtocol_Crc16(&uart_tx_frame_buffer[2], (uint16_t)(6U + payload_length));
  App_WriteU16(&uart_tx_frame_buffer[8U + payload_length], crc);

  HAL_UART_Transmit(&huart1, uart_tx_frame_buffer, (uint16_t)(10U + payload_length), HAL_MAX_DELAY);
}

static void App_SendAck(uint16_t sequence, uint8_t ref_command, const uint8_t *detail, uint8_t detail_length)
{
  uint8_t payload[260];

  payload[0] = SONAR_RESULT_OK;
  payload[1] = (uint8_t)g_sonar_app.status;
  payload[2] = ref_command;
  payload[3] = detail_length;
  if ((detail != NULL) && (detail_length > 0U))
  {
    memcpy(&payload[4], detail, detail_length);
  }

  App_SendFrame(SONAR_CMD_ACK, sequence, payload, (uint16_t)(4U + detail_length));
}

static void App_SendNack(uint16_t sequence, uint8_t ref_command, SonarResultCode result)
{
  uint8_t payload[4];

  payload[0] = (uint8_t)result;
  payload[1] = (uint8_t)g_sonar_app.status;
  payload[2] = ref_command;
  payload[3] = (uint8_t)result;

  App_SendFrame(SONAR_CMD_NACK, sequence, payload, sizeof(payload));
}

static void App_SendStatusEvent(uint8_t event, uint16_t detail)
{
  uint8_t payload[6];

  payload[0] = event;
  payload[1] = (uint8_t)g_sonar_app.status;
  App_WriteU16(&payload[2], g_sonar_app.current_angle);
  App_WriteU16(&payload[4], detail);

  App_SendFrame(SONAR_CMD_STATUS_EVENT, 0U, payload, sizeof(payload));
}

static void App_SendScanPoint(uint16_t point_index)
{
  uint16_t angle = (uint16_t)((uint32_t)point_index * (uint32_t)g_sonar_app.scan_config.scan_step);
  uint32_t distance = 0U;
  uint16_t quality = 0U;
  uint8_t payload[12];

  if (angle > 35999U)
  {
    angle = 35999U;
  }

  g_sonar_app.current_angle = angle;
  distance = App_GenerateDistance(angle);
  quality = (uint16_t)(80U + (point_index % 20U));

  App_WriteU16(&payload[0], point_index);
  App_WriteU16(&payload[2], angle);
  App_WriteU32(&payload[4], distance);
  App_WriteU16(&payload[8], quality);
  payload[10] = 1U;
  payload[11] = 0U;

  App_SendFrame(SONAR_CMD_SCAN_POINT, g_active_scan_sequence, payload, sizeof(payload));
}

static uint16_t App_GetScanPointCount(uint16_t scan_step)
{
  if (scan_step == 0U)
  {
    return 0U;
  }

  return (uint16_t)((36000U + scan_step - 1U) / scan_step);
}

static void App_SendDistanceResult(uint16_t sequence, uint16_t angle, uint32_t distance, uint16_t quality, uint8_t valid)
{
  uint8_t payload[9];

  App_WriteU16(&payload[0], angle);
  App_WriteU32(&payload[2], distance);
  App_WriteU16(&payload[6], quality);
  payload[8] = valid;
  App_SendFrame(SONAR_CMD_DISTANCE_RESULT, sequence, payload, sizeof(payload));
}

static void App_SendDebugSweepPoint(uint16_t sequence, uint16_t point_index, uint16_t angle, uint32_t distance, uint16_t quality, uint8_t has_wave)
{
  uint8_t payload[12];

  App_WriteU16(&payload[0], point_index);
  App_WriteU16(&payload[2], angle);
  App_WriteU32(&payload[4], distance);
  App_WriteU16(&payload[8], quality);
  payload[10] = 1U;
  payload[11] = has_wave;
  App_SendFrame(SONAR_CMD_DEBUG_SWEEP_POINT, sequence, payload, sizeof(payload));
}

static void App_SendWaveData(uint16_t sequence, uint16_t angle)
{
  uint8_t data_type = WAVE_DATA_TYPE_U8_RAW;
  uint16_t total_samples = g_sonar_app.debug_config.sample_count;
  uint16_t byte_length = App_BuildWavePayload(angle, g_sonar_app.debug_config.capture_type, total_samples, &data_type);
  uint16_t total_frames = (uint16_t)((byte_length + WAVE_CHUNK_DATA_SIZE - 1U) / WAVE_CHUNK_DATA_SIZE);
  uint16_t frame_id = 0U;

  if (byte_length == 0U)
  {
    return;
  }

  g_sonar_app.status = SONAR_STATUS_UPLOADING;
  App_SendStatusEvent(EVENT_UPLOAD_START, 0U);

  while (frame_id < total_frames)
  {
    uint16_t offset = (uint16_t)(frame_id * WAVE_CHUNK_DATA_SIZE);
    uint16_t chunk_size = (uint16_t)(byte_length - offset);
    uint8_t payload[11U + WAVE_CHUNK_DATA_SIZE];

    if (chunk_size > WAVE_CHUNK_DATA_SIZE)
    {
      chunk_size = WAVE_CHUNK_DATA_SIZE;
    }

    App_WriteU16(&payload[0], frame_id);
    App_WriteU16(&payload[2], total_frames);
    App_WriteU16(&payload[4], offset);
    App_WriteU16(&payload[6], chunk_size);
    App_WriteU16(&payload[8], angle);
    payload[10] = data_type;
    memcpy(&payload[11], &g_wave_bytes[offset], chunk_size);
    App_SendFrame(SONAR_CMD_WAVE_CHUNK, sequence, payload, (uint16_t)(11U + chunk_size));
    frame_id++;
  }

  {
    uint8_t finish_payload[5];
    App_WriteU16(&finish_payload[0], angle);
    App_WriteU16(&finish_payload[2], total_samples);
    finish_payload[4] = 0U;
    App_SendFrame(SONAR_CMD_WAVE_FINISH, sequence, finish_payload, sizeof(finish_payload));
  }

  g_sonar_app.status = SONAR_STATUS_MEASURING;
  App_SendStatusEvent(EVENT_UPLOAD_DONE, 0U);
}

static uint32_t App_GenerateDistance(uint16_t angle)
{
  float phase_shift = ((float)(g_scan_cycle > 0U ? (g_scan_cycle - 1U) : 0U)) * 0.08f;
  return App_GenerateDistanceForAngle(angle, 1200UL, 900UL, phase_shift);
}

static uint32_t App_GenerateDistanceForAngle(uint16_t angle, uint32_t base, uint32_t span, float phase_shift)
{
  float radians = (((float)angle) / 100.0f) * (APP_PI_F / 180.0f) + phase_shift;
  float ripple = (0.58f * cosf(radians - 0.30f))
               + (0.27f * cosf((2.0f * radians) + 0.85f))
               + (0.15f * sinf((3.0f * radians) - 0.55f));
  float distance = ((float)base) + (ripple * (float)span);

  if (distance < 100.0f)
  {
    distance = 100.0f;
  }

  return (uint32_t)distance;
}

static uint16_t App_BuildWavePayload(uint16_t angle, uint8_t capture_type, uint16_t sample_count, uint8_t *data_type)
{
  uint16_t index = 0U;

  if ((sample_count < 32U) || (sample_count > 4096U))
  {
    sample_count = 256U;
  }

  if (capture_type == CAPTURE_TYPE_ENVELOPE)
  {
    *data_type = WAVE_DATA_TYPE_U16_ENVELOPE;
    for (index = 0U; index < sample_count; ++index)
    {
      float radians = ((float)index / (float)sample_count) * (APP_PI_F * 4.0f);
      float shifted = radians + (((float)angle) / 3000.0f);
      uint16_t value = (uint16_t)(1200.0f + (sinf(shifted) * 500.0f));
      App_WriteU16(&g_wave_bytes[index * 2U], value);
    }

    return (uint16_t)(sample_count * 2U);
  }

  *data_type = WAVE_DATA_TYPE_U8_RAW;
  for (index = 0U; index < sample_count; ++index)
  {
    float radians = ((float)index / (float)sample_count) * (APP_PI_F * 6.0f);
    float shifted = radians + (((float)angle) / 2000.0f);
    int32_t value = (int32_t)(128.0f + (sinf(shifted) * 90.0f));

    if (value < 0)
    {
      value = 0;
    }
    else if (value > 255)
    {
      value = 255;
    }

    g_wave_bytes[index] = (uint8_t)value;
  }

  return sample_count;
}

static uint8_t App_ComputeMoveDirection(uint16_t current_angle, uint16_t target_angle)
{
  uint16_t clockwise = 0U;
  uint16_t counter_clockwise = 0U;

  if (current_angle == target_angle)
  {
    return 0U;
  }

  clockwise = (uint16_t)((target_angle + 36000U - current_angle) % 36000U);
  counter_clockwise = (uint16_t)((current_angle + 36000U - target_angle) % 36000U);
  return (clockwise <= counter_clockwise) ? 1U : 2U;
}

static void App_WriteU16(uint8_t *target, uint16_t value)
{
  target[0] = (uint8_t)(value & 0xFFU);
  target[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void App_WriteU32(uint8_t *target, uint32_t value)
{
  target[0] = (uint8_t)(value & 0xFFUL);
  target[1] = (uint8_t)((value >> 8U) & 0xFFUL);
  target[2] = (uint8_t)((value >> 16U) & 0xFFUL);
  target[3] = (uint8_t)((value >> 24U) & 0xFFUL);
}

static uint16_t App_ReadU16(const uint8_t *source)
{
  return (uint16_t)(((uint16_t)source[1] << 8U) | source[0]);
}

static int32_t App_FindSof(const uint8_t *buffer, uint16_t length)
{
  for (uint16_t index = 0U; index + 1U < length; ++index)
  {
    if ((buffer[index] == SONAR_PROTOCOL_SOF_HIGH) && (buffer[index + 1U] == SONAR_PROTOCOL_SOF_LOW))
    {
      return (int32_t)index;
    }
  }

  return -1;
}

static uint16_t App_StringLength(const char *text)
{
  uint16_t length = 0U;

  while (text[length] != '\0')
  {
    length++;
  }

  return length;
}

static uint8_t App_BuildDeviceInfoDetail(uint8_t *detail_buffer, uint16_t buffer_size)
{
  uint16_t serial_length = App_StringLength(DEVICE_SERIAL_NUMBER);
  uint16_t total_length = (uint16_t)(13U + serial_length);

  if (buffer_size < total_length)
  {
    return 0U;
  }

  detail_buffer[0] = 1U;
  detail_buffer[1] = 1U;
  detail_buffer[2] = 1U;
  detail_buffer[3] = 0U;
  App_WriteU32(&detail_buffer[4], 460800UL);
  App_WriteU32(&detail_buffer[8], APP_CAPABILITIES);
  memcpy(&detail_buffer[12], DEVICE_SERIAL_NUMBER, serial_length);
  detail_buffer[12U + serial_length] = '\0';

  return (uint8_t)total_length;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (uart_stream_length < UART_STREAM_BUFFER_SIZE)
    {
      uart_stream_buffer[uart_stream_length++] = uart_rx_byte;
    }
    else
    {
      uart_stream_length = 0U;
    }

    App_StartUartReception();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    App_StartUartReception();
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  SonarApp_Init(&g_sonar_app);
  App_StartUartReception();
  App_SendStatusEvent(0x02U, 0U);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_ProcessHeartbeat();
    App_ProcessProtocol();
    App_ProcessStateMachine();
    App_ProcessDeviceInfoBroadcast();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
