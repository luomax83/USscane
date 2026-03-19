/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>

#include "motor_driver.h"
#include "sonar_app.h"
#include "sonar_protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SCAN_POINT_INTERVAL_MS 60U
#define DEVICE_INFO_BROADCAST_MS 10000U
#define UART_STREAM_BUFFER_SIZE 512U
#define UART_TX_FRAME_BUFFER_SIZE 1040U
#define WAVE_CHUNK_DATA_SIZE 240U
#define DEPTH_SAMPLE_COUNT 7500U
#define DEPTH_BUFFER_COUNT 1U
#define DEPTH_BUFFER_INVALID 0xFFU
#define DEPTH_DEFAULT_PULSE_COUNT 6U
#define DEPTH_PROBE_500KHZ_PERIOD 80U
#define DEPTH_PROBE_500KHZ_PULSE (DEPTH_PROBE_500KHZ_PERIOD / 2U)
#define DEBUG_PROBE_TYPE_SIDE_SCAN 0x00U
#define DEBUG_PROBE_TYPE_DEPTH 0x01U
#define MOTOR_SLAVE_ADDRESS 0x01U
#define MOTOR_SINGLE_TURN_STEPS 6400U
#define MOTOR_MOVE_SPEED_RPM 100U
#define MOTOR_HOME_SPEED_RPM 100U
#define MOTOR_MOTION_TIMEOUT_MS 15000U
#define MOTOR_FORWARD_START_SPEED_RPM_X10 300U
#define MOTOR_FORWARD_MAX_SPEED_RPM 500U
#define MOTOR_FORWARD_ACCEL_TIME_MS 250U
#define MOTOR_FORWARD_TOLERANCE_STEPS 50U
#define MOTOR_STOP_STABLE_POLLS 4U
#define MOTOR_TARGET_STABLE_POLLS 3U
#define MOTOR_STABLE_STEP_TOLERANCE 4U
#define MOTOR_TARGET_STEP_TOLERANCE 16U
#define MOTOR_SIDE_SCAN_ANGLE_QUANTUM_CDEG 180U
#define MOTOR_MIN_SIDE_SCAN_STEP_CDEG MOTOR_SIDE_SCAN_ANGLE_QUANTUM_CDEG
#define MOTOR_MAX_SIDE_SCAN_STEP_CDEG 1980U
#define MOTOR_DEFAULT_SIDE_SCAN_STEP_CDEG 540U
#define DEVICE_SERIAL_NUMBER "H743-EMU-001"
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
#define DEPTH_BUFFER_STATE_FREE 0x00U
#define DEPTH_BUFFER_STATE_FILLING 0x01U
#define DEPTH_BUFFER_STATE_READY 0x02U
#define DEPTH_BUFFER_STATE_UPLOADING 0x03U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef struct
{
  uint8_t state;
  uint16_t sample_count;
  uint16_t sequence;
  uint16_t angle;
} DepthBufferSlot;

typedef struct
{
  uint8_t active;
  uint8_t buffer_index;
  uint16_t frame_id;
  uint16_t total_frames;
  uint16_t total_bytes;
  uint16_t sample_count;
  uint16_t sequence;
  uint16_t angle;
} DepthUploadContext;

static uint8_t uart_rx_byte = 0U;
static uint8_t uart_stream_buffer[UART_STREAM_BUFFER_SIZE];
static volatile uint16_t uart_stream_length = 0U;
static uint8_t frame_payload_buffer[SONAR_PROTOCOL_MAX_PAYLOAD];
static uint8_t uart_tx_frame_buffer[UART_TX_FRAME_BUFFER_SIZE];
static __attribute__((section(".dma_buffer"), aligned(32))) uint16_t g_depth_samples[DEPTH_BUFFER_COUNT][DEPTH_SAMPLE_COUNT];
static DepthBufferSlot g_depth_buffer_slots[DEPTH_BUFFER_COUNT];
static DepthUploadContext g_depth_upload;
static SonarAppContext g_sonar_app;
static uint32_t g_state_entered_ms = 0U;
static uint32_t g_scan_last_emit_ms = 0U;
static uint16_t g_scan_point_index = 0U;
static uint16_t g_scan_point_count = 0U;
static uint16_t g_scan_cycle = 0U;
static uint16_t g_active_scan_sequence = 0U;
static uint32_t g_last_device_info_broadcast_ms = 0U;
static uint16_t g_active_measure_sequence = 0U;
static volatile uint8_t g_depth_capture_active = 0U;
static volatile uint8_t g_depth_capture_complete = 0U;
static volatile uint8_t g_depth_capture_timeout = 0U;
static volatile uint8_t g_depth_pulse_count = 0U;
static ADC_HandleTypeDef *g_active_capture_adc = NULL;
static uint32_t g_active_capture_pwm_channel = TIM_CHANNEL_3;
static volatile uint8_t g_depth_capture_buffer_index = DEPTH_BUFFER_INVALID;
static volatile uint8_t g_depth_completed_buffer_index = DEPTH_BUFFER_INVALID;
static volatile uint8_t g_depth_timeout_buffer_index = DEPTH_BUFFER_INVALID;
static uint8_t g_depth_buffer_round_robin = 0U;
static MotorDriverContext g_motor_driver;
volatile uint32_t g_debug_capture_request = 0U;
volatile uint32_t g_debug_capture_done = 0U;
volatile uint32_t g_debug_capture_result = 0U;
volatile uint32_t g_debug_upload_stage = 0U;
volatile uint32_t g_debug_upload_chunks = 0U;
volatile uint32_t g_debug_fault_marker = 0U;
volatile uint32_t g_debug_motor_request = 0U;
volatile uint32_t g_debug_motor_done = 0U;
volatile uint32_t g_debug_motor_result = 0U;
volatile uint32_t g_debug_motor_status = 0U;
volatile int32_t g_debug_motor_actual_steps = 0;
volatile uint32_t g_debug_motor_target_angle = 0U;
volatile uint32_t g_debug_motor_arrived_angle = 0U;
static volatile uint32_t g_capture_started_cycles = 0U;
static volatile uint32_t g_capture_finished_cycles = 0U;
static volatile uint32_t g_last_capture_sample_rate_hz = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void App_StartUartReception(void);
static void App_PollUartReception(void);
static void App_ProcessProtocol(void);
static void App_ProcessStateMachine(void);
static void App_ProcessDeviceInfoBroadcast(void);
static void App_ProcessMotorDiagnostic(void);
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
static void App_FinishScan(uint8_t end_reason);
static void App_SendFrame(uint8_t command, uint16_t sequence, const uint8_t *payload, uint16_t payload_length);
static void App_SendAck(uint16_t sequence, uint8_t ref_command, const uint8_t *detail, uint8_t detail_length);
static void App_SendNack(uint16_t sequence, uint8_t ref_command, SonarResultCode result);
static uint16_t App_AppendMotorTrace(uint8_t *payload, uint16_t payload_length, uint16_t payload_capacity);
static void App_SendStatusEvent(uint8_t event, uint16_t detail);
static void App_SendScanPoint(uint16_t point_index);
static void App_SendDebugSweepPoint(uint16_t sequence, uint16_t point_index, uint16_t angle, uint32_t distance, uint16_t quality, uint8_t has_wave);
static uint16_t App_GetScanPointCount(uint16_t scan_step);
static uint32_t App_GenerateDistance(uint16_t angle);
static uint32_t App_GenerateDistanceForAngle(uint16_t angle, uint32_t base, uint32_t span, float phase_shift);
static uint32_t App_CalculateDistanceMmFromSamples(const uint16_t *samples, uint16_t sample_count, uint16_t sound_speed_m_s);
static void App_SendWaveFromBuffer(uint16_t sequence, uint16_t angle, uint8_t buffer_index, uint16_t sample_count);
static SonarResultCode App_RunBlockingMeasurement(uint16_t sequence, uint16_t angle, uint8_t upload_wave, uint32_t *distance_mm, uint16_t *quality);
static SonarResultCode App_CaptureSweepMeasurement(uint16_t sequence, uint32_t *distance_mm, uint16_t *quality, uint8_t upload_wave, uint8_t *buffer_index);
static uint8_t App_ComputeMoveDirection(uint16_t current_angle, uint16_t target_angle);
static void App_WriteU16(uint8_t *target, uint16_t value);
static void App_WriteU32(uint8_t *target, uint32_t value);
static uint16_t App_ReadU16(const uint8_t *source);
static int32_t App_FindSof(const uint8_t *buffer, uint16_t length);
static uint16_t App_StringLength(const char *text);
static uint8_t App_BuildDeviceInfoDetail(uint8_t *detail_buffer, uint16_t buffer_size);
static void App_StartDepthCapture(uint16_t sequence);
static void App_StopDepthCapture(void);
static void App_FinalizeDepthCapture(void);
static void App_ProcessDepthUpload(void);
static void App_InitMotorController(void);
static SonarResultCode App_MotorResultToSonarResult(MotorDriverResult result);
static SonarResultCode App_AcceptMotorWriteResult(MotorDriverResult result);
static uint16_t App_GetDefaultSideScanStepCdeg(void);
static uint16_t App_ClampSideScanStepCdeg(uint16_t step_cdeg);
static uint16_t App_NormalizeSideScanAngleCdeg(uint16_t angle_cdeg);
static uint16_t App_NormalizeMotorAngleCdeg(uint16_t angle_cdeg);
static uint16_t App_NormalizeMotorStepCdeg(uint16_t step_cdeg);
static void App_EnableCycleCounter(void);
static uint32_t App_GetApb1TimerKernelClockHz(void);
static uint32_t App_GetCaptureSampleRateHz(void);
static uint32_t App_GetValidatedCaptureSampleRateHz(void);
static SonarResultCode App_UpdateCurrentAngleFromMotor(void);
static uint16_t App_ComputeSingleTurnStepDelta(uint16_t lhs, uint16_t rhs);
static SonarResultCode App_WaitMotorForStopBySteps(uint32_t timeout_ms, uint16_t *arrived_angle);
static SonarResultCode App_WaitMotorForTargetBySteps(uint16_t target_angle, uint32_t timeout_ms, uint16_t *arrived_angle);
static SonarResultCode App_ZeroMotorBlocking(void);
static SonarResultCode App_MoveMotorToAngleBlocking(uint16_t target_angle, uint8_t *move_dir, uint16_t *arrived_angle);
static SonarResultCode App_MoveMotorByAngleDeltaBlocking(int32_t angle_delta_cdeg, uint16_t *arrived_angle);
static uint32_t App_EstimateMotorMoveWaitMs(int32_t step_delta, uint16_t speed_rpm, uint16_t accel_time_ms);
static void App_ServiceBackgroundTasks(uint32_t duration_ms);
static void App_FlushDepthUploads(uint32_t timeout_ms);
static uint8_t App_HasFreeDepthBuffer(void);
static uint8_t App_WaitForFreeDepthBuffer(uint32_t timeout_ms);
static uint8_t App_IsDepthProbeMode(void);
static uint8_t App_IsSideScanProbeMode(void);
static uint32_t App_DCacheAlignDown(uint32_t address);
static uint32_t App_DCacheAlignUp(uint32_t value);
static uint8_t App_AllocateDepthBuffer(void);
static uint8_t App_FindReadyDepthBuffer(void);
static void App_ReleaseDepthBuffer(uint8_t buffer_index);
static void App_InvalidateDepthBufferCache(uint8_t buffer_index);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void App_StartUartReception(void)
{
  __HAL_UART_CLEAR_PEFLAG(&huart2);
  __HAL_UART_CLEAR_FEFLAG(&huart2);
  __HAL_UART_CLEAR_NEFLAG(&huart2);
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  SET_BIT(huart2.Instance->RQR, USART_RQR_RXFRQ);
  if (HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1U) != HAL_OK)
  {
    __HAL_UART_CLEAR_PEFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_OREFLAG(&huart2);
  }
}

static void App_PollUartReception(void)
{
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

    {
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
      uart_stream_length = 0U;
      __enable_irq();
      continue;
    }

    total_frame_length = (uint16_t)(2U + 6U + payload_length + 2U);
    if (uart_stream_length < total_frame_length)
    {
      __enable_irq();
      return;
    }

    {
      uint16_t expected_crc = App_ReadU16(&uart_stream_buffer[8U + payload_length]);
      uint16_t actual_crc = SonarProtocol_Crc16(&uart_stream_buffer[2], (uint16_t)(6U + payload_length));
      if (expected_crc != actual_crc)
      {
        uart_stream_length = 0U;
        __enable_irq();
        continue;
      }
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

  if ((g_debug_capture_request != 0U) &&
      (g_depth_capture_active == 0U) &&
      ((g_sonar_app.status == SONAR_STATUS_IDLE) || (g_sonar_app.status == SONAR_STATUS_READY) || (g_sonar_app.status == SONAR_STATUS_ERROR)))
  {
    g_debug_capture_request = 0U;
    g_debug_capture_done = 0U;
    g_debug_capture_result = 0U;
    g_debug_upload_stage = 0U;
    g_debug_upload_chunks = 0U;
    g_sonar_app.debug_config.probe_type = DEBUG_PROBE_TYPE_DEPTH;
    g_sonar_app.debug_config.capture_type = CAPTURE_TYPE_RAW_WAVE;
    g_sonar_app.debug_config.sample_count = DEPTH_SAMPLE_COUNT;
    if (g_sonar_app.debug_config.pulse_count == 0U)
    {
      g_sonar_app.debug_config.pulse_count = DEPTH_DEFAULT_PULSE_COUNT;
    }
    g_sonar_app.debug_config.flags = 0U;
    g_sonar_app.mode = SONAR_APP_MODE_DEBUG_SINGLE;
    g_sonar_app.status = SONAR_STATUS_MEASURING;
    g_active_measure_sequence = 0U;
    App_StartDepthCapture(0U);
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

  if ((g_depth_capture_complete != 0U) && (g_depth_capture_active == 0U))
  {
    g_depth_capture_complete = 0U;
    App_FinalizeDepthCapture();
    g_debug_capture_done = 1U;
    g_debug_capture_result = 1U;
  }

  if ((g_depth_capture_timeout != 0U) && (g_depth_capture_active == 0U))
  {
    if (g_depth_timeout_buffer_index < DEPTH_BUFFER_COUNT)
    {
      App_ReleaseDepthBuffer(g_depth_timeout_buffer_index);
      g_depth_timeout_buffer_index = DEPTH_BUFFER_INVALID;
    }
    g_depth_capture_timeout = 0U;
    g_debug_capture_done = 1U;
    g_debug_capture_result = 2U;
    g_sonar_app.status = SONAR_STATUS_READY;
    App_SendStatusEvent(0xE0U, 0U);
    App_SendNack(g_active_measure_sequence, SONAR_CMD_START_SINGLE_MEASURE, SONAR_RESULT_TIMEOUT);
  }

  App_ProcessDepthUpload();
  App_ProcessMotorDiagnostic();
  SonarApp_Process(&g_sonar_app, now);
}

static void App_ProcessDeviceInfoBroadcast(void)
{
  uint32_t now = HAL_GetTick();

  if ((g_sonar_app.status == SONAR_STATUS_SCANNING) ||
      (g_sonar_app.status == SONAR_STATUS_DEBUG_SCANNING) ||
      (g_sonar_app.status == SONAR_STATUS_UPLOADING) ||
      (g_sonar_app.status == SONAR_STATUS_MEASURING))
  {
    return;
  }

  if ((now - g_last_device_info_broadcast_ms) >= DEVICE_INFO_BROADCAST_MS)
  {
    uint8_t detail[32];
    uint8_t detail_length = App_BuildDeviceInfoDetail(detail, sizeof(detail));
    g_last_device_info_broadcast_ms = now;
    App_SendAck(0U, SONAR_CMD_GET_DEVICE_INFO, detail, detail_length);
  }
}

static void App_ProcessMotorDiagnostic(void)
{
  uint32_t request = g_debug_motor_request;
  SonarResultCode result = SONAR_RESULT_OK;
  uint8_t move_dir = 0U;
  uint16_t arrived_angle = 0U;

  if (request == 0U)
  {
    return;
  }

  g_debug_motor_request = 0U;
  g_debug_motor_done = 0U;
  g_debug_motor_result = 0U;
  g_debug_motor_status = 0U;
  g_debug_motor_actual_steps = 0;
  g_debug_motor_arrived_angle = 0U;

  switch (request)
  {
    case 1U:
    {
      uint16_t status = 0U;
      MotorDriverResult driver_result = MotorDriver_ReadStatus(&g_motor_driver, &status);
      g_debug_motor_status = status;
      g_debug_motor_result = (uint32_t)driver_result;
      break;
    }
    case 2U:
    {
      int32_t actual_steps = 0;
      MotorDriverResult driver_result = MotorDriver_ReadActualSteps(&g_motor_driver, &actual_steps);
      g_debug_motor_actual_steps = actual_steps;
      g_debug_motor_result = (uint32_t)driver_result;
      break;
    }
    case 3U:
      result = App_MoveMotorToAngleBlocking((uint16_t)g_debug_motor_target_angle, &move_dir, &arrived_angle);
      g_debug_motor_arrived_angle = arrived_angle;
      g_debug_motor_result = (uint32_t)result;
      break;
    case 4U:
      result = App_ZeroMotorBlocking();
      g_debug_motor_result = (uint32_t)result;
      break;
    default:
      g_debug_motor_result = 0xEEEE0000UL | request;
      break;
  }

  g_debug_motor_done = 1U;
}

static void App_InitMotorController(void)
{
  int32_t actual_steps = 0;

  MotorDriver_Init(&g_motor_driver, &huart4, MOTOR_SLAVE_ADDRESS, MOTOR_SINGLE_TURN_STEPS);
  g_motor_driver.response_timeout_ms = 1000U;
  HAL_Delay(100U);

  if (MotorDriver_ReadActualSteps(&g_motor_driver, &actual_steps) == MOTOR_DRIVER_OK)
  {
    g_sonar_app.current_angle = MotorDriver_SingleTurnStepsToAngleCdeg(
        &g_motor_driver,
        MotorDriver_ActualStepsToSingleTurnSteps(&g_motor_driver, actual_steps));
    g_sonar_app.target_angle = g_sonar_app.current_angle;
  }
  else
  {
    g_sonar_app.current_angle = 0U;
    g_sonar_app.target_angle = 0U;
  }
}

static SonarResultCode App_MotorResultToSonarResult(MotorDriverResult result)
{
  switch (result)
  {
    case MOTOR_DRIVER_OK:
      return SONAR_RESULT_OK;
    case MOTOR_DRIVER_TIMEOUT:
      return SONAR_RESULT_TIMEOUT;
    case MOTOR_DRIVER_INVALID_PARAM:
      return SONAR_RESULT_INVALID_PARAM;
    case MOTOR_DRIVER_MOTION_ERROR:
      return SONAR_RESULT_FAIL;
    case MOTOR_DRIVER_UART_ERROR:
    case MOTOR_DRIVER_PROTOCOL_ERROR:
    case MOTOR_DRIVER_REMOTE_ERROR:
    default:
      return SONAR_RESULT_HARDWARE_ERROR;
  }
}

static SonarResultCode App_AcceptMotorWriteResult(MotorDriverResult result)
{
  if ((result == MOTOR_DRIVER_OK) || (result == MOTOR_DRIVER_TIMEOUT))
  {
    return SONAR_RESULT_OK;
  }

  return App_MotorResultToSonarResult(result);
}

static uint16_t App_GetDefaultSideScanStepCdeg(void)
{
  return App_NormalizeMotorStepCdeg(MOTOR_DEFAULT_SIDE_SCAN_STEP_CDEG);
}

static uint16_t App_ClampSideScanStepCdeg(uint16_t step_cdeg)
{
  if (step_cdeg < MOTOR_MIN_SIDE_SCAN_STEP_CDEG)
  {
    step_cdeg = MOTOR_MIN_SIDE_SCAN_STEP_CDEG;
  }

  if (step_cdeg > MOTOR_MAX_SIDE_SCAN_STEP_CDEG)
  {
    step_cdeg = MOTOR_MAX_SIDE_SCAN_STEP_CDEG;
  }

  return App_NormalizeMotorStepCdeg(step_cdeg);
}

static uint16_t App_NormalizeSideScanAngleCdeg(uint16_t angle_cdeg)
{
  uint32_t normalized = angle_cdeg % 36000U;
  normalized = ((normalized + (MOTOR_SIDE_SCAN_ANGLE_QUANTUM_CDEG / 2U)) / MOTOR_SIDE_SCAN_ANGLE_QUANTUM_CDEG) * MOTOR_SIDE_SCAN_ANGLE_QUANTUM_CDEG;
  if (normalized >= 36000U)
  {
    normalized = 0U;
  }
  return (uint16_t)normalized;
}

static uint16_t App_NormalizeMotorAngleCdeg(uint16_t angle_cdeg)
{
  return MotorDriver_NormalizeAngleCdeg(&g_motor_driver, angle_cdeg);
}

static uint16_t App_NormalizeMotorStepCdeg(uint16_t step_cdeg)
{
  uint16_t normalized = MotorDriver_NormalizeStepCdeg(&g_motor_driver, step_cdeg);

  if ((step_cdeg != 0U) && (normalized == 0U))
  {
    normalized = MotorDriver_SingleTurnStepsToAngleCdeg(&g_motor_driver, 1U);
  }

  return normalized;
}

static void App_EnableCycleCounter(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t App_GetApb1TimerKernelClockHz(void)
{
  RCC_ClkInitTypeDef clock_config = {0};
  uint32_t flash_latency = 0U;
  uint32_t hclk_hz = HAL_RCC_GetHCLKFreq();
  uint32_t pclk1_hz = HAL_RCC_GetPCLK1Freq();
  uint32_t timer_prescaler = READ_BIT(RCC->CFGR, RCC_CFGR_TIMPRE);

  HAL_RCC_GetClockConfig(&clock_config, &flash_latency);

  if (timer_prescaler == RCC_TIMPRES_DESACTIVATED)
  {
    if ((clock_config.APB1CLKDivider == RCC_APB1_DIV1) || (clock_config.APB1CLKDivider == RCC_APB1_DIV2))
    {
      return hclk_hz;
    }

    return (2U * pclk1_hz);
  }

  if ((clock_config.APB1CLKDivider == RCC_APB1_DIV1) ||
      (clock_config.APB1CLKDivider == RCC_APB1_DIV2) ||
      (clock_config.APB1CLKDivider == RCC_APB1_DIV4))
  {
    return hclk_hz;
  }

  return (4U * pclk1_hz);
}

static uint32_t App_GetCaptureSampleRateHz(void)
{
  uint32_t timer_clock_hz = App_GetApb1TimerKernelClockHz();
  uint32_t prescaler = htim4.Init.Prescaler + 1U;
  uint32_t period = htim4.Init.Period + 1U;

  if ((timer_clock_hz == 0U) || (prescaler == 0U) || (period == 0U))
  {
    return 0U;
  }

  return timer_clock_hz / (prescaler * period);
}

static uint32_t App_GetValidatedCaptureSampleRateHz(void)
{
  if (g_last_capture_sample_rate_hz != 0U)
  {
    return g_last_capture_sample_rate_hz;
  }

  return App_GetCaptureSampleRateHz();
}

static SonarResultCode App_UpdateCurrentAngleFromMotor(void)
{
  int32_t actual_steps = 0;
  MotorDriverResult result = MotorDriver_ReadActualSteps(&g_motor_driver, &actual_steps);

  if (result != MOTOR_DRIVER_OK)
  {
    return App_MotorResultToSonarResult(result);
  }

  g_sonar_app.current_angle = MotorDriver_SingleTurnStepsToAngleCdeg(
      &g_motor_driver,
      MotorDriver_ActualStepsToSingleTurnSteps(&g_motor_driver, actual_steps));
  return SONAR_RESULT_OK;
}

static uint16_t App_ComputeSingleTurnStepDelta(uint16_t lhs, uint16_t rhs)
{
  uint16_t delta = (lhs > rhs) ? (uint16_t)(lhs - rhs) : (uint16_t)(rhs - lhs);

  if (delta > (MOTOR_SINGLE_TURN_STEPS / 2U))
  {
    delta = (uint16_t)(MOTOR_SINGLE_TURN_STEPS - delta);
  }

  return delta;
}

static SonarResultCode App_WaitMotorForStopBySteps(uint32_t timeout_ms, uint16_t *arrived_angle)
{
  uint32_t started_ms = HAL_GetTick();
  uint16_t stable_polls = 0U;
  uint16_t last_steps = 0U;
  uint8_t has_last_steps = 0U;

  HAL_Delay(150U);

  while ((HAL_GetTick() - started_ms) < timeout_ms)
  {
    int32_t actual_steps = 0;
    uint16_t current_steps = 0U;
    uint16_t status = 0U;
    MotorDriverResult status_result = MotorDriver_ReadStatus(&g_motor_driver, &status);

    if (status_result != MOTOR_DRIVER_OK)
    {
      return App_MotorResultToSonarResult(status_result);
    }

    if (MotorDriver_ReadActualSteps(&g_motor_driver, &actual_steps) != MOTOR_DRIVER_OK)
    {
      return SONAR_RESULT_HARDWARE_ERROR;
    }

    current_steps = MotorDriver_ActualStepsToSingleTurnSteps(&g_motor_driver, actual_steps);
    g_sonar_app.current_angle = MotorDriver_SingleTurnStepsToAngleCdeg(&g_motor_driver, current_steps);

    if (has_last_steps != 0U)
    {
      if (App_ComputeSingleTurnStepDelta(current_steps, last_steps) <= MOTOR_STABLE_STEP_TOLERANCE)
      {
        stable_polls++;
      }
      else
      {
        stable_polls = 0U;
      }
    }

    last_steps = current_steps;
    has_last_steps = 1U;

    if ((status == 0U) && (stable_polls >= MOTOR_STOP_STABLE_POLLS))
    {
      if (arrived_angle != NULL)
      {
        *arrived_angle = g_sonar_app.current_angle;
      }
      return SONAR_RESULT_OK;
    }

    HAL_Delay(60U);
  }

  return SONAR_RESULT_TIMEOUT;
}

static SonarResultCode App_WaitMotorForTargetBySteps(uint16_t target_angle, uint32_t timeout_ms, uint16_t *arrived_angle)
{
  uint32_t started_ms = HAL_GetTick();
  uint16_t target_steps = MotorDriver_AngleCdegToSingleTurnSteps(&g_motor_driver, target_angle);
  uint16_t stable_polls = 0U;

  HAL_Delay(120U);

  while ((HAL_GetTick() - started_ms) < timeout_ms)
  {
    int32_t actual_steps = 0;
    uint16_t current_steps = 0U;
    uint16_t status = 0U;
    MotorDriverResult status_result = MotorDriver_ReadStatus(&g_motor_driver, &status);

    if (status_result != MOTOR_DRIVER_OK)
    {
      return App_MotorResultToSonarResult(status_result);
    }

    if (MotorDriver_ReadActualSteps(&g_motor_driver, &actual_steps) != MOTOR_DRIVER_OK)
    {
      return SONAR_RESULT_HARDWARE_ERROR;
    }

    current_steps = MotorDriver_ActualStepsToSingleTurnSteps(&g_motor_driver, actual_steps);
    g_sonar_app.current_angle = MotorDriver_SingleTurnStepsToAngleCdeg(&g_motor_driver, current_steps);

    if (App_ComputeSingleTurnStepDelta(current_steps, target_steps) <= MOTOR_TARGET_STEP_TOLERANCE)
    {
      stable_polls++;
      if ((status == 0U) && (stable_polls >= MOTOR_TARGET_STABLE_POLLS))
      {
        if (arrived_angle != NULL)
        {
          *arrived_angle = g_sonar_app.current_angle;
        }
        return SONAR_RESULT_OK;
      }
    }
    else
    {
      stable_polls = 0U;
    }

    HAL_Delay(50U);
  }

  return SONAR_RESULT_TIMEOUT;
}

static SonarResultCode App_ZeroMotorBlocking(void)
{
  MotorDriverResult result = MotorDriver_Home(&g_motor_driver, MOTOR_HOME_SPEED_RPM);

  if (App_AcceptMotorWriteResult(result) != SONAR_RESULT_OK)
  {
    return App_MotorResultToSonarResult(result);
  }

  HAL_Delay(5000U);
  result = MotorDriver_SetZero(&g_motor_driver);
  if (App_AcceptMotorWriteResult(result) != SONAR_RESULT_OK)
  {
    return App_MotorResultToSonarResult(result);
  }

  HAL_Delay(200U);
  g_sonar_app.current_angle = 0U;
  g_sonar_app.target_angle = 0U;
  return SONAR_RESULT_OK;
}

static SonarResultCode App_MoveMotorToAngleBlocking(uint16_t target_angle, uint8_t *move_dir, uint16_t *arrived_angle)
{
  uint16_t normalized_angle = App_NormalizeMotorAngleCdeg(target_angle);
  uint16_t motor_steps = 0U;
  uint16_t current_angle = g_sonar_app.current_angle;
  MotorDriverResult result = MOTOR_DRIVER_OK;

  if (move_dir != NULL)
  {
    *move_dir = App_ComputeMoveDirection(current_angle, normalized_angle);
  }

  if (normalized_angle == current_angle)
  {
    g_sonar_app.target_angle = normalized_angle;
    if (arrived_angle != NULL)
    {
      *arrived_angle = normalized_angle;
    }
    return SONAR_RESULT_OK;
  }

  motor_steps = MotorDriver_AngleCdegToSingleTurnSteps(&g_motor_driver, normalized_angle);
  result = MotorDriver_MoveSingleTurn(&g_motor_driver, motor_steps, MOTOR_MOVE_SPEED_RPM, 0U);
  if (App_AcceptMotorWriteResult(result) != SONAR_RESULT_OK)
  {
    return App_MotorResultToSonarResult(result);
  }

  App_ServiceBackgroundTasks(App_EstimateMotorMoveWaitMs(
      (int32_t)MotorDriver_AngleCdegToSingleTurnSteps(&g_motor_driver, normalized_angle) -
      (int32_t)MotorDriver_AngleCdegToSingleTurnSteps(&g_motor_driver, current_angle),
      MOTOR_MOVE_SPEED_RPM,
      200U));
  g_sonar_app.current_angle = normalized_angle;
  g_sonar_app.target_angle = normalized_angle;
  if (arrived_angle != NULL)
  {
    *arrived_angle = normalized_angle;
  }
  return SONAR_RESULT_OK;
}

static SonarResultCode App_MoveMotorByAngleDeltaBlocking(int32_t angle_delta_cdeg, uint16_t *arrived_angle)
{
  int64_t step_delta = 0;
  uint32_t wait_ms = 0U;
  int32_t target_angle = 0;
  MotorDriverResult result = MOTOR_DRIVER_OK;

  if (angle_delta_cdeg == 0)
  {
    if (arrived_angle != NULL)
    {
      *arrived_angle = g_sonar_app.current_angle;
    }
    return SONAR_RESULT_OK;
  }

  step_delta = ((int64_t)angle_delta_cdeg * (int64_t)MOTOR_SINGLE_TURN_STEPS) / 36000LL;
  if (step_delta == 0)
  {
    step_delta = (angle_delta_cdeg > 0) ? 1 : -1;
  }

  result = MotorDriver_MoveForwardSteps(
      &g_motor_driver,
      (int32_t)step_delta,
      MOTOR_FORWARD_START_SPEED_RPM_X10,
      MOTOR_FORWARD_MAX_SPEED_RPM,
      MOTOR_FORWARD_ACCEL_TIME_MS,
      MOTOR_FORWARD_TOLERANCE_STEPS);
  if (App_AcceptMotorWriteResult(result) != SONAR_RESULT_OK)
  {
    return App_MotorResultToSonarResult(result);
  }

  wait_ms = App_EstimateMotorMoveWaitMs((int32_t)step_delta, MOTOR_FORWARD_MAX_SPEED_RPM, MOTOR_FORWARD_ACCEL_TIME_MS);
  App_ServiceBackgroundTasks(wait_ms);
  target_angle = (int32_t)g_sonar_app.current_angle + angle_delta_cdeg;
  while (target_angle < 0)
  {
    target_angle += 36000;
  }
  target_angle %= 36000;
  g_sonar_app.current_angle = App_NormalizeMotorAngleCdeg((uint16_t)target_angle);
  if (arrived_angle != NULL)
  {
    *arrived_angle = g_sonar_app.current_angle;
  }

  return SONAR_RESULT_OK;
}

static uint32_t App_EstimateMotorMoveWaitMs(int32_t step_delta, uint16_t speed_rpm, uint16_t accel_time_ms)
{
  uint32_t abs_steps = 0U;
  uint32_t speed_steps_per_second = 0U;
  uint32_t move_time_ms = 0U;

  if (step_delta < 0)
  {
    abs_steps = (uint32_t)(-step_delta);
  }
  else
  {
    abs_steps = (uint32_t)step_delta;
  }

  if (speed_rpm == 0U)
  {
    speed_rpm = 100U;
  }

  speed_steps_per_second = ((uint32_t)speed_rpm * MOTOR_SINGLE_TURN_STEPS) / 60U;
  if (speed_steps_per_second == 0U)
  {
    speed_steps_per_second = 1U;
  }

  move_time_ms = (abs_steps * 1000U) / speed_steps_per_second;
  move_time_ms += accel_time_ms;
  move_time_ms += 200U;

  if (move_time_ms < 300U)
  {
    move_time_ms = 300U;
  }
  if (move_time_ms > MOTOR_MOTION_TIMEOUT_MS)
  {
    move_time_ms = MOTOR_MOTION_TIMEOUT_MS;
  }

  return move_time_ms;
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
  uint16_t scan_step = 0U;
  uint16_t sound_speed = 0U;
  uint8_t medium = 0U;
  uint8_t flags = 0U;

  if (payload_length != 6U)
  {
    App_SendNack(sequence, SONAR_CMD_CONFIG_SCAN, SONAR_RESULT_INVALID_PARAM);
    return;
  }

  if ((g_sonar_app.status != SONAR_STATUS_IDLE) && (g_sonar_app.status != SONAR_STATUS_READY) && (g_sonar_app.status != SONAR_STATUS_ERROR))
  {
    App_SendNack(sequence, SONAR_CMD_CONFIG_SCAN, SONAR_RESULT_INVALID_STATE);
    return;
  }

  scan_step = App_ReadU16(&payload[0]);
  sound_speed = App_ReadU16(&payload[2]);
  medium = payload[4];
  flags = payload[5];

  scan_step = App_NormalizeMotorStepCdeg(scan_step);
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
  uint8_t detail[6] = {1U, 1U, 0U, 0U, 0U, 0U};
  uint16_t original_start = 0U;
  uint16_t original_end = 0U;
  uint16_t original_step = 0U;

  if ((payload_length != 9U) && (payload_length != 10U) && (payload_length != 12U))
  {
    App_SendNack(sequence, SONAR_CMD_CONFIG_DEBUG, SONAR_RESULT_INVALID_PARAM);
    return;
  }

  if ((g_sonar_app.status != SONAR_STATUS_IDLE) && (g_sonar_app.status != SONAR_STATUS_READY) && (g_sonar_app.status != SONAR_STATUS_ERROR))
  {
    App_SendNack(sequence, SONAR_CMD_CONFIG_DEBUG, SONAR_RESULT_INVALID_STATE);
    return;
  }

  g_sonar_app.debug_config.start_angle = App_ReadU16(&payload[0]);
  g_sonar_app.debug_config.end_angle = App_ReadU16(&payload[2]);
  g_sonar_app.debug_config.step_angle = App_ReadU16(&payload[4]);
  original_start = g_sonar_app.debug_config.start_angle;
  original_end = g_sonar_app.debug_config.end_angle;
  original_step = g_sonar_app.debug_config.step_angle;
  g_sonar_app.debug_config.capture_type = payload[6];
  if (payload_length >= 12U)
  {
    g_sonar_app.debug_config.probe_type = payload[7];
    g_sonar_app.debug_config.sample_count = App_ReadU16(&payload[8]);
    g_sonar_app.debug_config.pulse_count = payload[10];
    g_sonar_app.debug_config.flags = payload[11];
  }
  else
  {
    g_sonar_app.debug_config.probe_type = DEBUG_PROBE_TYPE_SIDE_SCAN;
    g_sonar_app.debug_config.sample_count = App_ReadU16(&payload[7]);
    g_sonar_app.debug_config.pulse_count = DEPTH_DEFAULT_PULSE_COUNT;
    g_sonar_app.debug_config.flags = (payload_length == 10U) ? payload[9] : payload[8];
  }

  if ((g_sonar_app.debug_config.capture_type > CAPTURE_TYPE_ENVELOPE) ||
      (g_sonar_app.debug_config.probe_type > DEBUG_PROBE_TYPE_DEPTH) ||
      (g_sonar_app.debug_config.pulse_count == 0U))
  {
    App_SendNack(sequence, SONAR_CMD_CONFIG_DEBUG, SONAR_RESULT_INVALID_PARAM);
    return;
  }

  if (g_sonar_app.debug_config.probe_type == DEBUG_PROBE_TYPE_DEPTH)
  {
    g_sonar_app.debug_config.start_angle = 0U;
    g_sonar_app.debug_config.end_angle = 0U;
    g_sonar_app.debug_config.step_angle = 0U;
    g_sonar_app.debug_config.capture_type = CAPTURE_TYPE_RAW_WAVE;
    g_sonar_app.debug_config.sample_count = DEPTH_SAMPLE_COUNT;
    g_sonar_app.mode = SONAR_APP_MODE_DEBUG_SINGLE;
  }
  else
  {
    if (g_sonar_app.debug_config.end_angle < g_sonar_app.debug_config.start_angle)
    {
      App_SendNack(sequence, SONAR_CMD_CONFIG_DEBUG, SONAR_RESULT_INVALID_PARAM);
      return;
    }

    g_sonar_app.debug_config.start_angle = App_NormalizeSideScanAngleCdeg(g_sonar_app.debug_config.start_angle);
    g_sonar_app.debug_config.end_angle = App_NormalizeSideScanAngleCdeg(g_sonar_app.debug_config.end_angle);
    if (g_sonar_app.debug_config.end_angle < g_sonar_app.debug_config.start_angle)
    {
      g_sonar_app.debug_config.end_angle = g_sonar_app.debug_config.start_angle;
    }
    if ((g_sonar_app.debug_config.end_angle > g_sonar_app.debug_config.start_angle) &&
        (g_sonar_app.debug_config.step_angle == 0U))
    {
      g_sonar_app.debug_config.step_angle = App_GetDefaultSideScanStepCdeg();
    }
    else if (g_sonar_app.debug_config.step_angle > 0U)
    {
      g_sonar_app.debug_config.step_angle = App_ClampSideScanStepCdeg(g_sonar_app.debug_config.step_angle);
    }

    g_sonar_app.debug_config.capture_type = CAPTURE_TYPE_RAW_WAVE;
    g_sonar_app.debug_config.sample_count = DEPTH_SAMPLE_COUNT;
  }

  if (g_sonar_app.debug_config.flags & 0x02U)
  {
    g_sonar_app.mode = SONAR_APP_MODE_DEBUG_SWEEP;
  }
  else
  {
    g_sonar_app.mode = SONAR_APP_MODE_DEBUG_SINGLE;
  }
  g_sonar_app.status = SONAR_STATUS_READY;
  detail[1] = ((original_start != g_sonar_app.debug_config.start_angle) ||
               (original_end != g_sonar_app.debug_config.end_angle) ||
               (original_step != g_sonar_app.debug_config.step_angle)) ? 1U : 0U;
  App_WriteU32(&detail[2], App_GetCaptureSampleRateHz());
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
  uint16_t arrived_angle = 0U;
  SonarResultCode move_result = SONAR_RESULT_OK;

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

  target_angle = App_NormalizeSideScanAngleCdeg(App_ReadU16(payload));
  move_dir = App_ComputeMoveDirection(g_sonar_app.current_angle, target_angle);

  g_sonar_app.status = SONAR_STATUS_MOVING;
  g_sonar_app.target_angle = target_angle;
  App_SendStatusEvent(EVENT_MOVE_START, target_angle);

  move_result = App_MoveMotorToAngleBlocking(target_angle, &move_dir, &arrived_angle);
  if (move_result != SONAR_RESULT_OK)
  {
    g_sonar_app.status = SONAR_STATUS_READY;
    App_SendNack(sequence, SONAR_CMD_MOVE_TO_ANGLE, move_result);
    return;
  }

  SonarApp_OnMoveComplete(&g_sonar_app, arrived_angle);
  App_SendStatusEvent(EVENT_MOVE_ARRIVED, g_sonar_app.current_angle);

  App_WriteU16(&detail[0], g_sonar_app.current_angle);
  App_WriteU16(&detail[2], target_angle);
  detail[4] = move_dir;
  detail[5] = (g_sonar_app.current_angle == target_angle) ? 1U : 0U;
  App_SendAck(sequence, SONAR_CMD_MOVE_TO_ANGLE, detail, sizeof(detail));
}

static void App_HandleStartSingleMeasure(uint16_t sequence)
{
  uint8_t detail[1] = {1U};
  uint8_t move_dir = 0U;
  uint16_t arrived_angle = 0U;
  SonarResultCode move_result = SONAR_RESULT_OK;

  if (g_sonar_app.status != SONAR_STATUS_READY)
  {
    App_SendNack(sequence, SONAR_CMD_START_SINGLE_MEASURE, SONAR_RESULT_INVALID_STATE);
    return;
  }

  if (App_IsSideScanProbeMode() != 0U)
  {
    g_sonar_app.status = SONAR_STATUS_MOVING;
    g_sonar_app.target_angle = g_sonar_app.debug_config.start_angle;
    App_SendStatusEvent(EVENT_MOVE_START, g_sonar_app.target_angle);
    move_result = App_MoveMotorToAngleBlocking(g_sonar_app.target_angle, &move_dir, &arrived_angle);
    if (move_result != SONAR_RESULT_OK)
    {
      g_sonar_app.status = SONAR_STATUS_READY;
      App_SendNack(sequence, SONAR_CMD_START_SINGLE_MEASURE, move_result);
      return;
    }
    SonarApp_OnMoveComplete(&g_sonar_app, arrived_angle);
    App_SendStatusEvent(EVENT_MOVE_ARRIVED, g_sonar_app.current_angle);
  }

  g_sonar_app.status = SONAR_STATUS_MEASURING;
  g_sonar_app.mode = SONAR_APP_MODE_DEBUG_SINGLE;
  g_active_measure_sequence = sequence;
  App_SendAck(sequence, SONAR_CMD_START_SINGLE_MEASURE, detail, sizeof(detail));
  App_SendStatusEvent(EVENT_MEASURE_START, 0U);

  App_StartDepthCapture(sequence);
}

static void App_HandleStartDebugSweep(uint16_t sequence)
{
  uint8_t detail[1] = {1U};
  uint8_t finish_reason = 0U;
  uint16_t point_index = 0U;
  uint16_t angle = g_sonar_app.debug_config.start_angle;
  uint16_t step = g_sonar_app.debug_config.step_angle;
  uint8_t has_wave = ((g_sonar_app.debug_config.flags & 0x08U) != 0U) ? 1U : 0U;
  uint8_t single_only = 0U;
  uint8_t move_dir = 0U;
  uint16_t arrived_angle = 0U;
  SonarResultCode move_result = SONAR_RESULT_OK;
  SonarResultCode measure_result = SONAR_RESULT_OK;
  uint32_t distance_mm = 0U;
  uint16_t quality = 0U;
  uint8_t uploaded_buffer_index = DEPTH_BUFFER_INVALID;

  if (g_sonar_app.status != SONAR_STATUS_READY)
  {
    App_SendNack(sequence, SONAR_CMD_START_DEBUG_SWEEP, SONAR_RESULT_INVALID_STATE);
    return;
  }

  if (App_IsDepthProbeMode() != 0U)
  {
    App_SendNack(sequence, SONAR_CMD_START_DEBUG_SWEEP, SONAR_RESULT_INVALID_STATE);
    return;
  }

  if (step == 0U)
  {
    step = App_GetDefaultSideScanStepCdeg();
  }

  single_only = (g_sonar_app.debug_config.end_angle == g_sonar_app.debug_config.start_angle) ? 1U : 0U;
  if ((single_only == 0U) && (step > 0U))
  {
    if ((uint32_t)(g_sonar_app.debug_config.end_angle - g_sonar_app.debug_config.start_angle) < step)
    {
      single_only = 1U;
    }
  }
  g_sonar_app.status = SONAR_STATUS_DEBUG_SCANNING;
  g_sonar_app.mode = SONAR_APP_MODE_DEBUG_SWEEP;
  App_SendAck(sequence, SONAR_CMD_START_DEBUG_SWEEP, detail, sizeof(detail));
  App_SendStatusEvent(EVENT_MOVE_START, g_sonar_app.debug_config.start_angle);

  g_sonar_app.status = SONAR_STATUS_MOVING;
  move_result = App_MoveMotorToAngleBlocking(angle, &move_dir, &arrived_angle);
  if (move_result != SONAR_RESULT_OK)
  {
    g_sonar_app.status = SONAR_STATUS_READY;
    App_SendNack(sequence, SONAR_CMD_START_DEBUG_SWEEP, move_result);
    return;
  }
  g_sonar_app.current_angle = arrived_angle;
  App_SendStatusEvent(EVENT_MOVE_ARRIVED, g_sonar_app.current_angle);
  g_sonar_app.status = SONAR_STATUS_DEBUG_SCANNING;

  while (1)
  {
    g_sonar_app.status = SONAR_STATUS_MEASURING;
    App_SendStatusEvent(EVENT_MEASURE_START, g_sonar_app.current_angle);
    measure_result = App_CaptureSweepMeasurement(sequence, &distance_mm, &quality, has_wave, &uploaded_buffer_index);
    if (measure_result != SONAR_RESULT_OK)
    {
      finish_reason = 0x03U;
      g_sonar_app.status = SONAR_STATUS_READY;
      break;
    }
    App_SendDebugSweepPoint(sequence, point_index, g_sonar_app.current_angle, distance_mm, quality, has_wave);
    App_SendStatusEvent(EVENT_MEASURE_DONE, g_sonar_app.current_angle);
    point_index++;

    if ((single_only != 0U) || (angle == g_sonar_app.debug_config.end_angle))
    {
      break;
    }

    if ((uint32_t)angle + step >= g_sonar_app.debug_config.end_angle)
    {
      angle = g_sonar_app.debug_config.end_angle;
    }
    else
    {
      angle = (uint16_t)(angle + step);
    }

    g_sonar_app.status = SONAR_STATUS_MOVING;
    move_result = App_MoveMotorByAngleDeltaBlocking((int32_t)angle - (int32_t)g_sonar_app.current_angle, &arrived_angle);
    if (move_result != SONAR_RESULT_OK)
    {
      finish_reason = 0x02U;
      g_sonar_app.status = SONAR_STATUS_READY;
      break;
    }

    if (has_wave != 0U)
    {
      App_FlushDepthUploads(5000U);
      uploaded_buffer_index = DEPTH_BUFFER_INVALID;
      if ((g_depth_upload.active != 0U) || (App_FindReadyDepthBuffer() < DEPTH_BUFFER_COUNT))
      {
        finish_reason = 0x04U;
        g_sonar_app.status = SONAR_STATUS_READY;
        break;
      }
    }

    g_sonar_app.status = SONAR_STATUS_DEBUG_SCANNING;
    g_sonar_app.current_angle = arrived_angle;
    App_SendStatusEvent(EVENT_MOVE_ARRIVED, g_sonar_app.current_angle);
  }

  if (has_wave != 0U)
  {
    App_FlushDepthUploads(5000U);
  }

  {
    uint8_t finish_payload[3];
    App_WriteU16(&finish_payload[0], point_index);
    finish_payload[2] = finish_reason;
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
    SonarApp_OnZeroingComplete(&g_sonar_app);
    App_SendAck(sequence, SONAR_CMD_STOP, detail, sizeof(detail));
    App_SendStatusEvent(EVENT_MOVE_ARRIVED, 0U);
    return;
  }

  if (g_sonar_app.status != SONAR_STATUS_SCANNING)
  {
    App_StopDepthCapture();
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
  uint8_t detail[5];
  SonarResultCode result = SONAR_RESULT_OK;

  g_sonar_app.status = SONAR_STATUS_ZEROING;
  g_sonar_app.current_angle = 0U;
  g_sonar_app.target_angle = 0U;
  App_SendStatusEvent(EVENT_ZEROING_START, 0U);
  result = App_ZeroMotorBlocking();
  if (result != SONAR_RESULT_OK)
  {
    g_sonar_app.status = SONAR_STATUS_READY;
    App_SendNack(sequence, ref_command, result);
    return;
  }

  SonarApp_OnZeroingComplete(&g_sonar_app);
  App_SendStatusEvent(EVENT_ZEROING_DONE, 0U);

  if (ref_command == SONAR_CMD_CONFIG_SCAN)
  {
    detail[0] = 1U;
    App_WriteU16(&detail[1], g_sonar_app.current_angle);
    App_WriteU16(&detail[3], App_GetScanPointCount(g_sonar_app.scan_config.scan_step));
    App_SendAck(sequence, SONAR_CMD_CONFIG_SCAN, detail, sizeof(detail));
  }
  else if (ref_command == SONAR_CMD_ZEROING)
  {
    detail[0] = 1U;
    App_WriteU16(&detail[1], g_sonar_app.current_angle);
    App_SendAck(sequence, SONAR_CMD_ZEROING, detail, 3U);
  }
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
  uint16_t crc = 0U;

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

  crc = SonarProtocol_Crc16(&uart_tx_frame_buffer[2], (uint16_t)(6U + payload_length));
  App_WriteU16(&uart_tx_frame_buffer[8U + payload_length], crc);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_UART_Transmit(&huart2, uart_tx_frame_buffer, (uint16_t)(10U + payload_length), HAL_MAX_DELAY);
  while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET)
  {
  }
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
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
  uint8_t payload[32];
  uint16_t payload_length = 4U;

  payload[0] = (uint8_t)result;
  payload[1] = (uint8_t)g_sonar_app.status;
  payload[2] = ref_command;
  payload[3] = (uint8_t)result;

  payload_length = App_AppendMotorTrace(payload, payload_length, (uint16_t)sizeof(payload));
  App_SendFrame(SONAR_CMD_NACK, sequence, payload, payload_length);
}

static uint16_t App_AppendMotorTrace(uint8_t *payload, uint16_t payload_length, uint16_t payload_capacity)
{
  uint16_t copy_length = 0U;

  if ((payload == NULL) || (payload_capacity <= payload_length) || ((payload_capacity - payload_length) < 4U))
  {
    return payload_length;
  }

  payload[payload_length++] = (uint8_t)g_motor_driver_trace.result;
  payload[payload_length++] = (uint8_t)g_motor_driver_trace.request_length;
  payload[payload_length++] = (uint8_t)g_motor_driver_trace.response_length;
  copy_length = (uint16_t)(payload_capacity - payload_length);
  if (copy_length > 6U)
  {
    copy_length = 6U;
  }
  memcpy(&payload[payload_length], (const void *)g_motor_driver_trace.request, copy_length);
  payload_length = (uint16_t)(payload_length + copy_length);

  copy_length = (uint16_t)(payload_capacity - payload_length);
  if (copy_length > 6U)
  {
    copy_length = 6U;
  }
  memcpy(&payload[payload_length], (const void *)g_motor_driver_trace.response, copy_length);
  payload_length = (uint16_t)(payload_length + copy_length);

  return payload_length;
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

static void App_ProcessDepthUpload(void)
{
  uint8_t buffer_index = 0U;
  uint8_t preserve_debug_status = (g_sonar_app.mode == SONAR_APP_MODE_DEBUG_SWEEP) ? 1U : 0U;
  SonarDeviceStatus previous_status = g_sonar_app.status;

  if (g_depth_upload.active == 0U)
  {
    buffer_index = App_FindReadyDepthBuffer();
    if (buffer_index >= DEPTH_BUFFER_COUNT)
    {
      return;
    }

    g_depth_buffer_slots[buffer_index].state = DEPTH_BUFFER_STATE_UPLOADING;
    g_depth_upload.active = 1U;
    g_depth_upload.buffer_index = buffer_index;
    g_depth_upload.frame_id = 0U;
    g_depth_upload.sequence = g_depth_buffer_slots[buffer_index].sequence;
    g_depth_upload.angle = g_depth_buffer_slots[buffer_index].angle;
    g_depth_upload.sample_count = g_depth_buffer_slots[buffer_index].sample_count;
    g_depth_upload.total_bytes = (uint16_t)(g_depth_upload.sample_count * 2U);
    g_depth_upload.total_frames = (uint16_t)((g_depth_upload.total_bytes + WAVE_CHUNK_DATA_SIZE - 1U) / WAVE_CHUNK_DATA_SIZE);
    if (preserve_debug_status == 0U)
    {
      g_sonar_app.status = SONAR_STATUS_UPLOADING;
    }
    g_debug_upload_stage = 5U;
    App_SendStatusEvent(EVENT_UPLOAD_START, g_depth_upload.angle);
    if (g_depth_upload.total_frames == 0U)
    {
      g_depth_upload.total_frames = 1U;
    }
  }

  if (g_depth_upload.frame_id < g_depth_upload.total_frames)
  {
    uint16_t offset = (uint16_t)(g_depth_upload.frame_id * WAVE_CHUNK_DATA_SIZE);
    uint16_t chunk_size = (uint16_t)(g_depth_upload.total_bytes - offset);
    uint8_t payload[11U + WAVE_CHUNK_DATA_SIZE];

    if (chunk_size > WAVE_CHUNK_DATA_SIZE)
    {
      chunk_size = WAVE_CHUNK_DATA_SIZE;
    }

    App_WriteU16(&payload[0], g_depth_upload.frame_id);
    App_WriteU16(&payload[2], g_depth_upload.total_frames);
    App_WriteU16(&payload[4], offset);
    App_WriteU16(&payload[6], chunk_size);
    App_WriteU16(&payload[8], g_depth_upload.angle);
    payload[10] = WAVE_DATA_TYPE_U16_RAW;
    if (g_depth_upload.frame_id == 0U)
    {
      g_debug_upload_stage = 6U;
    }
    memcpy(&payload[11], ((const uint8_t *)g_depth_samples[g_depth_upload.buffer_index]) + offset, chunk_size);
    App_SendFrame(SONAR_CMD_WAVE_CHUNK, g_depth_upload.sequence, payload, (uint16_t)(11U + chunk_size));
    g_debug_upload_chunks++;
    if (g_depth_upload.frame_id == 0U)
    {
      g_debug_upload_stage = 7U;
    }
    g_depth_upload.frame_id++;
    return;
  }

  {
    uint8_t finish_payload[9];
    App_WriteU16(&finish_payload[0], g_depth_upload.angle);
    App_WriteU16(&finish_payload[2], g_depth_upload.sample_count);
    finish_payload[4] = 0U;
    App_WriteU32(&finish_payload[5], App_GetValidatedCaptureSampleRateHz());
    App_SendFrame(SONAR_CMD_WAVE_FINISH, g_depth_upload.sequence, finish_payload, sizeof(finish_payload));
  }

  {
    uint16_t uploaded_angle = g_depth_upload.angle;

  App_ReleaseDepthBuffer(g_depth_upload.buffer_index);
  memset(&g_depth_upload, 0, sizeof(g_depth_upload));
  g_debug_upload_stage = 8U;
  if (preserve_debug_status == 0U)
  {
    SonarApp_OnMeasureComplete(&g_sonar_app);
    App_SendStatusEvent(EVENT_MEASURE_DONE, 0U);
  }
  else
  {
    g_sonar_app.status = previous_status;
  }
  App_SendStatusEvent(EVENT_UPLOAD_DONE, uploaded_angle);
  }
  g_debug_upload_stage = 9U;
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

static uint32_t App_CalculateDistanceMmFromSamples(const uint16_t *samples, uint16_t sample_count, uint16_t sound_speed_m_s)
{
  uint32_t tail_start = 0U;
  uint32_t baseline_sum = 0U;
  uint32_t threshold = 0U;
  uint16_t baseline = 0U;
  uint16_t index = 0U;
  const uint16_t baseline_window = 128U;
  const uint16_t blind_zone_samples = 550U;
  const uint16_t threshold_counts = 500U;
  const float sample_rate_hz = (float)App_GetValidatedCaptureSampleRateHz();

  if ((samples == NULL) || (sample_count < (blind_zone_samples + 2U)) || (sample_rate_hz <= 0.0f))
  {
    return 0U;
  }

  if (sound_speed_m_s == 0U)
  {
    sound_speed_m_s = (g_sonar_app.scan_config.sound_speed != 0U) ? g_sonar_app.scan_config.sound_speed : 1500U;
  }

  tail_start = (sample_count > baseline_window) ? (uint32_t)(sample_count - baseline_window) : 0U;
  for (index = (uint16_t)tail_start; index < sample_count; ++index)
  {
    baseline_sum += samples[index];
  }

  baseline = (uint16_t)(baseline_sum / (uint32_t)(sample_count - tail_start));
  threshold = (uint32_t)baseline + threshold_counts;

  for (index = blind_zone_samples; index < sample_count; ++index)
  {
    if ((uint32_t)samples[index] >= threshold)
    {
      float prev_value = (float)samples[index - 1U];
      float curr_value = (float)samples[index];
      float crossing = (float)index;

      if (curr_value > prev_value)
      {
        crossing = ((float)(index - 1U)) + (((float)threshold - prev_value) / (curr_value - prev_value));
      }

      return (uint32_t)((crossing * (float)sound_speed_m_s * 1000.0f) / (sample_rate_hz * 2.0f));
    }
  }

  return 0U;
}

static void App_SendWaveFromBuffer(uint16_t sequence, uint16_t angle, uint8_t buffer_index, uint16_t sample_count)
{
  uint16_t total_bytes = 0U;
  uint16_t total_frames = 0U;
  uint16_t frame_id = 0U;

  if ((buffer_index >= DEPTH_BUFFER_COUNT) || (sample_count == 0U))
  {
    return;
  }

  total_bytes = (uint16_t)(sample_count * 2U);
  total_frames = (uint16_t)((total_bytes + WAVE_CHUNK_DATA_SIZE - 1U) / WAVE_CHUNK_DATA_SIZE);
  if (total_frames == 0U)
  {
    total_frames = 1U;
  }

  for (frame_id = 0U; frame_id < total_frames; ++frame_id)
  {
    uint16_t offset = (uint16_t)(frame_id * WAVE_CHUNK_DATA_SIZE);
    uint16_t chunk_size = (uint16_t)(total_bytes - offset);
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
    payload[10] = WAVE_DATA_TYPE_U16_RAW;
    memcpy(&payload[11], ((const uint8_t *)g_depth_samples[buffer_index]) + offset, chunk_size);
    App_SendFrame(SONAR_CMD_WAVE_CHUNK, sequence, payload, (uint16_t)(11U + chunk_size));
  }

  {
    uint8_t finish_payload[9];
    App_WriteU16(&finish_payload[0], angle);
    App_WriteU16(&finish_payload[2], sample_count);
    finish_payload[4] = 0U;
    App_WriteU32(&finish_payload[5], App_GetValidatedCaptureSampleRateHz());
    App_SendFrame(SONAR_CMD_WAVE_FINISH, sequence, finish_payload, sizeof(finish_payload));
  }
}

static SonarResultCode App_RunBlockingMeasurement(uint16_t sequence, uint16_t angle, uint8_t upload_wave, uint32_t *distance_mm, uint16_t *quality)
{
  uint32_t started_ms = HAL_GetTick();
  uint8_t buffer_index = DEPTH_BUFFER_INVALID;

  if (App_WaitForFreeDepthBuffer(5000U) == 0U)
  {
    return SONAR_RESULT_BUSY;
  }

  g_active_measure_sequence = sequence;
  App_StartDepthCapture(sequence);

  while ((HAL_GetTick() - started_ms) < 500U)
  {
    if ((g_depth_capture_complete != 0U) && (g_depth_capture_active == 0U))
    {
      g_depth_capture_complete = 0U;
      buffer_index = g_depth_completed_buffer_index;
      g_depth_completed_buffer_index = DEPTH_BUFFER_INVALID;

      if (buffer_index >= DEPTH_BUFFER_COUNT)
      {
        return SONAR_RESULT_HARDWARE_ERROR;
      }

      App_InvalidateDepthBufferCache(buffer_index);

      if (distance_mm != NULL)
      {
        *distance_mm = App_CalculateDistanceMmFromSamples(
            g_depth_samples[buffer_index],
            g_depth_buffer_slots[buffer_index].sample_count,
            (g_sonar_app.scan_config.sound_speed != 0U) ? g_sonar_app.scan_config.sound_speed : 1500U);
      }

      if (quality != NULL)
      {
        uint16_t sample_index = 0U;
        uint16_t baseline = 0U;
        uint16_t peak = 0U;
        uint32_t tail_sum = 0U;
        uint16_t tail_start = (g_depth_buffer_slots[buffer_index].sample_count > 128U) ? (uint16_t)(g_depth_buffer_slots[buffer_index].sample_count - 128U) : 0U;

        for (sample_index = tail_start; sample_index < g_depth_buffer_slots[buffer_index].sample_count; ++sample_index)
        {
          tail_sum += g_depth_samples[buffer_index][sample_index];
        }
        baseline = (uint16_t)(tail_sum / (uint32_t)(g_depth_buffer_slots[buffer_index].sample_count - tail_start));

        for (sample_index = 0U; sample_index < g_depth_buffer_slots[buffer_index].sample_count; ++sample_index)
        {
          if (g_depth_samples[buffer_index][sample_index] > peak)
          {
            peak = g_depth_samples[buffer_index][sample_index];
          }
        }

        *quality = (uint16_t)((peak > baseline) ? ((peak - baseline) > 1000U ? 1000U : (peak - baseline)) : 0U);
      }

      if (upload_wave != 0U)
      {
        if (g_sonar_app.mode == SONAR_APP_MODE_DEBUG_SWEEP)
        {
          g_depth_buffer_slots[buffer_index].state = DEPTH_BUFFER_STATE_READY;
          return SONAR_RESULT_OK;
        }

        App_SendStatusEvent(EVENT_UPLOAD_START, angle);
        App_SendWaveFromBuffer(sequence, angle, buffer_index, g_depth_buffer_slots[buffer_index].sample_count);
        App_SendStatusEvent(EVENT_UPLOAD_DONE, angle);
      }

      App_ReleaseDepthBuffer(buffer_index);
      return SONAR_RESULT_OK;
    }

    if ((g_depth_capture_timeout != 0U) && (g_depth_capture_active == 0U))
    {
      buffer_index = g_depth_timeout_buffer_index;
      if (buffer_index < DEPTH_BUFFER_COUNT)
      {
        App_ReleaseDepthBuffer(buffer_index);
      }
      g_depth_timeout_buffer_index = DEPTH_BUFFER_INVALID;
      g_depth_capture_timeout = 0U;
      return SONAR_RESULT_TIMEOUT;
    }
  }

  App_StopDepthCapture();
  if (g_depth_capture_buffer_index < DEPTH_BUFFER_COUNT)
  {
    App_ReleaseDepthBuffer(g_depth_capture_buffer_index);
    g_depth_capture_buffer_index = DEPTH_BUFFER_INVALID;
  }
  return SONAR_RESULT_TIMEOUT;
}

static SonarResultCode App_CaptureSweepMeasurement(uint16_t sequence, uint32_t *distance_mm, uint16_t *quality, uint8_t upload_wave, uint8_t *buffer_index)
{
  uint32_t started_ms = HAL_GetTick();
  uint8_t local_buffer_index = DEPTH_BUFFER_INVALID;

  if (buffer_index != NULL)
  {
    *buffer_index = DEPTH_BUFFER_INVALID;
  }

  if (App_WaitForFreeDepthBuffer(8000U) == 0U)
  {
    return SONAR_RESULT_BUSY;
  }

  g_active_measure_sequence = sequence;
  App_StartDepthCapture(sequence);

  while ((HAL_GetTick() - started_ms) < 1000U)
  {
    if ((g_depth_capture_complete != 0U) && (g_depth_capture_active == 0U))
    {
      uint16_t sample_index = 0U;
      uint16_t baseline = 0U;
      uint16_t peak = 0U;
      uint32_t tail_sum = 0U;
      uint16_t tail_start = 0U;

      g_depth_capture_complete = 0U;
      local_buffer_index = g_depth_completed_buffer_index;
      g_depth_completed_buffer_index = DEPTH_BUFFER_INVALID;

      if (local_buffer_index >= DEPTH_BUFFER_COUNT)
      {
        return SONAR_RESULT_HARDWARE_ERROR;
      }

      App_InvalidateDepthBufferCache(local_buffer_index);

      if (distance_mm != NULL)
      {
        *distance_mm = App_CalculateDistanceMmFromSamples(
            g_depth_samples[local_buffer_index],
            g_depth_buffer_slots[local_buffer_index].sample_count,
            (g_sonar_app.scan_config.sound_speed != 0U) ? g_sonar_app.scan_config.sound_speed : 1500U);
      }

      if (quality != NULL)
      {
        tail_start = (g_depth_buffer_slots[local_buffer_index].sample_count > 128U) ? (uint16_t)(g_depth_buffer_slots[local_buffer_index].sample_count - 128U) : 0U;
        for (sample_index = tail_start; sample_index < g_depth_buffer_slots[local_buffer_index].sample_count; ++sample_index)
        {
          tail_sum += g_depth_samples[local_buffer_index][sample_index];
        }
        baseline = (uint16_t)(tail_sum / (uint32_t)(g_depth_buffer_slots[local_buffer_index].sample_count - tail_start));
        for (sample_index = 0U; sample_index < g_depth_buffer_slots[local_buffer_index].sample_count; ++sample_index)
        {
          if (g_depth_samples[local_buffer_index][sample_index] > peak)
          {
            peak = g_depth_samples[local_buffer_index][sample_index];
          }
        }
        *quality = (uint16_t)((peak > baseline) ? ((peak - baseline) > 1000U ? 1000U : (peak - baseline)) : 0U);
      }

      if (upload_wave != 0U)
      {
        g_depth_buffer_slots[local_buffer_index].state = DEPTH_BUFFER_STATE_READY;
        if (buffer_index != NULL)
        {
          *buffer_index = local_buffer_index;
        }
      }
      else
      {
        App_ReleaseDepthBuffer(local_buffer_index);
      }

      return SONAR_RESULT_OK;
    }

    if ((g_depth_capture_timeout != 0U) && (g_depth_capture_active == 0U))
    {
      local_buffer_index = g_depth_timeout_buffer_index;
      if (local_buffer_index < DEPTH_BUFFER_COUNT)
      {
        App_ReleaseDepthBuffer(local_buffer_index);
      }
      g_depth_timeout_buffer_index = DEPTH_BUFFER_INVALID;
      g_depth_capture_timeout = 0U;
      return SONAR_RESULT_TIMEOUT;
    }

    HAL_Delay(1U);
  }

  App_StopDepthCapture();
  if (g_depth_capture_buffer_index < DEPTH_BUFFER_COUNT)
  {
    App_ReleaseDepthBuffer(g_depth_capture_buffer_index);
    g_depth_capture_buffer_index = DEPTH_BUFFER_INVALID;
  }
  return SONAR_RESULT_TIMEOUT;
}

static void App_ServiceBackgroundTasks(uint32_t duration_ms)
{
  uint32_t started_ms = HAL_GetTick();

  do
  {
    App_ProcessDepthUpload();
    HAL_Delay(2U);
  } while ((HAL_GetTick() - started_ms) < duration_ms);
}

static void App_FlushDepthUploads(uint32_t timeout_ms)
{
  uint32_t started_ms = HAL_GetTick();

  while ((HAL_GetTick() - started_ms) < timeout_ms)
  {
    if ((g_depth_upload.active == 0U) && (App_FindReadyDepthBuffer() >= DEPTH_BUFFER_COUNT))
    {
      return;
    }

    App_ProcessDepthUpload();
    HAL_Delay(2U);
  }
}

static uint8_t App_HasFreeDepthBuffer(void)
{
  uint8_t index = 0U;

  for (index = 0U; index < DEPTH_BUFFER_COUNT; ++index)
  {
    if (g_depth_buffer_slots[index].state == DEPTH_BUFFER_STATE_FREE)
    {
      return 1U;
    }
  }

  return 0U;
}

static uint8_t App_WaitForFreeDepthBuffer(uint32_t timeout_ms)
{
  uint32_t started_ms = HAL_GetTick();

  while ((HAL_GetTick() - started_ms) < timeout_ms)
  {
    if (App_HasFreeDepthBuffer() != 0U)
    {
      return 1U;
    }

    App_ProcessDepthUpload();
    HAL_Delay(2U);
  }

  return App_HasFreeDepthBuffer();
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

static void App_StartDepthCapture(uint16_t sequence)
{
  uint8_t buffer_index = App_AllocateDepthBuffer();
  uint32_t address = 0U;
  uint32_t length = 0U;

  (void)sequence;
  if (buffer_index >= DEPTH_BUFFER_COUNT)
  {
    g_debug_capture_done = 1U;
    g_debug_capture_result = 4U;
    App_SendNack(g_active_measure_sequence, SONAR_CMD_START_SINGLE_MEASURE, SONAR_RESULT_BUSY);
    g_sonar_app.status = SONAR_STATUS_READY;
    return;
  }

  address = App_DCacheAlignDown((uint32_t)g_depth_samples[buffer_index]);
  length = App_DCacheAlignUp(sizeof(g_depth_samples[buffer_index]) + ((uint32_t)g_depth_samples[buffer_index] - address));
  if (App_IsDepthProbeMode() != 0U)
  {
    g_active_capture_adc = &hadc2;
    g_active_capture_pwm_channel = TIM_CHANNEL_3;
  }
  else if (App_IsSideScanProbeMode() != 0U)
  {
    g_active_capture_adc = &hadc1;
    g_active_capture_pwm_channel = TIM_CHANNEL_4;
  }
  else
  {
    App_ReleaseDepthBuffer(buffer_index);
    App_SendNack(g_active_measure_sequence, SONAR_CMD_START_SINGLE_MEASURE, SONAR_RESULT_INVALID_PARAM);
    g_sonar_app.status = SONAR_STATUS_READY;
    return;
  }
  g_depth_capture_complete = 0U;
  g_depth_capture_timeout = 0U;
  g_depth_capture_active = 1U;
  g_depth_pulse_count = 0U;
  g_capture_started_cycles = 0U;
  g_capture_finished_cycles = 0U;
  g_last_capture_sample_rate_hz = 0U;
  g_depth_capture_buffer_index = buffer_index;
  g_depth_completed_buffer_index = DEPTH_BUFFER_INVALID;
  g_depth_timeout_buffer_index = DEPTH_BUFFER_INVALID;
  g_depth_buffer_slots[buffer_index].state = DEPTH_BUFFER_STATE_FILLING;
  g_depth_buffer_slots[buffer_index].sample_count = DEPTH_SAMPLE_COUNT;
  g_depth_buffer_slots[buffer_index].sequence = sequence;
  g_depth_buffer_slots[buffer_index].angle = g_sonar_app.current_angle;
  g_debug_upload_stage = 1U;

  SCB_CleanInvalidateDCache_by_Addr((uint32_t *)address, (int32_t)length);
  __HAL_TIM_SET_COMPARE(&htim3, g_active_capture_pwm_channel, DEPTH_PROBE_500KHZ_PULSE);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  __HAL_TIM_SET_COUNTER(&htim4, 0U);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
  __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);

  if (HAL_ADC_Start_DMA(g_active_capture_adc, (uint32_t *)g_depth_samples[buffer_index], DEPTH_SAMPLE_COUNT) != HAL_OK)
  {
    App_ReleaseDepthBuffer(buffer_index);
    g_depth_capture_buffer_index = DEPTH_BUFFER_INVALID;
    g_active_capture_adc = NULL;
    g_depth_capture_active = 0U;
    g_debug_capture_done = 1U;
    g_debug_capture_result = 3U;
    g_debug_upload_stage = 0xE1U;
    App_SendNack(g_active_measure_sequence, SONAR_CMD_START_SINGLE_MEASURE, SONAR_RESULT_HARDWARE_ERROR);
    g_sonar_app.status = SONAR_STATUS_READY;
    App_SendStatusEvent(0xE2U, 0U);
    return;
  }

  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    App_StopDepthCapture();
    g_debug_capture_done = 1U;
    g_debug_capture_result = 3U;
    g_debug_upload_stage = 0xE2U;
    App_SendNack(g_active_measure_sequence, SONAR_CMD_START_SINGLE_MEASURE, SONAR_RESULT_HARDWARE_ERROR);
    g_sonar_app.status = SONAR_STATUS_READY;
    App_SendStatusEvent(0xE2U, 0U);
    return;
  }

  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
  {
    App_StopDepthCapture();
    g_debug_capture_done = 1U;
    g_debug_capture_result = 3U;
    g_debug_upload_stage = 0xE3U;
    App_SendNack(g_active_measure_sequence, SONAR_CMD_START_SINGLE_MEASURE, SONAR_RESULT_HARDWARE_ERROR);
    g_sonar_app.status = SONAR_STATUS_READY;
    App_SendStatusEvent(0xE2U, 0U);
    return;
  }
  g_capture_started_cycles = DWT->CYCCNT;

  if (HAL_TIM_PWM_Start(&htim3, g_active_capture_pwm_channel) != HAL_OK)
  {
    App_StopDepthCapture();
    g_debug_capture_done = 1U;
    g_debug_capture_result = 3U;
    g_debug_upload_stage = 0xE4U;
    App_SendNack(g_active_measure_sequence, SONAR_CMD_START_SINGLE_MEASURE, SONAR_RESULT_HARDWARE_ERROR);
    g_sonar_app.status = SONAR_STATUS_READY;
    App_SendStatusEvent(0xE2U, 0U);
  }
}

static void App_StopDepthCapture(void)
{
  __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
  (void)HAL_TIM_PWM_Stop(&htim3, g_active_capture_pwm_channel);
  (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
  (void)HAL_TIM_Base_Stop_IT(&htim2);
  if (g_active_capture_adc != NULL)
  {
    (void)HAL_ADC_Stop_DMA(g_active_capture_adc);
    g_active_capture_adc = NULL;
  }
  g_depth_capture_active = 0U;
}

static void App_FinalizeDepthCapture(void)
{
  uint8_t buffer_index = g_depth_completed_buffer_index;

  if (buffer_index >= DEPTH_BUFFER_COUNT)
  {
    return;
  }

  App_InvalidateDepthBufferCache(buffer_index);
  g_depth_completed_buffer_index = DEPTH_BUFFER_INVALID;

  if ((g_sonar_app.debug_config.flags & 0x08U) != 0U)
  {
    g_depth_buffer_slots[buffer_index].state = DEPTH_BUFFER_STATE_READY;
    return;
  }

  App_ReleaseDepthBuffer(buffer_index);
  SonarApp_OnMeasureComplete(&g_sonar_app);
  App_SendStatusEvent(EVENT_MEASURE_DONE, 0U);
  g_sonar_app.status = SONAR_STATUS_READY;
}

static uint8_t App_IsDepthProbeMode(void)
{
  return (g_sonar_app.debug_config.probe_type == DEBUG_PROBE_TYPE_DEPTH) ? 1U : 0U;
}

static uint8_t App_IsSideScanProbeMode(void)
{
  return (g_sonar_app.debug_config.probe_type == DEBUG_PROBE_TYPE_SIDE_SCAN) ? 1U : 0U;
}

static uint32_t App_DCacheAlignDown(uint32_t address)
{
  return address & ~((uint32_t)31U);
}

static uint32_t App_DCacheAlignUp(uint32_t value)
{
  return (value + 31U) & ~((uint32_t)31U);
}

static uint8_t App_AllocateDepthBuffer(void)
{
  uint8_t attempt = 0U;

  for (attempt = 0U; attempt < DEPTH_BUFFER_COUNT; ++attempt)
  {
    uint8_t index = (uint8_t)((g_depth_buffer_round_robin + attempt) % DEPTH_BUFFER_COUNT);
    if (g_depth_buffer_slots[index].state == DEPTH_BUFFER_STATE_FREE)
    {
      g_depth_buffer_round_robin = (uint8_t)((index + 1U) % DEPTH_BUFFER_COUNT);
      return index;
    }
  }

  return DEPTH_BUFFER_INVALID;
}

static uint8_t App_FindReadyDepthBuffer(void)
{
  uint8_t index = 0U;

  for (index = 0U; index < DEPTH_BUFFER_COUNT; ++index)
  {
    if (g_depth_buffer_slots[index].state == DEPTH_BUFFER_STATE_READY)
    {
      return index;
    }
  }

  return DEPTH_BUFFER_INVALID;
}

static void App_ReleaseDepthBuffer(uint8_t buffer_index)
{
  if (buffer_index >= DEPTH_BUFFER_COUNT)
  {
    return;
  }

  g_depth_buffer_slots[buffer_index].state = DEPTH_BUFFER_STATE_FREE;
  g_depth_buffer_slots[buffer_index].sample_count = 0U;
  g_depth_buffer_slots[buffer_index].sequence = 0U;
  g_depth_buffer_slots[buffer_index].angle = 0U;
}

static void App_InvalidateDepthBufferCache(uint8_t buffer_index)
{
  uint32_t address = 0U;
  uint32_t length = 0U;

  if (buffer_index >= DEPTH_BUFFER_COUNT)
  {
    return;
  }

  address = App_DCacheAlignDown((uint32_t)g_depth_samples[buffer_index]);
  length = App_DCacheAlignUp(sizeof(g_depth_samples[buffer_index]) + ((uint32_t)g_depth_samples[buffer_index] - address));
  SCB_InvalidateDCache_by_Addr((uint32_t *)address, (int32_t)length);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
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
  if (huart->Instance == USART2)
  {
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    App_StartUartReception();
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if ((g_depth_capture_active != 0U) && (g_active_capture_adc != NULL) && (hadc->Instance == g_active_capture_adc->Instance))
  {
    uint8_t buffer_index = g_depth_capture_buffer_index;
    uint16_t sample_count = DEPTH_SAMPLE_COUNT;

    g_capture_finished_cycles = DWT->CYCCNT;
    if (buffer_index < DEPTH_BUFFER_COUNT)
    {
      sample_count = g_depth_buffer_slots[buffer_index].sample_count;
    }
    if ((g_capture_started_cycles != 0U) && (g_capture_finished_cycles != g_capture_started_cycles) && (sample_count != 0U))
    {
      uint32_t core_clock_hz = HAL_RCC_GetSysClockFreq();
      uint32_t elapsed_cycles = g_capture_finished_cycles - g_capture_started_cycles;

      if ((core_clock_hz != 0U) && (elapsed_cycles != 0U))
      {
        g_last_capture_sample_rate_hz = (uint32_t)((((uint64_t)sample_count * (uint64_t)core_clock_hz) + ((uint64_t)elapsed_cycles / 2ULL)) / (uint64_t)elapsed_cycles);
      }
    }

    g_depth_completed_buffer_index = g_depth_capture_buffer_index;
    g_depth_capture_buffer_index = DEPTH_BUFFER_INVALID;
    App_StopDepthCapture();
    g_debug_upload_stage = 4U;
    g_depth_capture_complete = 1U;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if ((htim->Instance == TIM3) && (g_depth_capture_active != 0U))
  {
    g_depth_pulse_count++;
    if (g_depth_pulse_count >= g_sonar_app.debug_config.pulse_count)
    {
      __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
      (void)HAL_TIM_PWM_Stop(&htim3, g_active_capture_pwm_channel);
    }
  }
  else if ((htim->Instance == TIM2) && (g_depth_capture_active != 0U))
  {
    g_depth_timeout_buffer_index = g_depth_capture_buffer_index;
    g_depth_capture_buffer_index = DEPTH_BUFFER_INVALID;
    App_StopDepthCapture();
    g_debug_upload_stage = 0xE5U;
    g_depth_capture_timeout = 1U;
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  SonarApp_Init(&g_sonar_app);
  App_InitMotorController();
  g_last_device_info_broadcast_ms = HAL_GetTick();
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  App_EnableCycleCounter();
  App_StartUartReception();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_PollUartReception();
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 25;
  PeriphClkInitStruct.PLL2.PLL2N = 288;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x024000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512MB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
