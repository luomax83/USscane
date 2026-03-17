#include "motor_driver.h"

#include <string.h>

#define MOTOR_REG_STATUS 0x0000U
#define MOTOR_REG_ACTUAL_STEPS_HI 0x0001U
#define MOTOR_REG_STOP 0x0004U
#define MOTOR_REG_HOME 0x001FU
#define MOTOR_REG_SINGLE_TURN_TARGET 0x0020U
#define MOTOR_REG_SINGLE_TURN_SPEED 0x0021U
#define MOTOR_REG_SINGLE_TURN_MODE 0x0022U
#define MOTOR_REG_FORWARD_TARGET_HI 0x0040U
#define MOTOR_REG_SET_ZERO 0x00D5U

#define MOTOR_MODBUS_FUNC_READ_HOLDING 0x03U
#define MOTOR_MODBUS_FUNC_WRITE_SINGLE 0x06U
#define MOTOR_MODBUS_FUNC_WRITE_MULTI 0x10U

#define MOTOR_STATUS_IDLE 0x0000U
#define MOTOR_STATUS_RUNNING 0x0001U
#define MOTOR_STATUS_COLLISION_STOP 0x0002U
#define MOTOR_STATUS_POS_LIMIT_STOP 0x0003U
#define MOTOR_STATUS_NEG_LIMIT_STOP 0x0004U
#define MOTOR_RS485_TX_PORT GPIOB
#define MOTOR_RS485_TX_PIN GPIO_PIN_7

static uint16_t MotorDriver_ModbusCrc16(const uint8_t *data, uint16_t length);
static void MotorDriver_FlushRx(MotorDriverContext *context);
static void MotorDriver_SetTransmitMode(void);
static void MotorDriver_SetReceiveMode(void);
static MotorDriverResult MotorDriver_Transfer(MotorDriverContext *context, const uint8_t *request, uint16_t request_length, uint8_t *response, uint16_t response_length);
static MotorDriverResult MotorDriver_ReadRegisters(MotorDriverContext *context, uint16_t register_address, uint16_t register_count, uint16_t *registers);
static MotorDriverResult MotorDriver_WriteSingleRegister(MotorDriverContext *context, uint16_t register_address, uint16_t value);
static MotorDriverResult MotorDriver_WriteMultipleRegisters(MotorDriverContext *context, uint16_t register_address, uint16_t register_count, const uint16_t *values);
static uint16_t MotorDriver_ReadU16(const uint8_t *data);
static void MotorDriver_WriteU16(uint8_t *data, uint16_t value);
static MotorDriverResult MotorDriver_ReceiveBytes(MotorDriverContext *context, uint8_t *response, uint16_t response_length, uint16_t *actual_length);

volatile MotorDriverTrace g_motor_driver_trace;

void MotorDriver_Init(MotorDriverContext *context, UART_HandleTypeDef *uart, uint8_t slave_address, uint16_t single_turn_steps)
{
  memset(context, 0, sizeof(*context));
  context->uart = uart;
  context->slave_address = slave_address;
  context->single_turn_steps = single_turn_steps;
  context->response_timeout_ms = 100U;
}

MotorDriverResult MotorDriver_ReadStatus(MotorDriverContext *context, uint16_t *status)
{
  uint16_t value = 0U;
  MotorDriverResult result = MotorDriver_ReadRegisters(context, MOTOR_REG_STATUS, 1U, &value);

  if ((result == MOTOR_DRIVER_OK) && (status != NULL))
  {
    *status = value;
  }

  return result;
}

MotorDriverResult MotorDriver_ReadActualSteps(MotorDriverContext *context, int32_t *steps)
{
  uint16_t registers[2];
  MotorDriverResult result = MotorDriver_ReadRegisters(context, MOTOR_REG_ACTUAL_STEPS_HI, 2U, registers);

  if ((result == MOTOR_DRIVER_OK) && (steps != NULL))
  {
    *steps = (int32_t)(((uint32_t)registers[0] << 16U) | registers[1]);
  }

  return result;
}

MotorDriverResult MotorDriver_MoveSingleTurn(MotorDriverContext *context, uint16_t target_steps, uint16_t speed_rpm, uint16_t move_mode)
{
  uint16_t registers[3];

  if ((context == NULL) || (context->uart == NULL) || (context->single_turn_steps == 0U) || (target_steps >= context->single_turn_steps))
  {
    return MOTOR_DRIVER_INVALID_PARAM;
  }

  registers[0] = target_steps;
  registers[1] = speed_rpm;
  registers[2] = move_mode;
  return MotorDriver_WriteMultipleRegisters(context, MOTOR_REG_SINGLE_TURN_TARGET, 3U, registers);
}

MotorDriverResult MotorDriver_MoveForwardSteps(MotorDriverContext *context, int32_t step_delta, uint16_t start_speed_rpm_x10, uint16_t max_speed_rpm, uint16_t accel_time_ms, uint16_t tolerance_steps)
{
  uint16_t registers[6];
  uint32_t steps = (uint32_t)step_delta;

  if ((context == NULL) || (context->uart == NULL))
  {
    return MOTOR_DRIVER_INVALID_PARAM;
  }

  registers[0] = (uint16_t)((steps >> 16U) & 0xFFFFU);
  registers[1] = (uint16_t)(steps & 0xFFFFU);
  registers[2] = start_speed_rpm_x10;
  registers[3] = max_speed_rpm;
  registers[4] = accel_time_ms;
  registers[5] = tolerance_steps;
  return MotorDriver_WriteMultipleRegisters(context, MOTOR_REG_FORWARD_TARGET_HI, 6U, registers);
}

MotorDriverResult MotorDriver_Home(MotorDriverContext *context, uint16_t speed_rpm)
{
  if ((context == NULL) || (context->uart == NULL))
  {
    return MOTOR_DRIVER_INVALID_PARAM;
  }

  return MotorDriver_WriteSingleRegister(context, MOTOR_REG_HOME, speed_rpm);
}

MotorDriverResult MotorDriver_SetZero(MotorDriverContext *context)
{
  if ((context == NULL) || (context->uart == NULL))
  {
    return MOTOR_DRIVER_INVALID_PARAM;
  }

  return MotorDriver_WriteSingleRegister(context, MOTOR_REG_SET_ZERO, 0U);
}

MotorDriverResult MotorDriver_Stop(MotorDriverContext *context)
{
  if ((context == NULL) || (context->uart == NULL))
  {
    return MOTOR_DRIVER_INVALID_PARAM;
  }

  return MotorDriver_WriteSingleRegister(context, MOTOR_REG_STOP, 0U);
}

MotorDriverResult MotorDriver_WaitForIdle(MotorDriverContext *context, uint32_t timeout_ms, uint16_t *final_status)
{
  uint32_t started_ms = HAL_GetTick();
  uint16_t status = MOTOR_STATUS_IDLE;
  MotorDriverResult result = MOTOR_DRIVER_OK;

  do
  {
    result = MotorDriver_ReadStatus(context, &status);
    if (result != MOTOR_DRIVER_OK)
    {
      return result;
    }

    if (status == MOTOR_STATUS_IDLE)
    {
      if (final_status != NULL)
      {
        *final_status = status;
      }
      return MOTOR_DRIVER_OK;
    }

    if ((status == MOTOR_STATUS_COLLISION_STOP) || (status == MOTOR_STATUS_POS_LIMIT_STOP) || (status == MOTOR_STATUS_NEG_LIMIT_STOP))
    {
      if (final_status != NULL)
      {
        *final_status = status;
      }
      return MOTOR_DRIVER_MOTION_ERROR;
    }

    HAL_Delay(20U);
  } while ((HAL_GetTick() - started_ms) < timeout_ms);

  if (final_status != NULL)
  {
    *final_status = status;
  }
  return MOTOR_DRIVER_TIMEOUT;
}

uint16_t MotorDriver_AngleCdegToSingleTurnSteps(const MotorDriverContext *context, uint16_t angle_cdeg)
{
  uint32_t normalized_angle = angle_cdeg % 36000U;

  if ((context == NULL) || (context->single_turn_steps == 0U))
  {
    return 0U;
  }

  return (uint16_t)(((normalized_angle * context->single_turn_steps) + 18000U) / 36000U) % context->single_turn_steps;
}

uint16_t MotorDriver_SingleTurnStepsToAngleCdeg(const MotorDriverContext *context, uint16_t single_turn_steps)
{
  if ((context == NULL) || (context->single_turn_steps == 0U))
  {
    return 0U;
  }

  return (uint16_t)((((uint32_t)single_turn_steps % context->single_turn_steps) * 36000U + (context->single_turn_steps / 2U)) / context->single_turn_steps);
}

uint16_t MotorDriver_NormalizeAngleCdeg(const MotorDriverContext *context, uint16_t angle_cdeg)
{
  return MotorDriver_SingleTurnStepsToAngleCdeg(context, MotorDriver_AngleCdegToSingleTurnSteps(context, angle_cdeg));
}

uint16_t MotorDriver_NormalizeStepCdeg(const MotorDriverContext *context, uint16_t step_cdeg)
{
  uint16_t steps = MotorDriver_AngleCdegToSingleTurnSteps(context, step_cdeg);

  if ((context == NULL) || (context->single_turn_steps == 0U))
  {
    return step_cdeg;
  }

  if (steps == 0U)
  {
    steps = 1U;
  }

  return MotorDriver_SingleTurnStepsToAngleCdeg(context, steps);
}

uint16_t MotorDriver_ActualStepsToSingleTurnSteps(const MotorDriverContext *context, int32_t actual_steps)
{
  int32_t wrapped_steps = 0;

  if ((context == NULL) || (context->single_turn_steps == 0U))
  {
    return 0U;
  }

  wrapped_steps = actual_steps % (int32_t)context->single_turn_steps;
  if (wrapped_steps < 0)
  {
    wrapped_steps += context->single_turn_steps;
  }

  return (uint16_t)wrapped_steps;
}

static uint16_t MotorDriver_ModbusCrc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  uint16_t index = 0U;

  for (index = 0U; index < length; ++index)
  {
    uint8_t bit = 0U;
    crc ^= data[index];
    for (bit = 0U; bit < 8U; ++bit)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc >>= 1U;
        crc ^= 0xA001U;
      }
      else
      {
        crc >>= 1U;
      }
    }
  }

  return crc;
}

static void MotorDriver_FlushRx(MotorDriverContext *context)
{
  __HAL_UART_CLEAR_PEFLAG(context->uart);
  __HAL_UART_CLEAR_FEFLAG(context->uart);
  __HAL_UART_CLEAR_NEFLAG(context->uart);
  __HAL_UART_CLEAR_OREFLAG(context->uart);
  SET_BIT(context->uart->Instance->RQR, USART_RQR_RXFRQ);
}

static void MotorDriver_SetTransmitMode(void)
{
  HAL_GPIO_WritePin(MOTOR_RS485_TX_PORT, MOTOR_RS485_TX_PIN, GPIO_PIN_SET);
  HAL_Delay(10U);
}

static void MotorDriver_SetReceiveMode(void)
{
  HAL_GPIO_WritePin(MOTOR_RS485_TX_PORT, MOTOR_RS485_TX_PIN, GPIO_PIN_RESET);
  HAL_Delay(1U);
}

static MotorDriverResult MotorDriver_Transfer(MotorDriverContext *context, const uint8_t *request, uint16_t request_length, uint8_t *response, uint16_t response_length)
{
  uint16_t actual_length = 0U;
  uint16_t trace_request_length = request_length;

  if ((context == NULL) || (context->uart == NULL) || (request == NULL) || (request_length == 0U))
  {
    return MOTOR_DRIVER_INVALID_PARAM;
  }

  g_motor_driver_trace.sequence++;
  g_motor_driver_trace.result = 0xFFFFFFFFUL;
  g_motor_driver_trace.request_length = request_length;
  g_motor_driver_trace.response_length = 0U;
  memset((void *)g_motor_driver_trace.request, 0, sizeof(g_motor_driver_trace.request));
  memset((void *)g_motor_driver_trace.response, 0, sizeof(g_motor_driver_trace.response));
  if (trace_request_length > sizeof(g_motor_driver_trace.request))
  {
    trace_request_length = (uint16_t)sizeof(g_motor_driver_trace.request);
  }
  memcpy((void *)g_motor_driver_trace.request, request, trace_request_length);

  MotorDriver_FlushRx(context);
  MotorDriver_SetTransmitMode();

  if (HAL_UART_Transmit(context->uart, (uint8_t *)request, request_length, context->response_timeout_ms) != HAL_OK)
  {
    MotorDriver_SetReceiveMode();
    g_motor_driver_trace.result = MOTOR_DRIVER_UART_ERROR;
    return MOTOR_DRIVER_UART_ERROR;
  }

  while (__HAL_UART_GET_FLAG(context->uart, UART_FLAG_TC) == RESET)
  {
  }
  HAL_Delay(5U);
  MotorDriver_SetReceiveMode();

  if ((response != NULL) && (response_length > 0U))
  {
    if (MotorDriver_ReceiveBytes(context, response, response_length, &actual_length) != MOTOR_DRIVER_OK)
    {
      g_motor_driver_trace.result = MOTOR_DRIVER_TIMEOUT;
      return MOTOR_DRIVER_TIMEOUT;
    }
    g_motor_driver_trace.response_length = actual_length;
    if (actual_length > sizeof(g_motor_driver_trace.response))
    {
      actual_length = (uint16_t)sizeof(g_motor_driver_trace.response);
    }
    memcpy((void *)g_motor_driver_trace.response, response, actual_length);
  }

  g_motor_driver_trace.result = MOTOR_DRIVER_OK;
  return MOTOR_DRIVER_OK;
}

static MotorDriverResult MotorDriver_ReadRegisters(MotorDriverContext *context, uint16_t register_address, uint16_t register_count, uint16_t *registers)
{
  uint8_t request[8];
  uint8_t response[64];
  uint16_t crc = 0U;
  uint16_t expected_length = 0U;
  uint16_t expected_crc = 0U;
  uint16_t actual_crc = 0U;
  uint16_t index = 0U;
  MotorDriverResult result = MOTOR_DRIVER_OK;

  if ((register_count == 0U) || (register_count > 28U) || (registers == NULL))
  {
    return MOTOR_DRIVER_INVALID_PARAM;
  }

  request[0] = context->slave_address;
  request[1] = MOTOR_MODBUS_FUNC_READ_HOLDING;
  MotorDriver_WriteU16(&request[2], register_address);
  MotorDriver_WriteU16(&request[4], register_count);
  crc = MotorDriver_ModbusCrc16(request, 6U);
  request[6] = (uint8_t)(crc & 0xFFU);
  request[7] = (uint8_t)((crc >> 8U) & 0xFFU);

  expected_length = (uint16_t)(5U + register_count * 2U);
  result = MotorDriver_Transfer(context, request, sizeof(request), response, expected_length);
  if (result != MOTOR_DRIVER_OK)
  {
    return result;
  }

  if ((response[0] != context->slave_address) || (response[1] == (MOTOR_MODBUS_FUNC_READ_HOLDING | 0x80U)))
  {
    return MOTOR_DRIVER_REMOTE_ERROR;
  }

  if ((response[1] != MOTOR_MODBUS_FUNC_READ_HOLDING) || (response[2] != (uint8_t)(register_count * 2U)))
  {
    return MOTOR_DRIVER_PROTOCOL_ERROR;
  }

  expected_crc = (uint16_t)(((uint16_t)response[expected_length - 1U] << 8U) | response[expected_length - 2U]);
  actual_crc = MotorDriver_ModbusCrc16(response, (uint16_t)(expected_length - 2U));
  if (expected_crc != actual_crc)
  {
    return MOTOR_DRIVER_PROTOCOL_ERROR;
  }

  for (index = 0U; index < register_count; ++index)
  {
    registers[index] = MotorDriver_ReadU16(&response[3U + index * 2U]);
  }

  return MOTOR_DRIVER_OK;
}

static MotorDriverResult MotorDriver_WriteSingleRegister(MotorDriverContext *context, uint16_t register_address, uint16_t value)
{
  uint8_t request[8];
  uint8_t response[8];
  uint16_t crc = 0U;
  MotorDriverResult result = MOTOR_DRIVER_OK;

  request[0] = context->slave_address;
  request[1] = MOTOR_MODBUS_FUNC_WRITE_SINGLE;
  MotorDriver_WriteU16(&request[2], register_address);
  MotorDriver_WriteU16(&request[4], value);
  crc = MotorDriver_ModbusCrc16(request, 6U);
  request[6] = (uint8_t)(crc & 0xFFU);
  request[7] = (uint8_t)((crc >> 8U) & 0xFFU);

  result = MotorDriver_Transfer(context, request, sizeof(request), response, sizeof(response));
  if (result != MOTOR_DRIVER_OK)
  {
    return result;
  }

  if (memcmp(request, response, sizeof(request)) != 0)
  {
    return MOTOR_DRIVER_PROTOCOL_ERROR;
  }

  return MOTOR_DRIVER_OK;
}

static MotorDriverResult MotorDriver_WriteMultipleRegisters(MotorDriverContext *context, uint16_t register_address, uint16_t register_count, const uint16_t *values)
{
  uint8_t request[32];
  uint8_t response[8];
  uint16_t crc = 0U;
  uint16_t request_length = 0U;
  uint16_t index = 0U;
  MotorDriverResult result = MOTOR_DRIVER_OK;

  if ((register_count == 0U) || (register_count > 10U) || (values == NULL))
  {
    return MOTOR_DRIVER_INVALID_PARAM;
  }

  request[0] = context->slave_address;
  request[1] = MOTOR_MODBUS_FUNC_WRITE_MULTI;
  MotorDriver_WriteU16(&request[2], register_address);
  MotorDriver_WriteU16(&request[4], register_count);
  request[6] = (uint8_t)(register_count * 2U);

  for (index = 0U; index < register_count; ++index)
  {
    MotorDriver_WriteU16(&request[7U + index * 2U], values[index]);
  }

  request_length = (uint16_t)(7U + register_count * 2U);
  crc = MotorDriver_ModbusCrc16(request, request_length);
  request[request_length] = (uint8_t)(crc & 0xFFU);
  request[request_length + 1U] = (uint8_t)((crc >> 8U) & 0xFFU);
  request_length = (uint16_t)(request_length + 2U);

  result = MotorDriver_Transfer(context, request, request_length, response, sizeof(response));
  if (result != MOTOR_DRIVER_OK)
  {
    return result;
  }

  if ((response[0] != context->slave_address) || (response[1] != MOTOR_MODBUS_FUNC_WRITE_MULTI))
  {
    return MOTOR_DRIVER_PROTOCOL_ERROR;
  }

  if ((MotorDriver_ReadU16(&response[2]) != register_address) || (MotorDriver_ReadU16(&response[4]) != register_count))
  {
    return MOTOR_DRIVER_PROTOCOL_ERROR;
  }

  crc = (uint16_t)(((uint16_t)response[7] << 8U) | response[6]);
  if (MotorDriver_ModbusCrc16(response, 6U) != crc)
  {
    return MOTOR_DRIVER_PROTOCOL_ERROR;
  }

  return MOTOR_DRIVER_OK;
}

static MotorDriverResult MotorDriver_ReceiveBytes(MotorDriverContext *context, uint8_t *response, uint16_t response_length, uint16_t *actual_length)
{
  uint32_t started_ms = HAL_GetTick();
  uint32_t last_rx_ms = started_ms;
  uint16_t index = 0U;

  if (actual_length != NULL)
  {
    *actual_length = 0U;
  }

  while ((HAL_GetTick() - started_ms) < context->response_timeout_ms)
  {
    if (__HAL_UART_GET_FLAG(context->uart, UART_FLAG_ORE) != RESET)
    {
      __HAL_UART_CLEAR_OREFLAG(context->uart);
    }
    if (__HAL_UART_GET_FLAG(context->uart, UART_FLAG_FE) != RESET)
    {
      __HAL_UART_CLEAR_FEFLAG(context->uart);
    }
    if (__HAL_UART_GET_FLAG(context->uart, UART_FLAG_NE) != RESET)
    {
      __HAL_UART_CLEAR_NEFLAG(context->uart);
    }

    if (__HAL_UART_GET_FLAG(context->uart, UART_FLAG_RXNE) != RESET)
    {
      response[index++] = (uint8_t)(context->uart->Instance->RDR & 0xFFU);
      last_rx_ms = HAL_GetTick();
      if (index >= response_length)
      {
        if (actual_length != NULL)
        {
          *actual_length = index;
        }
        return MOTOR_DRIVER_OK;
      }
    }
    else if ((index > 0U) && ((HAL_GetTick() - last_rx_ms) > 20U))
    {
      break;
    }
  }

  if (actual_length != NULL)
  {
    *actual_length = index;
  }
  return (index == response_length) ? MOTOR_DRIVER_OK : MOTOR_DRIVER_TIMEOUT;
}

static uint16_t MotorDriver_ReadU16(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static void MotorDriver_WriteU16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)((value >> 8U) & 0xFFU);
  data[1] = (uint8_t)(value & 0xFFU);
}
