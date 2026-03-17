#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum
{
  MOTOR_DRIVER_OK = 0,
  MOTOR_DRIVER_UART_ERROR = 1,
  MOTOR_DRIVER_TIMEOUT = 2,
  MOTOR_DRIVER_PROTOCOL_ERROR = 3,
  MOTOR_DRIVER_REMOTE_ERROR = 4,
  MOTOR_DRIVER_MOTION_ERROR = 5,
  MOTOR_DRIVER_INVALID_PARAM = 6
} MotorDriverResult;

typedef struct
{
  UART_HandleTypeDef *uart;
  uint8_t slave_address;
  uint16_t single_turn_steps;
  uint16_t response_timeout_ms;
} MotorDriverContext;

typedef struct
{
  uint32_t sequence;
  uint32_t result;
  uint16_t request_length;
  uint16_t response_length;
  uint8_t request[32];
  uint8_t response[64];
} MotorDriverTrace;

extern volatile MotorDriverTrace g_motor_driver_trace;

void MotorDriver_Init(MotorDriverContext *context, UART_HandleTypeDef *uart, uint8_t slave_address, uint16_t single_turn_steps);
MotorDriverResult MotorDriver_ReadStatus(MotorDriverContext *context, uint16_t *status);
MotorDriverResult MotorDriver_ReadActualSteps(MotorDriverContext *context, int32_t *steps);
MotorDriverResult MotorDriver_MoveSingleTurn(MotorDriverContext *context, uint16_t target_steps, uint16_t speed_rpm, uint16_t move_mode);
MotorDriverResult MotorDriver_MoveForwardSteps(MotorDriverContext *context, int32_t step_delta, uint16_t start_speed_rpm_x10, uint16_t max_speed_rpm, uint16_t accel_time_ms, uint16_t tolerance_steps);
MotorDriverResult MotorDriver_Home(MotorDriverContext *context, uint16_t speed_rpm);
MotorDriverResult MotorDriver_SetZero(MotorDriverContext *context);
MotorDriverResult MotorDriver_Stop(MotorDriverContext *context);
MotorDriverResult MotorDriver_WaitForIdle(MotorDriverContext *context, uint32_t timeout_ms, uint16_t *final_status);
uint16_t MotorDriver_AngleCdegToSingleTurnSteps(const MotorDriverContext *context, uint16_t angle_cdeg);
uint16_t MotorDriver_SingleTurnStepsToAngleCdeg(const MotorDriverContext *context, uint16_t single_turn_steps);
uint16_t MotorDriver_NormalizeAngleCdeg(const MotorDriverContext *context, uint16_t angle_cdeg);
uint16_t MotorDriver_NormalizeStepCdeg(const MotorDriverContext *context, uint16_t step_cdeg);
uint16_t MotorDriver_ActualStepsToSingleTurnSteps(const MotorDriverContext *context, int32_t actual_steps);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_DRIVER_H__ */
