#ifndef __SONAR_APP_H__
#define __SONAR_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "sonar_protocol.h"

typedef enum
{
  SONAR_APP_MODE_IDLE = 0x00U,
  SONAR_APP_MODE_SCAN = 0x01U,
  SONAR_APP_MODE_DEBUG_SINGLE = 0x02U,
  SONAR_APP_MODE_DEBUG_SWEEP = 0x03U
} SonarAppMode;

typedef struct
{
  uint16_t scan_step;
  uint16_t sound_speed;
  uint8_t medium;
  uint8_t flags;
} SonarScanConfig;

typedef struct
{
  uint16_t start_angle;
  uint16_t end_angle;
  uint16_t step_angle;
  uint8_t capture_type;
  uint16_t sample_count;
  uint8_t flags;
} SonarDebugConfig;

typedef struct
{
  SonarDeviceStatus status;
  SonarAppMode mode;
  uint16_t current_angle;
  uint16_t target_angle;
  SonarScanConfig scan_config;
  SonarDebugConfig debug_config;
} SonarAppContext;

void SonarApp_Init(SonarAppContext *context);
void SonarApp_Process(SonarAppContext *context, uint32_t now_ms);
void SonarApp_RequestStop(SonarAppContext *context);
void SonarApp_OnZeroingComplete(SonarAppContext *context);
void SonarApp_OnMoveComplete(SonarAppContext *context, uint16_t current_angle);
void SonarApp_OnMeasureComplete(SonarAppContext *context);

#ifdef __cplusplus
}
#endif

#endif /* __SONAR_APP_H__ */
