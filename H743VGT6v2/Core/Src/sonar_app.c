#include "sonar_app.h"

#include <string.h>

void SonarApp_Init(SonarAppContext *context)
{
  memset(context, 0, sizeof(*context));
  context->status = SONAR_STATUS_IDLE;
  context->mode = SONAR_APP_MODE_IDLE;
  context->scan_config.scan_step = 100U;
  context->scan_config.sound_speed = 1500U;
  context->debug_config.sample_count = 7500U;
  context->debug_config.pulse_count = 6U;
}

void SonarApp_Process(SonarAppContext *context, uint32_t now_ms)
{
  (void)context;
  (void)now_ms;
}

void SonarApp_RequestStop(SonarAppContext *context)
{
  context->status = SONAR_STATUS_STOPPING;
  context->mode = SONAR_APP_MODE_IDLE;
}

void SonarApp_OnZeroingComplete(SonarAppContext *context)
{
  context->current_angle = 0U;
  context->target_angle = 0U;
  context->status = SONAR_STATUS_READY;
}

void SonarApp_OnMoveComplete(SonarAppContext *context, uint16_t current_angle)
{
  context->current_angle = current_angle;
  context->status = SONAR_STATUS_READY;
}

void SonarApp_OnMeasureComplete(SonarAppContext *context)
{
  if (context->mode == SONAR_APP_MODE_SCAN)
  {
    context->status = SONAR_STATUS_SCANNING;
  }
  else if (context->mode == SONAR_APP_MODE_DEBUG_SWEEP)
  {
    context->status = SONAR_STATUS_DEBUG_SCANNING;
  }
  else
  {
    context->status = SONAR_STATUS_READY;
  }
}
