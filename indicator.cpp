#include "indicator.h"
#include <Arduino.h>

IndicatorState indicator_state = {};

void setup_nav_lights()
{
  pinMode(NavLight_Font, OUTPUT);
  pinMode(NavLight_Back, OUTPUT);
}

void set_nav_lights(uint8_t value)
{
  digitalWrite(NavLight_Font, value);
  digitalWrite(NavLight_Back, value);
}

void update_indicator_light()
{
  if(indicator_state.count == 0)
    return;
  indicator_state.timer += micros();
  if(indicator_state.timer < indicator_state.interval)
    return;
  indicator_state.timer = 0;

  set_nav_lights(indicator_state.count % 2);
  if(indicator_state.count != -1)
    indicator_state.count++;
}

void set_indicator_light(ErrorCode code)
{
  switch(code)
  {
    case ErrorCode::Normal:
      indicator_state.count    = -1;
      indicator_state.interval = 1000;
      break;
      
    case ErrorCode::RadioDisconnected:
      indicator_state.count    = 3 * 2;
      indicator_state.interval = 200;
      break;
      
    case ErrorCode::LowBattery:
      indicator_state.count    = 5 * 2;
      indicator_state.interval = 500;
      break;
      
    case ErrorCode::CriticalLowBattery:
      indicator_state.count    = 5 * 2;
      indicator_state.interval = 200;
      break;

    case ErrorCode::Unknown:
      indicator_state.count    = 5 * 2;
      indicator_state.interval = 500;
      break;
  }
}