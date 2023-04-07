#include "battery.h"
#include "config.h"
#include <Arduino.h>

float get_battery_level(uint8_t pin)
{
  // voltage = voltage * 0.92 + (analogRead(0) + 65) * 0.09853;
  const auto voltage = analogRead(pin) * 5.f / 1023.f;
  const auto level   = map(voltage, 3.6f, 4.2f, 0.f, 100.f);
  return level;
}

void check_battery_level(uint8_t pin)
{
  const auto level = get_battery_level(pin);

  if(level < critical_low_battery_level)
    set_indicator_light(ErrorCode::CriticalLowBattery);
  else if(level < low_battery_level)
    set_indicator_light(ErrorCode::LowBattery);
}