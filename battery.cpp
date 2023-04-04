#include "battery.h"
#include <Arduino.h>

float get_battery_level()
{
  const auto voltage       = analogRead(A0) * 5.f / 1023.f;
  const auto battery_level = map(voltage, 3.6f, 4.2f, 0.f, 100.f);
  return battery_level;
}

void check_battery_level()
{
  const auto battery_level = get_battery_level();

  if(battery_level < 20.f)
    set_indicator_light(ErrorCode::CriticalLowBattery);
  else if(battery_level < 30.f)
    set_indicator_light(ErrorCode::LowBattery);
}