#pragma once

#include <Arduino.h>
#include <stdint.h>

class Battery
{
public:
  Battery(uint8_t pin)
    : pin_(pin)
  {
  }

  /**
   * @brief 获取电池剩余电量, 范围 [0-100]
   * 
   * @return float 电池剩余电量
   */
  float get_battery_level() const noexcept
  {
    const auto voltage = analogRead(pin_) * 5.f / 1023.f;
    const auto level   = map(voltage, 3.6f, 4.2f, 0.f, 100.f);
    return level;
  }

private:
  uint8_t pin_;
};