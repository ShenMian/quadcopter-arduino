#pragma once

#include "../barometer.hpp"
#include "../temperature.hpp"
#include <Adafruit_BMP280.h>

/**
 * @brief BMP280 气压计
 */
class BMP280 : public Barometer, public Temperature
{
public:
  BMP280()
  {
    if(!bmp_.begin())
      abort();
  }

  float get_altitude() const override
  {
    return bmp_.readAltitude();
  }
  
  float get_pressure() const override
  {
    return bmp_.readPressure();
  }
  
  float get_temperature() const override
  {
    return bmp_.readTemperature();
  }

private:
  Adafruit_BMP280 bmp_;
};