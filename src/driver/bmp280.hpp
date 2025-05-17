#pragma once

#include <Adafruit_BMP280.h>

#include "../sensor/barometer.hpp"
#include "../sensor/temperature.hpp"
#include "../utility.hpp"

/**
 * @brief BMP280 气压计
 */
class BMP280: public Barometer, public Temperature {
  public:
    BMP280() {
        if (!bmp_.begin()) {
            Serial.println("Failed to find BMP280");
            terminate();
        }
    }

    float get_altitude() const override {
        return bmp_.readAltitude();
    }

    float get_pressure() const override {
        return bmp_.readPressure();
    }

    float get_temperature() const override {
        return bmp_.readTemperature();
    }

  private:
    Adafruit_BMP280 bmp_;
};