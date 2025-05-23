#pragma once

#include <Arduino.h>
#include <Servo.h>
#include <stdint.h>

#include "../actuator/motor.hpp"
#include "../utility.hpp"

/**
 * @brief 伺服电机
 */
class ServoMotor: public Motor {
  public:
    ServoMotor() = default;

    ServoMotor(uint8_t pin) {
        attach(pin);
    }

    void attach(uint8_t pin) override {
        if (servo_.attach(pin) == INVALID_SERVO) {
            Serial.println("Failed to attatch servo motor");
            terminate();
        }
    }

  private:
    void set_speed_impl(float speed) override {
        servo_.writeMicroseconds(map(speed, 0.f, 1.f, 1500, 2000));
    }

    Servo servo_;
};