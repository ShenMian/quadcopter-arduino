#pragma once

#include "../motor.hpp"
#include <Arduino.h>
#include <stdint.h>

/**
 * @brief 直流电机
 */
class DCMotor : public Motor
{
public:
	DCMotor() = default;

	DCMotor(uint8_t pin)
	{
		attach(pin);
	}

	void set_speed(float speed) override
	{
		if(speed < 0.f || speed > 1.f)
			abort();

		analogWrite(pin_, map(speed, 0.f, 1.f, 0, 255));
	}

	void attach(uint8_t pin) override
	{
    pin_ = pin;
		pinMode(pin_, OUTPUT);
	}

private:
  uint8_t pin_;
};