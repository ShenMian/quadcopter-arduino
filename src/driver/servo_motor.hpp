#pragma once

#include "../motor.hpp"
#include <Arduino.h>
#include <Servo.h>
#include <stdint.h>

/**
 * @brief 伺服电机
 */
class ServoMotor : public Motor
{
public:
	ServoMotor() = default;

	ServoMotor(uint8_t pin)
	{
		attach(pin);
	}

	void set_speed(float speed) override
	{
		if(speed < 0.f || speed > 1.f)
			abort();

		servo_.writeMicroseconds(map(speed, 0.f, 1.f, 1500, 2000));
	}

	void attach(uint8_t pin) override
	{
		if(servo_.attach(pin) == INVALID_SERVO)
			abort();
	}

private:
	Servo servo_;
};