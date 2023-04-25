#pragma once

#include "../actuator/motor.hpp"
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

	void attach(uint8_t pin) override
	{
		if(servo_.attach(pin) == INVALID_SERVO)
			abort();
	}

private:
	void set_speed_(float speed) override { servo_.writeMicroseconds(map(speed, 0.f, 1.f, 1500, 2000)); }

	Servo servo_;
};