#pragma once

#include "../actuator/motor.hpp"
#include "../utility.hpp"
#include <Arduino.h>
#include <stdint.h>

/**
 * @brief 直流电机
 */
class DCMotor : public Motor
{
public:
	DCMotor() = default;

	DCMotor(uint8_t pin) { attach(pin); }

	void attach(uint8_t pin) override
	{
		pin_ = pin;
		pinMode(pin_, OUTPUT);
	}

	/**
	 * @brief 设置 PWM 占空比
	 *
	 * @param duty_cycle 占空比, 范围 [0, 1]
	 */
	void set_pwm_duty_cycle(float duty_cycle)
	{
		duty_cycle = clamp(duty_cycle, 0.f, 1.f);
		analogWrite(pin_, map(duty_cycle, 0.f, 1.f, 0, 255));
	}

	/**
	 * @brief 设置角速率
	 *
	 * @param rate    角速率, 单位: RPM
	 * @param voltage 电压, 单位: V
	 *
	 * @warning 未测试/可以绕过锁定
	 */
	void set_angular_rate(uint8_t rate, float voltage)
	{
		const float duty_cycle = rate * internal_resis_ * rated_speed_ / (voltage * 60);
		set_pwm_duty_cycle(duty_cycle);
	}

private:
	void set_speed_(float speed) override { set_pwm_duty_cycle(speed); }

	uint8_t  pin_;
	uint16_t rated_speed_;
	float    internal_resis_;
};