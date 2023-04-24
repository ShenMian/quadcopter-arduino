#pragma once

#include <Arduino.h>
#include <stdint.h>

class Battery
{
public:
	Battery(uint8_t pin, float voltage_min, float voltage_max, uint8_t resis_1, uint8_t resis_2)
	    : pin_(pin), voltage_min_(voltage_min), voltage_max_(voltage_max), resis_1_(resis_1), resis_2_(resis_2)
	{
	}

	/**
	 * @brief 获取电池电压
	 *
	 * @return float 电池电压, 单位: V
	 */
	float get_voltage() const noexcept
	{
		constexpr float voltage_ref = 5.f;
		constexpr float adc_max     = 1023.f;

		const auto voltage = analogRead(pin_) * (voltage_ref / adc_max) * ((resis_1_ + resis_2_) / (float)resis_1_);
		return voltage;
	}

	/**
	 * @brief 获取电池剩余电量, 范围 [0-100]
	 *
	 * @return float 电池剩余电量
	 */
	float get_level() const noexcept
	{
		const auto level = map(get_voltage(), voltage_min_, voltage_max_, 0.f, 100.f);
		return level;
	}

private:
	uint8_t pin_;
	float   voltage_min_;
	float   voltage_max_;
	uint8_t resis_1_; ///< 电阻 1, 单位: kΩ
	uint8_t resis_2_; ///< 电阻 1, 单位: kΩ
};