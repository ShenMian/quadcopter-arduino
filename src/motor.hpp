#pragma once

#include <Arduino.h>
#include <stdint.h>

/**
 * @brief 电机
 */
class Motor
{
public:
	/**
	 * @brief 设置电机转速
	 *
	 * @param speed 电机转速, 范围: [0, 1]
	 */
	virtual void set_speed(float speed) = 0;

	virtual void attach(uint8_t pin) = 0;
};