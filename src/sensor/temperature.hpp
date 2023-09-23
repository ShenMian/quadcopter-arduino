#pragma once

/**
 * @brief 温度传感器
 */
class Temperature
{
public:
	/**
	 * @brief 获取温度, 单位: °C
	 */
	virtual float get_temperature() const = 0;
};