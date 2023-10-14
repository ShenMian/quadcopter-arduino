#pragma once

#include "../math.hpp"

/**
 * @brief 陀螺仪
 */
class Gyroscope
{
public:
	/**
	 * @brief 获取角速度, 单位: rad/s
	 */
	EulerAngles get_angular_velocity() const
	{
		const auto angular_velocity_raw = get_raw_angular_velocity();
		return angular_velocity_raw + offset_;
	}

	/**
	 * @brief 计算陀螺仪静止时的角速度.
	 * 
	 * @param duration 校准时间, 单位: 秒.
	 */
	void calc_offset(float duration)
	{
		size_t times = 0;
		auto start_timepoint = millis();
		do {
			offset_ += get_raw_angular_velocity();
			times++;
		} while(millis() - start_timepoint < duration * 1000);
		offset_ /= times;
		offset_ *= -1;
	}

	/**
	 * @brief 获取原始角速度, 单位: rad/s
	 */
	virtual EulerAngles get_raw_angular_velocity() const = 0;

private:
	EulerAngles offset_;
};