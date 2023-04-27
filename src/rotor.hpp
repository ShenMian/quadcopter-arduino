#pragma once

#include "actuator/motor.hpp"

/**
 * @brief 转子. 包含电机和电机在调整姿态时的缩放参数
 */
struct Rotor
{
	Rotor(Motor& motor, float yaw_scale, float pitch_scale, float roll_scale)
	    : motor(motor), yaw_scale(yaw_scale), pitch_scale(pitch_scale), roll_scale(roll_scale)
	{
	}

	Motor& motor;
	float  yaw_scale;
	float  pitch_scale;
	float  roll_scale;
	float  scale = 1.f;
};
