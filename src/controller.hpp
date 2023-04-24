#pragma once

#include "evaluator.hpp"
#include "pid.hpp"
#include "motor.hpp"

class Controller
{
public:
	/**
	 * @brief 构造函数
	 * 
	 * @param rotors      转子
	 * @param rotor_count 转子数量
	 */
	Controller(Rotor rotors[], uint8_t rotor_count) : rotors_(rotors), rotor_count_(rotor_count) {}

	/**
	 * @brief 更新电机转速
	 * 
	 * @param evaluator 评估器
	 * @param dt        时间变化量
	 */
	void update(Evaluator& evaluator, float dt)
	{
		const auto angles = evaluator.get_angles();

		EulerAngles output_angles;
		output_angles.yaw   = yaw_pid_.pid(target_angles_.yaw, angles.yaw, dt);
		output_angles.pitch = pitch_pid_.pid(target_angles_.pitch, angles.pitch, dt);
		output_angles.roll  = roll_pid_.pid(target_angles_.roll, angles.roll, dt);

		for(uint8_t i = 0; i < rotor_count_; i++)
		{
			rotors_[i].motor->set_speed((target_throttle_ + output_angles.yaw * rotors_[i].yaw_scale +
			                                                output_angles.pitch * rotors_[i].pitch_scale +
			                                                output_angles.roll * rotors_[i].roll_scale) *
			                                                   rotors_[i].scale);
		}
	}

	/**
	 * @brief 设置目标姿态角
	 *
	 * @param angles 目标姿态角
	 */
	void set_target_angles(const EulerAngles& angles) noexcept
	{
		target_angles_       = angles;
		target_angles_.pitch = constrain(angles.pitch, -max_pitch_angle_, max_pitch_angle_);
		target_angles_.roll  = constrain(angles.roll, -max_roll_angle_, max_roll_angle_);
	}

	/**
	 * @brief 设置目标节流阀
	 *
	 * @param target_throttle 目标节流阀
	 */
	void set_throttle(float throttle) { target_throttle_ = throttle; }

private:
	float       target_throttle_;
	EulerAngles target_angles_;

	Rotor*  rotors_;
	uint8_t rotor_count_;

	PID yaw_pid_{4.0, 0.02, 0};
	PID pitch_pid_{1.3, 0.04, 18};
	PID roll_pid_{1.3, 0.04, 18};

	float max_pitch_angle_ = deg_to_rad(30);
	float max_roll_angle_  = deg_to_rad(30);
};
