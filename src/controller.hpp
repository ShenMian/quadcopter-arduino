#pragma once

#include "estimator.hpp"
#include "pid.hpp"
#include "rotor.hpp"
#include "utility.hpp"

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
	void update(Estimator& estimator, float dt)
	{
		const auto angles = estimator.get_angles();

		EulerAngles output_angles;
		output_angles.yaw   = angle_yaw_pid_.pid(target_angles_.yaw, angles.yaw, dt);
		output_angles.pitch = angle_pitch_pid_.pid(target_angles_.pitch, angles.pitch, dt);
		output_angles.roll  = angle_roll_pid_.pid(target_angles_.roll, angles.roll, dt);

		const auto  angular_velocity = estimator.get_angular_velocity();
		EulerAngles output_angular;
		output_angular.yaw   = angular_yaw_pid_.pid(output_angles.yaw, angular_velocity.yaw, dt);
		output_angular.pitch = angular_pitch_pid_.pid(output_angles.pitch, angular_velocity.pitch, dt);
		output_angular.roll  = angular_roll_pid_.pid(output_angles.roll, angular_velocity.roll, dt);

		for(uint8_t i = 0; i < rotor_count_; i++)
		{
			rotors_[i].motor->set_speed((target_throttle_ + output_angular.yaw * rotors_[i].yaw_scale +
			                             output_angular.pitch * rotors_[i].pitch_scale +
			                             output_angular.roll * rotors_[i].roll_scale) *
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
		target_angles_.pitch = clamp(angles.pitch, -max_pitch_angle_, max_pitch_angle_);
		target_angles_.roll  = clamp(angles.roll, -max_roll_angle_, max_roll_angle_);
	}

	/**
	 * @brief 设置目标节流阀
	 *
	 * @param target_throttle 目标节流阀, 范围 [0, 1]
	 */
	void set_throttle(float throttle) { target_throttle_ = clamp(throttle, 0.f, 1.f); }

private:
	float       target_throttle_;
	EulerAngles target_angles_;

	Rotor*  rotors_;
	uint8_t rotor_count_;

	PID angle_yaw_pid_{4.0, 0.02, 0};
	PID angle_pitch_pid_{1.3, 0.04, 18};
	PID angle_roll_pid_{1.3, 0.04, 18};
	PID angular_yaw_pid_{1.0, 0.0, 0.0};
	PID angular_pitch_pid_{1.0, 0.0, 0.0};
	PID angular_roll_pid_{1.0, 0.0, 0.0};

	float max_pitch_angle_ = deg_to_rad(30);
	float max_roll_angle_  = deg_to_rad(30);
};
