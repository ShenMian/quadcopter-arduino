#pragma once

#include "math.hpp"
#include "sensor/accelerometer.hpp"
#include "sensor/barometer.hpp"
#include "sensor/gyroscope.hpp"
#include "sensor/temperature.hpp"

#include "driver/bmp280.hpp"
#include "driver/mpu6050.hpp"

#include <Arduino.h>
#include <Kalman.h>

/**
 * @brief 位置和姿态评估器
 */
class Estimator
{
public:
	Estimator()
	{
		accelerometer_ = static_cast<Accelerometer*>(&mpu_);
		gyroscope_     = static_cast<Gyroscope*>(&mpu_);
		barometer_     = static_cast<Barometer*>(&bmp_);
		temperature_   = static_cast<Temperature*>(&bmp_);

		const auto  acc = get_acceleration();
		const float acc_roll  = rad_to_deg(atan2(-acc.x, sqrt(acc.y * acc.y + acc.z * acc.z)));
		const float acc_pitch = rad_to_deg(atan2(acc.y, sqrt(acc.x * acc.x + acc.z * acc.z)));

		angles_.yaw   = 0.f;
		angles_.pitch = acc_pitch;
		angles_.roll  = acc_roll;
		pitch_kalman_filter_.setAngle(angles_.pitch);
		roll_kalman_filter_.setAngle(angles_.roll);
	}

	/**
	 * @brief 更新位置和姿态
	 *
	 * @param dt 时间变化量
	 */
	void update(float dt)
	{
		const auto acc  = get_acceleration();
		const auto gyro = get_angular_velocity();

		velocity_ += acc * dt;
		position_ += velocity_ * dt;
		position_.z = (get_altitude() - takeoff_altitude_) * 0.9 + position_.z * 0.1;

		const float acc_roll  = rad_to_deg(atan2(-acc.x, sqrt(acc.y * acc.y + acc.z * acc.z)));
		const float acc_pitch = rad_to_deg(atan2(acc.y, sqrt(acc.x * acc.x + acc.z * acc.z)));

		angles_.yaw += rad_to_deg(gyro.yaw) * dt;
		angles_.pitch = pitch_kalman_filter_.getAngle(angles_.pitch * 0.8 + acc_pitch * 0.2, rad_to_deg(gyro.pitch), dt);
		angles_.roll  = roll_kalman_filter_.getAngle(angles_.roll * 0.8 + acc_roll * 0.2, rad_to_deg(gyro.roll), dt);
	}

	Vector3     get_position() const noexcept { return position_; }
	Vector3     get_velocity() const noexcept { return velocity_; }
	EulerAngles get_angles() const noexcept { return angles_; }

	Vector3     get_acceleration() const { return accelerometer_->get_acceleration(); };
	EulerAngles get_angular_velocity() const { return gyroscope_->get_angular_velocity(); };
	float       get_altitude() const { return barometer_->get_altitude(); }
	float       get_pressure() const { return barometer_->get_pressure(); }
	float       get_temperature() const { return temperature_->get_temperature(); }

	/**
	 * @brief 获取空气密度, 单位: kg/m³
	 */
	float get_air_density() const
	{
		constexpr float std_pressure = 1.293f; // 标准大气压
		return 273.f * get_pressure() * std_pressure / 101325.f * (273.f + get_temperature());
	}

private:
	Accelerometer* accelerometer_;
	Gyroscope*     gyroscope_;
	Barometer*     barometer_;
	Temperature*   temperature_;

	float       takeoff_altitude_;
	Vector3     velocity_;
	Vector3     position_;
	EulerAngles angles_;

	// 卡尔曼滤波器
	Kalman pitch_kalman_filter_;
	Kalman roll_kalman_filter_;

	// 传感器组件
	MPU6050 mpu_;
	BMP280  bmp_;
};