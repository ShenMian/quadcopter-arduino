#pragma once

#include "../sensor/accelerometer.hpp"
#include "../sensor/gyroscope.hpp"
#include "../sensor/temperature.hpp"
#include "../utility.hpp"
#include <Adafruit_MPU6050.h>

/**
 * @brief MPU6050 6 轴运动处理组件
 */
class MPU6050 : public Accelerometer, public Gyroscope, public Temperature
{
public:
	MPU6050()
	{
		if(!mpu_.begin())
		{
			Serial.println("Failed to find MPU6050");
			terminate();
		}
		mpu_.setAccelerometerRange(MPU6050_RANGE_8_G);
		mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
		mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);
	}

	Vector3 get_acceleration() const override
	{
		sensors_event_t accel_event, gyro_event, temp_event;
		mpu_.getEvent(&accel_event, &gyro_event, &temp_event);
		return {accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z};
	}

	EulerAngles get_angular_velocity() const override
	{
		sensors_event_t accel_event, gyro_event, temp_event;
		mpu_.getEvent(&accel_event, &gyro_event, &temp_event);
		return {gyro_event.gyro.x, gyro_event.gyro.y, gyro_event.gyro.z};
	}

	float get_temperature() const override
	{
		sensors_event_t accel_event, gyro_event, temp_event;
		mpu_.getEvent(&accel_event, &gyro_event, &temp_event);
		return temp_event.temperature;
	}

private:
	Adafruit_MPU6050 mpu_;
};
