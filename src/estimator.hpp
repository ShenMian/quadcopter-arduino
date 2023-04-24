#pragma once

#include "math.hpp"
#include "sensor/accelerometer.hpp"
#include "sensor/gyroscope.hpp"
#include "sensor/barometer.hpp"

#include "driver/mpu6050.hpp"
#include "driver/bmp280.hpp"

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

    velocity_  += acc * dt;
    position_  += velocity_ * dt;
    position_.z = get_altitude() - takeoff_altitude_;

# if 1
    const float roll  = rad_to_deg(atan2(acc.y, acc.z));
    const float pitch = rad_to_deg(atan2(-acc.x, sqrt(acc.y * acc.y + acc.z * acc.z)));
#elif 0
    const float roll  = rad_to_deg(atan2(acc.y, acc.z));
    const float pitch = rad_to_deg(atan(-acc.x / sqrt(acc.y * acc.y + acc.z * acc.z)));
#else
    const float roll  = rad_to_degatan(acc.y / sqrt(acc.x * acc.x + acc.z * acc.z));
    const float pitch = rad_to_deg(atan2(-acc.x, acc.z));
#endif

    angles_.pitch = pitch_kalman_filter_.getAngle(pitch, gyro.pitch, dt);
    angles_.roll  = roll_kalman_filter_.getAngle(roll, gyro.roll, dt);
  }

  Vector3     get_position() const noexcept { return position_; }
  Vector3     get_velocity() const noexcept { return velocity_; }
  EulerAngles get_angles() const noexcept { return angles_; }

  Vector3     get_acceleration() const { return accelerometer_->get_acceleration(); };
  EulerAngles get_angular_velocity() const { return gyroscope_->get_angular_velocity(); };
  float       get_altitude() const { return barometer_->get_altitude(); }

  private:
  Accelerometer* accelerometer_;
  Gyroscope*     gyroscope_;
  Barometer*     barometer_;

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