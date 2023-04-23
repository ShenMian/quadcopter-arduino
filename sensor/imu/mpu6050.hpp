#pragma once

#include "../imu.hpp"
#include "../../config.h"
#include "../../indicator.h"
#include <Adafruit_MPU6050.h>

/**
 * @brief MPU6050
 * 
 * MPU 6050 坐标系:
 *
 *   z
 * y ↑
 *  \|
 *   +----→ x
 *
 * 安装的时候可以绕 Z 轴旋转 90°, 转换为机体坐标系.
 */
class MPU6050 : public IMU
{
public:
  MPU6050()
  {
    if(!mpu.begin()) {
      // TOOD: failed to find MPU6050 chip
      stop_with_error(ErrorCode::SelfTestFailed);
    }
  }

  Vector3 get_acceleration() const override
  {
    sensors_event_t acc;
    mpu.getEvent(&acc, nullptr, nullptr);
    // return {acc.acceleration.x, acc.acceleration.y, acc.acceleration.z};
    return {-acc.acceleration.y, acc.acceleration.x, acc.acceleration.z}; // 绕 Z 轴旋转 -90°
  }

  EulerAngles get_angular_velocity() const override
  {
    sensors_event_t gyro;
    mpu.getEvent(nullptr, &gyro, nullptr);
    // return {gyro.gyro.x, gyro.gyro.y, gyro.gyro.z};
    return {-gyro.gyro.y, gyro.gyro.x, gyro.gyro.z}; // 绕 Z 轴旋转 -90°
  }

  EulerAngles get_angles() const override
  {
  }

private:
  Adafruit_MPU6050 mpu;
};