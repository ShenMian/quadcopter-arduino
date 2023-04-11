#pragma once

#include "../imu.hpp"
#include "../../config.h"
#include <JY901.h>

/**
 * @brief JY901 上的 MPU6050
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
class JY901_IMU : public IMU
{
public:
  Vector3 get_acceleration() const override
  {
    // JY901.GetAngle();
    return {JY901.stcAcc.a[0] / (SHRT_MAX + 1) * 16.f,
            JY901.stcAcc.a[1] / (SHRT_MAX + 1) * 16.f,
            JY901.stcAcc.a[2] / (SHRT_MAX + 1) * 16.f};
  }

  EulerAngles get_angular_velocity() const override
  {
    return {(float)JY901.stcGyro.w[0] / (SHRT_MAX + 1) * 2000,  // yaw
            (float)JY901.stcGyro.w[1] / (SHRT_MAX + 1) * 2000,  // pitch
            (float)JY901.stcGyro.w[2] / (SHRT_MAX + 1) * 2000}; // roll
  }

  EulerAngles get_angles() const override
  {
    return {(float)JY901.stcAngle.Angle[0] / (SHRT_MAX + 1) / pi,  // yaw
            (float)JY901.stcAngle.Angle[1] / (SHRT_MAX + 1) / pi,  // pitch
            (float)JY901.stcAngle.Angle[2] / (SHRT_MAX + 1) / pi}; // roll
  }
};