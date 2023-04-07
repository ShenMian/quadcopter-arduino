#pragma once

#include "../imu.hpp"
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
  /**
  * @brief 更新 JY901 数据
  *
  * 包含三轴加速度计, 三轴陀螺仪, 气压计等
  */
  /*
  void update_jy901()
  {
    // TODO: 用途不明
    while (Serial.available()) {
      JY901.CopeSerialData(Serial.read()); // Call JY901 data cope function
    }
  }
  */

  Vector3 get_acceleration() const override
  {
    return {JY901.stcAcc.a[0], JY901.stcAcc.a[1], JY901.stcAcc.a[2]};
  }

  EulerAngles get_angular_velocity() const override
  {
    return { (float)JY901.stcGyro.w[0] / (SHRT_MAX + 1),   // yaw
             (float)JY901.stcGyro.w[1] / (SHRT_MAX + 1),   // pitch
             (float)JY901.stcGyro.w[2] / (SHRT_MAX + 1) }; // roll
  }

  EulerAngles get_angles() const override
  {
    return { (float)JY901.stcAngle.Angle[0] / (SHRT_MAX + 1),   // yaw
             (float)JY901.stcAngle.Angle[1] / (SHRT_MAX + 1),   // pitch
             (float)JY901.stcAngle.Angle[2] / (SHRT_MAX + 1) }; // roll
  }
};