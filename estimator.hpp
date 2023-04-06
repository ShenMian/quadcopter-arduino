#pragma once

#include <JY901.h>
#include "types.h"

/**
 * @brief 位置和姿态估计器
 *
 * 从传感器获取数据并推测机体的位置和姿态.
 */
class Estimator
{
public:
  void update()
  {
    // update_jy901();
    update_position();
    update_angles();
  }

  /**
  * @brief 更新 JY901 数据
  *
  * 包含三轴加速度计, 三轴陀螺仪, 气压计等
  */
  void update_jy901()
  {
    // TODO: 用途不明
    while (Serial.available()) {
      JY901.CopeSerialData(Serial.read()); // Call JY901 data cope function
    }
  }

  /**
  * @brief 获取加速度
  *
  * MPU 6050 坐标系:
  *
  *   z
  * y ↑
  *  \|
  *   +----→ x
  *
  * 安装的时候可以绕 Z 轴旋转 90°, 转换为机体坐标系
  */
  Vector3 get_acceleration()
  {
    return {JY901.stcAcc.a[0], JY901.stcAcc.a[1], JY901.stcAcc.a[2]};
    // return {JY901.stcAcc.a[1], -JY901.stcAcc.a[0], JY901.stcAcc.a[2]};
  }

  /**
  * @brief 获取姿态角
  *
  * @see get_acceleration
  */
  EulerAngles get_angles_by_jy901()
  {
    return { (float)JY901.stcAngle.Angle[0] / (SHRT_MAX + 1),    // yaw
             (float)JY901.stcAngle.Angle[1] / (SHRT_MAX + 1),    // pitch
             (float)JY901.stcAngle.Angle[2] / (SHRT_MAX + 1) };  // roll
    /*
    return { (float)JY901.stcAngle.Angle[2] / (SHRT_MAX + 1),    // yaw
             (float)JY901.stcAngle.Angle[0] / (SHRT_MAX + 1),    // pitch
             (float)JY901.stcAngle.Angle[1] / (SHRT_MAX + 1) };  // roll
    */
  }

  /**
  * @brief 获取角速度
  */
  EulerAngles get_angular_velocity()
  {
    return { (float)JY901.stcGyro.w[0] / (SHRT_MAX + 1),    // yaw
             (float)JY901.stcGyro.w[1] / (SHRT_MAX + 1),    // pitch
             (float)JY901.stcGyro.w[2] / (SHRT_MAX + 1) };  // roll
  }

  /**
  * @brief 获取估计姿态角
  */
  EulerAngles get_angles() const noexcept { return actual_angles; }

  /**
  * @brief 获取估计位置
  */
  Vector3 get_position() const noexcept { return actual_position; }

  /**
  * @brief 获取气压计测量海拔
  *
  * 精度: ±1m
  */
  long get_altitude() const noexcept { return JY901.stcPress.lAltitude; }

  void set_takeoff_altitude(float altitude) noexcept { takeoff_altitude = altitude; };

private:
  /**
  * @brief 更新坐标和速度
  */
  void update_position()
  {
    // TODO: 加速度计在震动环境下误差较大, 结合加速度计和陀螺仪的数据来获得更加准确的姿态

    const Vector3 acceleration = get_acceleration();
    actual_velocity += acceleration * dt;
    actual_position += actual_velocity * dt;

    actual_position.z = get_altitude() - takeoff_altitude;
  }

  /**
  * @brief 更新姿态角
  */
  void update_angles()
  {
    /*
    const EulerAngles angular_velocity = get_angular_velocity();
    for(int i = 0; i < 3; i++)
      actual_angles.v[i] += angular_velocity.v[i] * dt;
    */
    actual_angles = get_angles_by_jy901();
  }

  // ZYX 欧拉角, 弧度制
  EulerAngles actual_angles = {}; // 当前姿态角

  // 以起飞点为坐标系原点
  Vector3 actual_position = {}; // 当前坐标
  Vector3 actual_velocity = {}; // 当前速度
  
  long takeoff_altitude; // 起飞时海拔, 用于计算相对海拔
};