#pragma once

#include <limits.h>
#include "types.h"

#include "sensor/imu/jy901_imu.hpp"
#include "sensor/barometer/jy901_barometer.hpp"

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
    update_position();
    update_angles();
  }

  /**
  * @brief 获取估计姿态角
  */
  EulerAngles get_angles() const noexcept { return estimate_angles; }

  /**
  * @brief 获取估计位置
  */
  Vector3 get_position() const noexcept { return estimate_position; }

  /**
  * @brief 起飞
  *
  * 设置起飞海拔.
  */
  void takeoff() noexcept { takeoff_altitude = get_altitude(); }

  /**
  * @brief 获取加速度
  */
  Vector3 get_acceleration() const { return imu.get_acceleration(); }

  /**
  * @brief 获取角速度
  */
  EulerAngles get_angular_velocity() const { return imu.get_angular_velocity(); }

  /**
  * @brief 获取气压计测量海拔
  */
  long get_altitude() const noexcept { return barometer.get_altitude(); }

private:
  /**
  * @brief 更新坐标和速度
  */
  void update_position()
  {
    const Vector3 acceleration = get_acceleration();
    estimate_velocity += acceleration * dt;
    estimate_position += estimate_velocity * dt;

    estimate_position.z = get_altitude() - takeoff_altitude;
  }

  /**
  * @brief 更新姿态角
  */
  void update_angles()
  {
    estimate_angles = imu.get_angles();
    /*
    const EulerAngles angular_velocity = get_angular_velocity();
    for(int i = 0; i < 3; i++)
      estimate_angles.v[i] += angular_velocity.v[i] * dt;
    */
  }

  // ZYX 欧拉角, 弧度制
  EulerAngles estimate_angles = {}; // 当前姿态角

  // 以起飞点为坐标系原点
  Vector3 estimate_position = {}; // 当前坐标
  Vector3 estimate_velocity = {}; // 当前速度
  
  long takeoff_altitude; // 起飞时海拔, 用于计算相对海拔
  
  JY901_IMU       imu;
  JY901_Barometer barometer;
};