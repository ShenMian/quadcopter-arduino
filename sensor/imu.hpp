#pragma once

#include "../types.h"

class IMU
{
public:
  /**
  * @brief 获取加速度
  */
  virtual Vector3 get_acceleration() const = 0;

  /**
  * @brief 获取角速度
  */
  virtual EulerAngles get_angular_velocity() const = 0;

  /**
  * @brief 获取姿态角
  */
  virtual EulerAngles get_angles() const = 0;
};