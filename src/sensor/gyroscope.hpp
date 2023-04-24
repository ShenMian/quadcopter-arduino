#pragma once

#include "../math.hpp"

/**
 * @brief 陀螺仪
 */
class Gyroscope
{
public:
  /**
   * @brief 获取角速度, 单位: rad/s
   */
  virtual EulerAngles get_angular_velocity() const = 0;
};