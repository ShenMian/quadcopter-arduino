#pragma once

#include "math.hpp"

/**
 * @brief 加速度计
 */
class Accelerometer
{
public:
  /**
   * @brief 获取加速度, 单位: m/s²
   */
  virtual Vector3 get_acceleration() const = 0;
};