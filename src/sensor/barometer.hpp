#pragma once

/**
 * @brief 气压计
 */
class Barometer
{
public:
  /**
   * @brief 获取海拔高度, 单位: m
   */
  virtual float get_altitude() const = 0;

  /**
   * @brief 获取气压, 单位: Pa
   */
  virtual float get_pressure() const = 0;
};