#pragma once

#include "utility.hpp"
#include <Arduino.h>
#include <float.h>

/**
 * @brief PID 控制器
 */
class PID
{
public:
  float kP, kI, kD; ///< PID 参数

  PID(float kP, float kI, float kD)
    : kP(kP), kI(kI), kD(kD)
  {}

  /**
  * @brief PID 控制器
  *
  * @param target 设定点
  * @param actual 测量的过程值
  * @param dt     时间变化量
  */
  float pid(float target, float actual, float dt)
  {
    const float error = target - actual;
    prev_error_ = error;

    const float p = kP * error;
    integral_    += kI * error * dt;
    const float d = kD * (error - prev_error_) / dt;

    integral_ = clamp(integral_, -max_integral_abs_, max_integral_abs_);

    return p + integral_ + d;
  }

  /**
  * @brief 设置积分项绝对最大值, 避免积分饱和
  *
  * @param max_abs 积分项绝对最大值
  */
  void set_max_integral(float max_abs)
  {
    if(max_abs < 0)
      abort();
    max_integral_abs_ = max_abs;
  }

private:
  float integral_         = 0.f;     ///< 积分
  float max_integral_abs_ = FLT_MAX; ///< 积分最大值
  float prev_error_;                 ///< 上一次误差
};
