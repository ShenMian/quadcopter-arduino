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
  float kp, ki, kd; ///< PID 参数

  PID(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd)
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

    const float p = kp * error;
    integral_    += ki * error * dt;
    const float d = kd * (error - prev_error_) / dt;

    integral_ = clamp(integral_, -integral_limit_, integral_limit_);

    return p + integral_ + d;
  }

  /**
  * @brief 设置积分项绝对最大值, 避免积分饱和
  *
  * @param max_abs 积分项绝对最大值
  */
  void set_integral_limit(float limit) { integral_limit_ = abs(limit); }

private:
  float integral_       = 0.f;     ///< 积分
  float integral_limit_ = FLT_MAX; ///< 积分最大值
  float prev_error_;               ///< 上一次误差
};
