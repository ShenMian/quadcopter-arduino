#pragma once

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
  * @param target 设定点(setpoint)
  * @param actual 测量的过程值(process variable)
  * @param dt     时间变化量
  */
  float pid(float target, float actual, float dt)
  {
    const float error = target - actual;
    prev_error_ = error;

    const float p = kP * error;
    integral_    += kI * error * dt;
    const float d = kD * (error - prev_error_) / dt;

    return p + integral_ + d;
  }

private:
  float integral_ = 0.f; ///< 积分
  float prev_error_;     ///< 上一次误差
};
