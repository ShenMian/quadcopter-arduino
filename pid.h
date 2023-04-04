#pragma once

/**
 * @brief PID 状态
 *
 * 存储计算 PID 所需的参数.
 */
struct PIDState
{
  PIDState(float kP, float kI, float kD)
    : kP(kP), kI(kI), kD(kD)
  {}

  float kP, kI, kD;     ///< PID 参数
  float integral = 0.f; ///< 积分
  float prev_error;     ///< 上一次误差
};

/**
 * @brief PID 控制器
 *
 * @param target 设定点(setpoint)
 * @param actual 测量的过程值(process variable)
 * @param dt     时间变化量
 * @param state  PID 状态
 */
float pid(float target, float actual, float dt, PIDState& state);