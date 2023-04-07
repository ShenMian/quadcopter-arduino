#include "motors.h"
#include <Arduino.h>

Servo motors[4];

void setup_motors()
{
#if QUAD_TYPE == QUAD_X
  if(motors[FR].attach(Motor_FR) == INVALID_SERVO)
    while(true)
    {
      set_indicator_light(ErrorCode::Unknown);
      delay(1000);
    }
  if(motors[FL].attach(Motor_FL) == INVALID_SERVO)
    while(true)
    {
      set_indicator_light(ErrorCode::Unknown);
      delay(1000);
    }
  if(motors[BR].attach(Motor_BR) == INVALID_SERVO)
    while(true)
    {
      set_indicator_light(ErrorCode::Unknown);
      delay(1000);
    }
  if(motors[BL].attach(Motor_BL) == INVALID_SERVO)
    while(true)
    {
      set_indicator_light(ErrorCode::Unknown);
      delay(1000);
    }
#else
  if(motors[F].attach(Motor_F) == INVALID_SERVO)
    while(true)
    {
      set_indicator_light(ErrorCode::Unknown);
      delay(1000);
    }
  if(motors[B].attach(Motor_B) == INVALID_SERVO)
    while(true)
    {
      set_indicator_light(ErrorCode::Unknown);
      delay(1000);
    }
  if(motors[R].attach(Motor_R) == INVALID_SERVO)
    while(true)
    {
      set_indicator_light(ErrorCode::Unknown);
      delay(1000);
    }
  if(motors[L].attach(Motor_L) == INVALID_SERVO)
    while(true)
    {
      set_indicator_light(ErrorCode::Unknown);
      delay(1000);
    }
#endif
}

void update_motors(const EulerAngles& angles, float throttle)
{
  // FIXME:
  // 1. writeMicroseconds 理论上可以让电机反转, 但实际上不行, 桨叶的气动结构基本都是设计为单向转的. 反向转的效果只用减小电机力, 让重力实现就行
  // 2. 为了适应不同的电机需要加一些东西, writeMicroseconds 是针对无刷电调的 PWM 控制的, 如果是有刷电调(或普通 MOS 管)可能需要用 analogWrite, 因为 writeMicroseconds 貌似不能做到输出满占空比

  // TODO: 电压跌落补偿

  // TODO: Servo 库默认的频率只有 50Hz

  // 假设 1000-1499 为逆时针旋转, 1501-2000 为顺时针旋转, 1500 为停止
#if QUAD_TYPE == QUAD_X
  motors[FR].writeMicroseconds(constrain(1500 - (throttle * 400 + angles.pitch + angles.roll + angles.yaw), 1000, 1500));
  motors[FL].writeMicroseconds(constrain(1500 + (throttle * 400 + angles.pitch - angles.roll - angles.yaw), 1000, 1500));
  
  motors[BR].writeMicroseconds(constrain(1500 + (throttle * 400 - angles.pitch + angles.roll - angles.yaw), 1000, 1500));
  motors[BL].writeMicroseconds(constrain(1500 - (throttle * 400 - angles.pitch - angles.roll + angles.yaw), 1000, 1500));
#else
  motors[F].writeMicroseconds(constrain(1500 + (throttle * 400 + angles.pitch - angles.yaw), 1500, 2000));
  motors[B].writeMicroseconds(constrain(1500 + (throttle * 400 - angles.pitch - angles.yaw), 1500, 2000));
  
  motors[R].writeMicroseconds(constrain(1500 - (throttle * 400 + angles.roll + angles.yaw), 1000, 1500));
  motors[L].writeMicroseconds(constrain(1500 - (throttle * 400 - angles.roll + angles.yaw), 1000, 1500));
#endif
}