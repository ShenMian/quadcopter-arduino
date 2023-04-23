#include "motors.h"
#include "indicator.h"
#include <Arduino.h>

Servo motors[4];

void setup_motors()
{
#if QUAD_TYPE == QUAD_X
  if(motors[FR].attach(Motor_FR) == INVALID_SERVO)
    stop_with_error(ErrorCode::SelfTestFailed);
  if(motors[FL].attach(Motor_FL) == INVALID_SERVO)
    stop_with_error(ErrorCode::SelfTestFailed);
  if(motors[BR].attach(Motor_BR) == INVALID_SERVO)
    stop_with_error(ErrorCode::SelfTestFailed);
  if(motors[BL].attach(Motor_BL) == INVALID_SERVO)
    stop_with_error(ErrorCode::SelfTestFailed);
#else
  if(motors[F].attach(Motor_F) == INVALID_SERVO)
    stop_with_error(ErrorCode::DeviceFailed);
  if(motors[B].attach(Motor_B) == INVALID_SERVO)
    stop_with_error(ErrorCode::DeviceFailed);
  if(motors[R].attach(Motor_R) == INVALID_SERVO)
    stop_with_error(ErrorCode::DeviceFailed);
  if(motors[L].attach(Motor_L) == INVALID_SERVO)
    stop_with_error(ErrorCode::DeviceFailed);
#endif
}

void update_motors(EulerAngles angles, float throttle)
{

  // FIXME:
  // 1. writeMicroseconds 理论上可以让电机反转, 但实际上不行, 桨叶的气动结构基本都是设计为单向转的. 反向转的效果只用减小电机力, 让重力实现就行
  // 2. 为了适应不同的电机需要加一些东西, writeMicroseconds 是针对无刷电调的 PWM 控制的, 如果是有刷电调(或普通 MOS 管)可能需要用 analogWrite, 因为 writeMicroseconds 貌似不能做到输出满占空比

  // TODO: 电压跌落补偿

  // TODO: Servo 库默认的频率只有 50Hz

  // 全部正转信号, 反接需要反转的电机
  // 大概 1540 开始正常转
  throttle *= 0.8f;

#if QUAD_TYPE == QUAD_X
  angles.roll *= 0.707f;
  angles.pitch *= 0.707f;
  
  const int value = (throttle - 0.f < __FLT_EPSILON__ ? 1500 : 1540) + throttle * (500 - 100 - 40);
  // 顺时针
  motors[FR].writeMicroseconds(min(value + angles.pitch + angles.roll + angles.yaw, 2000));
  motors[FL].writeMicroseconds(min(value + angles.pitch - angles.roll - angles.yaw, 2000));
  
  // 逆时针
  motors[BR].writeMicroseconds(min(value - angles.pitch + angles.roll - angles.yaw, 2000));
  motors[BL].writeMicroseconds(min(value - angles.pitch - angles.roll + angles.yaw, 2000));
#else
  // 顺时针
  motors[F].writeMicroseconds(min(value + angles.pitch - angles.yaw, 2000));
  motors[B].writeMicroseconds(min(value - angles.pitch - angles.yaw), 2000));
  
  // 逆时针
  motors[R].writeMicroseconds(min(1540 + (value + angles.roll + angles.yaw, 2000));
  motors[L].writeMicroseconds(min(1540 + (value - angles.roll + angles.yaw, 2000));
#endif
  
  /*
  Serial.print("throttle: ");
  Serial.println(throttle);
  Serial.print("front right: ");
  Serial.print(min(value + angles.pitch + angles.roll + angles.yaw, 2000));
  Serial.print(", front left: ");
  Serial.print(min(value + angles.pitch - angles.roll - angles.yaw, 2000));
  Serial.print(", back right: ");
  Serial.print(min(value - angles.pitch + angles.roll - angles.yaw, 2000));
  Serial.print(", back left: ");
  Serial.print(min(value - angles.pitch - angles.roll + angles.yaw, 2000));
  Serial.println();
  */
}