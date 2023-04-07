#pragma once

#include <Arduino.h>
#include <stdint.h>

#define QUAD_PLUS 0x01 ///< 十字形模式, QUAD +
#define QUAD_X    0x02 ///< X型模式

#define QUAD_TYPE QUAD_X
#if QUAD_TYPE != QUAD_PLUS && QUAD_TYPE != QUAD_X
#error Unsupported QUAD type
#endif

/**
 * 机体坐标系:
 *
 *        z
 *      x ↑
 *       \|
 * y ←----+
 * 
 * 坐标系原点为飞行器重心, 飞行器正前方为 X 轴正方向, 正左方为 Y 轴正方向, 正上方为 Z 轴正方向
 */

enum Pin : uint8_t
{
  /**
  * (FL) (FR)
  *    \ /
  *     X
  *    / \
  * (BL) (BR)
  *
  * 电机 FL & BR 顺时针旋转.
  * 电机 FR & BL 逆顺时针旋转.
  */
  Motor_FR = 5,
  Motor_FL = 6,
  Motor_BR = 9,
  Motor_BL = 3,

  /**
  *      (F)
  *       |
  * (L)---o---(R)
  *       |
  *      (B)
  *
  * 电机 F & B 顺时针旋转.
  * 电机 L & R 逆顺时针旋转.
  */
  Motor_F = 5,
  Motor_B = 6,
  Motor_R = 9,
  Motor_L = 3,
  
  RF24_CE  = 7,
  RF24_CSN = 8,

  NavLight_Font = 2,
  NavLight_Back = 4,

  Battery = A7,
};

/**
 * @brief 电机数组下标别名
 */
enum Motor : uint8_t
{
  FR = 0, ///< 右前
  FL,     ///< 左前
  BR,     ///< 右后
  BL,     ///< 左后

  F = 0,  ///< 前
  B,      ///< 后
  R,      ///< 右
  L,      ///< 左
};

constexpr float pi = 3.1415926535897932384626433832795;

/**
 * @brief 角度制转弧度制
 */
constexpr float deg_to_rad(float deg) { return deg * pi / 180.f; }

/**
 * @brief 弧度制转角度制
 */
constexpr float rad_to_deg(float deg) { return deg * 180.f / pi; }

constexpr float dt = 0.002;   ///< 时间变化量

constexpr uint8_t radio_address[] = "00004";

constexpr float max_angle         = deg_to_rad(20.f); ///< 最大倾角, 弧度制
constexpr float max_takeoff_angle = deg_to_rad(5.f);  ///< 起飞时最大倾角, 弧度制

constexpr float low_battery_level          = 30.f;
constexpr float critical_low_battery_level = 20.f;