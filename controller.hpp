#pragma once

#include <stdint.h>
#include "types.h"
#include "pid.h"
#include "estimator.hpp"

PIDState angle_states[3] = {
  {4.0, 0.02, 0},  // yaw
  {1.3, 0.04, 18}, // pitch
  {1.3, 0.04, 18}, // roll
};

PIDState position_states[3] = {
  {0.1, 0.1, 20}, // x
  {0.1, 0.1, 20}, // y
  {0.1, 0.1, 20}, // z
};

/**
 * @brief 控制模式
 */
enum Mode : uint8_t
{
  Manual = 0, ///< 纯手动, 无自动控制

  Yaw_Hold   = 1 << 0,
  Pitch_Hold = 1 << 1,
  Roll_Hold  = 1 << 2,

  Z_Hold = 1 << 3,
  X_Hold = 1 << 4,
  Y_Hold = 1 << 5,

  Altitude_Hold = Z_Hold,                   ///< 定高
  Position_Hold = X_Hold | Y_Hold | Z_Hold, ///< 定点
};

/**
 * @brief 位置和姿态控制器
 *
 * 将机体调整到目标位置和姿态.
 */
class Controller
{
public:
  void update(const Estimator& estimator)
  {
    EulerAngles actuator_angles; // 姿态控制量
    if(mode & Mode::Yaw_Hold)
      actuator_angles.yaw = pid(target_angles.yaw, estimator.get_angles().yaw, dt, angle_states[0]);
      
    if(mode & Mode::Pitch_Hold)
      actuator_angles.pitch = pid(target_angles.pitch, estimator.get_angles().pitch, dt, angle_states[1]);
      
    if(mode & Mode::Roll_Hold)
      actuator_angles.roll = pid(target_angles.roll, estimator.get_angles().roll, dt, angle_states[2]);
      
    if(mode & Mode::X_Hold)
      target_angles.pitch -= pid(target_position.x, estimator.get_position().x, dt, position_states[0]);
    
    if(mode & Mode::Y_Hold)
      target_angles.roll -= pid(target_position.y, estimator.get_position().y, dt, position_states[1]);

    if(mode & Mode::Z_Hold)
      throttle += pid(target_position.z, estimator.get_position().z, dt, position_states[2]) > 0.f ? 0.1f : -0.1f;

    // 限定目标角度最大倾角
    target_angles.pitch = constrain(target_angles.pitch, -max_angle, max_angle);
    target_angles.roll  = constrain(target_angles.roll, -max_angle, max_angle);

    update_motors(actuator_angles, throttle);
  }

  /**
  * @brief 设置目标姿态角
  */
  void set_target_angles(const EulerAngles& angles) noexcept { target_angles = angles; }
  
  /**
  * @brief 设置目标位置
  */
  void set_target_position(const Vector3& position) noexcept { target_position = position; }

  /**
  * @brief 获取目标姿态角
  */
  EulerAngles get_target_angles() const noexcept { return target_angles; }

  /**
  * @brief 获取目标位置
  */
  Vector3 get_target_position() const noexcept { return target_position; }

private:
  Vector3     target_position = {};  // 目标坐标
  EulerAngles target_angles   = {};  // 目标姿态角

  float throttle = 0.0f;  // 节流阀

  Mode mode = Mode::Altitude_Hold;
};