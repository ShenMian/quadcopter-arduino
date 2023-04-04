#include <SPI.h>
#include <JY901.h>
#include <limits.h>

#include "config.h"
#include "indicator.h"
#include "pid.h"
#include "battery.h"
#include "radio.h"
#include "types.h"
#include "motors.h"

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

// ZYX 欧拉角, 弧度制
EulerAngles target_angles = {}; // 目标姿态

// 以起飞点为坐标系原点
Vector3 target_position = {}; // 目标坐标
Vector3 actual_position = {}; // 当前坐标
Vector3 actual_velocity = {}; // 当前速度

float throttle = 0.0f;  // 节流阀
long  takeoff_altitude; // 起飞时海拔, 用于计算相对海拔

enum class Status
{
  Ready,       ///< 准备起飞
  FixedHeight, ///< 定高
  FixedPoint,  ///< 定点
};

Status status;

void print_actual_angles()
{
  const EulerAngles actual_angles = get_actual_angles();
  Serial.print(F("Actual angles: "));
  for (int i = 0; i < 3; i++) {
    Serial.print(actual_angles.v[i] * rad_to_deg);
    Serial.print(F(", "));
  }
  Serial.print(F("\n"));
}

void print_actual_position()
{
  Serial.print(F("Actual position: "));
  for (int i = 0; i < 3; i++) {
    Serial.print(actual_position.v[i]);
    Serial.print(F(", "));
  }
  Serial.print(F("\n"));

  Serial.print(abs(get_actual_altitude() - actual_position.z));
  Serial.print(F("\n"));
}

void setup_nav_lights()
{
  pinMode(NavLight_FR, OUTPUT);
  pinMode(NavLight_FL, OUTPUT);
  pinMode(NavLight_BR, OUTPUT);
  pinMode(NavLight_BL, OUTPUT);
}

/**
 * @brief 更新 IMU 数据
 */
void update_jy901()
{
  while (Serial.available()) {
    JY901.CopeSerialData(Serial.read()); // Call JY901 data cope function
  }
}

void update_actual_position_and_velocity()
{
  // TODO: 加速度计在震动环境下误差较大, 结合加速度计和陀螺仪的数据来获得更加准确的姿态
  actual_velocity.x = JY901.stcAcc.a[1];
  actual_velocity.y = -JY901.stcAcc.a[0];
  actual_velocity.z = JY901.stcAcc.a[2];
  for(int i = 0; i < 3; i++)
    actual_position.v[i] += actual_velocity.v[i] * dt;
}

/**
 * @brief 获取实际姿态角
 *
 * MPU 6050 坐标系:
 *
 *   z
 * y ↑
 *  \|
 *   +----→ x
 */
EulerAngles get_actual_angles()
{
  return { (float)JY901.stcAngle.Angle[2] / (SHRT_MAX + 1),   // yaw
           (float)JY901.stcAngle.Angle[0] / (SHRT_MAX + 1),   // pitch
           (float)JY901.stcAngle.Angle[1] / (SHRT_MAX + 1) }; // roll
}

/**
 * @brief 获取气压计测量海拔
 *
 * 精度: ±1m
 */
long get_actual_altitude()
{
  return JY901.stcPress.lAltitude;
}

void setup()
{
  Serial.begin(115200);
  while(!Serial);

  setup_nav_lights();
  setup_radio(100);
  setup_motors();

  set_nav_lights(LOW);
  indicator_state = {};

  EulerAngles actual_angles;
  // 等待倾角小于起飞时最大倾角
  do {
    update_jy901();
    actual_angles = get_actual_angles();
  } while(abs(actual_angles.pitch) > max_takeoff_angle || abs(actual_angles.pitch) > max_takeoff_angle);

  target_angles.yaw   = actual_angles.yaw;
  target_angles.pitch = 0;
  target_angles.roll  = 0;

  target_position.x = 0;
  target_position.y = 0;
  target_position.z = 1;

  set_indicator_light(ErrorCode::Normal);

  status = Status::Ready;
  while(status == Status::Ready)
  {
    check_battery_level();
    update_indicator_light();
    update_radio(target_angles, target_position, throttle);
    if(throttle > 0.f)
      status = Status::FixedHeight;
  }

  takeoff_altitude = get_actual_altitude();
}

void loop()
{
  check_battery_level();
  update_radio(target_angles, target_position, throttle);
  update_jy901();
  update_actual_position_and_velocity();

  update_indicator_light();

  // 调整到目标姿态
  const EulerAngles actual_angles = get_actual_angles(); // 实际姿态, 弧度制
  if(abs(actual_angles.pitch) > max_angle || abs(actual_angles.roll) > max_angle)
  {
    // TODO: 倾角过大
  }

  EulerAngles actuator_angles; // 姿态控制量
  for(int i = 0; i < 3; i++)
    actuator_angles.v[i] = pid(target_angles.v[i], actual_angles.v[i], dt, angle_states[i]);
    
  Vector3 actuator_position;

  if(status == Status::FixedHeight || status == Status::FixedPoint)
  {
    // 调整到指定高度(Z)
    actual_position.z   = get_actual_altitude() - takeoff_altitude;
    actuator_position.z = pid(target_position.z, actual_position.z, dt, position_states[2]);
    throttle += actuator_position.z > 0.f ? 0.1f : -0.1f;
  }

  if(status == Status::FixedPoint)
  {
    // 调整姿态来到达指定 X, Y
    actuator_position.x  = pid(target_position.x, actual_position.x, dt, position_states[0]);
    actuator_position.y  = pid(target_position.y, actual_position.y, dt, position_states[1]);
    target_angles.pitch -= actuator_position.x;
    target_angles.roll  -= actuator_position.y;

    // 限定目标角度最大倾角
    target_angles.pitch = constrain(target_angles.pitch, -max_angle, max_angle);
    target_angles.roll  = constrain(target_angles.roll, -max_angle, max_angle);
  }

  // TODO: 感觉需要乘以一个系数, 具体多少不太清楚
  const float factor = 3.f;
  for(int i = 0; i < 3; i++)
    actuator_angles.v[i] *= factor;

  update_motors(actuator_angles, throttle);
}
