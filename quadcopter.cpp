#include <SPI.h>
#include <stdint.h>

#include "radio.h"
#include "indicator.h"
#include "motors.h"
#include "battery.h"

#include "estimator.hpp"
#include "controller.hpp"

Estimator  estimator;
Controller controller;

#define MAX_WAYPOINT_HISTORIES 15

Vector3 waypoint_histories[MAX_WAYPOINT_HISTORIES]; ///< 最短历史航点路径, 用于自动返航

constexpr float waypoint_distance = 3.f; ///< 航点之间的最小距离, 超过距离才会记录航点

void record_waypoint_histories()
{
  static uint8_t index = 0;
  if(index == MAX_WAYPOINT_HISTORIES)
    return; // 无法继续记录航点

  const auto position = estimator.get_position();
  if((waypoint_histories[index] - position).norm() <= waypoint_distance)
    return;
  
  for(int16_t i = index - 1; i > 0; i--)
  {
    if((waypoint_histories[i] - position).norm() <= waypoint_distance)
    {
      index = i;
      return;
    }
  }

  waypoint_histories[index++] = position;
  Serial.println("Add new waypoint");
}

void print_actual_angles()
{
  Serial.print(F("Actual angles: "));
  for (int i = 0; i < 3; i++) {
    Serial.print(rad_to_deg(estimator.get_angles().v[i]));
    Serial.print(F(", "));
  }
  Serial.print(F("\n"));
}

void print_actual_position()
{
  Serial.print(F("Actual position: "));
  for (int i = 0; i < 3; i++) {
    Serial.print(estimator.get_position().v[i]);
    Serial.print(F(", "));
  }
  Serial.print(F("\n"));

  // Serial.print(abs(estimator.get_altitude() - estimator.get_position().z));
  // Serial.print(F("\n"));
}

void setup()
{
  Serial.begin(115200); // JY901 Serial 通讯频率
  while(!Serial);

  // setup_nav_lights();
  // setup_radio(100);
  setup_motors();

  indicator_state = {};

  // 等待倾角小于起飞时最大倾角
  /*
  do {
    estimator.update();
  } while(abs(estimator.get_angles().pitch) > max_takeoff_angle || abs(estimator.get_angles().pitch) > max_takeoff_angle);
  */

  controller.set_target_angles({estimator.get_angles().yaw, 0, 0});
  controller.set_target_position({0, 0, 1});

  // set_indicator_light(ErrorCode::Normal);

  estimator.takeoff();
}

void loop()
{
  // check_battery_level(Battery);
  estimator.update();
  record_waypoint_histories();
  // update_indicator_light();

  // 调整到目标姿态
  if(abs(estimator.get_angles().pitch) > max_angle || abs(estimator.get_angles().roll) > max_angle)
  {
    // TODO: 倾角过大
  }
  
  /*
  float       throttle;
  EulerAngles target_angles;
  Vector3     target_position;
  update_radio(target_angles, target_position, throttle);
  controller.set_target_angles(target_angles);
  controller.set_target_position(target_position);
  controller.set_throttle(throttle);
  */

  controller.update(estimator);
}

/**
 * @brief 更新 JY901 数据
 */
void serialEvent()
{
  // TODO: 新板子将使用 IIC 与 JY901 通讯
  while (Serial.available()) {
    JY901.CopeSerialData(Serial.read()); // Call JY901 data cope function
  }
}
