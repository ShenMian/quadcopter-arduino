#include <SPI.h>

#include "radio.h"
#include "indicator.h"
#include "motors.h"
#include "battery.h"

#include "estimator.hpp"
#include "controller.hpp"

Estimator  estimator;
Controller controller;

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
  Serial.begin(115200);
  while(!Serial);

  setup_nav_lights();
  setup_radio(100);
  setup_motors();

  indicator_state = {};

  // 等待倾角小于起飞时最大倾角
  do {
    estimator.update();
  } while(abs(estimator.get_angles().pitch) > max_takeoff_angle || abs(estimator.get_angles().pitch) > max_takeoff_angle);

  controller.set_target_angles({estimator.get_angles().yaw, 0, 0});
  controller.set_target_position({0, 0, 1});

  set_indicator_light(ErrorCode::Normal);

  estimator.set_takeoff_altitude(estimator.get_altitude());
}

void loop()
{
  check_battery_level();
  // update_radio(target_angles, target_position, throttle);
  estimator.update();

  update_indicator_light();

  // 调整到目标姿态
  if(abs(estimator.get_angles().pitch) > max_angle || abs(estimator.get_angles().roll) > max_angle)
  {
    // TODO: 倾角过大
  }

  controller.update(estimator);
}
