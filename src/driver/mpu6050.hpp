#pragma once

#include "../accelerometer.hpp"
#include "../gyroscope.hpp"
#include "../temperature.hpp"
#include <Adafruit_MPU6050.h>

/**
 * @brief MPU6050 6 轴运动处理组件
 */
class MPU6050 : public Accelerometer, public Gyroscope, public Temperature
{
public:
  MPU6050()
  {
    if(!mpu_.begin())
      abort();
  }
  
  Vector3 get_acceleration() const override
  {
    sensors_event_t event;
    mpu_.getEvent(&event, nullptr, nullptr);
    return {event.acceleration.x, event.acceleration.y, event.acceleration.z};
  }

  EulerAngles get_angular_velocity() const override
  {
    sensors_event_t event;
    mpu_.getEvent(nullptr, &event, nullptr);
    return {event.gyro.x, event.gyro.y, event.gyro.z};
  }
  
  float get_temperature() const override
  {
    sensors_event_t event;
    mpu_.getEvent(nullptr, nullptr, &event);
    return event.temperature;
  }

private:
  Adafruit_MPU6050 mpu_;
};