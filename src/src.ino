#include "pid.hpp"
#include "math.hpp"
#include "rotor.hpp"

#include "estimator.hpp"
#include "controller.hpp"

#include "sensor/battery.hpp"
#include "driver/dc_motor.hpp"

#include <Arduino.h>

enum Pin : uint8_t
{
  motor_front_right = 5,
  motor_front_left = 6,
  motor_back_right = 9,
  motor_back_left = 3,

  battery_ = A7,
};

DCMotor motors[4] = {Pin::motor_front_right, Pin::motor_front_left, Pin::motor_back_right, Pin::motor_back_left};

Rotor rotors[4] = {
  {&motors[0], -1.0, -0.70710678118654752440084436210485f, -0.70710678118654752440084436210485f},
  {&motors[1],  1.0, -0.70710678118654752440084436210485f,  0.70710678118654752440084436210485f},
  {&motors[2],  1.0,  0.70710678118654752440084436210485f, -0.70710678118654752440084436210485f},
  {&motors[3], -1.0,  0.70710678118654752440084436210485f,  0.70710678118654752440084436210485f},
};

Evaluator  evaluator;
Battery    battery(Pin::battery_);
Controller controller(rotors, 4);

void setup()
{
}

void loop()
{
  auto prev_timepoint = millis();
  while(true)
  {
    const auto curr_timepoint = millis();
    const float dt            = (curr_timepoint - prev_timepoint) * 1000.f;
    prev_timepoint            = curr_timepoint;

    evaluator.update(dt);
    controller.update(evaluator, dt);
  }
}
