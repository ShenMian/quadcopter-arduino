#include "math.hpp"
#include "pid.hpp"
#include "rotor.hpp"

#include "controller.hpp"
#include "estimator.hpp"

#include "driver/dc_motor.hpp"
#include "sensor/battery.hpp"

#include <Arduino.h>

enum Pin : uint8_t
{
	motor_front_right = 5,
	motor_front_left  = 6,
	motor_back_right  = 9,
	motor_back_left   = 3,

	battery_ = A7,
};

DCMotor motors[4] = {Pin::motor_front_right, Pin::motor_front_left, Pin::motor_back_right, Pin::motor_back_left};

// constexpr float scale = 0.70710678118654752440084436210485f; ///< cos(45°)
constexpr float scale     = 1.f;
Rotor           rotors[4] = {
    {&motors[0], -1.0, -scale, -scale},
    {&motors[1],  1.0, -scale,  scale},
    {&motors[2],  1.0,  scale, -scale},
    {&motors[3], -1.0,  scale,  scale},
};

Estimator  estimator;
Battery    battery(Pin::battery_, 3.2f, 4.2f, 10, 10);
Controller controller(rotors, 4);

void setup()
{
}

void loop()
{
	auto prev_timepoint = millis();
	while(true)
	{
		const auto  curr_timepoint = millis();
		const float dt             = (curr_timepoint - prev_timepoint) * 1000.f;
		prev_timepoint             = curr_timepoint;

		estimator.update(dt);
		controller.update(estimator, dt);
	}
}
