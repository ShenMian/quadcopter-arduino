#include "math.hpp"
#include "pid.hpp"
#include "rotor.hpp"
#include "config.hpp"
#include "radio.hpp"

#include "controller.hpp"
#include "estimator.hpp"

#include "driver/dc_motor.hpp"
#include "sensor/battery.hpp"

#include <Arduino.h>

DCMotor motors[4] = {Pin::motor_front_right, Pin::motor_front_left, Pin::motor_back_right, Pin::motor_back_left};

// constexpr float scale = 0.70710678118654752440084436210485f; ///< cos(45°)
constexpr float scale     = 1.f;
Rotor           rotors[4] = {
    {motors[0], -1.0, -scale, -scale},
    {motors[1],  1.0, -scale,  scale},
    {motors[2],  1.0,  scale, -scale},
    {motors[3], -1.0,  scale,  scale},
};

Estimator  estimator;
Battery    battery(Pin::battery_, 3.2f, 4.2f, 10, 10);
Controller controller(rotors, 4);
Radio      radio(Pin::radio_ce, Pin::radio_csn, "00001", "00002");

// TODO: 对 float 类型的变量进行标准化
struct Package
{
	EulerAngles target_angles;
	float       target_throttle;
};

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

		Package package;
		if(radio.read(package))
		{
			controller.set_target_angles(package.target_angles);
			controller.set_target_throttle(package.target_throttle);
		}

		estimator.update(dt);
		controller.update(estimator, dt);

		prev_timepoint = curr_timepoint;
	}
}
