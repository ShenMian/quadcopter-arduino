#include "estimator.hpp"

Estimator estimator;

void setup()
{
}

void loop()
{
	estimator.update(dt);

	const auto angles = estimator.get_angles();

	Serial.print("$");
	Serial.print(angles.yaw);
	Serial.print(" ");
	Serial.print(angles.pitch);
	Serial.print(" ");
	Serial.print(angles.roll);
	Serial.print(";");
}
