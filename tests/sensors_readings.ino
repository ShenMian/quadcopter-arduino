#include "driver/bmp280.hpp"
#include "driver/mpu6050.hpp"

MPU6050 mpu;
BMP280  bmp;

void setup()
{
}

void loop()
{
	Accelerometer& acc  = mpu;
	Gyroscope&     gyro = mpu;
	Barometer&     baro = bmp;

	Serial.print("acc : ");
	Serial.print(acc.get_acceleration().x);
	Serial.print(", ");
	Serial.print(acc.get_acceleration().y);
	Serial.print(", ");
	Serial.print(acc.get_acceleration().z);
	Serial.println(" m/s^2");

	Serial.print("gyro: ");
	Serial.print(gyro.get_angular_velocity().yaw);
	Serial.print(", ");
	Serial.print(gyro.get_angular_velocity().pitch);
	Serial.print(", ");
	Serial.print(gyro.get_angular_velocity().roll);
	Serial.println(" rad/s");

	Serial.print("baro: ");
	Serial.print(baro.get_altitude());
	Serial.println(" m");
}