#include "driver/bmp280.hpp"
#include "driver/mpu6050.hpp"

MPU6050 mpu;
BMP280  bmp;

void setup()
{
	Serial.begin(115200);
	while(!Serial)
		delay(10);
}

/**
 * 输出加速度计, 陀螺仪, 气压计的读数到串口.
 * 用于检测传感器是否能正常工作.
 */
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