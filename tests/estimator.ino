#include "estimator.hpp"

void setup()
{
	Serial.begin(115200);
	while(!Serial)
		delay(10);
}

/**
 * 输出估测的机体姿态到串口, 可以结合 [serial_port_plotter] 查看可视化结果.
 * 用于检测评估器是否能正常工作, 检测传感器请使用测试用例 `sensors_readings`.
 * [serial_port_plotter]: https://github.com/CieNTi/serial_port_plotter
 */
void loop()
{
	Estimator estimator;

	auto prev_timepoint = millis();
	while(true)
	{
		const auto  curr_timepoint = millis();
		const float dt             = (curr_timepoint - prev_timepoint) / 1000.f;

		estimator.update(dt);

		const auto angles = estimator.get_angles();
#if 0
		// Serial.print("yaw:");
		// Serial.println(angles.pitch);
		Serial.print("pitch:");
		Serial.println(angles.pitch);
		Serial.print("roll:");
		Serial.println(angles.roll);
#else
		Serial.print("$");
		// Serial.print(angles.yaw);
		// Serial.print(" ");
		Serial.print(angles.pitch);
		Serial.print(" ");
		Serial.print(angles.roll);
		Serial.println(";");
#endif

		prev_timepoint = curr_timepoint;
	}
}
