#include "estimator.hpp"

Estimator estimator;

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
