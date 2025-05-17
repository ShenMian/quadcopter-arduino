#include <Arduino.h>

#include "driver/dc_motor.hpp"
#include "utility.hpp"

enum Pin : uint8_t {
    motor_front_right = 5,
    motor_front_left = 6,
    motor_back_right = 9,
    motor_back_left = 3,
};

DCMotor motors[4] = {
    Pin::motor_front_right,
    Pin::motor_front_left,
    Pin::motor_back_right,
    Pin::motor_back_left
};

void setup() {
    Serial.begin(115200);
    while (!Serial)
        delay(10);
}

/**
 * 逐个电机以 20% 的转速转动 100ms.
 * 用于检测电机是否能正常工作.
 */
void loop() {
    delay(5000);
    for (uint8_t i = 0; i < ARRAY_SIZE(motors); i++) {
        motors[i].arm();
        motors[i].set_speed(0.2f);
        delay(100);
        motors[i].disarm();
        delay(1000);
    }
    terminate();
}
