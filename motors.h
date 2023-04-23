#pragma once

#include "config.h"
#include "math.h"
#include <Servo.h>

extern Servo motors[4];

/**
 * @brief 初始化电机
 */
void setup_motors();

void update_motors(EulerAngles angles, float throttle);