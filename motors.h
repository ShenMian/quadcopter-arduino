#pragma once

#include "config.h"
#include "types.h"
#include "indicator.h"
#include <Servo.h>

extern Servo motors[4];

/**
 * @brief 初始化电机
 */
void setup_motors();

void update_motors(const EulerAngles& angles, float throttle);