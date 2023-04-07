#pragma once

#include "indicator.h"
#include <Arduino.h>

/**
 * @brief 获取电池电量, 范围 [0-100]
 */
float get_battery_level(uint8_t pin);

/**
 * @brief 检查电池电量, 并在电量低时设置指示灯
 */
void check_battery_level(uint8_t pin);