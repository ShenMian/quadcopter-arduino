#pragma once

#include "indicator.h"

/**
 * @brief 获取电池电量, 范围 [0-100]
 */
float get_battery_level();

void check_battery_level();