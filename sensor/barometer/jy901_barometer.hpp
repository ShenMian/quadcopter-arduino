#pragma once

#include "../barometer.hpp"
#include <JY901.h>

/**
 * @brief JY901 上的 BMP280
 *
 * 精度: ±1m
 */
class JY901_Barometer : public Barometer
{
public:
  long get_altitude() const override { return JY901.stcPress.lAltitude; }
};