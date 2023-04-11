#pragma once

#include "config.h"
#include "math.h"
#include <RF24.h>
#include <nRF24L01.h>

/**
 * @brief 无线电数据包
 */
struct RadioPackage
{
  EulerAngles target_angles;
  Vector3     target_position;
  float       throttle;
};

extern RF24 radio;

/**
 * @brief 初始化无线电
 */
void setup_radio(uint8_t channel);

void update_radio(EulerAngles& target_angles, Vector3& target_position, float& throttle);