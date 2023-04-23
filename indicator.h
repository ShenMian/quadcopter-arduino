#pragma once

#include <stdint.h>
#include "config.h"

/**
 * @brief 错误代码
 */
enum class ErrorCode
{
  Normal,             ///< 正常
  SelfTestFailed,     ///< 设备错误
  RadioDisconnected,  ///< 无线电信号中断
  NoGPS,              ///< 无 GPS 信号
  LowBattery,         ///< 低电量
  CriticalLowBattery, ///< 严重低电量
  Unknown,            ///< 未知错误
};

/**
 * @brief 指示器状态
 */
struct IndicatorState
{
  uint8_t  timer;
  uint16_t interval; ///< 闪烁间隔
  int8_t   count;    ///< 闪烁次数
};

extern IndicatorState indicator_state;

void setup_nav_lights();
void set_nav_lights(uint8_t value);
void update_indicator_light();
void set_indicator_light(ErrorCode code);

void stop_with_error(ErrorCode code);