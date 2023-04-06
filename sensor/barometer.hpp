#pragma once

class Barometer
{
public:
  /**
  * @brief 获取海拔
  */
  virtual long get_altitude() const = 0;
};