#pragma once

#include <Arduino.h>

/**
 * @brief 三维向量
 */
union Vector3
{
  struct
  {
    float x;
    float y;
    float z;
  };
  float v[3];

  float norm() const noexcept
  {
    return sqrt(x * x + y * y + z * z);
  }

  Vector3 operator*(float scale) const noexcept
  {
    return Vector3(*this) *= scale;
  }

  Vector3 operator*=(float scale) noexcept
  {
    x *= scale, y *= scale, z *= scale;
    return *this;
  }

  Vector3& operator+=(const Vector3& rhs) noexcept
  {
    x += rhs.x, y += rhs.y, z += rhs.z;
    return *this;
  }
};

/**
 * @brief 欧拉角
 */
union EulerAngles
{
  struct
  {
    float yaw;
    float pitch;
    float roll;
  };
  float v[3];
};