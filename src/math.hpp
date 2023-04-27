#pragma once

#include <Arduino.h>

constexpr float pi = 3.1415926535897932384626433832795;

/**
 * @brief 角度制转弧度制
 */
constexpr float deg_to_rad(float deg)
{
	return deg * pi / 180.f;
}

/**
 * @brief 弧度制转角度制
 */
constexpr float rad_to_deg(float rad)
{
	return rad * 180.f / pi;
}

/**
 * @brief 三维向量
 */
union Vector3 {
	struct
	{
		float x;
		float y;
		float z;
	};
	float v[3];

	float norm() const noexcept { return sqrt(norm_sq()); }

	float norm_sq() const noexcept { return x * x + y * y + z * z; }

	Vector3 operator+(const Vector3& rhs) const noexcept { return Vector3(*this) += rhs; }

	Vector3 operator-(const Vector3& rhs) const noexcept { return Vector3(*this) -= rhs; }

	Vector3 operator*(float scale) const noexcept { return Vector3(*this) *= scale; }

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

	Vector3& operator-=(const Vector3& rhs) noexcept
	{
		x -= rhs.x, y -= rhs.y, z -= rhs.z;
		return *this;
	}
};

/**
 * @brief 欧拉角
 */
union EulerAngles {
	struct
	{
		float yaw;
		float pitch;
		float roll;
	};
	float v[3];

	EulerAngles operator-(const EulerAngles& rhs) const noexcept { return EulerAngles(*this) -= rhs; }

	EulerAngles& operator-=(const EulerAngles& rhs) noexcept
	{
		yaw -= rhs.yaw, pitch -= rhs.pitch, roll -= rhs.roll;
		return *this;
	}
};

/**
 * @brief 四元数
 */
struct Quaternion
{
	float x;
	float y;
	float z;
	float w;
};

/**
 * @brief 欧拉角转四元数
 */
inline Quaternion ToQuaternion(const EulerAngles& angles)
{
	const float cr = cos(angles.roll * 0.5);
	const float sr = sin(angles.roll * 0.5);
	const float cp = cos(angles.pitch * 0.5);
	const float sp = sin(angles.pitch * 0.5);
	const float cy = cos(angles.yaw * 0.5);
	const float sy = sin(angles.yaw * 0.5);

	Quaternion quat;
	quat.w = cr * cp * cy + sr * sp * sy;
	quat.x = sr * cp * cy - cr * sp * sy;
	quat.y = cr * sp * cy + sr * cp * sy;
	quat.z = cr * cp * sy - sr * sp * cy;

	return quat;
}

/**
 * @brief 四元数转欧拉角
 */
inline EulerAngles ToEulerAngles(const Quaternion& quat)
{
	EulerAngles angles;

	// roll (x-axis rotation)
	const float sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z);
	const float cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y);
	angles.roll           = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	const float sinp = sqrt(1 + 2 * (quat.w * quat.y - quat.x * quat.z));
	const float cosp = sqrt(1 - 2 * (quat.w * quat.y - quat.x * quat.z));
	angles.pitch     = 2 * atan2(sinp, cosp) - pi / 2;

	// yaw (z-axis rotation)
	const float siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
	const float cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
	angles.yaw            = atan2(siny_cosp, cosy_cosp);

	return angles;
}