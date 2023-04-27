#pragma once

template <typename T>
constexpr float clamp(T v, T min, T max)
{
	return v < min ? min : v > max ? max : v;
}

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
