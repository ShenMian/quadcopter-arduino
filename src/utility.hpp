#pragma once

constexpr float clamp(float v, float min, float max)
{
	return v < min ? min : v > max ? max : v;
}

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
