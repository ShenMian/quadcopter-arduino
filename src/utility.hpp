#pragma once

template <typename T>
constexpr float clamp(T v, T min, T max)
{
	return v < min ? min : v > max ? max : v;
}

void terminate()
{
	while(1)
	{
		delay(10);
	}
}

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
