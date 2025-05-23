#pragma once

#include <Arduino.h>

enum Pin : uint8_t {
    motor_front_right = 5,
    motor_front_left = 6,
    motor_back_right = 9,
    motor_back_left = 3,

    radio_ce = 7,
    radio_csn = 8,

    battery_ = A7,
};
