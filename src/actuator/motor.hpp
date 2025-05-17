#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "../utility.hpp"

/**
 * @brief 电机
 */
class Motor {
  public:
    /**
	 * @brief 设置电机转速
	 *
	 * @param speed 电机转速, 范围: [0, 1]
	 */
    void set_speed(float speed) {
        if (!armed_)
            return;
        speed = clamp(speed, 0.f, 1.f);
        set_speed_impl(speed);
    };

    void armed(bool arm) {
        if (arm == false)
            set_speed_impl(0.f);
        armed_ = arm;
    }

    bool armed() const noexcept {
        return armed_;
    }

    virtual void attach(uint8_t pin) = 0;

  protected:
    virtual void set_speed_impl(float speed) = 0;

  private:
    bool armed_ = false;
};