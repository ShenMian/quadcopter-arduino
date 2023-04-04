#include "pid.h"

float pid(float target, float actual, float dt, PIDState& state)
{
  // TODO: 可能要对积分项进行一个阈值的限定, 因为不能无限累积(可能会累积过度)

  const float error = target - actual;
  state.prev_error  = error;

  const float p   = state.kP * error;
  state.integral += state.kI * error * dt;
  const float d   = state.kD * (error - state.prev_error) / dt;

  return p + state.integral + d;
}