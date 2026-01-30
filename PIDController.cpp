#include "PIDController.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string_view>

PIDController::PIDController(Params params)
    : kp_(params.kp),
      ki_(params.ti != 0 ? kp_ / params.ti : 0.0f),
      kd_(params.td * kp_),
      output_range_(params.output_range) {}

float PIDController::Compute(float setpoint, float measure, float dt) {
  const float derivative = (measure - last_measure_) / dt;
  last_measure_ = measure;
  return Compute(setpoint, measure, derivative, dt);
}

float PIDController::Compute(float setpoint, float measure,
                             float measure_derivative, float dt) {
  error_ = setpoint - measure;

  d_term_ = kd_ * -measure_derivative;
  const float integral_add = error_ * dt;
  integral_ += integral_add;
  i_term_ = ki_ * integral_;
  p_term_ = kp_ * error_;

  float result = i_term_ + p_term_ + d_term_;
  if (output_range_) {
    const float orig_result = result;
    result = std::clamp(result, output_range_->first, output_range_->second);
    const bool saturated = result != orig_result;

    if (saturated) {
      // Anti-windup: if we're saturated and integral_add was in the same
      // direction as the saturation, undo it.
      if ((integral_add > 0) == (result == output_range_->second))
        integral_ -= integral_add;
    }
  }
  return result;
}

void PIDController::DebugPrint(std::string_view name) const {
  printf(">%*s_P:%f\n>%*s_I:%f\n>%*s_D:%f\n>%*s_Error:%f\n", name.size(),
         name.data(), p_term_, name.size(), name.data(), i_term_, name.size(),
         name.data(), d_term_, name.size(), name.data(), error_);
}
