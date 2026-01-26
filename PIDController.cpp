#include "PIDController.h"

#include <algorithm>
#include <cmath>

PIDController::PIDController(Params params)
    : kp_(params.kp),
      ki_(params.ti != 0 ? 1.0f / params.ti : 0.0f),
      kd_(params.td),
      output_range_(params.output_range) {}

float PIDController::Compute(float setpoint, float measured, float dt) {
  error_ = setpoint - measured;
  d_term_ = kd_ * (error_ - last_error_) / dt;
  const float integral_add = error_ * dt;
  integral_ += integral_add;
  i_term_ = ki_ * integral_;
  p_term_ = kp_ * error_;

  last_error_ = error_;
  float result = i_term_ + p_term_ + d_term_;
  if (output_range_) {
    result = std::clamp(result, output_range_->first, output_range_->second);
    if ((result == output_range_->first && integral_ < 0) ||
        (result == output_range_->second && integral_ > 0)) {
      // Undo our addition to the integral term to prevent windup.
      integral_ -= integral_add;
    }
  }
  return result;
}
