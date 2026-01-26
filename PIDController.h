#pragma once

#include <optional>
#include <utility>

class PIDController {
 public:
  struct Params {
    float kp;
    float ti;
    float td;
    std::optional<std::pair<float, float>> output_range;
  };

  PIDController(Params params);

  float Compute(float setpoint, float measured, float dt);
  void Reset() {
    last_error_ = 0.0f;
    integral_ = 0.0f;
  }

  // Accessors for the individual components of the computation
  // during the previous run.
  float p() const { return p_term_; }
  float i() const { return i_term_; }
  float d() const { return d_term_; }
  float error() const { return error_; }

 private:
  float kp_;
  float ki_;
  float kd_;
  std::optional<std::pair<float, float>> output_range_;

  float error_ = 0.0f;
  float p_term_ = 0.0f;
  float i_term_ = 0.0f;
  float d_term_ = 0.0f;

  float last_error_ = 0.0f;
  float integral_ = 0.0f;
};
