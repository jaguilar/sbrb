#pragma once

#include <optional>
#include <string_view>
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

  // This overload allows passing the change in the measured value per unit
  // time, which can be useful if the sensors report it directly.
  float Compute(float setpoint, float measured, float measure_derivative,
                float dt);

  void Reset() { integral_ = 0.0f; }

  // Accessors for the individual components of the computation
  // during the previous run.
  float p() const { return p_term_; }
  float i() const { return i_term_; }
  float d() const { return d_term_; }
  float error() const { return error_; }

  void set_kp(float kp) { kp_ = kp; }
  void set_ti(float ti) { ki_ = ti != 0 ? kp_ / ti : 0; }
  void set_td(float td) { kd_ = kp_ * td; }

  float kp() const { return kp_; }
  float ti() const { return ki_ != 0 ? kp_ / ki_ : 0; }
  float td() const { return kd_ / kp_; }

  void DebugPrint(std::string_view name) const;

 private:
  float kp_;
  float ki_;
  float kd_;
  std::optional<std::pair<float, float>> output_range_;

  float error_ = 0.0f;
  float p_term_ = 0.0f;
  float i_term_ = 0.0f;
  float d_term_ = 0.0f;

  float last_measure_ = 0.0f;
  float integral_ = 0.0f;
};
