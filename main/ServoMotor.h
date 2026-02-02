#pragma once

#include <Arduino.h>
#include <driver/pulse_cnt.h>
#include <jesl.h>
#include <jesl/freertos/mutex.h>

#include <memory>

#include "PIDController.h"
#include "freertos/portmacro.h"

class ServoMotor {
 public:
  struct Config {
    int pulse_pin;
    int control_pin;
    int pwm_pin;
    int fwd_pin;
    int rev_pin;

    // DC motors often have a deadband where they won't move at all if commanded
    // to move at a duty cycle in that deadband. This variable sets the motor
    // to automatically compensate for the deadband. When a duty not equal to
    // zero is commanded, the motor will run at a minimum of the deadband duty
    // value. The actual duty cycle will be scaled in between the deadband value
    // and the maximum duty cycle.
    int deadband = 0;
  };

  static std::unique_ptr<ServoMotor> Create(const Config& config);

  ~ServoMotor();

  void SetDuty(int duty);
  void SetSpeed(int deg_per_s);
  void SetPosition(int deg);
  void Brake();

  void DeepSleepPrepare();
  void DeepSleepResume();

  int64_t GetCount();

  float GetSpeed() const {
    portENTER_CRITICAL(&control_mutex_);
    float s = speed_;
    portEXIT_CRITICAL(&control_mutex_);
    return s;
  }

  void set_deadband(int deadband) { deadband_ = deadband; }

 private:
  ServoMotor(pcnt_unit_handle_t unit, pcnt_channel_handle_t chan_a,
             pcnt_channel_handle_t chan_b, int pwm_pin, int fwd_pin,
             int rev_pin, int deadband);

  void Update();
  void OutputDuty(int duty);

  const pcnt_unit_handle_t unit_;
  const pcnt_channel_handle_t chan_a_;
  const pcnt_channel_handle_t chan_b_;
  const int pwm_pin_;
  const int fwd_pin_;
  const int rev_pin_;
  int deadband_;

  // If the control setpoint is this value during duty-based control, we'll
  // brake instead of setting the duty.
  static constexpr int kBrakeDuty = -300;  

  enum class ControlMode {
    kDuty, kSpeed, kPosition
  };
  struct Control {
    int setpoint = 0;
    uint8_t seq = 0;  // Used by the control loop to detect changes.
    ControlMode mode = ControlMode::kDuty;
  };
  Control control_;
  mutable portMUX_TYPE control_mutex_ = portMUX_INITIALIZER_UNLOCKED;

  float speed_;  // deg/s

  TaskHandle_t task_;
  
  PIDController pid_;
};
