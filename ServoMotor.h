#pragma once

#include <Arduino.h>
#include <memory>
#include "portmacro.h"
#include <driver/pulse_cnt.h>
#include <freertos/FreeRTOS.h>
#include <jesl.h>
#include <jesl/freertos/mutex.h>
#include "PIDController.h"

class ServoMotor {
 public:
  struct Config {
    int pulse_pin;
    int control_pin;
    int pwm_pin;
    int fwd_pin;
    int rev_pin;
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

  float GetSpeed() {
    portENTER_CRITICAL(&control_mutex_);
    float s = speed_;
    portEXIT_CRITICAL(&control_mutex_);
    return s;
  }

 private:
  ServoMotor(pcnt_unit_handle_t unit, pcnt_channel_handle_t chan_a,
             pcnt_channel_handle_t chan_b, int pwm_pin, int fwd_pin,
             int rev_pin);

  void Update();
  void OutputDuty(int duty);

  const pcnt_unit_handle_t unit_;
  const pcnt_channel_handle_t chan_a_;
  const pcnt_channel_handle_t chan_b_;
  const int pwm_pin_;
  const int fwd_pin_;
  const int rev_pin_;

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
  portMUX_TYPE control_mutex_ = portMUX_INITIALIZER_UNLOCKED;

  float speed_;  // deg/s

  TaskHandle_t task_;
  
  PIDController pid_;
};
