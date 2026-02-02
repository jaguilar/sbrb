#include "ServoMotor.h"

#include <Arduino.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <utility>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_check.h"
#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "portmacro.h"
#include "freertos/semphr.h"
#include "soc/gpio_num.h"

// Define LOG_TAG if not already defined for ESP logging macros
#ifndef LOG_TAG
#define LOG_TAG "ServoMotor"
#endif

ServoMotor::ServoMotor(pcnt_unit_handle_t unit, pcnt_channel_handle_t chan_a,
                       pcnt_channel_handle_t chan_b, int pwm_pin, int fwd_pin,
                       int rev_pin, int deadband)
    : unit_(unit),
      chan_a_(chan_a),
      chan_b_(chan_b),
      pwm_pin_(pwm_pin),
      fwd_pin_(fwd_pin),
      rev_pin_(rev_pin),
      deadband_(deadband),
      pid_(PIDController::Params{
          .kp = .5f,
          .ti = 0.05f,
          .td = 0.01f,
          .output_range = std::make_pair(-255.0f, 255.0f)}) {}

ServoMotor::~ServoMotor() {
  // Clean up PCNT resources if needed.
  // Note: PCNT resource lifecycle management might be needed here depending on
  // if we want to support destruction. For now, assuming static lifetime or
  // user handles it. Ideally: disable, del_channel, del_unit. But since this is
  // a refactor of static setup, simple is fine.
}

std::unique_ptr<ServoMotor> ServoMotor::Create(const Config& config) {
  pcnt_unit_handle_t pcnt_unit = NULL;
  pcnt_unit_config_t cfg = {
      .low_limit = -32768,
      .high_limit = 32767,
      .flags =
          {
              .accum_count = true,
          },
  };
  // Using ESP_ERROR_CHECK usually aborts. For a factory, maybe we should return
  // nullptr on failure? But original code used ESP_ERROR_CHECK, so we stick to
  // that or let it crash on failure. To allow graceful failure, we could check
  // return codes. Sticking to original behavior:
  ESP_ERROR_CHECK(pcnt_new_unit(&cfg, &pcnt_unit));

  pcnt_glitch_filter_config_t filter_cfg = {
      .max_glitch_ns = 1000,
  };
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_cfg));

  pcnt_chan_config_t chan_a_cfg = {
      .edge_gpio_num = config.pulse_pin,
      .level_gpio_num = config.control_pin,
  };
  pcnt_channel_handle_t chan_a;
  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_cfg, &chan_a));

  pcnt_chan_config_t chan_b_cfg = {
      .edge_gpio_num = config.control_pin,
      .level_gpio_num = config.pulse_pin,
  };
  pcnt_channel_handle_t chan_b;
  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_cfg, &chan_b));

  ESP_ERROR_CHECK(
      pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(
      pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  ESP_ERROR_CHECK(
      pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(
      pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, cfg.high_limit));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, cfg.low_limit));

  ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

  // Initialize Motor Pins
  pinMode(config.pwm_pin, OUTPUT);
  pinMode(config.rev_pin, OUTPUT);
  pinMode(config.fwd_pin, OUTPUT);
  digitalWrite(config.rev_pin, LOW);
  digitalWrite(config.fwd_pin, LOW);
  analogWriteFrequency(config.pwm_pin, 20000);
  analogWriteResolution(config.pwm_pin, 8);
  analogWrite(config.pwm_pin, 0);

  auto servo_motor =
      new ServoMotor(pcnt_unit, chan_a, chan_b, config.pwm_pin, config.fwd_pin,
                     config.rev_pin, config.deadband);

  xTaskCreate(
      +[](void* arg) { reinterpret_cast<ServoMotor*>(arg)->Update(); },
      "ServoMotor", 4098, servo_motor, 0, &servo_motor->task_);

  return std::unique_ptr<ServoMotor>(servo_motor);
}

void ServoMotor::DeepSleepPrepare() {
  vTaskSuspend(task_);
  OutputDuty(0);
  gpio_hold_en(static_cast<gpio_num_t>(fwd_pin_));
  gpio_hold_en(static_cast<gpio_num_t>(rev_pin_));
}

void ServoMotor::DeepSleepResume() {
  // Note that when we resume from deep sleep we do not need to resume
  // the task_ or set the duty. Those values are lost to us since the
  // memory went away. But what we do need to do is disable the holds
  // on these GPIOs.
  gpio_hold_dis(static_cast<gpio_num_t>(fwd_pin_));
  gpio_hold_dis(static_cast<gpio_num_t>(rev_pin_));
}

void ServoMotor::Update() {
  constexpr TickType_t update_freq = pdMS_TO_TICKS(2);
  TickType_t last_wake_time = xTaskGetTickCount();

  int prev_pulse = 0;
  ESP_ERROR_CHECK(pcnt_unit_get_count(this->unit_, &prev_pulse));
  uint64_t last_micros = micros();

  float prev_speed = 0;
  const float alpha = 0.1;

  Control control, new_control;
  int seq = 0;
  uint64_t last_log_micros = micros();
  while (true) {
    if (!xTaskDelayUntil(&last_wake_time, update_freq)) {
      log_d("missed deadline");
    }
    portENTER_CRITICAL(&control_mutex_);
    new_control = control_;
    portEXIT_CRITICAL(&control_mutex_);

    if (control.mode != new_control.mode) {
      pid_.Reset();
    }
    control = new_control;
    
    int cur_pulse = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(this->unit_, &cur_pulse));
    uint64_t current_micros = micros();
    int dpulse = cur_pulse - prev_pulse;
    uint64_t delta_micros = current_micros - last_micros;
    const float pulse_per_output_rev = 7 * 4 * 30;
    
    const float pps = (1.0 * dpulse / delta_micros) * 1000000;
    const float rps = pps / pulse_per_output_rev;
    const float dps = rps * 360;

    float new_speed = alpha * dps + (1 - alpha) * prev_speed;
    portENTER_CRITICAL(&control_mutex_);
    speed_ = new_speed;
    portEXIT_CRITICAL(&control_mutex_);

    int output_duty;

    if (control.mode == ControlMode::kSpeed) {
      float dt = static_cast<float>(delta_micros) / 1000000.0f;
      float output = pid_.Compute(control.setpoint, new_speed, dt);
      output_duty = static_cast<int>(output);
      OutputDuty(output_duty);
    } else  {
      output_duty = control.setpoint;
      OutputDuty(output_duty);
    }

#if 0
    if (current_micros - last_log_micros > 20000) {
      last_log_micros = current_micros;
      uint32_t timestamp = current_micros / 1000;
      printf(">setpoint:%d:%d\n", timestamp, control.setpoint);
      printf(">state:%d:%f\n", timestamp, new_speed);
      printf(">output:%d:%d\n", timestamp, output_duty);
      printf(">i:%d:%f\n", timestamp, pid_.i());  
      printf(">p:%d:%f\n", timestamp, pid_.p());
      printf(">d:%d:%f\n", timestamp, pid_.d());
      printf(">err:%d:%f\n", timestamp, pid_.error());
    }
#endif

    prev_speed = new_speed;
    prev_pulse = cur_pulse;
    last_micros = current_micros;
  }
}

void ServoMotor::OutputDuty(int duty) {
  if (duty == kBrakeDuty) {
    digitalWrite(rev_pin_, HIGH);
    digitalWrite(fwd_pin_, HIGH);
    analogWrite(pwm_pin_, 0);
    return;
  }
  if (duty == 0) {
    digitalWrite(rev_pin_, LOW);
    digitalWrite(fwd_pin_, LOW);
    analogWrite(pwm_pin_, 0);
  }

  if (duty > 0) {
    digitalWrite(rev_pin_, LOW);
    digitalWrite(fwd_pin_, HIGH);
  } else if (duty < 0) {
    digitalWrite(rev_pin_, HIGH);
    digitalWrite(fwd_pin_, LOW);
  }

  duty = std::abs(duty);
  if (deadband_ > 0) {
    // Scale the duty cycle so that it is between deadband_ and 255.
    const int deadband_width = 255 - deadband_;
    duty = deadband_ + duty * deadband_width / 255;
  }
  analogWrite(pwm_pin_, duty);
}

void ServoMotor::SetDuty(int duty) {
  duty = std::clamp(duty, -255, 255);
  portENTER_CRITICAL(&control_mutex_);
  control_.mode = ControlMode::kDuty;
  control_.setpoint = duty;
  ++control_.seq;
  portEXIT_CRITICAL(&control_mutex_);
}

void ServoMotor::Brake() {
  portENTER_CRITICAL(&control_mutex_);
  control_.mode = ControlMode::kDuty;
  control_.setpoint = 0;
  ++control_.seq;
  portEXIT_CRITICAL(&control_mutex_);
}

void ServoMotor::SetSpeed(int speed) {
  portENTER_CRITICAL(&control_mutex_);
  control_.mode = ControlMode::kSpeed;
  control_.setpoint = speed;
  ++control_.seq;
  portEXIT_CRITICAL(&control_mutex_);
}

void ServoMotor::SetPosition(int position) {
  portENTER_CRITICAL(&control_mutex_);
  control_.mode = ControlMode::kPosition;
  control_.setpoint = position;
  ++control_.seq;
  portEXIT_CRITICAL(&control_mutex_);
}

int64_t ServoMotor::GetCount() {
  int count;
  ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_get_count(unit_, &count));
  return count;
}
