#include <Arduino.h>
#include <ESPmDNS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <sys/unistd.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string_view>

#include "Adafruit_NeoPixel.h"
#include "FreeRTOSConfig.h"
#include "IPAddress.h"
#include "MPU6050.h"
#include "PIDController.h"
#include "ServoMotor.h"
#include "WiFi.h"
#include "WiFiUdp.h"
#include "Wire.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp32-hal-adc.h"
#include "esp32-hal-gpio.h"
#include "esp32-hal-log.h"
#include "esp32-hal.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "pins_arduino.h"
#include "portmacro.h"
#include "soc/gpio_num.h"
#include "uni_property.h"

#ifdef CONFIG_BT_MODE_TUNE
#include <charconv>
#include <format>
#include <string>
#include <system_error>

#include "NimBLEAdvertising.h"
#include "NimBLEAttValue.h"
#include "NimBLEDevice.h"
#endif
#ifdef CONFIG_BT_MODE_PAD
#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <gap.h>
#include <uni.h>
#endif

#define TAG "SBRB"

namespace sbrb {

constexpr int kMPU6050SDA = 5;
constexpr int kMPU6050SCL = 6;
constexpr int kBattVoltagePin = 7;
constexpr int kSleepPin = 8;

constexpr int kMotorLPwmPin = 12;
constexpr int kMotorLFwdPin = RX;
constexpr int kMotorLRevPin = 13;
constexpr int kMotorLPulsePin = 2;
constexpr int kMotorLControlPin = 1;

constexpr int kMotorRPwmPin = 9;
constexpr int kMotorRFwdPin = 11;
constexpr int kMotorRRevPin = 10;
constexpr int kMotorRPulsePin = 3;
constexpr int kMotorRControlPin = 4;

constexpr int kNeoPixelPin = 21;
constexpr int kNeoPixelCount = 1;
static Adafruit_NeoPixel neo_pixel(kNeoPixelCount, kNeoPixelPin,
                                   NEO_RGB + NEO_KHZ800);

constexpr float kBattVoltageScale = 3.3f;
constexpr float kDividerRatio = 1.0f / 3.0f;

std::unique_ptr<ServoMotor> motor_l;
std::unique_ptr<ServoMotor> motor_r;
std::unique_ptr<MPU6050> mpu;

struct State {
  int64_t now = millis();
  int loop_iter = 0;

  int last_steering_iter = 0;
  uint32_t now_us = micros();
  float dt = 0;  // The change in time (s) since the last sample.

  float pitch = -90;  // Degrees, negative if we're tilted forward.
  float pitch_rate =
      0;  // Rate in the change of pitch (i.e. the raw gyro reading.)

  float yaw_rate = 0;     // Degrees/s, smoothed via EWMA.
  float wheel_speed = 0;  // Degrees/s, averaged between the two motors,
                          // smoothed via EWMA.
  float accel = 0;        // Degrees/s^2, smoothed via EWMA.

  float wheel_speed_left = 0;
  float wheel_speed_right = 0;

  int64_t last_time_standing = now - 1000;
  bool fallen() const { return now - last_time_standing > 250; }

  // Command state, useful for debugging.
  float commanded_yaw_rate;
  float commanded_speed;
  float commanded_pitch;
  float commanded_duty_left;
  float commanded_duty_right;

  // The "now_us" time of the sample that was last used to compute the balance
  // and steering commands. Used to compute dt for the PID controller update
  // loops.
  uint32_t last_balance_cmd_us = 0;
  uint32_t last_steering_cmd_us = 0;
};

// The controller for the robot. Contains the PID controllers for the robot.
struct Controller {
  // The pitch controller takes in the current pitch and the desired pitch and
  // emits motor average duty cycle to achieve that pitch. Basic tuning
  // intuition is if we're pitched forward of where we want to be, we should be
  // running the motors *toward* the direction we're pitched. Running the motors
  // in one direction tends to push the wheels in the opposite direction,
  // gain will be positive.
  PIDController pitch_controller{PIDController::Params{
      .kp = 30.f,
      .ti = 0.6f,
      .td = 0.035f,
      .output_range = {{-255, 255}},
  }};

  // Controls the speed of the wheels by adjusting the target pitch angle.
  // The gain on this controller is quite low, because the wheel speed is in
  // degrees per second. The idea here is that the desired pitch might be
  // only a couple of degrees forward when we want to go 720 degrees per second.
  //
  // The gain's sign is set this way: if we want to go forward, we need to pitch
  // forward. To pitch forward, we need to the pitch to be negative.
  PIDController wheel_speed_controller{{
      .kp = -0.005f,
      .ti = 2.5f,
      .td = 0.01f,
      .output_range = {{-20, 20}},
  }};

  // Controls the differential drive that controls the yaw rate. (I.e. for each
  // degree per second of yaw rate error, how much duty cycle should we add to
  // the left motor and subtract from the right motor?)
  //
  // As a reminder to myself, in robotics, clockwise rotation about Z is
  // negative. Error is setpoint - measurement. So, let's suppose setpoint is
  // zero, and I observe that I'm turning to the right (measurement is
  // negative). That will make my error positive. When my error is positive, I
  // need to turn left to compensate, which is to say that I need to add duty to
  // the right motor and subtract from the left. So the output of this is the
  // differential duty cycle to add to the right motor and subtract from the
  // left.
  PIDController yaw_rate_controller{PIDController::Params{
      .kp = 0.25f, .ti = 0.0, .td = 0.0f, .output_range = {{-50, 50}}}};

  int32_t axis_y = 0;  // Used to compute setpoints.
  int32_t axis_rx = 0;
};
Controller pid_controllers;

static volatile bool sleep_soon = false;
void IRAM_ATTR SleepPinISR() { sleep_soon = true; }

[[noreturn]] void GoToSleep() {
  motor_l->DeepSleepPrepare();
  motor_r->DeepSleepPrepare();

  neo_pixel.clear();
  neo_pixel.show();

  const auto pin = static_cast<gpio_num_t>(kSleepPin);
  pinMode(pin, INPUT_PULLUP);
  do {
    delay(100);  // Debounce, the sleep button.
  } while (!digitalRead(pin));
  delay(25);

  // We need to respect all holds, particularly on the motor pins,
  // while we are asleep.
  gpio_deep_sleep_hold_en();

  rtc_gpio_init(pin);
  rtc_gpio_pullup_en(pin);
  esp_sleep_enable_ext0_wakeup(pin, 0);
  esp_deep_sleep_start();
}

void WakeUp() {
  rtc_gpio_deinit(static_cast<gpio_num_t>(kSleepPin));
  motor_l->DeepSleepResume();
  motor_r->DeepSleepResume();

  // Don't finish waking up until the sleep button is released.
  pinMode(kSleepPin, INPUT_PULLUP);
  while (!digitalRead(kSleepPin)) {
    delay(100);
  }
  delay(50);
}

static void InitTuningService();

// The battery is connected to kBattVoltagePin via a voltage divider.
// The divider is 20k/10k. This means we'll see a voltage which is roughly
// 2/3rds of the actual battery voltage.
int BatteryMillivolts() {
  constexpr int kMillivoltsScale = 16 / kDividerRatio;
  const int batt_milivolts =
      (analogReadMilliVolts(kBattVoltagePin) * kMillivoltsScale) / 16;
  return batt_milivolts;
}

static int smoothed_battery_millivolts = -1000;
void MonitorBattery(void*) {
  constexpr float kAlpha = 0.05;
  if (smoothed_battery_millivolts < -100) {
    smoothed_battery_millivolts = BatteryMillivolts();
  } else {
    smoothed_battery_millivolts = (smoothed_battery_millivolts * (1 - kAlpha) +
                                   BatteryMillivolts() * kAlpha);
  }
}

#if CONFIG_TELEMETRY_ENABLED
struct TelemetryData {
  uint32_t now_ms;
  float yaw_rate;
  float yaw_p;
  float yaw_i;
  float yaw_d;
  float yaw_err;
  float yaw_cmd;
  float motor_l_duty;
  float motor_r_duty;
  float pitch;
};

static portMUX_TYPE telemetry_state_mu = portMUX_INITIALIZER_UNLOCKED;
static TelemetryData telemetry_state;
static IPAddress telemetry_addr;
static constexpr int kTeleplotPort = 47269;
static WiFiUDP udp;

void QueueTelemetry(const TelemetryData& d) {
  if (portTRY_ENTER_CRITICAL(&telemetry_state_mu, 0)) {
    telemetry_state = d;
    portEXIT_CRITICAL(&telemetry_state_mu);
  }
}

void SendTelemetry(void*) {
  static uint32_t last_send = 0;
  if (!portTRY_ENTER_CRITICAL(&telemetry_state_mu, 0)) {
    ESP_LOGW("SBRB", "Failed to enter critical section for telemetry");
    return;
  }
  TelemetryData state = telemetry_state;
  IPAddress addr = telemetry_addr;
  portEXIT_CRITICAL(&telemetry_state_mu);
  if (addr == IPAddress()) {
    ESP_LOGW("SBRB", "Telemetry address not resolved");
    return;
  }
  if (state.now_ms == last_send) {
    ESP_LOGW("SBRB", "Telemetry not updated");
    return;
  }
  last_send = state.now_ms;

  // Assemble a UDP packet containing the telemetry. For the sake of argument we
  // will assume no more than one packet is needed, but this is a hack.
  char packet_buf[1432];
  int packet_len = 0;

  auto add_field = [&](std::string_view name, float value) {
    int written = snprintf(&packet_buf[packet_len],
                           sizeof(packet_buf) - packet_len, "%*s:%lu:%.4f\n",
                           name.size(), name.data(), state.now_ms, value);
    if (written < 0 || written >= sizeof(packet_buf) - packet_len) {
      packet_buf[packet_len] = 0;
    }
    packet_len += written;
  };

  add_field("dyaw", state.yaw_rate);
  add_field("yaw_p", state.yaw_p);
  add_field("yaw_i", state.yaw_i);
  add_field("yaw_d", state.yaw_d);
  add_field("yaw_err", state.yaw_err);
  add_field("yaw_cmd", state.yaw_cmd);
  add_field("motor_l_duty", state.motor_l_duty);
  add_field("motor_r_duty", state.motor_r_duty);
  add_field("pitch", state.pitch);

  udp.beginPacket(addr, kTeleplotPort);
  udp.write(reinterpret_cast<const uint8_t*>(packet_buf), packet_len);
  if (udp.endPacket() < 0) {
    ESP_LOGD("SBRB", "Failed to send telemetry packet of length %d",
             packet_len);
  } else {
    ESP_LOGD("SBRB", "Sent telemetry packet of length %d", packet_len);
  }
}

TelemetryData MakeTelemetryPacket(const State& state,
                                  const Controller& controller) {
  TelemetryData packet;
  packet.now_ms = state.now;
  packet.yaw_rate = state.yaw_rate;
  packet.yaw_p = controller.yaw_rate_controller.p();
  packet.yaw_i = controller.yaw_rate_controller.i();
  packet.yaw_d = controller.yaw_rate_controller.d();
  packet.yaw_err = controller.yaw_rate_controller.error();
  packet.yaw_cmd = state.commanded_yaw_rate;
  packet.motor_l_duty = state.commanded_duty_left;
  packet.motor_r_duty = state.commanded_duty_right;
  packet.pitch = state.pitch;
  return packet;
}

#endif

void LowPriorityTasks(void*) {
  struct {
    void (*work)(void*);
    uint32_t period;
    uint32_t last_done = millis() - period;
    bool IsExpired() const { return millis() - last_done > period; }
    void MarkDone() { last_done = millis(); }
    uint32_t SleepDuration() const {
      const uint32_t since_done = millis() - last_done;
      if (since_done > period) {
        return 0;
      }
      return period - since_done;
    }
  } jobs[] = {
      {.work = MonitorBattery, .period = 1000},
#if CONFIG_TELEMETRY_ENABLED
      {.work = SendTelemetry, .period = 50},
#endif
  };

  while (true) {
    uint32_t sleep_time = std::numeric_limits<uint32_t>::max();
    for (auto& job : jobs) {
      if (job.IsExpired()) {
        job.work(nullptr);
        job.MarkDone();
      }
      sleep_time = std::min(sleep_time, job.SleepDuration());
    }
    if (sleep_time > 50) {
      ESP_LOGW(TAG, "Sleep duration %d too long", sleep_time);
      sleep_time = 50;
    }
    vTaskDelay(pdMS_TO_TICKS(sleep_time));
  }
}

constexpr int kBalanceFreq = 500;
constexpr int kSteeringFreq = 20;
constexpr int kSteerOncePerIters = kBalanceFreq / kSteeringFreq;

void UpdateState(State& state, bool log_details = false) {
  state.now = millis();
  state.loop_iter++;
  auto now_us = micros();
  state.dt = (now_us - state.now_us) / 1000000.0f;
  state.now_us = now_us;

  if (state.dt < 0.0005f) {
    // Not enough time has passed since the last sample. Skip this iteration.
    return;
  }

  constexpr float kAlpha = 0.98f;
  constexpr float kRadToDeg = 180.0f / 3.14159265f;

  // MPU6050 Defaults
  constexpr float kAccelScale = 16384.0f;
  constexpr float kGyroScale = 32.8f;
  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
  mpu->getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

  // Convert to physical units
  const float ax = ax_raw / kAccelScale;
  const float ay = ay_raw / kAccelScale;
  const float az = az_raw / kAccelScale;
  const float gy = gy_raw / kGyroScale;
  const float gz = gz_raw / kGyroScale;

  float acc_angle_y = atan2(-ax, sqrt(ay * ay + az * az)) * kRadToDeg;
  state.pitch =
      (state.pitch + gy * state.dt) * kAlpha + (1.0f - kAlpha) * acc_angle_y;

  // Update pitch and yaw rates. Tuned for a cutoff frequency of ~4Hz.
  constexpr auto kGyroAlpha = 0.05f;
  constexpr auto kGryoAlphaInv = 1.0f - kGyroAlpha;
  state.pitch_rate = kGyroAlpha * gy + kGryoAlphaInv * state.pitch_rate;
  state.yaw_rate = kGyroAlpha * gz + kGryoAlphaInv * state.yaw_rate;

  state.wheel_speed_left = motor_l->GetSpeed();
  state.wheel_speed_right = motor_r->GetSpeed();
  const float avg_speed =
      (state.wheel_speed_left + state.wheel_speed_right) / 2.0f;
  // Wheel speed is filtered less aggressively than the gyro since we expect
  // a higher frequency of real wheel speed changes.
  constexpr auto kSpeedAlpha = 0.1f;
  const float raw_accel = (avg_speed - state.wheel_speed) / state.dt;
  state.accel = kSpeedAlpha * raw_accel + (1.0f - kSpeedAlpha) * state.accel;
  state.wheel_speed =
      kSpeedAlpha * avg_speed + (1.0f - kSpeedAlpha) * state.wheel_speed;

  const float abspitch = std::abs(state.pitch);
  if (state.fallen() && abspitch <= 3) {
    state.last_time_standing = state.now;
  } else if (abspitch <= 30) {
    state.last_time_standing = state.now;
  }
}

void Control(State& state, Controller& control) {
  // Compute the setpoints for desired turn rate and forward and reverse.
  constexpr float kSpeedRange = 920.0f;  // 120 RPM is max commanded speed.
  constexpr float kYawRange =
      180.0f;  // 90 degrees per second is max turn rate.
  constexpr float kSpeedSaturation =
      512.0f;  // Once the y-axis reaches +-512, the output will be saturated.
  constexpr float kYawSaturation =
      450.0f;  // Once the rx-axis reaches +-450, the output will be saturated.

  // axis_y+ is backward, but speed+ is forward, so we negate the axis.
  const float speed_setpoint =
      std::clamp(-control.axis_y * kSpeedRange / kSpeedSaturation, -kSpeedRange,
                 kSpeedRange);
  // axis_rx+ is clockwise rotation, but yaw rate is negative for clockwise.
  const float yaw_setpoint = std::clamp(
      -control.axis_rx * kYawRange / kYawSaturation, -kYawRange, kYawRange);

  if (state.loop_iter - state.last_steering_iter >= kSteerOncePerIters) {
    state.last_steering_iter = state.loop_iter;
    // Try to find a pitch that will get us to the desired speed.
    const float dt = (state.now_us - state.last_steering_cmd_us) / 1000000.0f;
    state.commanded_pitch = control.wheel_speed_controller.Compute(
        speed_setpoint, state.wheel_speed, state.accel, dt);
    state.last_steering_cmd_us = state.now_us;
  }

  // Try to find a duty cycle that will get us to the desired pitch.
  const float dt = (state.now_us - state.last_balance_cmd_us) / 1000000.0f;
  float duty = control.pitch_controller.Compute(
      state.commanded_pitch, state.pitch, state.pitch_rate, dt);
  float yaw_duty =
      control.yaw_rate_controller.Compute(yaw_setpoint, state.yaw_rate, dt);
  state.last_balance_cmd_us = state.now_us;

  // Scale the duty cycle based on the battery voltage.
  // We did the initial robot tuning at 7V on a bench power supply.
  // If the battery voltage is higher than that we need to command
  // a lower duty cycle to compensate. This does also mean we'll never
  // run the motor at full power when the battery is fully charged,
  // but that's okay because doing it this way means we'll give a
  // consistent experience no matter the charge level. Note that there
  // is no need for clamping here as the robot will shut off if the
  // power is substantially lower than the tuning voltage to protect
  // the battery.
  constexpr float kTuningVbattMV = 7000;
  if (smoothed_battery_millivolts > 6000) {
    // Don't bother with this step if we're not connected to motor power.
    duty *= kTuningVbattMV / smoothed_battery_millivolts;
    yaw_duty *= kTuningVbattMV / smoothed_battery_millivolts;
  }

  state.commanded_duty_left = duty - yaw_duty;
  state.commanded_duty_right = duty + yaw_duty;
  motor_l->SetDuty(state.commanded_duty_left);
  motor_r->SetDuty(state.commanded_duty_right);
}

void SetPixelColor(float pitch) {
  constexpr float kRedPitch = 10.0f;

  const float lerp = std::min(1.0f, std::abs(pitch) / kRedPitch);
  neo_pixel.setPixelColor(0, lerp * 255, (1.0f - lerp) * 255, 0);
  neo_pixel.show();
}

#if CONFIG_BT_MODE_TUNE

char cmd_buf[100];
int cmd_idx = 0;

void DoTuningCommand(std::string_view cmd) {
  while (true) {
    // Strip leading whitespace
    while (!cmd.empty() && std::isspace(cmd.front())) {
      cmd.remove_prefix(1);
    }
    if (cmd.empty()) break;

    // Find the end of the command string (first whitespace)
    size_t cmd_end = 0;
    while (cmd_end < cmd.size() && !std::isspace(cmd[cmd_end])) {
      cmd_end++;
    }

    std::string_view command = cmd.substr(0, cmd_end);
    cmd.remove_prefix(cmd_end);

    // Strip whitespace before value
    while (!cmd.empty() && std::isspace(cmd.front())) {
      cmd.remove_prefix(1);
    }

    if (cmd.empty()) {
      log_e("Command %.*s missing value", static_cast<int>(command.size()),
            command.data());
      break;
    }

    float value;
    auto result = std::from_chars(cmd.data(), cmd.data() + cmd.size(), value);
    if (result.ec == std::errc()) {
      cmd.remove_prefix(result.ptr - cmd.data());

      if (command == "pp") {
        pid_controllers.pitch_controller.set_kp(value);
        ESP_LOGI(TAG, "Pitch: Set kp to %f", value);
      } else if (command == "pi") {
        pid_controllers.pitch_controller.set_ti(value);
        ESP_LOGI(TAG, "Pitch: Set ti to %f", value);
      } else if (command == "pd") {
        pid_controllers.pitch_controller.set_td(value);
        ESP_LOGI(TAG, "Pitch: Set td to %f", value);
      } else if (command == "wp") {
        pid_controllers.wheel_speed_controller.set_kp(value);
        ESP_LOGI(TAG, "Wheel: Set kp to %f", value);
      } else if (command == "wi") {
        pid_controllers.wheel_speed_controller.set_ti(value);
        ESP_LOGI(TAG, "Wheel: Set ti to %f", value);
      } else if (command == "wd") {
        pid_controllers.wheel_speed_controller.set_td(value);
        ESP_LOGI(TAG, "Wheel: Set td to %f", value);
      } else if (command == "db") {
        motor_l->set_deadband(static_cast<int>(value));
        motor_r->set_deadband(static_cast<int>(value));
        ESP_LOGI(TAG, "Set deadband to %d", static_cast<int>(value));
      } else {
        log_e("Unknown command: %.*s", static_cast<int>(command.size()),
              command.data());
      }
    } else {
      log_e("Failed to parse value for command %.*s",
            static_cast<int>(command.size()), command.data());
      // Consume the bad token
      while (!cmd.empty() && !std::isspace(cmd.front())) {
        cmd.remove_prefix(1);
      }
    }
  }
}

void MaybeReadCommand() {
  if (!Serial.available()) return;
  int c;
  while (cmd_idx < sizeof(cmd_buf) - 1 &&
         (cmd_idx == 0 || cmd_buf[cmd_idx - 1] != '\n') &&
         (c = Serial.read()) != -1) {
    cmd_buf[cmd_idx++] = c;
    ESP_LOGI(TAG, "Received char: %c", c);
  }

  std::string_view cmd_view(cmd_buf, cmd_idx);
  if (!cmd_view.empty() &&
      (cmd_view.back() == '\n' || cmd_view.back() == '\r')) {
    DoTuningCommand(cmd_view);
    std::memset(cmd_buf, 0, sizeof(cmd_buf));
    cmd_idx = 0;
  }
}

static NimBLECharacteristicCallbacks& TuningCharacteristicCallbacks() {
  class Callbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* characteristic,
                 NimBLEConnInfo& conn_info) {
      ESP_LOGI(TAG, "Received write to tuning characteristic");
      NimBLEAttValue v = characteristic->getValue();
      DoTuningCommand(std::string_view(reinterpret_cast<const char*>(v.data()),
                                       v.length()));
    }
    void onRead(NimBLECharacteristic* characteristic,
                NimBLEConnInfo& conn_info) {
      ESP_LOGI(TAG, "Received read to tuning characteristic");
      std::string result = std::format(
          "pp {:.4f} pi {:.4f} pd {:.4f} wp {:.4f} wi {:.4f} wd {:.4f}",
          pid_controllers.pitch_controller.kp(),
          pid_controllers.pitch_controller.ti(),
          pid_controllers.pitch_controller.td(),
          pid_controllers.wheel_speed_controller.kp(),
          pid_controllers.wheel_speed_controller.ti(),
          pid_controllers.wheel_speed_controller.td());
      characteristic->setValue(result);
    }
  };
  static Callbacks callbacks;
  return callbacks;
}

void InitTuningService() {
  auto init_nimble = [](void*) {
    ESP_LOGI(TAG, "NimBLE init started.");

    NimBLEDevice::init("SBRB");
    NimBLEServer* server = NimBLEDevice::createServer();
    // Create a service at the nordic SPP service UUID.
    NimBLEService* service = server->createService(
        NimBLEUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E"));
    NimBLECharacteristic* characteristic = service->createCharacteristic(
        NimBLEUUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"),
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ);
    characteristic->setCallbacks(&TuningCharacteristicCallbacks());
    service->start();

    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(service->getUUID());
    advertising->setName("SBRB");
    advertising->start();
    server->advertiseOnDisconnect(true);

    ESP_LOGI(TAG, "NimBLE initialized");

    vTaskDelete(nullptr);  // Intentionally exitting.
  };

  xTaskCreate(init_nimble, "NimBLEInit", 4096, nullptr, 1, nullptr);
}

#else
static inline void MaybeReadCommand() {}
static void InitTuningService() {}
#endif

#if CONFIG_BT_MODE_PAD

uni_platform my_platform = {
    .name = "SBRB",
    .init = [](int, const char**) {},
    .on_init_complete =
        []() {
          uni_bt_start_scanning_and_autoconnect_unsafe();
          uni_bt_allow_incoming_connections(true);
        },
    .on_device_discovered =
        [](bd_addr_t addr, const char* name, uint16_t cod, uint8_t rssi) {
          ESP_LOGI(TAG, "Device discovered: %s", name);
          return UNI_ERROR_SUCCESS;
        },
    .on_device_connected =
        [](uni_hid_device_t* d) { ESP_LOGI(TAG, "Device connected"); },
    .on_device_disconnected =
        [](uni_hid_device_t* d) { ESP_LOGI(TAG, "Device disconnected"); },
    .on_device_ready =
        [](uni_hid_device_t* d) {
          ESP_LOGI(TAG, "Device ready");
          return UNI_ERROR_SUCCESS;
        },
    .on_gamepad_data =
        [](uni_hid_device_t* d, uni_gamepad_t* gp) {
          static int count = 0;
          if (count++ < 10) {
            return;
          }
          count = 0;
          ESP_LOGD(TAG, "X: %05d Y: %05d RX: %05d RY: %05d", gp->axis_x);
        },
    .on_controller_data =
        [](uni_hid_device_t* d, uni_controller_t* ctl) {
          static int count = 0;
          pid_controllers.axis_y = ctl->gamepad.axis_y;
          pid_controllers.axis_rx = ctl->gamepad.axis_rx;
          if (count++ < 10) {
            return;
          }
          count = 0;
          // axis_y forward is negative
          // axis_rx left is negative
          ESP_LOGD(TAG, "X: %05d Y: %05d RX: %05d RY: %05d",
                   ctl->gamepad.axis_x, ctl->gamepad.axis_y,
                   ctl->gamepad.axis_rx, ctl->gamepad.axis_ry);
        },
    .get_property = [](uni_property_idx_t idx) -> const uni_property_t* {
      return nullptr;
    },
    .on_oob_event = [](uni_platform_oob_event_t event,
                       void* data) { ESP_LOGI(TAG, "OOB event"); },
    .device_dump = [](uni_hid_device_t* d) { ESP_LOGI(TAG, "Device dump"); },
    .register_console_cmds =
        []() { ESP_LOGI(TAG, "Register console commands"); },
};

void InitBluepad32() {
  auto start_bluepad = [](void*) {
    // Configure BTstack for ESP32 VHCI Controller
    btstack_init();

    // Must be called before uni_init()
    uni_platform_set_custom(&my_platform);

    // Init Bluepad32.
    uni_init(0 /* argc */, NULL /* argv */);

    auto print_bdaddr = [](void*) {
      bd_addr_t addr;
      gap_local_bd_addr(addr);
      ESP_LOGI(TAG, "Local bdaddr: %s", bd_addr_to_str(addr));
    };
    static btstack_context_callback_registration_t callback_registration;
    callback_registration.callback = print_bdaddr;
    callback_registration.context = nullptr;
    btstack_run_loop_execute_on_main_thread(&callback_registration);

    // Does not return.
    btstack_run_loop_execute();
  };

  xTaskCreate(start_bluepad, "BluePadThread", 4096, nullptr, 0, nullptr);
}

#else
static inline void InitBluepad32() {}
#endif

}  // namespace sbrb

TaskHandle_t loop_task;
esp_timer_handle_t loop_timer;

static void IRAM_ATTR AllowLoopToRun(void* arg) {
  BaseType_t higher_priority_woken;
  xTaskNotifyFromISR(loop_task, 0, eNoAction, &higher_priority_woken);
  portYIELD_FROM_ISR(higher_priority_woken);
}

void setup() {
  using namespace sbrb;
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  neo_pixel.setBrightness(0.2 * 255);

  const bool did_wake_from_deep_sleep =
      esp_sleep_get_wakeup_causes() & BIT(ESP_SLEEP_WAKEUP_EXT0);
  if (!did_wake_from_deep_sleep) {
    // Give the serial monitor some time to connect.
    delay(5000);
    printf("Woke up from not deep sleep!\n");
  }

  pinMode(kBattVoltagePin, INPUT);
  analogSetAttenuation(ADC_11db);

  // Set up the motors.
  ServoMotor::Config config = {
      .pulse_pin = kMotorLPulsePin,
      .control_pin = kMotorLControlPin,
      .pwm_pin = kMotorLPwmPin,
      .fwd_pin = kMotorLFwdPin,
      .rev_pin = kMotorLRevPin,
      .deadband = 8,
  };
  motor_l = ServoMotor::Create(config);
  motor_l->SetDuty(0);

  ServoMotor::Config config2 = {
      .pulse_pin = kMotorRPulsePin,
      .control_pin = kMotorRControlPin,
      .pwm_pin = kMotorRPwmPin,
      .fwd_pin = kMotorRFwdPin,
      .rev_pin = kMotorRRevPin,
      .deadband = 8,
  };
  motor_r = ServoMotor::Create(config2);
  motor_r->SetDuty(0);

  WakeUp();

  if (!Wire.begin(kMPU6050SDA, kMPU6050SCL)) {
    ESP_LOGE(TAG, "Failed to initialize I2C");
  } else {
    const uint8_t reboot_mpu_6050[] = {0x6B, 0x80};
    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
    Wire.write(reboot_mpu_6050, 2);
    Wire.endTransmission();

    // Wait for reboot to complete.
    delay(100);

    Wire.setClock(400000);
    ESP_LOGI(TAG, "I2C initialized");
    mpu = std::make_unique<MPU6050>(MPU6050_DEFAULT_ADDRESS, &Wire);
    mpu->initialize();
    mpu->setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    mpu->setDLPFMode(MPU6050_DLPF_BW_42);
    mpu->setRate(0);  // No samplerate divider (== 1kHz)
  }

#if 0
  if (!did_wake_from_deep_sleep) {
    // Any wakeup cause besides EXT0 means that we didn't wake up from
    // someone pressing the button. Let's go directly to sleep and ask them
    // to press the button before we actually wake up!
    ESP_LOGI(TAG, "Woke for cause other than button press, sleeping!\n");
    GoToSleep();
  }
#endif

  attachInterrupt(kSleepPin, SleepPinISR, FALLING);

  InitTuningService();
  InitBluepad32();

  // This timer exists to ensure the loop task unblocks at a fixed frequency,
  // which is not possible using xTaskDelayUntil, at least when we're talking
  // about 500Hz or more.
  loop_task = xTaskGetCurrentTaskHandle();
  esp_timer_create_args_t args{
      .callback = AllowLoopToRun,
      .arg = nullptr,
      .name = "LoopTimer",
      .skip_unhandled_events = true,
  };
  esp_timer_create(&args, &loop_timer);
  esp_timer_start_periodic(loop_timer, 1000000 / kBalanceFreq);

  // Run low priority tasks on core zero -- we can't tolerate the PID loop being
  // blocked by this stuff.
  xTaskCreatePinnedToCore(LowPriorityTasks, "LowPriorityTasks", 8192, nullptr,
                          0, nullptr, 0);

#if CONFIG_TELEMETRY_ENABLED
  xTaskCreate(
      +[](void*) {
        if (std::string_view(CONFIG_WIFI_SSID) == "") {
          ESP_LOGI(TAG, "No WiFi SSID configured");
          vTaskDelete(nullptr);
        }
        WiFi.mode(WIFI_MODE_STA);
        WiFi.begin(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
        while (WiFi.status() != WL_CONNECTED) {
          delay(500);
        }
        ESP_LOGI(TAG, "Connected to WiFi %s",
                 WiFi.localIP().toString().c_str());

        if (!MDNS.begin("sbrb")) {
          ESP_LOGE(TAG, "MDNS.begin() failed");
        }

        IPAddress addr;

        int res;
        do {
          res = WiFi.hostByName(CONFIG_TELEPLOT_HOSTNAME, addr);
          if (res != 1) {
            ESP_LOGE(TAG, "Unable to resolve telemetry host: %d", res);
          }
          delay(500);
        } while (res != 1);
        portENTER_CRITICAL(&telemetry_state_mu);
        telemetry_addr = addr;
        portEXIT_CRITICAL(&telemetry_state_mu);
        vTaskDelete(nullptr);
      },
      "WiFiTask", 4096, nullptr, 0, nullptr);
#endif
  // The priority of the loop task needs to be relatively high since it is
  // running the PID controllers.
  vTaskPrioritySet(nullptr, configMAX_PRIORITIES - 2);
}

void loop() {
  using namespace sbrb;
  static State state;
  static int print_counter = 0;
  bool print_details = false;
  if (print_details) print_counter = 0;

  static auto last_loop_start = micros();
  const auto wait_start = micros();
  auto interloop_time = wait_start - last_loop_start;
  last_loop_start = wait_start;

  xTaskNotifyWait(0, 1, nullptr, portMAX_DELAY);
  const auto loop_start = micros();
  UpdateState(state, print_details);

  // Do shutdown checks first and foremost.
  if (smoothed_battery_millivolts < 6800 || sleep_soon) {
    // GoToSleep();
  }

  SetPixelColor(state.pitch);

  if (state.fallen()) {
    motor_l->SetDuty(0);
    motor_r->SetDuty(0);
    pid_controllers.pitch_controller.Reset();
    pid_controllers.wheel_speed_controller.Reset();
    pid_controllers.yaw_rate_controller.Reset();
  } else {
    Control(state, pid_controllers);
  }

#if CONFIG_TELEMETRY_ENABLED
  static int32_t last_telemetry = state.now;
  if (state.now - last_telemetry > 50) {
    last_telemetry = state.now;
    TelemetryData packet = MakeTelemetryPacket(state, pid_controllers);
    QueueTelemetry(packet);
  }
#endif

  const auto loop_end = micros();
  static int last_loop_log = millis();
  if (millis() - last_loop_log > 1000 && !print_details && print_counter != 0) {
    // Check for violated loop timing expectations.
    if (interloop_time > 2500) {
      ESP_LOGE(TAG, "Interloop time was %d usec",
               static_cast<int>(interloop_time));
    } else if (loop_end - loop_start > 1500) {
      ESP_LOGE(TAG, "Control loop took %d usec",
               static_cast<int>(loop_end - loop_start));
    } else {
      return;  // We didn't print anything.
    }
    last_loop_log = millis();
  }
}
