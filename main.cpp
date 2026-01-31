#include <Arduino.h>
#include <FreeRTOS.h>
#include <stdio.h>
#include <sys/unistd.h>
#include <task.h>

#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <format>
#include <memory>
#include <string>
#include <string_view>
#include <system_error>

#include "Adafruit_NeoPixel.h"
#include "MPU6050.h"
#include "NimBLEAdvertising.h"
#include "NimBLEAttValue.h"
#include "NimBLEDevice.h"
#include "PIDController.h"
#include "ServoMotor.h"
#include "Wire.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp32-hal-adc.h"
#include "esp32-hal-gpio.h"
#include "esp32-hal-log.h"
#include "esp32-hal.h"
#include "esp_err.h"
#include "esp_sleep.h"
#include "pins_arduino.h"
#include "soc/gpio_num.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#define LOG_LOCAL_TAG "SBRB"

constexpr int kMPU6050SDA = 5;
constexpr int kMPU6050SCL = 6;
constexpr int kBattVoltagePin = 7;
constexpr int kSleepPin = 8;

constexpr int kMotorRPwmPin = 12;
constexpr int kMotorRFwdPin = RX;
constexpr int kMotorRRevPin = 13;
constexpr int kMotorRPulsePin = 2;
constexpr int kMotorRControlPin = 1;

constexpr int kMotorLPwmPin = 9;
constexpr int kMotorLFwdPin = 11;
constexpr int kMotorLRevPin = 10;
constexpr int kMotorLPulsePin = 3;
constexpr int kMotorLControlPin = 4;

constexpr int kNeoPixelPin = 21;
constexpr int kNeoPixelCount = 1;
static Adafruit_NeoPixel neo_pixel(kNeoPixelCount, kNeoPixelPin,
                                   NEO_RGB + NEO_KHZ800);

constexpr float kBattVoltageScale = 3.3f;
constexpr float kDividerRatio = 1.0f / 3.0f;

std::unique_ptr<ServoMotor> motor_l;
std::unique_ptr<ServoMotor> motor_r;
std::unique_ptr<MPU6050> mpu;

static volatile bool sleep_soon = false;
void IRAM_ATTR SleepPinISR() { sleep_soon = true; }

static NimBLECharacteristicCallbacks& TuningCharacteristicCallbacks();

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

void InitNimBLE(void*) {
  log_i("NimBLE init started.");

  NimBLEDevice::init("SBRB");
  NimBLEServer* server = NimBLEDevice::createServer();
  // Create a service at the nordic SPP service UUID.
  NimBLEService* service =
      server->createService(NimBLEUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E"));
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

  log_i("NimBLE initialized");

  vTaskDelete(nullptr);  // Intentionally exitting.
}

void setup() {
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
    log_e("Failed to initialize I2C");
  } else {
    const uint8_t reboot_mpu_6050[] = {0x6B, 0x80};
    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
    Wire.write(reboot_mpu_6050, 2);
    Wire.endTransmission();

    // Wait for reboot to complete.
    delay(100);

    Wire.setClock(400000);
    log_i("I2C initialized");
    mpu = std::make_unique<MPU6050>(MPU6050_DEFAULT_ADDRESS, &Wire);
    mpu->initialize();
    mpu->setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
  }

#if 0
  if (!did_wake_from_deep_sleep) {
    // Any wakeup cause besides EXT0 means that we didn't wake up from
    // someone pressing the button. Let's go directly to sleep and ask them
    // to press the button before we actually wake up!
    log_i("Woke for cause other than button press, sleeping!\n");
    GoToSleep();
  }
#endif

  attachInterrupt(kSleepPin, SleepPinISR, FALLING);

  xTaskCreate(InitNimBLE, "InitNimBLE", 4096, nullptr, 1, nullptr);
}

// The battery is connected to kBattVoltagePin via a voltage divider.
// The divider is 20k/10k. This means we'll see a voltage which is roughly
// 2/3rds of the actual battery voltage.
int BatteryMillivolts() {
  constexpr int kMillivoltsScale = 16 / kDividerRatio;
  const int batt_milivolts =
      (analogReadMilliVolts(kBattVoltagePin) * kMillivoltsScale) / 16;
  return batt_milivolts;
}

// The state of the robot. Contains only the data needed to control the robot,
// not any intermediate values.
struct State {
  int64_t now = millis();
  int loop_iter = 0;

  int last_steering_iter = 0;

  float pitch = -90;  // Degrees, negative if we're tilted forward.
  float pitch_rate;  // Rate in the change of pitch (i.e. the raw gyro reading.)

  float yaw_rate = 0;     // Degrees/s, smoothed via EWMA.
  float wheel_speed = 0;  // Degrees/s, averaged between the two motors,
                          // smoothed via EWMA.
  float accel = 0;        // Degrees/s^2, smoothed via EWMA.

  float wheel_speed_left;
  float wheel_speed_right;

  int64_t last_time_batt_good = now;
  int vbatt_mv = 0;

  int64_t last_time_standing = now - 1000;
  bool fallen() const { return now - last_time_standing > 250; }

  // Command state, useful for debugging.
  float commanded_yaw_rate;
  float commanded_speed;
  float commanded_pitch;
  float commanded_duty_left;
  float commanded_duty_right;
};

constexpr int kBalanceFreq = 500;
constexpr int kSteeringFreq = 20;
constexpr int kSteerOncePerIters = kBalanceFreq / kSteeringFreq;

void UpdateState(State& state, bool log_details = false) {
  state.now = millis();
  state.loop_iter++;
  auto phase_start = micros();

  constexpr float kDt = 0.002f;  // 2ms = 500Hz
  constexpr float kAlpha = 0.98f;
  constexpr float kRadToDeg = 180.0f / 3.14159265f;

  // MPU6050 Defaults
  constexpr float kAccelScale = 16384.0f;
  constexpr float kGyroScale = 32.8f;
  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
  mpu->getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
  if (log_details) {
    log_i("getMotion6: %d us", micros() - phase_start);
    phase_start = micros();
  }

  // Convert to physical units
  const float ax = ax_raw / kAccelScale;
  const float ay = ay_raw / kAccelScale;
  const float az = az_raw / kAccelScale;
  const float gx = gx_raw / kGyroScale;
  const float gy = gy_raw / kGyroScale;
  const float gz = gz_raw / kGyroScale;

  float acc_angle_y = atan2(-ax, sqrt(ay * ay + az * az)) * kRadToDeg;
  state.pitch =
      (state.pitch + gy * kDt) * kAlpha + (1.0f - kAlpha) * acc_angle_y;

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
  const float raw_accel = (avg_speed - state.wheel_speed) / kDt;
  state.accel = kSpeedAlpha * raw_accel + (1.0f - kSpeedAlpha) * state.accel;
  state.wheel_speed =
      kSpeedAlpha * avg_speed + (1.0f - kSpeedAlpha) * state.wheel_speed;

  if (log_details) {
    log_i("physics: %d us", micros() - phase_start);
    phase_start = micros();
  }

  constexpr auto kBatteryAlpha = 0.02;
  if (state.vbatt_mv == 0) {
    state.vbatt_mv = BatteryMillivolts();
  } else {
    state.vbatt_mv = kBatteryAlpha * BatteryMillivolts() +
                     (1.0f - kBatteryAlpha) * state.vbatt_mv;
  }

  if (state.vbatt_mv < 1000 || state.vbatt_mv > 6800) {
    // Note: if the battery voltage is <1000, that either means that the 2S
    // LiPo is uber screwed, or we're just not connected to battery power.
    // Either way no harm done by staying awake in that case.
    state.last_time_batt_good = state.now;
  }

  if (log_details) {
    log_i("battery: %d us", micros() - phase_start);
    phase_start = micros();
  }

  const float abspitch = std::abs(state.pitch);
  if (state.fallen() && abspitch <= 3) {
    state.last_time_standing = state.now;
  } else if (abspitch <= 30) {
    state.last_time_standing = state.now;
  }

  if (log_details) {
    log_i("standing: %d us", micros() - phase_start);
    phase_start = micros();
  }
}

// The controller for the robot. Contains the PID controllers for the robot.
struct Controller {
  // The pitch controller takes in the current pitch and the desired pitch and
  // emits motor average duty cycle to achieve that pitch. Basic tuning
  // intuition is if we're pitched forward of where we want to be, we should be
  // running the motors *toward* the direction we're pitched. Running the motors
  // in one direction tends to push the wheels in the opposite direction,
  // gain will be positive.
  PIDController pitch_controller{PIDController::Params{
      .kp = 35.f,
      .ti = 1.4f,
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
      .kp = -0.01f,
      .ti = 2.5f,
      .td = 0.005f,
      .output_range = {{-20, 20}},
  }};

  PIDController yaw_rate_controller{PIDController::Params{.kp = 0.0f}};

  float temp_test_duty = 1.0f;
};
Controller controller;

void Control(State& state, Controller& control) {
#if 1
  if (state.loop_iter - state.last_steering_iter >= kSteerOncePerIters) {
    state.last_steering_iter = state.loop_iter;
    // Try to find a pitch that will get us to the desired speed.
    state.commanded_pitch = control.wheel_speed_controller.Compute(
        0.0f, state.wheel_speed, state.accel, 1.0f / kSteeringFreq);
  }

  // Try to find a duty cycle that will get us to the desired pitch.
  float duty =
      control.pitch_controller.Compute(state.commanded_pitch, state.pitch,
                                       state.pitch_rate, 1.0f / kBalanceFreq);
#else

  // Temporarily, we're trying to detect the deadzone of these motors.
  // Each second we'll increase the duty cycle by 1. We're trying to see
  // when the torque gets high enough that the motors start moving.
  static int last_duty_change = millis();

  if (millis() - last_duty_change > 1000) {
    controller.temp_test_duty += 1.0f;
    last_duty_change = millis();
  }
  float duty = controller.temp_test_duty;
#endif

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
  if (state.vbatt_mv > 6000) {
    // Don't bother with this step if we're not connected to motor power.
    duty *= kTuningVbattMV / state.vbatt_mv;
  }

  motor_l->SetDuty(duty);
  motor_r->SetDuty(duty);

  state.commanded_duty_left = duty;
  state.commanded_duty_right = duty;
}

void SetPixelColor(float pitch) {
  constexpr float kRedPitch = 10.0f;

  const float lerp = std::min(1.0f, std::abs(pitch) / kRedPitch);
  neo_pixel.setPixelColor(0, lerp * 255, (1.0f - lerp) * 255, 0);
  neo_pixel.show();
}

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
        controller.pitch_controller.set_kp(value);
        log_i("Pitch: Set kp to %f", value);
      } else if (command == "pi") {
        controller.pitch_controller.set_ti(value);
        log_i("Pitch: Set ti to %f", value);
      } else if (command == "pd") {
        controller.pitch_controller.set_td(value);
        log_i("Pitch: Set td to %f", value);
      } else if (command == "wp") {
        controller.wheel_speed_controller.set_kp(value);
        log_i("Wheel: Set kp to %f", value);
      } else if (command == "wi") {
        controller.wheel_speed_controller.set_ti(value);
        log_i("Wheel: Set ti to %f", value);
      } else if (command == "wd") {
        controller.wheel_speed_controller.set_td(value);
        log_i("Wheel: Set td to %f", value);
      } else if (command == "db") {
        motor_l->set_deadband(static_cast<int>(value));
        motor_r->set_deadband(static_cast<int>(value));
        log_i("Set deadband to %d", static_cast<int>(value));
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
    log_i("Received char: %c", c);
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
      log_i("Received write to tuning characteristic");
      NimBLEAttValue v = characteristic->getValue();
      DoTuningCommand(std::string_view(reinterpret_cast<const char*>(v.data()),
                                       v.length()));
    }
    void onRead(NimBLECharacteristic* characteristic,
                NimBLEConnInfo& conn_info) {
      log_i("Received read to tuning characteristic");
      std::string result = std::format(
          "pp {:.4f} pi {:.4f} pd {:.4f} wp {:.4f} wi {:.4f} wd {:.4f}",
          controller.pitch_controller.kp(), controller.pitch_controller.ti(),
          controller.pitch_controller.td(),
          controller.wheel_speed_controller.kp(),
          controller.wheel_speed_controller.ti(),
          controller.wheel_speed_controller.td());
      characteristic->setValue(result);
    }
  };
  static Callbacks callbacks;
  return callbacks;
}

void loop() {
  static State state;
  static TickType_t last_wake_time = xTaskGetTickCount();
  constexpr TickType_t frequency = pdMS_TO_TICKS(2);
  static int64_t last_print_time = millis();
  static bool fallen = true;
  static int64_t last_time_standing = 0;

  const auto loop_start = micros();
  UpdateState(state);

  // Do shutdown checks first and foremost.
  if (state.now - state.last_time_batt_good > 1000 || sleep_soon) {
    // GoToSleep();
  }

  SetPixelColor(state.pitch);

  if (state.fallen()) {
    motor_l->SetDuty(0);
    motor_r->SetDuty(0);
    controller.pitch_controller.Reset();
    controller.wheel_speed_controller.Reset();
    controller.yaw_rate_controller.Reset();
  } else {
    Control(state, controller);
  }

  MaybeReadCommand();

  bool did_print = 0;
  if (state.now - last_print_time > 50) {
    const int loop_us = micros() - loop_start;
    controller.pitch_controller.DebugPrint("pitch");
    controller.wheel_speed_controller.DebugPrint("wheel_speed");
    printf(
        ">pitch_deg:%f\npitch_rate:%f\n>cmd_pitch:%f\n>wheel_speed:%f\n>duty:%"
        "f\n>vbatt:%d\n",
        state.pitch, state.pitch_rate, state.commanded_pitch, state.wheel_speed,
        (state.commanded_duty_left + state.commanded_duty_right) / 2.0f,
        state.vbatt_mv);
    last_print_time = state.now;
    did_print = 1;
  }

  if (!xTaskDelayUntil(&last_wake_time, frequency)) {
    static int last_loop_log = millis();
    if (millis() - last_loop_log > 1000 && !did_print) {
      log_e("Loop took more than %d ticks", static_cast<int>(frequency));
      last_loop_log = millis();
    }
  }
}
