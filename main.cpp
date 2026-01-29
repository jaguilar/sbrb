#include <Arduino.h>
#include <FreeRTOS.h>
#include <stdio.h>
#include <sys/unistd.h>
#include <task.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>

#include "Adafruit_NeoPixel.h"
#include "MPU6050.h"
#include "ServoMotor.h"
#include "Wire.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp32-hal-adc.h"
#include "esp32-hal-gpio.h"
#include "esp32-hal-log.h"
#include "esp_err.h"
#include "esp_sleep.h"
#include "pins_arduino.h"
#include "soc/gpio_num.h"

constexpr int kMPU6050SDA = 5;
constexpr int kMPU6050SCL = 6;
constexpr int kBattVoltagePin = 7;
constexpr int kSleepPin = 8;

constexpr int kMotor1PwmPin = 12;
constexpr int kMotor1FwdPin = 13;
constexpr int kMotor1RevPin = RX;
constexpr int kMotor2PwmPin = 11;
constexpr int kMotor2FwdPin = 10;
constexpr int kMotor2RevPin = 9;

constexpr int kNeoPixelPin = 21;
constexpr int kNeoPixelCount = 1;
static Adafruit_NeoPixel neo_pixel(kNeoPixelCount, kNeoPixelPin,
                                   NEO_RGB + NEO_KHZ800);

constexpr float kBattVoltageScale = 3.3f;
constexpr float kDividerRatio = 1.0f / 3.0f;

std::unique_ptr<ServoMotor> motor1;
std::unique_ptr<ServoMotor> motor2;
std::unique_ptr<MPU6050> mpu;

static volatile bool sleep_soon = false;
void IRAM_ATTR SleepPinISR() { sleep_soon = true; }

void GoToSleep() {
  motor1->DeepSleepPrepare();
  motor2->DeepSleepPrepare();

  neo_pixel.clear();
  neo_pixel.show();

  const auto pin = static_cast<gpio_num_t>(kSleepPin);
  pinMode(pin, INPUT_PULLUP);
  do {
    delay(100);  // Debounce, effectively.
    printf("Debouncing sleep pin\n");
  } while (!digitalRead(pin));
  delay(100);

  // We need to respect all holds, particularly on the motor pins,
  // while we are asleep.
  gpio_deep_sleep_hold_en();

  rtc_gpio_init(pin);
  rtc_gpio_pullup_en(pin);
  esp_sleep_enable_ext0_wakeup(pin, 0);
  int start = millis();
  do {
    printf("Will sleep imminently!\n");
    delay(100);
  } while (millis() - start < 1000);
  esp_deep_sleep_start();
}

void WakeUp() {
  rtc_gpio_deinit(static_cast<gpio_num_t>(kSleepPin));
  motor1->DeepSleepResume();
  motor2->DeepSleepResume();

  pinMode(kSleepPin, INPUT_PULLUP);
  while (!digitalRead(kSleepPin)) {
    printf("Debouncing sleep pin!\n");
    delay(100);
  }
  delay(50);

  printf("Wakeup done!\n");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  neo_pixel.setBrightness(0.2 * 255);

  pinMode(kBattVoltagePin, INPUT);
  analogSetAttenuation(ADC_11db);

  // Set up the motors.
  ServoMotor::Config config = {
      .pulse_pin = 1,
      .control_pin = 2,
      .pwm_pin = kMotor1PwmPin,
      .fwd_pin = kMotor1FwdPin,
      .rev_pin = kMotor1RevPin,
  };
  motor1 = ServoMotor::Create(config);
  motor1->SetDuty(.2 * 255);

  ServoMotor::Config config2 = {
      .pulse_pin = 3,
      .control_pin = 4,
      .pwm_pin = kMotor2PwmPin,
      .fwd_pin = kMotor2FwdPin,
      .rev_pin = kMotor2RevPin,
  };
  motor2 = ServoMotor::Create(config2);
  motor2->SetDuty(0);

  WakeUp();

  if (!Wire.begin(kMPU6050SDA, kMPU6050SCL)) {
    log_e("Failed to initialize I2C");
  } else {
    log_i("I2C initialized");
    mpu = std::make_unique<MPU6050>(MPU6050_DEFAULT_ADDRESS, &Wire);
    mpu->initialize();
    mpu->setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
  }

  if (!(esp_sleep_get_wakeup_causes() & BIT(ESP_SLEEP_WAKEUP_EXT0))) {
    // Any wakeup cause besides EXT0 means that we didn't wake up from
    // someone pressing the button. Let's go directly to sleep and ask them
    // to press the button before we actually wake up!
    printf("Woke for cause other than button press, sleeping!\n");
    GoToSleep();
  }

  attachInterrupt(kSleepPin, SleepPinISR, FALLING);
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

void loop() {
  constexpr float kDt = 0.002f;  // 2ms = 500Hz
  constexpr float kAlpha = 0.98f;
  constexpr float kRadToDeg = 180.0f / 3.14159265f;

  // MPU6050 Defaults
  constexpr float kAccelScale = 16384.0f;
  constexpr float kGyroScale = 32.8f;

  float angle_x = 0;
  float angle_y = 0;

  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t frequency = pdMS_TO_TICKS(2);

  int print_counter = 0;
  int low_voltage_counter = 0;

  while (true) {
    if (sleep_soon) {
      GoToSleep();
      break;
    }

    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    mpu->getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

    // Convert to physical units
    float ax = ax_raw / kAccelScale;
    float ay = ay_raw / kAccelScale;
    float az = az_raw / kAccelScale;
    float gx = gx_raw / kGyroScale;
    float gy = gy_raw / kGyroScale;
    // gz not used for lean estimation of X and Y in this simple filter

    float acc_angle_x = atan2(ay, az) * kRadToDeg;
    float acc_angle_y = atan2(-ax, sqrt(ay * ay + az * az)) * kRadToDeg;
    angle_x = kAlpha * (angle_x + gx * kDt) + (1.0f - kAlpha) * acc_angle_x;
    angle_y = kAlpha * (angle_y + gy * kDt) + (1.0f - kAlpha) * acc_angle_y;

    // Linearly interpolate between red and green depending on the distance from
    // pointing staight up. Full red at 10 degrees.
    const float total_angle = sqrt(angle_x * angle_x + angle_y * angle_y);
    const float lerp = std::min(1.0f, total_angle / 10.0f);
    const uint32_t color = neo_pixel.Color(lerp * 255, (1.0f - lerp) * 255, 0);
    neo_pixel.setPixelColor(0, color);
    neo_pixel.show();

    // Print every 100ms (50 iterations of 2ms)
    print_counter++;
    if (print_counter >= 200) {
      int batt_millivolts = BatteryMillivolts();
      if (batt_millivolts < 6800) {
        if (++low_voltage_counter > 100) {
          printf("Battery voltage critical.\n");
          GoToSleep();
        }
      } else {
        low_voltage_counter = 0;
      }

      printf(">speed:%f\n", motor1->GetSpeed());
      printf(">angle_x:%f\n", angle_x);
      printf(">angle_y:%f\n", angle_y);
      printf(">batt_millivoltage:%d\n", batt_millivolts);
      print_counter = 0;
    }

    xTaskDelayUntil(&last_wake_time, frequency);
  }
}
