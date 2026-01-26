#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>

#include "MPU6050.h"
#include "ServoMotor.h"
#include "Wire.h"
#include "esp32-hal-log.h"

constexpr int kMPU6050SCL = 4;
constexpr int kMPU6050SDA = 5;

std::unique_ptr<ServoMotor> motor;
std::unique_ptr<MPU6050> mpu;

void setup() {
  Serial.begin(115200);

  delay(10000);

// Set up the motor.
#if 0
  ServoMotor::Config config = {
      .pulse_pin = 15,
      .control_pin = 16,
      .pwm_pin = 42,
      .fwd_pin = 40,
      .rev_pin = 41,
  };
  motor = ServoMotor::Create(config);
#endif

  if (!Wire.begin(kMPU6050SDA, kMPU6050SCL)) {
    log_e("Failed to initialize I2C");
    return;
  }
  log_i("I2C initialized");
  mpu = std::make_unique<MPU6050>(MPU6050_DEFAULT_ADDRESS, &Wire);
  mpu->initialize();
  mpu->setFullScaleGyroRange(MPU6050_GYRO_FS_1000);

  delay(100);
  // Ensure we are in the correct range for the calculations below.
  // 2g range, 250 deg/s range.
}

void loop() {
  if (mpu == nullptr) {
    log_e("MPU6050 not initialized");
    return;
  }
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

  while (true) {
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

    // Print every 100ms (50 iterations of 2ms)
    print_counter++;
    if (print_counter >= 50) {
      printf(">angle_x:%f\n", angle_x);
      printf(">angle_y:%f\n", angle_y);
      print_counter = 0;
    }

    xTaskDelayUntil(&last_wake_time, frequency);
  }
}
