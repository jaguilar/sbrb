idf_component_register(SRCS "main.cpp" "PIDController.cpp" "ServoMotor.cpp"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES arduino Adafruit_NeoPixel_ESPIDF mpu6050_ESPIDF jesl esp_wifi)

if(CONFIG_BT_MODE_PAD)
  message(STATUS "linking bluepad32")
  target_link_libraries(${COMPONENT_LIB} idf::ricardoquesada__bluepad32)
elseif(CONFIG_BT_MODE_TUNE)
message(STATUS "linking nimble-arduino-espidf")
  target_link_libraries(${COMPONENT_LIB} idf::NimBLE-Arduino_ESPIDF)
endif()
