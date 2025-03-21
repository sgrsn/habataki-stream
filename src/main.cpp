#include <Arduino.h>
#include <sstream>
#include <iostream>
#include "i2cmaster.hpp"
#include "i2c_register.h"

I2CMaster i2cMaster;

struct ControlData
{
  int X;
  int Y;
  int slider;
  int start;
};

ControlData controlData;

void setup() 
{
  // シリアル通信の初期化
  Serial.begin(115200);
  Serial.println("Teensy 4.1 I2C Master with Register Access");  
  
  // I2Cマスターの初期化
  i2cMaster.init();

  // ESP32が準備完了するまで待つ
  int esp32Ready = false;
  while(!esp32Ready)
  {
    Serial.println("Waiting for ESP32 to be ready...");
    i2cMaster.readRegister(ESP32_I2C_ADDR, ESP32_IS_READY_REG, esp32Ready);
    delay(5000);
  }
}

void loop() 
{
  uint32_t now = millis();

  i2cMaster.readRegister(ESP32_I2C_ADDR, MODEM_JOYSTICK_X_REG,  controlData.X);
  i2cMaster.readRegister(ESP32_I2C_ADDR, MODEM_JOYSTICK_Y_REG,  controlData.Y);
  i2cMaster.readRegister(ESP32_I2C_ADDR, MODEM_SLIDER_REG,      controlData.slider);
  i2cMaster.readRegister(ESP32_I2C_ADDR, MODEM_START_STOP_REG,  controlData.start);
  
  std::cout << "X: " << controlData.X << " Y: " << controlData.Y << " Slider: " << controlData.slider << " Start: " << controlData.start << std::endl;

  delay(50);
}
