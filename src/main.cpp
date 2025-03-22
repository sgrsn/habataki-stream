#include <Arduino.h>
#include <sstream>
#include <iostream>
#include <IntervalTimer.h>
#include "Interface/Sensor.hpp"
#include "AS5600.h"
#include "i2cmaster.hpp"
#include "i2c_register.h"
#include "RCInterface.hpp"
#include "config.h"

/*I2C devices*/
I2CMaster   i2cMaster;
AS5600      as5600;
ESC         esc(ESC_PIN, 800, 2100, ESC_ACCEL);
RCServo     servo1(SERVO1_PIN, 60, 90, 120);
RCServo     servo2(SERVO2_PIN, 25, 75, 120);
VN::Sensor  sensor;

// 最大4つのIntervalTimerを使える
IntervalTimer timer_i2c;
IntervalTimer timer_imu;

int gnss_fix = 0;

static void I2CTimer()
{
  // ESP32から制御信号を受信
  ControlData controlData;
  i2cMaster.readRegister(ESP32_I2C_ADDR, MODEM_JOYSTICK_X_REG,  controlData.X);
  i2cMaster.readRegister(ESP32_I2C_ADDR, MODEM_JOYSTICK_Y_REG,  controlData.Y);
  i2cMaster.readRegister(ESP32_I2C_ADDR, MODEM_SLIDER_REG,      controlData.slider);
  i2cMaster.readRegister(ESP32_I2C_ADDR, MODEM_START_STOP_REG,  controlData.start);
  //float deg_per_sec = as5600.getAngularSpeed();
  //float flapping_freq = deg_per_sec / 360.0f / GEAR_RATIO;
  uint16_t angle = as5600.readAngle();

  // std::cout << "X: " << controlData.X << ", Y: " << controlData.Y << ", Slider: " << controlData.slider << ", Start: " << controlData.start << ", angle: " << angle << std::endl;

  // GNSSの状態をESP32に送信
  i2cMaster.writeRegister(ESP32_I2C_ADDR, GNSS_STATUS_REG, gnss_fix);

  // ESC, サーボの制御
  servo1.setPosition(controlData.X);
  servo2.setPosition(controlData.Y);
  if (controlData.start) {
    esc.setAccel(ESC_ACCEL);
    esc.setSpeed((float)controlData.slider);
  } else {
    esc.setAccel(ESC_STOP_ACCEL);
    esc.setSpeed(0);
  }
  esc.update();

  static uint32_t i2clastTime = 0;
  uint32_t past = millis() - i2clastTime;
  i2clastTime = millis();
  std::cout << "I2CTimer Past: " << past << std::endl;
}

static void imuTimer()
{
  uint32_t start = millis();
  // Configure binary output
  VN::Registers::System::BinaryOutput1 binaryOutput1Register;
  binaryOutput1Register.asyncMode = 3;
  binaryOutput1Register.rateDivisor = 40;
  binaryOutput1Register.common = 0x7FFF;
  
  auto compositeData = sensor.getNextMeasurement();
  // if (!compositeData) continue;
  
  if (compositeData->matchesMessage(binaryOutput1Register)) {
    VN::Vec3f accel = compositeData->imu.accel.value();
    std::cout << "Accel: " << accel[0] << ", " << accel[1] << ", " << accel[2] << std::endl;
  }

  std::cout << "imuTimer take: " << millis() - start << std::endl;

  static uint32_t imulastTime = 0;
  uint32_t past = millis() - imulastTime;
  imulastTime = millis();
  std::cout << "imuTimer Past: " << past << std::endl;
}


void initializeTimer()
{
  timer_i2c.begin(I2CTimer, I2C_RATE_US);
  timer_imu.begin(imuTimer, IMU_RATE_US);
}

void setup() 
{
  // シリアル通信の初期化
  Serial.begin(115200);
  Serial.println("Teensy 4.1 I2C Master with Register Access");  

  // ESCの初期化
  esc.begin();
  esc.setAccel(ESC_STOP_ACCEL);
  esc.setSpeed(0);
  esc.update();

  // サーボの初期化
  servo1.begin();
  servo2.begin();
  servo1.setPosition(0);
  servo2.setPosition(0);
  
  // I2Cの初期化
  Wire.begin();

  // AS5600の初期化
  as5600.begin(255);
  as5600.setDirection(AS5600_CLOCK_WISE);

  // vectornavの初期化
  VN::Error latestError = sensor.autoConnect(VN200_COMPORT);

  // Configure binary output
  VN::Registers::System::BinaryOutput1 binaryOutput1Register;
  binaryOutput1Register.asyncMode = 3;
  binaryOutput1Register.rateDivisor = 40;
  binaryOutput1Register.common = 0x7FFF;
  sensor.writeRegister(&binaryOutput1Register);

  // ESP32が準備完了するまで待つ
  int esp32Ready = false;
  while(!esp32Ready)
  {
    Serial.println("Waiting for ESP32 to be ready...");
    i2cMaster.readRegister(ESP32_I2C_ADDR, ESP32_IS_READY_REG, esp32Ready);
    delay(5000);
  }

  initializeTimer();
}

void loop() 
{
  // 何もしない
}
