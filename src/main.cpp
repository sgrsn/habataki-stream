#include <Arduino.h>
#include <sstream>
#include <iostream>
#include <IntervalTimer.h>
#include "Interface/Sensor.hpp"
#include "AS5600.h"
#include "i2cmaster.hpp"
#include "i2c_register.h"
#include "RCInterface.hpp"
#include "SDLogger.h"
#include "config.h"


/*I2C devices*/
I2CMaster   i2cMaster;
AS5600      as5600;

/*RC devices*/
ESC         esc(ESC_PIN, 800, 2100, ESC_ACCEL);
RCServo     servo1(SERVO1_PIN, 60, 90, 120);
RCServo     servo2(SERVO2_PIN, 25, 75, 120);

/*VectorNav VN-200*/
VN::Sensor  sensor;

/*SDLogger*/
SDLogger<SensorData> logger(CSV_FILE_NAME, 256, 100);

// 最大4つのIntervalTimerを使える
IntervalTimer timer_i2c;
IntervalTimer timer_imu;

EdgeStatus status;

bool is_logging = false;
int start_logging = 0;
int file_count = 0;
std::string filename = "data";

static void I2CTimer()
{
  // ESP32から制御信号を受信
  ControlData controlData;
  i2cMaster.readRegister(ESP32_I2C_ADDR, MODEM_JOYSTICK_X_REG,  controlData.X);
  i2cMaster.readRegister(ESP32_I2C_ADDR, MODEM_JOYSTICK_Y_REG,  controlData.Y);
  i2cMaster.readRegister(ESP32_I2C_ADDR, MODEM_SLIDER_REG,      controlData.slider);
  i2cMaster.readRegister(ESP32_I2C_ADDR, MODEM_START_STOP_REG,  controlData.start);
  i2cMaster.readRegister(ESP32_I2C_ADDR, LOGGING_STATUS_REG,    start_logging);
  float deg_per_sec = as5600.getAngularSpeed();
  float flapping_freq = deg_per_sec / 360.0f / GEAR_RATIO;
  uint16_t angle = as5600.readAngle();

  // std::cout << "X: " << controlData.X << ", Y: " << controlData.Y << ", Slider: " << controlData.slider << ", Start: " << controlData.start << ", angle: " << angle << std::endl;

  // StatusをESP32に送信
  i2cMaster.writeRegister(ESP32_I2C_ADDR, TEENSY41_STATUS_REG, status);

  // ロギングデータの更新
  SensorData& currentData = logger.getCurrentData();
  currentData.controlData = controlData;
  currentData.flapping_freq = (float)angle;

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
  // std::cout << "I2CTimer Past: " << past << std::endl;
}

void imuTimer()
{
  uint32_t start = millis();
  // Configure binary output
  VN::Registers::System::BinaryOutput1 binaryOutput1Register;
  binaryOutput1Register.asyncMode = 3;
  binaryOutput1Register.rateDivisor = 40;
  binaryOutput1Register.common = 0x7FFF;
  
  const bool needsMoreData = sensor.processNextPacket();
  if (needsMoreData) 
  {
    sensor.loadMainBufferFromSerial();
  } 
  else 
  {
    auto nextMeasurement_maybe = sensor.getNextMeasurement(false);
    if (nextMeasurement_maybe->matchesMessage(binaryOutput1Register)) 
    {
      VN::InsStatus ins_status = nextMeasurement_maybe->ins.insStatus.value();
      if ( ins_status.gnssFix == 1 ) {
        status.gnss_fix = ErrorCode::SUCCESS;
      } else {
        status.gnss_fix = ErrorCode::FAILED;
      }
      VN::Lla lla = nextMeasurement_maybe->ins.posLla.value();

      // std::cout << "Fix: " << ins_status.gnssFix << std::endl;
      // std::cout << "Lat: " << lla.lat << ", Lon: " << lla.lon << ", Alt: " << lla.alt << std::endl;

      VN::Quat quat = nextMeasurement_maybe->attitude.quaternion.value();
      // std::cout << "Quat: " << quat.vector[0] << ", " << quat.vector[1] << ", " << quat.vector[2] << ", " << quat.scalar << std::endl;  
      SensorData& currentData = logger.getCurrentData();
      currentData.quatX = quat.vector[0];
      currentData.quatY = quat.vector[1];
      currentData.quatZ = quat.vector[2];
      currentData.quatW = quat.scalar;
      currentData.latitude = lla.lat;
      currentData.longitude = lla.lon;
      currentData.altitude = lla.alt;
    }
  }

  static uint32_t imulastTime = 0;
  uint32_t past = millis() - imulastTime;
  imulastTime = millis();
  //std::cout << "imuTimer Past: " << past << std::endl;
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
  Serial.println("Teensy 4.1 Setup");  

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
  Serial.println("Servo Setup Done");
 
  // I2Cの初期化
  Wire.begin();
  Serial.println("Wire Setup Done");

  // AS5600の初期化
  as5600.begin(255);
  as5600.setDirection(AS5600_CLOCK_WISE);

  // vectornavの初期化
  VN::Error latestError = sensor.connect(VN200_COMPORT, VN::Sensor::BaudRate::Baud115200);
  if (latestError != VN::Error::None) {
    Serial.println("vectornav autoConnect failed");
    // エラーをESP32に通知
    while(true)
    {
      status.vectornav = ErrorCode::FAILED;
      i2cMaster.writeRegister(ESP32_I2C_ADDR, TEENSY41_STATUS_REG, static_cast<int>(status));
      delay(500);
    }
  }
  Serial.println("Connected to VectorNav VN-200");
  delay(500);

  // Configure binary output
  VN::Registers::System::BinaryOutput1 binaryOutput1Register;
  binaryOutput1Register.asyncMode = 3;
  binaryOutput1Register.rateDivisor = 40;
  binaryOutput1Register.common = 0x7FFF;
  sensor.writeRegister(&binaryOutput1Register);
  delay(500);
  sensor.asyncOutputEnable(VN::AsyncOutputEnable::State::Enable);
  status.vectornav = ErrorCode::SUCCESS;
  Serial.println("Setup Done");

  if (!logger.begin()) 
  {
    Serial.println("Failed to initialize SDLogger");
    // エラーをESP32に通知
    status.sdcard = ErrorCode::FAILED;
    i2cMaster.writeRegister(ESP32_I2C_ADDR, TEENSY41_STATUS_REG, status);
    while (1) { /* エラー時は停止 */ }
  }

  // ESP32が準備完了するまで待つ
  int esp32Ready = false;
  while(!esp32Ready)
  {
    Serial.println("Waiting for ESP32 to be ready...");
    i2cMaster.readRegister(ESP32_I2C_ADDR, ESP32_IS_READY_REG, esp32Ready);
    delay(5000);
  }

  // ESP32に準備完了を通知
  i2cMaster.writeRegister(ESP32_I2C_ADDR, TEENSY41_STATUS_REG, 1);

  // タイマー起動
  initializeTimer();

  Serial.println("Habataki Streamer is ready!");
}

void loop() {
  // SDLoggerにデータを書き込む
  if (start_logging == 1)
  {
    if (!is_logging) 
    {
      // 次のファイルを作成
      logger.setCsvHeader("timestamp_ms,quatX,quatY,quatZ,quatW,latitude,longitude,altitude,fix,flapping_freq,X,Y,slider,start");
        
      bool file_exists = true;
      std::string file;
      while(file_exists)
      {
        file = filename + std::to_string(file_count) + ".csv";
        file_exists = logger.fileExists(file.c_str());
        file_count++;
      }
      if (!logger.begin(file.c_str())) {
        Serial.println("Failed to initialize SDLogger");
        // エラーをESP32に通知
        status.sdcard = ErrorCode::FAILED;
        i2cMaster.writeRegister(ESP32_I2C_ADDR, TEENSY41_STATUS_REG, status);
        while (1) { /* エラー時は停止 */ }
      }
      status.sdcard = ErrorCode::SUCCESS;
      logger.startLogging(10000);
      Serial.println(millis());
    }

    // 更新
    logger.update();
    is_logging = true;
  }
  else if (is_logging)
  {
    // ファイル保存
    logger.close();
    is_logging = false;
    // タイマーを停止
    logger.stopLogging();
    Serial.println(millis());
  }
}