#pragma once
#include <string>

const uint8_t ESC_PIN = 33;
const uint8_t SERVO1_PIN = 36;
const uint8_t SERVO2_PIN = 37;

const float ESC_ACCEL = 50.0f;
const float ESC_STOP_ACCEL = 100.0f;

std::string VN200_COMPORT = "Serial2";

const uint32_t I2C_RATE_US = 20 * 1e3;
const uint32_t IMU_RATE_US = 100;

const char* CSV_FILE_NAME = "data.csv";

const float GEAR_RATIO = 7.0f;

// 1回のロギングデータサイズ
#define DATA_SIZE 100
// リングバッファの長さ
#define RING_BUFFER_LENGTH 256
#define FLUSH_INTERVAL 100

struct ControlData
{
  int X = 0;
  int Y = 0;
  int slider = 0;
  int start = 0;
};

// ユーザー定義のデータ構造体
struct SensorData {
  // IMU
  float quatX = 0.0f;
  float quatY = 0.0f;
  float quatZ = 0.0f;
  float quatW = 0.0f;

  // GNSS
  float latitude = 0.0f;
  float longitude = 0.0f;
  float altitude = 0.0f;
  int fix = 0;

  // Encoder
  float flapping_freq = 0.0f;

  // control
  ControlData controlData;

  // CSVへの書き込み方法を定義する
  void writeToCSV(Print& output) const 
  {
    output.print(quatX, 6);
    output.print(',');
    output.print(quatY, 6);
    output.print(',');
    output.print(quatZ, 6);
    output.print(',');
    output.print(quatW, 6);
    output.print(',');
    output.print(latitude, 6);
    output.print(',');
    output.print(longitude, 6);
    output.print(',');
    output.print(altitude, 6);
    output.print(',');
    output.print(fix);
    output.print(',');
    output.print(flapping_freq, 2);
    output.print(',');
    output.print(controlData.X);
    output.print(',');
    output.print(controlData.Y);
    output.print(',');
    output.print(controlData.slider);
    output.print(',');
    output.print(controlData.start);
  }
};