#pragma once
#include <string>

const uint8_t ESC_PIN = 33;
const uint8_t SERVO1_PIN = 36;
const uint8_t SERVO2_PIN = 37;

const float ESC_ACCEL = 50.0f;
const float ESC_STOP_ACCEL = 100.0f;

std::string VN200_COMPORT = "Serial2";

const uint32_t I2C_RATE_US = 10 * 1e3;
const uint32_t IMU_RATE_US = 100 * 1e3;

const float GEAR_RATIO = 7.0f;

struct ControlData
{
  int X;
  int Y;
  int slider;
  int start;
};