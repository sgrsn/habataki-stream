#pragma once
#include <Arduino.h>
#include <Wire.h>

// レジスタの数
#define NUM_REGISTERS 8
#define i2c_data_type_t int
#define I2C_DATA_SIZE sizeof(i2c_data_type_t)

class I2CMaster
{
 private:
  // 各レジスタの現在の値を保持する配列
  i2c_data_type_t registers[NUM_REGISTERS] = {0};

 public:
  I2CMaster() {}

  void init() 
  { 
    // I2Cマスターの初期化
    Wire.begin();  // マスターとして初期化
  }

  // 指定したレジスタの値を読み出す関数
  bool readRegister(uint8_t slave_addr, byte regAddr, i2c_data_type_t& value) 
  {
    if (regAddr >= NUM_REGISTERS) {
      return false;
    }
    
    // まずレジスタアドレスを送信
    Wire.beginTransmission(slave_addr);
    Wire.write(regAddr);
    Wire.endTransmission();
    
    // 該当レジスタの値（4バイト）を要求
    Wire.requestFrom(slave_addr, I2C_DATA_SIZE);  // int型は4バイト
    
    // 受信したバイトからint型の値を組み立て
    if (Wire.available() >= I2C_DATA_SIZE) {
      byte buffer[I2C_DATA_SIZE];
      for (int i = 0; i < I2C_DATA_SIZE; i++) {
        buffer[i] = Wire.read();
      }
      
      value = 0;
      // バイト配列からint型に変換
      for (int i = 0; i < I2C_DATA_SIZE; i++) {
        value |= buffer[i] << (8 * (I2C_DATA_SIZE - i - 1));
      }
      
    } else {
      return false;
    }
    
    return true;
  }

  // 指定したレジスタに値を書き込む関数
  void writeRegister(uint8_t slave_addr, byte regAddr, i2c_data_type_t value) 
  {
    if (regAddr >= NUM_REGISTERS) {
      return;  // 無効なレジスタアドレス
    }
    
    Wire.beginTransmission(slave_addr);
    Wire.write(regAddr);  // レジスタアドレスを送信
    
    // int型の値をバイト単位に分解して送信
    byte buffer[I2C_DATA_SIZE];
    for (int i = 0; i < I2C_DATA_SIZE; i++) {
      buffer[i] = (value >> (8 * (I2C_DATA_SIZE - i - 1))) & 0xFF;
    }
    for (int i = 0; i < I2C_DATA_SIZE; i++) {
      Wire.write(buffer[i]);
    }
    
    Wire.endTransmission();
  }

};