//MPU5060_ESP REV 1.0.2
//.h
#ifndef MPU6050_ESP_h
#define MPU6050_ESP_h

#include "Arduino.h"
#include "Wire.h"

//Register/Values for MPU5060
#define MPU6050_ADDR         0x68
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_TEMP_H       0x41
#define MPU6050_TEMP_L       0x42
#define MPU6050_FIFO_EN      0x23

class MPU6050_ESP {
  public:

    void Init(uint8_t PowerPin);

    void Run();
    void PowerOn();
    void PowerOff();
    bool Error() {
      return _MPU_Error;
    };

    //Raw Gyro, Accel, Temp Vals:
    int16_t RawGyro[3] = {0, 0, 0}; //X,Y,Z
    int16_t RawAccel[3] = {0, 0, 0}; //X,Y,Z
    uint16_t RawTemp = 0;

  private:
    //If we are Init
    bool _Init = false;

    //We only want to periodically configure the MPU:
    uint32_t _MPU_ConfigTimer = 0;

    //These Val are used to check if MPU is functioning properly:
    uint8_t _DataCompareCount = 0;
    int16_t _PrevRawGyro[3];
    int16_t _PrevRawAccel[3];

    //These are used to manage the power to the MPU
    uint8_t _PowerPin;
    bool _MPU_Power = false;
    bool _MPU_PowerCycle = false;
    uint32_t _MPU_PowerCycleTimer;

    //High if error with MPU
    bool _MPU_Error = true;

    //Use this to write to Individual MPU Registers
    void _WriteMPU6050(byte reg, byte data) {
      Wire.beginTransmission(MPU6050_ADDR);
      Wire.write(reg);
      Wire.write(data);
      Wire.endTransmission();
    }
};

//.cpp
//#include "MPU5060_ESP.h"
//#include "Arduino.h"

void MPU6050_ESP::Init(uint8_t PowerPin) {
  _Init = true;
  _DataCompareCount = 0;
  _MPU_Error = true;
  _PowerPin = PowerPin;
  this->PowerOn();
  Wire.begin();
  {
    _MPU_ConfigTimer = millis();
    this->_WriteMPU6050(MPU6050_SMPLRT_DIV, 0x00);
    this->_WriteMPU6050(MPU6050_CONFIG, 0x00);
    this->_WriteMPU6050(MPU6050_GYRO_CONFIG, 0x08);
    this->_WriteMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
    this->_WriteMPU6050(MPU6050_PWR_MGMT_1, 0x01);
    this->_WriteMPU6050(MPU6050_FIFO_EN, 0xFF);
  }
}

void MPU6050_ESP::PowerOn() {
  if (_Init) {
    digitalWrite(_PowerPin, HIGH);
    pinMode(_PowerPin, OUTPUT);
    _MPU_Power = true;
  }
}

void  MPU6050_ESP::PowerOff() {
  if (_Init) {
    digitalWrite(_PowerPin, LOW);
    pinMode(_PowerPin, INPUT);
    _MPU_Power = false;
  }
}

void MPU6050_ESP::Run() {
  if (_Init) {
    if (_MPU_Power) {
      //If MPU Dissconnects and Reconnects, This will ensure it will keep working:
      //We only do this 10 times a second:
      if (millis() - _MPU_ConfigTimer >= 100) {
        _MPU_ConfigTimer = millis();
        this->_WriteMPU6050(MPU6050_SMPLRT_DIV, 0x00);
        this->_WriteMPU6050(MPU6050_CONFIG, 0x00);
        this->_WriteMPU6050(MPU6050_GYRO_CONFIG, 0x08);
        this->_WriteMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
        this->_WriteMPU6050(MPU6050_PWR_MGMT_1, 0x01);
        this->_WriteMPU6050(MPU6050_FIFO_EN, 0xFF);
      }

      //Collect the Raw Data:
      Wire.beginTransmission(MPU6050_ADDR);
      Wire.write(0x3B); //Start of Raw Data Reg, Accel X,Y,Z TEMP, Gyro X,Y,Z: H-L
      Wire.endTransmission(); //default True, Release Line After Transmission
      Wire.requestFrom(MPU6050_ADDR, 14);
      for (uint8_t X = 0; X < 3; X++) {
        RawAccel[X] = ((Wire.read() << 8) | Wire.read());
      }
      RawTemp = ((Wire.read() << 8) | Wire.read());
      for (uint8_t X = 0; X < 3; X++) {
        RawGyro[X] = ((Wire.read() << 8) | Wire.read());
      }
      bool CompareMatch = true;
      for (uint8_t X = 0; X < 3; X++) {
        if (_PrevRawGyro[X] != RawGyro[X]) CompareMatch = false;
        if (_PrevRawAccel[X] != RawAccel[X]) CompareMatch = false;
        _PrevRawGyro[X] = RawGyro[X];
        _PrevRawAccel[X] = RawAccel[X];
      }
      if (CompareMatch) {
        _DataCompareCount++;
      }
      else {
        _DataCompareCount = 0;
        _MPU_Error = false;
      }
      if (_DataCompareCount >= 10) {
        _MPU_Error = true;
        _MPU_PowerCycle = true;
        this->PowerOff();
        _MPU_PowerCycleTimer = millis();
      }
    }
    else {
      if (_MPU_PowerCycle) {
        if (millis() - _MPU_PowerCycleTimer >= 100) {
          this->PowerOn();
          _MPU_PowerCycle = false;
          _DataCompareCount = 0;
        }
      }
    }
  }
  else _MPU_Error = true;
}

//Undefine All Defined Values to prevent problems in future Code:
#undef MPU6050_ADDR
#undef MPU6050_SMPLRT_DIV
#undef MPU6050_CONFIG
#undef MPU6050_GYRO_CONFIG
#undef MPU6050_ACCEL_CONFIG
#undef MPU6050_PWR_MGMT_1
#undef MPU6050_TEMP_H
#undef MPU6050_TEMP_L
#undef MPU6050_FIFO_EN

#endif
