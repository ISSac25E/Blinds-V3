//MPU5060_ESP REV 1.0.0
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

    MPU6050_ESP(uint8_t PowerPin);

    void Run();
    void CalcGyroOffset(uint16_t Times);
    void CalcAccelOffset(uint16_t Times);
    void PowerOn();
    void PowerOff();
    void GyroOffSet(int16_t X_OffSet, int16_t Y_OffSet, int16_t Z_OffSet, bool OffSet_EN);
    void AccelOffSet(int16_t X_OffSet, int16_t Y_OffSet, int16_t Z_OffSet, bool OffSet_EN);
    bool Error() {
      return _MPU_Error;
    };

    //Raw Gyro, Accel, Temp Vals:
    int16_t RawGyro[3] = {0, 0, 0}; //X,Y,Z
    int16_t RawAccel[3] = {0, 0, 0}; //X,Y,Z
    uint16_t RawTemp = 0;

  private:
    int16_t _GyroOffset[3] = {0, 0, 0}; //X,Y,Z
    bool _GyroOffset_EN = false;

    int16_t _AccelOffset[3] = {0, 0, 0}; //X,Y,Z
    bool _AccelOffset_EN = false;

    uint8_t _DataCompareCount;

    int16_t _PrevRawGyro[3];
    int16_t _PrevRawAccel[3];

    uint8_t _PowerPin;
    bool _MPU_Power = false;
    bool _MPU_PowerCycle = false;
    uint32_t _MPU_PowerCycleTimer;

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

MPU6050_ESP::MPU6050_ESP(uint8_t PowerPin) {
  _PowerPin = PowerPin;
  this->PowerOn();
  Wire.begin();
}

void MPU6050_ESP::PowerOn() {
  digitalWrite(_PowerPin, HIGH);
  pinMode(_PowerPin, OUTPUT);
  _MPU_Power = true;
}

void  MPU6050_ESP::PowerOff() {
  digitalWrite(_PowerPin, LOW);
  pinMode(_PowerPin, INPUT);
  _MPU_Power = false;
}

void MPU6050_ESP::GyroOffSet(int16_t X_OffSet, int16_t Y_OffSet, int16_t Z_OffSet, bool OffSet_EN) {
  _GyroOffset[0] = X_OffSet;
  _GyroOffset[1] = Y_OffSet;
  _GyroOffset[2] = Z_OffSet;
  _GyroOffset_EN = OffSet_EN;
};

void MPU6050_ESP::AccelOffSet(int16_t X_OffSet, int16_t Y_OffSet, int16_t Z_OffSet, bool OffSet_EN) {
  _AccelOffset[0] = X_OffSet;
  _AccelOffset[1] = Y_OffSet;
  _AccelOffset[2] = Z_OffSet;
  _AccelOffset_EN = OffSet_EN;
};

void MPU6050_ESP::CalcGyroOffset(uint16_t Times) {
  int32_t GyroOffSetHold[3] = {0, 0, 0};
  for (uint16_t X = 0; X < Times; X++) {
    //If MPU Dissconnects and Reconnects, This will ensure it will keep working:
    this->_WriteMPU6050(MPU6050_SMPLRT_DIV, 0x00);
    this->_WriteMPU6050(MPU6050_CONFIG, 0x00);
    this->_WriteMPU6050(MPU6050_GYRO_CONFIG, 0x08);
    this->_WriteMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
    this->_WriteMPU6050(MPU6050_PWR_MGMT_1, 0x01);
    this->_WriteMPU6050(MPU6050_FIFO_EN, 0xFF);

    //Collect the Raw Data:
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); //Start of Raw Data Reg, Accel X,Y,Z TEMP, Gyro X,Y,Z: H-L
    Wire.endTransmission(); //default True, Release Line After Transmission
    Wire.requestFrom(MPU6050_ADDR, 14);
    for (volatile uint8_t Y = 0; Y < 4; Y++) {
      Wire.read();
      Wire.read();
    }
    for (volatile uint8_t Y = 0; Y < 3; Y++) {
      GyroOffSetHold[Y] += ((Wire.read() << 8) | Wire.read());
    }
  }
  for (uint8_t X = 0; X < 3; X++) {
    GyroOffSetHold[X] /= Times;
    _GyroOffset[X] = GyroOffSetHold[X];
  }
  _GyroOffset_EN = true;
}

void MPU6050_ESP::CalcAccelOffset(uint16_t Times) {
  int32_t AccelOffSetHold[3] = {0, 0, 0};
  for (volatile uint16_t X = 0; X < Times; X++) {
    //If MPU Dissconnects and Reconnects, This will ensure it will keep working:
    this->_WriteMPU6050(MPU6050_SMPLRT_DIV, 0x00);
    this->_WriteMPU6050(MPU6050_CONFIG, 0x00);
    this->_WriteMPU6050(MPU6050_GYRO_CONFIG, 0x08);
    this->_WriteMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
    this->_WriteMPU6050(MPU6050_PWR_MGMT_1, 0x01);
    this->_WriteMPU6050(MPU6050_FIFO_EN, 0xFF);

    //Collect the Raw Data:
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); //Start of Raw Data Reg, Accel X,Y,Z TEMP, Gyro X,Y,Z: H-L
    Wire.endTransmission(); //default True, Release Line After Transmission
    Wire.requestFrom(MPU6050_ADDR, 14);
    for (volatile uint8_t Y = 0; Y < 3; Y++) {
      AccelOffSetHold[Y] += ((Wire.read() << 8) | Wire.read());
    }
    for (volatile uint8_t Y = 0; Y < 4; Y++) {
      Wire.read();
      Wire.read();
    }
    yield();
  }
  for (volatile uint8_t X = 0; X < 3; X++) {
    AccelOffSetHold[X] /= Times;
    _AccelOffset[X] = AccelOffSetHold[X];
  }
  _AccelOffset_EN = true;
}

void MPU6050_ESP::Run() {
  if (_MPU_Power) {
    //If MPU Dissconnects and Reconnects, This will ensure it will keep working:
    this->_WriteMPU6050(MPU6050_SMPLRT_DIV, 0x00);
    this->_WriteMPU6050(MPU6050_CONFIG, 0x00);
    this->_WriteMPU6050(MPU6050_GYRO_CONFIG, 0x08);
    this->_WriteMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
    this->_WriteMPU6050(MPU6050_PWR_MGMT_1, 0x01);
    this->_WriteMPU6050(MPU6050_FIFO_EN, 0xFF);

    //Collect the Raw Data:
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); //Start of Raw Data Reg, Accel X,Y,Z TEMP, Gyro X,Y,Z: H-L
    Wire.endTransmission(); //default True, Release Line After Transmission
    Wire.requestFrom(MPU6050_ADDR, 14);
    for (uint8_t X = 0; X < 3; X++) {
      RawAccel[X] = ((Wire.read() << 8) | Wire.read());
      if (_AccelOffset_EN) RawAccel[X] -= _AccelOffset[X];
    }
    RawTemp = ((Wire.read() << 8) | Wire.read());
    for (uint8_t X = 0; X < 3; X++) {
      RawGyro[X] = ((Wire.read() << 8) | Wire.read());
      if (_GyroOffset_EN) RawGyro[X] -= _GyroOffset[X];
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
