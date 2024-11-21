//VACUUM_GYRO_INTER REV 1.0.1
//.h
#ifndef VACUUM_GYRO_INTER_h
#define VACUUM_GYRO_INTER_h

#include "Arduino.h"
#include "MPU6050_ESP.h"

#ifndef VACUUM_GYRO_INTER_ACCEL_ORNT_FACT                 //FACTOR FOR ANGLE DETECTION, DEFAULT GOOD
#define VACUUM_GYRO_INTER_ACCEL_ORNT_FACT 500
#endif

#ifndef VACUUM_GYRO_INTER_ACCEL_ORNT_ANG                  //ANGLE AT WHICH ACCEL WILL ASSUME UPSIDE DOWN
#define VACUUM_GYRO_INTER_ACCEL_ORNT_ANG 4000
#endif

#ifndef VACUUM_GYRO_INTER_ACCEL_VIB_COMP_FACT              //COMP FACTOR FOR ACCEL VIBRATION DETECTION, DEFAULT IS ALREADY GOOD
#define VACUUM_GYRO_INTER_ACCEL_VIB_COMP_FACT 50
#endif

#ifndef VACUUM_GYRO_INTER_ACCEL_VIB_ABS_COMP_FACT          //COMP FACTOR FOR ACCEL VIBRATION DETECTION, DEFAULT IS ALREADY GOOD
#define VACUUM_GYRO_INTER_ACCEL_VIB_ABS_COMP_FACT 10
#endif

#ifndef VACUUM_GYRO_INTER_ACCEL_VIB_ABS_MV_AVG_THRES       //VIBRATION TRIGGER THRESHOLD
#define VACUUM_GYRO_INTER_ACCEL_VIB_ABS_MV_AVG_THRES 300
#endif

#ifndef VACUUM_GYRO_INTER_ACCEL_VIB_ON_CNT_THRES           //ON COUNT TRIGGER THRESHOLD
#define VACUUM_GYRO_INTER_ACCEL_VIB_ON_CNT_THRES 200
#endif

#ifndef VACUUM_GYRO_INTER_ACCEL_VIB_OFF_CNT_THRES          //OFF COUNT TRIGGER THRESHOLD
#define VACUUM_GYRO_INTER_ACCEL_VIB_OFF_CNT_THRES -600
#endif

class VAC_GYRO {
  public:

    VAC_GYRO(uint8_t Pin) {
      _MPU.Init(Pin);
    };

    //    void SetAccelOffset(int16_t X, int16_t Y, int16_t Z);
    //    void SetGyroOffset(int16_t X, int16_t Y, int16_t Z);

    //Calculates MPU Offsets, checks for Errors, returns True if Vac Moving:
    bool Run();

    //Return weather MPU is having a problem:
    bool Error() {
      return _MPU.Error();
    }

    //Offsets for Gyro and Accel:
    int16_t _AccelOffset[3] = {0, 0, 0};
    int16_t _GyroOffset[3] = {0, 0, 0};

    //To know if we are calculating offsets
    //This is also How you Trigger Calc Externally:
    bool CalcOffsets = false;

  private:
    //Create private Object For MPU:
    MPU6050_ESP _MPU;

};

//.cpp
//#include "VACUUM_GYRO_INTER.h"
//#include "Arduino.h"

//void VAC_GYRO::SetAccelOffset(int16_t X, int16_t Y, int16_t Z) {
//  _AccelOffset[0] = X;
//  _AccelOffset[1] = Y;
//  _AccelOffset[2] = Z;
//}
//void VAC_GYRO::SetGyroOffset(int16_t X, int16_t Y, int16_t Z) {
//  _GyroOffset[0] = X;
//  _GyroOffset[1] = Y;
//  _GyroOffset[2] = Z;
//}

bool VAC_GYRO::Run() {
  static int32_t CompAccel[3] = {0, 0, 0};
  static int32_t CompGyro[3] = {0, 0, 0};

  _MPU.Run();
  if (!_MPU.Error()) {
    bool OrientComp = true;
    //Calc Orientation
    {
      static int32_t CompAccelOrient[3] = {0, 0, 0};
      //We are using 500 because we only need orientation, not movment:
      for (uint8_t X = 0; X < 3; X++) {
        CompAccelOrient[X] -= (CompAccelOrient[X] / VACUUM_GYRO_INTER_ACCEL_ORNT_FACT);
        CompAccelOrient[X] += (_MPU.RawAccel[X] - _AccelOffset[X]);
        if (abs(CompAccelOrient[X] / VACUUM_GYRO_INTER_ACCEL_ORNT_FACT) > VACUUM_GYRO_INTER_ACCEL_ORNT_ANG) OrientComp = false;
      }
    }
    //Calc Accel Vibration and if its moving:
    bool VibrateComp;
    {
      static int32_t CompAccelAvg[3] = {0, 0, 0};
      static int32_t AccelAbsAvg[3] = {0, 0, 0};
      static int32_t PeakCount = 0;
      static bool MoveState = false;
      {
        CompAccelAvg[0] -= CompAccelAvg[0] / VACUUM_GYRO_INTER_ACCEL_VIB_COMP_FACT;
        CompAccelAvg[1] -= CompAccelAvg[1] / VACUUM_GYRO_INTER_ACCEL_VIB_COMP_FACT;
        CompAccelAvg[2] -= CompAccelAvg[2] / VACUUM_GYRO_INTER_ACCEL_VIB_COMP_FACT;

        CompAccelAvg[0] += _MPU.RawAccel[0];
        CompAccelAvg[1] += _MPU.RawAccel[1];
        CompAccelAvg[2] += _MPU.RawAccel[2];
      }
      {
        AccelAbsAvg[0] -= AccelAbsAvg[0] / VACUUM_GYRO_INTER_ACCEL_VIB_ABS_COMP_FACT;
        AccelAbsAvg[1] -= AccelAbsAvg[1] / VACUUM_GYRO_INTER_ACCEL_VIB_ABS_COMP_FACT;
        AccelAbsAvg[2] -= AccelAbsAvg[2] / VACUUM_GYRO_INTER_ACCEL_VIB_ABS_COMP_FACT;

        AccelAbsAvg[0] += abs((CompAccelAvg[0] / VACUUM_GYRO_INTER_ACCEL_VIB_COMP_FACT) - _MPU.RawAccel[0]);
        AccelAbsAvg[1] += abs((CompAccelAvg[1] / VACUUM_GYRO_INTER_ACCEL_VIB_COMP_FACT) - _MPU.RawAccel[1]);
        AccelAbsAvg[2] += abs((CompAccelAvg[2] / VACUUM_GYRO_INTER_ACCEL_VIB_COMP_FACT) - _MPU.RawAccel[2]);
      }
      int32_t MoveAvg = (((AccelAbsAvg[0] / VACUUM_GYRO_INTER_ACCEL_VIB_ABS_COMP_FACT) +
                          (AccelAbsAvg[1] / VACUUM_GYRO_INTER_ACCEL_VIB_ABS_COMP_FACT) +
                          (AccelAbsAvg[2] / VACUUM_GYRO_INTER_ACCEL_VIB_ABS_COMP_FACT)) / 3);
      if (MoveAvg > VACUUM_GYRO_INTER_ACCEL_VIB_ABS_MV_AVG_THRES) {
        if (PeakCount < 0) PeakCount = 0;
        else if (PeakCount < 10000)PeakCount++;
      }
      else {
        if (PeakCount > 0)
          PeakCount = 0;
        else if (PeakCount > -10000) PeakCount--;
      }
      if (MoveState) {
        if (PeakCount < VACUUM_GYRO_INTER_ACCEL_VIB_OFF_CNT_THRES) {
          MoveState = false;
        }
      }
      else {
        if (PeakCount > VACUUM_GYRO_INTER_ACCEL_VIB_ON_CNT_THRES) {
          MoveState = true;
        }
      }
      VibrateComp = MoveState;
    }
    //Run Offset Calc Loop:
    if (CalcOffsets) {
      //Use for Calculating Offsets:
      static int32_t OffsetAccelAvg[3] = {0, 0, 0};
      static int32_t OffsetGyroAvg[3] = {0, 0, 0};
      static int16_t PrevOffsetAccelAvg[3] = {0, 0, 0};
      static int16_t PrevOffsetGyroAvg[3] = {0, 0, 0};
      static uint16_t OffsetCount = 0;
      static bool BeginOffsetCalc = false;

      if (!BeginOffsetCalc) {
        BeginOffsetCalc = true;
        for (uint8_t X = 0; X < 3; X++) {
          OffsetAccelAvg[X] = 0;
          OffsetGyroAvg[X] = 0;
          PrevOffsetAccelAvg[X] = 0;
          PrevOffsetGyroAvg[X] = 0;
        }
        OffsetCount = 0;
      }

      for (uint8_t X = 0; X < 3; X++) {
        //Use 30 for a more reactive and precise measurement:
        OffsetAccelAvg[X] -= (OffsetAccelAvg[X] / 30);
        OffsetAccelAvg[X] += _MPU.RawAccel[X];

        OffsetGyroAvg[X] -= (OffsetGyroAvg[X] / 30);
        OffsetGyroAvg[X] += _MPU.RawGyro[X];

        if (abs((OffsetAccelAvg[0] / 30) - PrevOffsetAccelAvg[0]) <= 5 &&
            abs((OffsetAccelAvg[1] / 30) - PrevOffsetAccelAvg[1]) <= 5 &&
            abs((OffsetAccelAvg[2] / 30) - PrevOffsetAccelAvg[2]) <= 5) {
          OffsetCount++;
          if (OffsetCount >= 200) {
            CalcOffsets = false;
            BeginOffsetCalc = false;
            for (uint8_t Y = 0; Y < 3; Y++) {
              _AccelOffset[Y] = (OffsetAccelAvg[Y] / 30);
            }
          }
        }
        else {
          OffsetCount = 0;
        }
        for (uint8_t Y = 0; Y < 3; Y++) {
          PrevOffsetAccelAvg[Y] = (OffsetAccelAvg[Y] / 30);
        }
      }
    }
    if (VibrateComp && OrientComp) return true;
    else return false;
  }
  else {
    return false;
  }
}

#endif
