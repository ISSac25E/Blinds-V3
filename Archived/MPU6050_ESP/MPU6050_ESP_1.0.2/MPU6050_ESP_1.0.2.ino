#include "MPU6050_ESP.h"

MPU6050_ESP MPU;

int16_t OffsetAccel[3] = {0, 0, 0};

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  Serial.println("INIT");
  MPU.Init(14);
  uint32_t Timer = millis();
  while (millis() - Timer < 300) MPU.Run();
  //  int32_t AvgAccel[3] = {0, 0, 0};
  //  int32_t PrevAvgAccel[3] = {0, 0, 0};
  //  uint16_t COMP_CNT = 0;
  //  bool COMP = false;
  //  while (!COMP) {
  //    yield();
  //    MPU.Run();
  //    AvgAccel[0] -= (AvgAccel[0] / 50);
  //    AvgAccel[1] -= (AvgAccel[1] / 50);
  //    AvgAccel[2] -=  (AvgAccel[2] / 50);
  //
  //    AvgAccel[0] += MPU.RawAccel[0];
  //    AvgAccel[1] += MPU.RawAccel[1];
  //    AvgAccel[2] += MPU.RawAccel[2];
  //
  //    if (abs((AvgAccel[0] / 50) - PrevAvgAccel[0]) <= 5 &&
  //        abs((AvgAccel[1] / 50) - PrevAvgAccel[1]) <= 5 &&
  //        abs((AvgAccel[2] / 50) - PrevAvgAccel[2]) <= 5) {
  //      COMP_CNT++;
  //      if (COMP_CNT >= 200) COMP = true;
  //    }
  //    else COMP_CNT = 0;
  //    PrevAvgAccel[0] = (AvgAccel[0] / 50);
  //    PrevAvgAccel[1] = (AvgAccel[1] / 50);
  //    PrevAvgAccel[2] = (AvgAccel[2] / 50);
  //  }
  //  for (volatile uint8_t X = 0; X < 3; X++) {
  //    OffsetAccel[X] = (AvgAccel[X] / 50);
  //  }
}

void loop() {  
      static int32_t AbsAccel[3] = {0, 0, 0};
      static int16_t PrevAbsAccel[3] = {0, 0, 0};
      MPU.Run();
      AbsAccel[0] -= AbsAccel[0] / 1000;
      AbsAccel[1] -= AbsAccel[1] / 1000;
      AbsAccel[2] -=  AbsAccel[2] / 1000;
  
      AbsAccel[0] += (MPU.RawAccel[0]);
      AbsAccel[1] += (MPU.RawAccel[1]);
      AbsAccel[2] += (MPU.RawAccel[2]);
  
      AbsAccel[0] = abs(AbsAccel[0]);
      AbsAccel[1] = abs(AbsAccel[1]);
      AbsAccel[2] = abs(AbsAccel[2]);
  
      PrevAbsAccel[0] = MPU.RawAccel[0];
      PrevAbsAccel[1] = MPU.RawAccel[1];
      PrevAbsAccel[2] = MPU.RawAccel[2];
  
      Serial.print(AbsAccel[0] / 1000); Serial.print(' ');
      Serial.print(AbsAccel[1] / 1000); Serial.print(' ');
      Serial.print(AbsAccel[2] / 1000); Serial.println();


//  static int32_t CompAccelAvg[3] = {0, 0, 0};
//  static int32_t AccelAvg[3] = {0, 0, 0};
//  static int32_t AccelAbsAvg[3] = {0, 0, 0};
//
//  MPU.Run();
//  {
//    CompAccelAvg[0] -= CompAccelAvg[0] / 50;
//    CompAccelAvg[1] -= CompAccelAvg[1] / 50;
//    CompAccelAvg[2] -= CompAccelAvg[2] / 50;
//
//    CompAccelAvg[0] += MPU.RawAccel[0];
//    CompAccelAvg[1] += MPU.RawAccel[1];
//    CompAccelAvg[2] += MPU.RawAccel[2];
//  }
//  {
//    AccelAvg[0] -= AccelAvg[0] / 50;
//    AccelAvg[1] -= AccelAvg[1] / 50;
//    AccelAvg[2] -= AccelAvg[2] / 50;
//
//    AccelAvg[0] += MPU.RawAccel[0];
//    AccelAvg[1] += MPU.RawAccel[1];
//    AccelAvg[2] += MPU.RawAccel[2];
//  }
//  {
//    AccelAbsAvg[0] -= AccelAbsAvg[0] / 10;
//    AccelAbsAvg[1] -= AccelAbsAvg[1] / 10;
//    AccelAbsAvg[2] -= AccelAbsAvg[2] / 10;
//
//    AccelAbsAvg[0] += abs((CompAccelAvg[0] / 50) - MPU.RawAccel[0]);
//    AccelAbsAvg[1] += abs((CompAccelAvg[1] / 50) - MPU.RawAccel[1]);
//    AccelAbsAvg[2] += abs((CompAccelAvg[2] / 50) - MPU.RawAccel[2]);
//  }
//
//  //  Serial.print((AccelAbsAvg[0] / 100)); Serial.print(' ');
//  //  Serial.print((AccelAbsAvg[1] / 100)); Serial.print(' ');
//  //  Serial.print((AccelAbsAvg[2] / 100)); Serial.println();
//
//  int32_t MoveAvg = (((AccelAbsAvg[0] / 10) +
//                      (AccelAbsAvg[1] / 10) +
//                      (AccelAbsAvg[2] / 10)) / 3);
// static int32_t PeakCount = 0;
// static bool LED_State = false;
////  PeakCount = PeakCount / 100;
//  if (MoveAvg > 300) {
//    if(PeakCount < 0) PeakCount = 0;
//    else if(PeakCount < 10000)PeakCount++;
//  }
//  else {
//    if(PeakCount > 0)
//      PeakCount = 0;
//      else if(PeakCount > -10000) PeakCount--;
//  }
//  if(LED_State) {
//    if(PeakCount < -600){
//      LED_State = false;
//      digitalWrite(LED_BUILTIN, HIGH);
//    }
//  }
//  else {
//    if(PeakCount > 100){
//      LED_State = true;
//      digitalWrite(LED_BUILTIN, LOW);
//    }
//  }
//  Serial.print(PeakCount); Serial.println();
}
