#include "MPU6050_ESP.h"

MPU6050_ESP MPU(14);

void setup() {

  Serial.begin(115200);
  Serial.println("INIT");
  //    MPU.CalcAccelOffset(200);
  MPU.CalcAccelOffset(10);
  MPU.CalcAccelOffset(200);
}

void loop() {
  int32_t CompAccel[3] = {0, 0, 0};
  for (uint8_t X = 0; X < 100; X++) {
    MPU.Run();
    CompAccel[0] += MPU.RawAccel[0];
    CompAccel[1] += MPU.RawAccel[1];
    CompAccel[2] += MPU.RawAccel[2];
  }
  CompAccel[0] /= 1000;
  CompAccel[1] /= 1000;
  CompAccel[2] /= 1000;
  for (uint8_t X = 0; X < 10; X++) {
    Serial.print(CompAccel[0]); Serial.print(' ');
    Serial.print(CompAccel[1]); Serial.print(' ');
    Serial.print(CompAccel[2]); Serial.println();
  }
}
