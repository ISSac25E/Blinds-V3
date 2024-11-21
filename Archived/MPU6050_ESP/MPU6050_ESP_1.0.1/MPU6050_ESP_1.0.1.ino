#include "MPU6050_ESP.h"

MPU6050_ESP MPU(14);

void setup() {

  Serial.begin(115200);
  Serial.println("INIT");
}

void loop() {
  int32_t CompGyro[3] = {0, 0, 0};
  for (uint8_t X = 0; X < 100; X++) {
    MPU.Run();
    CompGyro[0] += MPU.RawGyro[0];
    CompGyro[1] += MPU.RawGyro[1];
    CompGyro[2] += MPU.RawGyro[2];
  }
  CompGyro[0] /= 100;
  CompGyro[1] /= 100;
  CompGyro[2] /= 100;
  for (uint8_t X = 0; X < 10; X++) {
    Serial.print(CompGyro[0]); Serial.print(' ');
    Serial.print(CompGyro[1]); Serial.print(' ');
    Serial.print(CompGyro[2]); Serial.println();
  }
}
