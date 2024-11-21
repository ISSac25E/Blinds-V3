#include "VACUUM_GYRO_INTER.h"

VAC_GYRO VAC_GYRO_1(14);  //Set Power Pin to 14 (D5)

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
uint32_t Timer = millis();
pinMode(LED_BUILTIN, OUTPUT);
digitalWrite(LED_BUILTIN, HIGH);
while(millis() - Timer < 500) VAC_GYRO_1.Run();
VAC_GYRO_1.CalcOffsets();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(VAC_GYRO_1.Run()) digitalWrite(LED_BUILTIN, LOW);
  else digitalWrite(LED_BUILTIN, HIGH);
}
