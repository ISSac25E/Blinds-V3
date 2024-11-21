#include "Arduino.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Blinds V3\Core\TimerInterval\TimerInterval_1.0.0.h"

void timerCallBack(void);

uint32_t timer = millis();
uint32_t potVal = 0;

void setup()
{
  // Serial.begin(2000000);
  pinMode(13, OUTPUT);
  pinMode(A0, OUTPUT);
  pinMode(A2, OUTPUT);
  digitalWrite(A0, LOW);
  digitalWrite(A2, HIGH);

  TimerInterval.setCallback(timerCallBack);
  TimerInterval.setTimer(0);
  analogWrite()
}

void loop()
{
  potVal++;
  // if (millis() - timer >= 10)
  // {
  //   TimerInterval.disableTimer();
  //   potVal = map(analogRead(A1), 0, 1023, 0, 256);
  //   TimerInterval.enableTimer();
  //   timer = millis();
  //   // Serial.println(potVal);
  // }
  // Serial.println(potVal);
}

void timerCallBack(void)
{
  digitalWrite(13, !digitalRead(13));
  TimerInterval.setTimer(potVal++);
}
