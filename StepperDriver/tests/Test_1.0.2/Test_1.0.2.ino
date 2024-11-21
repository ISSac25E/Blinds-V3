#include "Arduino.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Blinds V3\Core\StepperDriver\StepperDriver_1.0.1.h"

#define STEP_PIN 8
#define DIR_PIN 9

void stepperCallBack(bool);

StepperDriver stepper(stepperCallBack);

void setup()
{
  Serial.begin(115200);
  Serial.println("init");

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);

  digitalWrite(A0, LOW);
  digitalWrite(A2, HIGH);

  stepper.setAccel(500);
  stepper.setMaxSpeed(500);
}

uint32_t timer = millis();

void loop()
{
  static bool pinState[3] = {true, true, true};
  if (digitalRead(2) != pinState[0])
  {
    pinState[0] = !pinState[0];
    if (!pinState[0])
    {
      stepper.moveTo(0);
    }
  }
  if (digitalRead(3) != pinState[1])
  {
    pinState[1] = !pinState[1];
    if (!pinState[1])
    {
      stepper.stop();
    }
  }
  if (digitalRead(4) != pinState[2])
  {
    pinState[2] = !pinState[2];
    if (!pinState[2])
    {
      stepper.moveTo(1000);
    }
  }
  stepper.handle();
}

void stepperCallBack(bool dir)
{
  digitalWrite(DIR_PIN, dir);
  digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
}