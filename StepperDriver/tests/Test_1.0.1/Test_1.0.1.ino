#include "Arduino.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Blinds V3\Core\StepperDriver\StepperDriver_2.1.3.h"

#define STEP_PIN 8
#define DIR_PIN 9

void stepperCallBack(bool, bool);

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

  pinMode(A0, OUTPUT);
  pinMode(A2, OUTPUT);

  digitalWrite(A0, LOW);
  digitalWrite(A2, HIGH);

  stepper.setMaxSpeed(500);
  stepper.setAccel(800);
}

uint32_t timer = millis();

void loop()
{
  if (millis() - timer >= 30) {
    stepper.moveTo(map(analogRead(A1), 0, 1023, 0, 5000));
    timer = millis();
  }
}

// void loop()
// {
//   static bool pinState[3] = {true, true, true};
//   if (digitalRead(2) != pinState[0])
//   {
//     pinState[0] = !pinState[0];
//     if (!pinState[0])
//     {
//       stepper.setAccel(500);
//       stepper.moveTo(0);
//     }
//   }
//   if (digitalRead(3) != pinState[1])
//   {
//     pinState[1] = !pinState[1];
//     if (!pinState[1])
//     {
//       stepper.setAccel(2000);
//       stepper.stop();
//     }
//   }
//   if (digitalRead(4) != pinState[2])
//   {
//     pinState[2] = !pinState[2];
//     if (!pinState[2])
//     {
//       stepper.setAccel(500);
//       stepper.moveTo(2000);
//     }
//   }
// }

void stepperCallBack(bool par, bool dir)
{
  digitalWrite(DIR_PIN, dir);
  digitalWrite(STEP_PIN, par);
}