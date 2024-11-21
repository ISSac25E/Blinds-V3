#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\Vacuum_IR\Vacuum_IR_1.0.0.h"
volatile const bool start[] = {0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1};
volatile const bool stop[] = {0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0};

void setup()
{
  pinMode(3, INPUT_PULLUP);
  Vacuum_IR.setPin(11);
}

bool sig = false;

void loop()
{
  if (!digitalRead(3))
  {
    delay(20);
    if (!digitalRead(3))
    {
      sig ? Vacuum_IR.write((bool *)start, sizeof(start)) : Vacuum_IR.write((bool *)stop, sizeof(stop));
      sig = !sig;
      delay(20);
      while (!digitalRead(3))
        delay(20);
    }
  }
}