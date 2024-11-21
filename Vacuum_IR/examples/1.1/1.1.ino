#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\Vacuum_IR\Vacuum_IR_1.0.0.h"
volatile const uint8_t start[] PROGMEM = {22, 112, 0, 0, 255, 174, 0};
volatile const uint8_t stop[] PROGMEM = {22, 242, 0, 0, 255, 109, 0};

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
      sig ? Vacuum_IR.writeByte_PROGMEM((uint8_t *)start, 48) : Vacuum_IR.writeByte_PROGMEM((uint8_t *)stop, 48);
      sig = !sig;
      delay(20);
      while (!digitalRead(3))
        delay(20);
    }
  }
}