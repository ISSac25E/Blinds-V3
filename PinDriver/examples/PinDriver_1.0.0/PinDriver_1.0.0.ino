#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\WordOfLife\core\PinDriver\PinDriver_1.0.0.h"

PinDriver pin(3);

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  Serial.println(pin.run());
  delay(10);
}