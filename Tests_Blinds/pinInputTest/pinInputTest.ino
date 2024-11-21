#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\WordOfLife\core\InputMacro\InputMacro_1.0.1.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\WordOfLife\core\PinDriver\PinDriver_1.0.1.h"

PinDriver pinInput(3);
InputMacro pinMacro(true);

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  if (pinMacro(pinInput))
  {
    if (pinMacro)
    {
      Serial.print("|>> : ");
      Serial.println(pinMacro.prevInterval());
    }
    else if (!pinMacro)
    {
      Serial.print("|<< : ");
      Serial.println(pinMacro.prevInterval());
    }
  }

  if (pinMacro.interval() > 1000 && !pinMacro && !pinMacro.triggered())
  {
    Serial.print("Wifi Reset");
    Serial.println(pinMacro.interval());
    pinMacro.trigger();
  }
  // if (pinMacro.interval() > 1000 && pinMacro && !pinMacro.triggered())
  // {
  //   // Serial.print("|>| : ");
  //   // Serial.println(pinMacro.interval());
  //   pinMacro.trigger();
  // }
  
  if (Serial.available())
  {
    while (Serial.available())
      Serial.read();
    Serial.print("||| : ");
    Serial.println(pinMacro.interval());
  }
}
