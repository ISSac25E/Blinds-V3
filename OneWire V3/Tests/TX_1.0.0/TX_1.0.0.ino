#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Blinds V3\Core\OneWire V3\OneWire_V3_1.0.1.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Blinds V3\Core\RTX_V3\Core\BitED\BitED_1.0.0.h"

OneWire wire;

uint8_t buffer[30] = {'H', 'e', 'l', 'l', 'o', ' ', 'W', 'o', 'r', 'l', 'd', '!'};
uint8_t bitLength = 12 * 8;

void setup()
{
  Serial.begin(115200);
  wire.setPin(12);

  Serial.println(bitLength);
  for (uint8_t x = 0; x < bitLength; x++)
  {
    Serial.print(READ_BIT(buffer, x));
    Serial.print(',');
  }

  BitED.setED(buffer, bitLength, 0);

  Serial.println();
  Serial.println(bitLength);
  for (uint8_t x = 0; x < bitLength; x++)
  {
    Serial.print(READ_BIT(buffer, x));
    Serial.print(',');
  }
  wire.write(buffer, bitLength);
}

uint32_t timer = micros();

void loop()
{
  if (micros() - timer >= 80)
  {
    timer = micros();
    wire.run();
  }
  if (!wire.state())
  {
    Serial.println(wire.writeFlag());
    wire.write(buffer, bitLength);
  }
}