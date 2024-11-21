#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Blinds V3\Core\OneWire V3\OneWire_V3_1.0.1.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Blinds V3\Core\RTX_V3\Core\BitED\BitED_1.0.0.h"

OneWire wire;
OneWire wireTX;

uint8_t buffer[30];
uint8_t bitLength;

uint8_t txbuffer[30] = {'>', 'B', 'y', 'e', ' ', 'W', 'o', 'r', 'l', 'd', '!', '<'};
uint8_t txbitLength = 12 * 8;

void setup()
{
  Serial.begin(115200);

  wire.setPin(12);
  wireTX.setPin(10);

  // Serial.println(bitLength);
  // for (uint8_t x = 0; x < bitLength; x++)
  // {
  //   Serial.print(READ_BIT(buffer, x));
  //   Serial.print(',');
  // }

  BitED.setED(txbuffer, txbitLength, 0);

  // Serial.println();
  // Serial.println(bitLength);
  // for (uint8_t x = 0; x < bitLength; x++)
  // {
  //   Serial.print(READ_BIT(buffer, x));
  //   Serial.print(',');
  // }
  wire.read(buffer, bitLength, 30 * 8, 0);
  // wire.write(buffer, bitLength);
}

uint32_t timer = micros();

void loop()
{
  if (micros() - timer >= 80)
  {
    timer = micros();
    wire.run();
    wireTX.run();
  }
  if (!wireTX.state())
  {
    Serial.print("TX Flag: ");
    Serial.println(wireTX.writeFlag());
    wireTX.write(txbuffer, txbitLength);
  }
  if (wire.msgAvail())
  {
    Serial.println(bitLength);
    for (uint8_t x = 0; x < bitLength; x++)
    {
      Serial.print(READ_BIT(buffer, x));
      Serial.print(',');
    }
    Serial.println();
    for (uint8_t x = 0; x < (bitLength / 8); x++)
    {
      Serial.print((char)buffer[x]);
    }
    Serial.println();

    if (BitED.checkED(buffer, bitLength, 0))
    {
      Serial.println("ED True");
    }
    else
    {
      Serial.println("ED False");
    }

    Serial.println(bitLength);
    for (uint8_t x = 0; x < bitLength; x++)
    {
      Serial.print(READ_BIT(buffer, x));
      Serial.print(',');
    }
    Serial.println();
    for (uint8_t x = 0; x < (bitLength / 8); x++)
    {
      Serial.print((char)buffer[x]);
    }
    Serial.println();

    wire.read(buffer, bitLength, 30 * 8, 0);
  }
}