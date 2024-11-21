//PinPort
//.h
#ifndef PinPort_h
#define PinPort_h

#include "Arduino.h"

class PinPort
{
public:
  PinPort(uint8_t pin)
  {
    // set pin registers:
    pin %= 20; // make sure pin does not go out of bounds
    if (pin <= 7)
    {
      // DigitalPins (0 - 7)
      // Port D
      _pinPort_PIN = &PIND;
      _pinPort_DDR = (_pinPort_PIN + 1);
      _pinPort_PORT = (_pinPort_PIN + 2);
      _pinMask = (1 << pin);
      _pinMaskNot = ~_pinMask;
    }
    else if (pin <= 13)
    {
      // DigitalPins (8 - 13)
      // Port B
      _pinPort_PIN = &PINB;
      _pinPort_DDR = (_pinPort_PIN + 1);
      _pinPort_PORT = (_pinPort_PIN + 2);
      _pinMask = (1 << (pin - 8));
      _pinMaskNot = ~_pinMask;
    }
    else
    {
      // Analog Pins (A0 - A7)
      // Port C
      _pinPort_PIN = &PINC;
      _pinPort_DDR = (_pinPort_PIN + 1);
      _pinPort_PORT = (_pinPort_PIN + 2);
      _pinMask = (1 << (pin - 14));
      _pinMaskNot = ~_pinMask;
    }
  }

  inline void set(bool state)
  {
    state ? (*_pinPort_DDR |= _pinMask) : (*_pinPort_DDR &= _pinMaskNot);
  }
  inline bool read()
  {
    return *_pinPort_PIN & _pinMask;
  }
  inline void write(bool state)
  {
    state ? (*_pinPort_PORT |= _pinMask) : (*_pinPort_PORT &= _pinMaskNot);
  }

private:
  // all port register variables:
  volatile uint8_t *_pinPort_PIN;  // Pin read/state register
  volatile uint8_t *_pinPort_DDR;  // Pin set register (INPUT/OUTPUT)
  volatile uint8_t *_pinPort_PORT; // Pin write register (HIGH/LOW)

  volatile uint8_t _pinMask;    // eg. B00100000
  volatile uint8_t _pinMaskNot; // eg. B1101111
};

//.cpp

#endif
