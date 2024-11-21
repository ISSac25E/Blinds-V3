// RTX_V3.0
//.h
#ifndef OneWire_h
#define OneWire_h

#include "Arduino.h"
#include "util/bitArray.h"

#define wh_SetPin_OUTPUT (*_pinPort_DDR |= _pinMask)
#define wh_SetPin_INPUT (*_pinPort_DDR &= _pinMaskNot)

#define wh_SetPin_HIGH (*_pinPort_PORT |= _pinMask)
#define wh_SetPin_LOW (*_pinPort_PORT &= _pinMaskNot)

#define wh_ReadPin (*_pinPort_PIN & _pinMask)

class OneWire
{
public:
  /*
    OneWire():
      init OneWire pin
      defaults pin to INPUT_PULLUP
      state is set to 0(idle)

      inputs: ( (uint8_t)OneWire Pin)
  */
  OneWire(uint8_t);

  /*
    read():
      read a packet
      will overwrite any other current action (idle or reading)
      give a packet buffer and max amount of bits allowed to be read
      give max parsing time(optional) 0 = no timeout (will read until a packet is found)

      inputs: ( (uint8_t*) return packet array,
                (uint8_t&) return packet array length(bits),
                (uint8_t) maxbits allowed (or 'packet array' length),
                (uint16_t) max parse time in microseconds(us) (0 = no max parse time. read until packet found) )
  */
  void read(uint8_t *, uint8_t &, uint8_t, uint16_t) __attribute__((optimize("-O0")));

  /*
    write():
      write a packet immediately
      will overwrite any other current action (idle or reading)

      inputs: ( (uint8_t*) packet array (to be written),
                (uint8_t) bit (to write) )
  */
  void write(uint8_t *, uint8_t) __attribute__((optimize("-O0")));

  /*
    run():
      run handler for OneWire
      run at a consistant time interval. Run interval of Reader MUST be the same as transmitter
      bit-depth / transmission speed is determined by how often this handler is run

      originally designed for use with timer interrupts
  */
  inline void run() __attribute__((always_inline));

  /*
    state():
      return current run state of handler

      returns: (uint8_t) current run state
                  0 = idle(ready to read or write)
                  1 = reading
                  2 = writing
  */
  uint8_t state();

  /*
    msgAvail():
      use this after a read has been completed
      returns: true if message available in previously provided buffer(byte count too)
  */
  bool msgAvail();

private:
  // all port register variables:
  volatile uint8_t *_pinPort_PIN;  // Pin read/state register
  volatile uint8_t *_pinPort_DDR;  // Pin set register (INPUT/OUTPUT)
  volatile uint8_t *_pinPort_PORT; // Pin write register (HIGH/LOW)

  volatile uint8_t _pinMask;    // eg. B00100000
  volatile uint8_t _pinMaskNot; // eg. B1101111

  /*
  "_runState"
    0 = idle
    1 = reading
    2 = writing
  */
  uint8_t _runState = 0;

  // "_packet" used for both send and reading bits (dataBits are compressed(uint8_t))
  uint8_t *_packet;
  // "_bits" used for sending AND for maxByte of reading
  uint8_t _bits;
  // "_bitCount" used for keeping track of CURRENT bit sending
  uint8_t _bitCount;
  // "_returnBits" used for returning read bytes
  uint8_t *_returnBits;

  bool _msgAvailable = false;

  /* "_parseTimer" is used by micros() for msg timeout
     timeout timer begins immediately after "read()" is called
     if start bit is found, message will not timeout until message is complete or error condition  */
  int32_t _parseTimer;
  /* "_maxParse_us" will be used to record how much handler will wait for a message in microSeconds
  if "_maxParse_us" is set to zero, handler will indefinitely search for a message until a valid one is found */
  uint16_t _maxParse_us;

  /*
     "_readBitCount" holds current streak count for the current pinState "_readBitCount[0]"
      and previous streak "_readBitCount[1]"
      and the one before that "_readBitCount[2]"
      so on...
  */
  uint8_t _readBitCount[4];

  /*
    "_startBit " will be used to track when a start bit is located.
    A start bit must be found to collect data bits and the stop bit
  */
  bool _startBit;

  /*
  mark if previous data bit has already been found
  to prevent duplicate data bits
  */
  bool _dataBit;

  /*
  "_sendStage"
    0 = start bit
    1 = data bit
    2 = stop bit
  */
  uint8_t _sendStage;
  // "_sendBitStage" used for each "_sendStage"
  uint8_t _sendBitStage;

  // "_pinState" used for reading pinState and sending pinState
  bool _pinState;
};

//.cpp

OneWire::OneWire(uint8_t pin)
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
  wh_SetPin_INPUT; // set pin to input
  wh_SetPin_HIGH;  // set to pullup
}

void OneWire::read(uint8_t *packet, uint8_t &bits, uint8_t maxBits, uint16_t maxParse_us = 0)
{
  // stop all action until after variables are set:
  _runState = 0;

  _packet = packet;
  _returnBits = &bits;
  _bits = maxBits;

  _msgAvailable = false;

  _parseTimer = micros();
  _maxParse_us = maxParse_us;

  _readBitCount[0] = 0;
  _readBitCount[1] = 0;
  _readBitCount[2] = 0;

  _startBit = false;

  _pinState = true;

  wh_SetPin_INPUT;
  wh_SetPin_HIGH;

  // start reading actions:
  _runState = 1;
}

void OneWire::write(uint8_t *packet, uint8_t bits)
{
  // stop all action until after variables are set:
  _runState = 0;

  _packet = packet;
  _bits = bits;
  _bitCount = 0;

  _msgAvailable = false;

  _sendStage = 0;
  _sendBitStage = 0;

  _pinState = true;

  wh_SetPin_HIGH;
  wh_SetPin_OUTPUT;

  // start reading actions:
  _runState = 2;
}

inline void OneWire::run()
{
  switch (_runState)
  {
  case 1: // read
  {
    /*
      timeout handler:
      will timeout only if start bit is not found
      is a start bit is found, timeout will wait until a packet is
      found or an error occurs
    */
    {
      if (_maxParse_us != 0)
      {
        if (!_startBit && _pinState)
        {
          if (micros() - _parseTimer >= _maxParse_us)
          {
            // back to idle:
            _runState = 0;
          }
        }
      }
    }

    /*
      keep track of read segments
      also helps with consistency-
      (each bit will be read as closely as possible to the start of the run() function)
    */
    {
      bool pinStateHold = wh_ReadPin;
      if (pinStateHold != _pinState) // pin state changed(relative to last reading)
      {
        _readBitCount[3] = _readBitCount[2];
        _readBitCount[2] = _readBitCount[1];
        _readBitCount[1] = _readBitCount[0];
        _readBitCount[0] = 1; // always start with 1 (1 bit reading found so far)
      }
      else
      {
        if (_readBitCount[0] < 255)
          _readBitCount[0]++;
      }
      _pinState = pinStateHold;
    }

    /*
      find any start bit to reset message reading
    */
    {
      /*
      start bit:
        ‾\____10____/‾2‾
      */
      if (_pinState)
      {
        if (_readBitCount[1] >= 8 && _readBitCount[1] <= 14)
        {
          _startBit = true;
          _dataBit = true;
          *_returnBits = 0;
        }
      }
    }
    /*
      if start bit has been found, collect data bits and the final stop bit
    */
    if (_startBit)
    {
      /*
      find data bits:
        bit(0):
          ‾\_2_/‾‾‾6+‾‾‾
        bit(1):
          ‾\___6___/‾2+‾
      */
      {
        if (!_dataBit)
        {
          if (_pinState)
          {
            // bit(0):
            if (_readBitCount[1] >= 1 && _readBitCount[1] <= 3)
            {
              if (_readBitCount[0] >= 5)
              {
                if (*_returnBits >= _bits)
                {
                  _startBit = false;
                }
                else
                {
                  WRITE_BIT(_packet, (*_returnBits)++, false);
                  _dataBit = true;
                }
              }
            }
            // bit(1):
            else if (_readBitCount[1] >= 5 && _readBitCount[1] <= 7)
            {
              if (_readBitCount[0] >= 1)
              {
                if (*_returnBits >= _bits)
                {
                  _startBit = false;
                }
                else
                {
                  WRITE_BIT(_packet, (*_returnBits)++, true);
                  _dataBit = true;
                }
              }
            }
          }
        }
        else
        {
          if (!_pinState)
          {
            _dataBit = false;
          }
        }
      }
      /*
      find stop bit:
        bit(0):
          ‾\_2_/‾2‾\_2_/‾1+‾
      */
      {
        if (_pinState)
        {
          if (_readBitCount[0] >= 1)
          {
            if (_readBitCount[1] >= 1 && _readBitCount[1] <= 3) // second low-pulse
            {
              if (_readBitCount[2] >= 1 && _readBitCount[2] <= 3) // first high-pulse
              {
                if (_readBitCount[3] >= 1 && _readBitCount[3] <= 3) // first low-pulse
                {
                  // done reading:
                  _runState = 0;
                  _msgAvailable = true;
                }
              }
            }
          }
        }
      }
      /*
      check for hangs:
      any bit that is too long (low or high)
      */
      {
        // any bit longer than start-bit:
        if (_readBitCount[0] > 7)
        {
          _startBit = false;
        }
      }
    }
  }
  break;
  case 2: // write
  {
    /*
    write to pin as soon as possible in the loop
    for consistency*/
    {
      _pinState ? wh_SetPin_HIGH : wh_SetPin_LOW;
    }
    switch (_sendStage)
    {
      // Data Bits
    case 1:
    {
      if (_sendBitStage < 8)
        _sendBitStage++;

      if (_sendBitStage == 1)
      {
        _pinState = false;
      }
      if (READ_BIT(_packet, _bitCount)) // (1) bit
      {
        if (_sendBitStage == 7)
        {
          _pinState = true;
        }
      }
      else // (0) bit
      {
        if (_sendBitStage == 3)
        {
          _pinState = true;
        }
      }

      if (_sendBitStage == 8)
      {
        _sendBitStage = 0;
        _bitCount++;
      }
      if (_bitCount >= _bits)
      {
        // move on to stop bit:
        _sendStage = 2;
        _sendBitStage = 0;
      }
    }
    break;
      // Start Bit
    case 0:
    {
      if (_sendBitStage < 14)
        _sendBitStage++;

      if (_sendBitStage == 1)
      {
        wh_SetPin_OUTPUT;
        _pinState = true;
      }
      else if (_sendBitStage == 3)
      {
        _pinState = false;
      }
      else if (_sendBitStage == 13)
      {
        _pinState = true;
      }
      else if (_sendBitStage == 14)
      {
        // start bit completed, move on to data bits
        if (_bits)
          _sendStage = 1;
        else
          _sendStage = 2;
        _sendBitStage = 0;
      }
    }
    break;
      // Stop Bit
    case 2:
    {
      if (_sendBitStage < 9)
        _sendBitStage++;

      if (_sendBitStage == 1)
      {
        _pinState = false;
      }
      else if (_sendBitStage == 3)
      {
        _pinState = true;
      }
      else if (_sendBitStage == 5)
      {
        _pinState = false;
      }
      else if (_sendBitStage == 7)
      {
        _pinState = true;
      }
      else if (_sendBitStage == 9)
      {
        _pinState = true;
        wh_SetPin_HIGH;
        wh_SetPin_INPUT;
        _runState = 0; // msg completed, stop transmission
      }
    }
    break;
    default:
      _runState = 0;
      // no msg, break;
      break;
    }
  }
  break;
  case 0: // idle
    break;
  default:
    _runState = 0;
    break;
  }
}

uint8_t OneWire::state()
{
  return _runState;
}

bool OneWire::msgAvail()
{
  return _msgAvailable;
}

#endif
