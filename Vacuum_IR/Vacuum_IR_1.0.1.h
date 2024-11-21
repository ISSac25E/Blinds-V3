/*
  Vacuum_IR

  Highly specific IR Protocol for vacuum IR.
  Designed for direct wire IR manipulation, NOT with actual IR LED
  Tests proved IR LED to be greatly ineffective

  dependencies:
    - "TimerInterval" library for precise manipulation of timer2
    - "PinPort" library for fast pin manipulation

  ToDO:
    - Major issue with TimerInterrupt. Interrupts almost immediately on first "TimerSet" of each Transmission
      temporary patch with extra TimerSet before each Transmission

*/
//.h
#ifndef Vacuum_IR_h
#define Vacuum_IR_h

#include "Arduino.h"

/*
  - "TimerInterval" library
  - "PinPort" library
  - "bitArray" tool
*/
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\TimerInterval\TimerInterval_1.0.0.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\PinPort\PinPort_1.0.2.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\Utility\bitArray.h"

static void __timerCallback__(void);

class Vacuum_IR_class
{
public:
  Vacuum_IR_class();

  /*
    setPin:
      setup pin number for IR

      inputs: (uint8_t) IR Pin Number
  */
  void setPin(uint8_t);

  /*
    write:
      send to IR
      input a bool array of data bits to be sent
      If IR is currently sending, the new write request will be ignored

      inputs: (bool*) boolean array of data bits to be sent,
              (uint8_t) number of data bits to send (*from the boolean array)

      outputs: (bool) true if send request successful, false if request is ignored (because of ongoing transmission)
  */
  bool write(bool *, uint8_t);

  /*
    writeByte:
      send to IR USING a byte array of data bits
      If IR is currently sending, the new write request will be ignored
      note: slightly more processing is required for a byte-array of data bits compared to bool-array
            but, byte-array saves ram more efficiently

      inputs: (uint8_t*) byte array of data bits to be sent,
              (uint8_t) number of data bits to send (*from the boolean array)

      outputs: (bool) true if send request successful, false if request is ignored (because of ongoing transmission)
  */
  bool writeByte(uint8_t *, uint8_t);

  /*
    write_PROGMEM:
      send to IR
      input a bool PROGMEM array of data bits to be sent
      If IR is currently sending, the new write request will be ignored

      inputs: (bool*) boolean PROGMEM array of data bits to be sent,
              (uint8_t) number of data bits to send (*from the boolean PROGMEM array)

      outputs: (bool) true if send request successful, false if request is ignored (because of ongoing transmission)
  */
  bool write_PROGMEM(bool *, uint8_t);

  /*
    writeByte_PROGMEM:
      send to IR USING a byte PROGMEM array of data bits
      If IR is currently sending, the new write request will be ignored
      note: slightly more processing is required for a byte-array of data bits compared to bool-array
            but, byte-array saves ram more efficiently

      inputs: (uint8_t*) byte PROGMEM array of data bits to be sent,
              (uint8_t) number of data bits to send (*from the boolean array)

      outputs: (bool) true if send request successful, false if request is ignored (because of ongoing transmission)
  */
  bool writeByte_PROGMEM(uint8_t *, uint8_t);

  /*
    stop:
      stops IR Transmission regardless of stage:
      resets output pin
  */
  void stop();

private:
  PinPort _pin;

  /*
    _dataPtr:
      will be used for both bool and uint8_t data array's
  */
  volatile uint8_t *_dataPtr;

  /*
    _writeStage:
      0 = not writing
      1 = writing start bits
      2 = writing data bits
      3 = writing end bits
  */
  volatile uint8_t _writeStage = 0;
  volatile uint8_t _writeSubStage;

  volatile uint8_t _dataBitsLen;
  volatile uint8_t _dataBitsCnt;
  volatile uint8_t _writeCount;
  /*
    _writeMedium:
      0 = bool(RAM)
      1 = byte(RAM)
      2 = bool(PROGMEM)
      3 = byte(PROGMEM)
  */
  volatile uint8_t _writeMedium;

  friend void __timerCallback__(void);
};
Vacuum_IR_class Vacuum_IR;

//.cpp
Vacuum_IR_class::Vacuum_IR_class()
{
  TimerInterval.disableTimer();
  TimerInterval.setCallback(__timerCallback__);
}

void Vacuum_IR_class::setPin(uint8_t pin)
{
  this->stop();
  _pin.setPin(pin);
  _pin.digitalWrite(LOW);
  _pin.pinMode(INPUT);
}

bool Vacuum_IR_class::write(bool *data, uint8_t bitLen)
{
  if (!_writeStage)
  {
    // set pin LOW-INPUT and Disable timer just in case:
    this->stop();

    // setup all vars:
    _dataPtr = (uint8_t *)data;
    _writeStage = 1;
    _writeSubStage = 0;
    _dataBitsLen = bitLen;
    _dataBitsCnt = 0;
    _writeCount = 0;

    _writeMedium = 0;

    _pin.digitalWrite(HIGH);
    _pin.pinMode(OUTPUT);
    TimerInterval.setTimer(100); // Temporary patch for Timer instant-interrupt glitch. TODO
    return true;
  }
  return false;
}

bool Vacuum_IR_class::writeByte(uint8_t *data, uint8_t bitLen)
{
  if (!_writeStage)
  {
    // set pin LOW-INPUT and Disable timer just in case:
    this->stop();

    // setup all vars:
    _dataPtr = (uint8_t *)data;
    _writeStage = 1;
    _writeSubStage = 0;
    _dataBitsLen = bitLen;
    _dataBitsCnt = 0;
    _writeCount = 0;

    _writeMedium = 1;

    _pin.digitalWrite(HIGH);
    _pin.pinMode(OUTPUT);
    TimerInterval.setTimer(100); // Temporary patch for Timer instant-interrupt glitch. TODO
    return true;
  }
  return false;
}

bool Vacuum_IR_class::write_PROGMEM(bool *data, uint8_t bitLen)
{
  if (!_writeStage)
  {
    // set pin LOW-INPUT and Disable timer just in case:
    this->stop();

    // setup all vars:
    _dataPtr = (uint8_t *)data;
    _writeStage = 1;
    _writeSubStage = 0;
    _dataBitsLen = bitLen;
    _dataBitsCnt = 0;
    _writeCount = 0;

    _writeMedium = 2;

    _pin.digitalWrite(HIGH);
    _pin.pinMode(OUTPUT);
    TimerInterval.setTimer(100); // Temporary patch for Timer instant-interrupt glitch. TODO
    return true;
  }
  return false;
}

bool Vacuum_IR_class::writeByte_PROGMEM(uint8_t *data, uint8_t bitLen)
{
  if (!_writeStage)
  {
    // set pin LOW-INPUT and Disable timer just in case:
    this->stop();

    // setup all vars:
    _dataPtr = (uint8_t *)data;
    _writeStage = 1;
    _writeSubStage = 0;
    _dataBitsLen = bitLen;
    _dataBitsCnt = 0;
    _writeCount = 0;

    _writeMedium = 3;

    TimerInterval.setTimer(100); // Temporary patch for Timer instant-interrupt glitch. TODO
    return true;
  }
  return false;
}

void Vacuum_IR_class::stop()
{
  // disable timer:
  TimerInterval.disableTimer();

  // write pin INPUT:
  _pin.pinMode(INPUT);
  _pin.digitalWrite(LOW);

  // reset stage var:
  Vacuum_IR._writeStage = 0;
}

static void __timerCallback__(void)
{
  if (Vacuum_IR._writeStage)
  {
    switch (Vacuum_IR._writeStage)
    {
    case 1:
      // write Start Bits
      /*
        start bits:
          ‾‾‾idle‾‾‾\__3080us__/‾‾2980us‾‾\__data-bits__
      */
      switch (Vacuum_IR._writeSubStage)
      {
      case 0: // First-LOW bit(3080us):
        // write pin LOW:
        Vacuum_IR._pin.digitalWrite(LOW);

        // set vars:
        Vacuum_IR._writeSubStage++;

        // setTimer:
        TimerInterval.setTimer(3080);
        break;

      case 1: // Second-HIGH bit(2980us):
        // write pin INPUT/HIGH:
        Vacuum_IR._pin.digitalWrite(HIGH);

        // set vars:
        Vacuum_IR._writeSubStage = 0;
        Vacuum_IR._writeStage++;

        // check if there are any data bits to be written. If not, increment stage one more time:
        if (!Vacuum_IR._dataBitsLen)
          Vacuum_IR._writeStage++;

        // setTimer:
        TimerInterval.setTimer(2980);
        break;

      default: // ERROR, this value should not be possible. Reset pin and timer:
        Vacuum_IR.stop();
        break;
      }
      break;
    case 2:
      // write Data Bits
      /*
        data-bits:
          LOW:
            ‾‾‾prev-bit‾‾‾\__482us__/‾‾518us‾‾\__next-bit__

          HIGH:
            ‾‾‾prev-bit‾‾‾\__482us__/‾‾‾‾1500us‾‾‾‾\__next-bit__
      */
      switch (Vacuum_IR._writeSubStage)
      {
      case 0: // First-LOW bit(482us):
        // write pin LOW:
        Vacuum_IR._pin.digitalWrite(LOW);

        // set vars:
        Vacuum_IR._writeSubStage++;

        // setTimer:
        TimerInterval.setTimer(482);
        break;
      case 1: // Second-HIGH bit(518us or 1599us):
        // write pin INPUT/HIGH:
        Vacuum_IR._pin.digitalWrite(HIGH);

        bool bitWrite; // return bit write state

        // check write Medium:
        switch (Vacuum_IR._writeMedium)
        {
        case 0:
          // bool (RAM)
          bitWrite = (bool)Vacuum_IR._dataPtr[Vacuum_IR._dataBitsCnt];
          break;
        case 1:
          // byte (RAM)
          bitWrite = READ_BIT((uint8_t *)Vacuum_IR._dataPtr, Vacuum_IR._dataBitsCnt);
          break;
        case 2:
          // bool (PROGMEM)
          bitWrite = (bool)pgm_read_byte_near(Vacuum_IR._dataPtr + Vacuum_IR._dataBitsCnt);
          break;
        case 3:
          // byte (PROGMEM)
          bitWrite = READ_BIT_PROGMEM((uint8_t *)Vacuum_IR._dataPtr, Vacuum_IR._dataBitsCnt);
          break;
        default: // ERROR
          // set to default value:
          bitWrite = false;
          break;
        }

        // set vars:
        Vacuum_IR._writeSubStage = 0;
        Vacuum_IR._dataBitsCnt++;
        if (Vacuum_IR._dataBitsCnt >= Vacuum_IR._dataBitsLen)
          Vacuum_IR._writeStage++; // go to end bit

        // set timer:
        bitWrite ? TimerInterval.setTimer(1500) : TimerInterval.setTimer(518);
        break;
      default: // ERROR
        Vacuum_IR.stop();
        break;
      }
      break;
    case 3:
      // write end bits
      /*
        end-bits:
          Standard:
            ‾‾‾prev-bit‾‾‾\__482us__/‾‾‾‾‾‾‾‾‾19ms‾‾‾‾‾‾‾‾‾\__next-transmission__

          Last Transmission:
            ‾‾‾prev-bit‾‾‾\__482us__/‾‾‾‾‾‾‾‾‾‾‾‾‾80ms‾‾‾‾‾‾‾‾‾‾‾‾‾ready-for-next-transmission‾‾‾‾
      */
      switch (Vacuum_IR._writeSubStage)
      {
      case 0: // First-LOW bit(482us):
        // write pin LOW:
        Vacuum_IR._pin.digitalWrite(LOW);

        // set vars:
        Vacuum_IR._writeSubStage++;

        // setTimer:
        TimerInterval.setTimer(482);
        break;
      case 1: // Second_HIGH bit(19ms or 80ms):
        // write pin INPUT/HIGH:
        Vacuum_IR._pin.digitalWrite(HIGH);

        // check if we need to transmit again:
        Vacuum_IR._writeCount++;
        if (Vacuum_IR._writeCount >= 3) // we transmitted 3 times already, we are done
        {
          Vacuum_IR._writeSubStage++;
          TimerInterval.setTimer(80000);
        }
        else // transmit again
        {
          Vacuum_IR._writeStage = 1;
          Vacuum_IR._writeSubStage = 0;
          Vacuum_IR._dataBitsCnt = 0;
          TimerInterval.setTimer(19000);
        }
        break;
      case 2: // close transmission
        // write pin INPUT/HIGH:
        Vacuum_IR._pin.pinMode(INPUT);
        Vacuum_IR._pin.digitalWrite(LOW);

        // Set to done writing:
        Vacuum_IR._writeStage = 0;

        // disable timer:
        TimerInterval.disableTimer();
        break;
      default: // ERROR
        Vacuum_IR.stop();
        break;
      }
      break;
    default:
      Vacuum_IR.stop();
      break;
    }
  }
  else
  {
    // No message being written, reset pin and disable timer:
    Vacuum_IR.stop();
  }
}

#endif