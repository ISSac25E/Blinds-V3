// file_name
//.h
#ifndef StepperDriver_h
#define StepperDriver_h

#include "Arduino.h"

// for perfect inline Port manipulations within the class:
#define SetPin_OUTPUT (*_pinPort_DDR |= _pinMask)
#define SetPin_INPUT (*_pinPort_DDR &= _pinMaskNot)
#define SetPin(state) (state ? SetPin_OUTPUT : SetPin_INPUT)

#define WritePin_HIGH (*_pinPort_PORT |= _pinMask)
#define WritePin_LOW (*_pinPort_PORT &= _pinMaskNot)
#define WritePin(state) (state ? WritePin_HIGH : WritePin_LOW)

#define ReadPin (*_pinPort_PIN & _pinMask)

// all port register variables:
static volatile uint8_t *_pinPort_PIN;  // Pin read/state register
static volatile uint8_t *_pinPort_DDR;  // Pin set register (INPUT/OUTPUT)
static volatile uint8_t *_pinPort_PORT; // Pin write register (HIGH/LOW)

static volatile uint8_t _pinMask;    // eg. B00100000
static volatile uint8_t _pinMaskNot; // eg. B1101111

#ifndef STEPPER_STEPS_ACCEL
#error "STEPPER_STEPS_ACCEL not declared. Format to Declare: #define STEPPER_STEPS_ACCEL 50"
#endif

#ifndef STEPPER_MAX_SPEED
#error "STEPPER_MAX_SPEED not declared. Format to Declare: #define STEPPER_MAX_SPEED 200"
#endif

#ifndef STEPPER_MIN_SPEED
#define STEPPER_MIN_SPEED 10
#warning "STEPPER_MAX_SPEED not declared. Default Declared: #define STEPPER_MAX_SPEED 10"
#endif

#ifndef STEPPER_STEPS_STOP
#define STEPPER_STEPS_STOP (STEPPER_STEP_ACCEL/3)
#warning "STEPPER_STEPS_STOP not declared. Default Declared: #define STEPPER_STEPS_STOP (STEPPER_STEP_ACCEL/3)"
#endif



class StepperDriver
{
public:
  /*
    StepperDriver():
      Input: step pin, dir pin
  */
  StepperDriver(uint8_t, uint8_t);
  /*
    start():
      Input: What position to set Stepper too
      will automatically handle stopping stepper and 
  */
  void set(uint16_t);
  /*
    stop()
      Stop current stepping
      decelerate at "STEPPER_STEPS_STOP" speed
  */
  void stop();

  /*
    step():
      returns Step Position
  */
  uint16_t step();

private:


};

//.cpp
StepperDriver::StepperDriver(uint16_t frequency, uint8_t pin)
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
  SetPin_INPUT;
  WritePin_LOW;
}
void StepperDriver::start(uint16_t steps)
{
}
void StepperDriver::stop()
{
}

uint16_t StepperDriver::step()
{
}

#undef SetPin_OUTPUT
#undef SetPin_INPUT
#undef SetPin

#undef WritePin_HIGH
#undef WritePin_LOW
#undef WritePin

#undef ReadPin
#endif
