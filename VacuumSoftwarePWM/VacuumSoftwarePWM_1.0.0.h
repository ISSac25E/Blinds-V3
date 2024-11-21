// VacuumSoftwarePWM
//.h
#ifndef VacuumSoftwarePWM_h
#define VacuumSoftwarePWM_h

#include "Arduino.h"
#include "../PinPort/PinPort_1.0.2.h"

static inline void __write_pwm_high__(void) __attribute__((always_inline));
static inline void __write_pwm_low__(void) __attribute__((always_inline));

class VacuumSoftwarePWM_class
{
public:
  VacuumSoftwarePWM_class()
  {
    this->disable();
  }

  void setPin(uint8_t pin)
  {
    _pwmPin.setPin(pin);
    this->disable();
  }

  /*
    write:
      will not enable PWM. Must use "enable" first before using "write"
  */
  void write(uint8_t pwm)
  {
    if (_pwmEnable)
    {
      OCR0A = 0;
      if (!pwm)
      {
        TIMSK0 &= (~B110);
        _pwmPin.pinMode(INPUT);
        _pwmPin.digitalWrite(LOW);
      }
      else if (pwm == 255)
      {
        TIMSK0 &= (~B110);
        _pwmPin.digitalWrite(HIGH);
        _pwmPin.pinMode(OUTPUT);
      }
      else
      {
        OCR0B = pwm;
        TIMSK0 |= B110;
        _pwmPin.pinMode(OUTPUT);
      }
    }
    else
    {
      TIMSK0 &= (~B110);
      _pwmPin.pinMode(INPUT);
      _pwmPin.digitalWrite(LOW);
    }
  }

  /*
    enable:
      sets pin to output
  */
  void enable()
  {
    _pwmEnable = true;
    this->write(0);
  }

  /*
    enable(uint8_t):
      sets pin to output
      enable with initial value
  */
  void enable(uint8_t pwm)
  {
    _pwmEnable = true;
    this->write(pwm);
  }

  /*
    disable:
      sets pin to input
  */
  void disable()
  {
    _pwmEnable = false;
    _pwmPin.pinMode(INPUT);
    _pwmPin.digitalWrite(LOW);
    TIMSK0 &= (~B110);
  }

private:
  PinPort _pwmPin;
  volatile bool _pwmEnable = false;

  friend void __write_pwm_high__(void);
  friend void __write_pwm_low__(void);
};
VacuumSoftwarePWM_class VacuumSoftwarePWM;

//.cpp

static inline void __write_pwm_high__(void)
{
  if (VacuumSoftwarePWM._pwmEnable)
    VacuumSoftwarePWM._pwmPin.digitalWrite(HIGH);
}

static inline void __write_pwm_low__(void)
{
  if (VacuumSoftwarePWM._pwmEnable)
    VacuumSoftwarePWM._pwmPin.digitalWrite(LOW);
}

ISR(TIMER0_COMPA_vect)
{
  __write_pwm_high__();
}

ISR(TIMER0_COMPB_vect)
{
  __write_pwm_low__();
}

#endif
