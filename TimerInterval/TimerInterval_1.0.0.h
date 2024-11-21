// TimerInterval
//.h
#ifndef TimerInterval_h
#define TimerInterval_h

#include "Arduino.h"

static inline void __place_holder_funct__(void);

class TimerIntervalClass
{
public:
  /*
    TimerIntervalClass():
      set timer 2 up
      only one instance can be used
  */
  TimerIntervalClass();

  /*
    disableTimer():
      disables timer completely, may cause unpredictable results,
      use as short as possible, re-enable timer using '_enableTimer()' or 'setTimer()'
  */
  inline void disableTimer();
  /*
    enableTimer():
      re-enables interrupt timer
  */
  inline void enableTimer();
  /*
    setTimer():
      set timer interrupt interval in microseconds(us)
      this will reenable timer if it was disabled

      Valid Interval Range: 1-1048576us

      This function is optimized for time.

      inputs: ( (uint32_t)timer interrupt interval in microseconds(us) (1-1048576) )
  */
  inline void setTimer(uint32_t);

  /*
    setCallback():
      set the function/method to be called on each interrupt
      callback can be changed dynamically whenever needed
  */
  inline void setCallback(void (*)(void));
};
TimerIntervalClass TimerInterval;

//.cpp
/*
    used to set secondary pre-scale to expand timer 2 interval range
*/
static volatile uint8_t __timerPrescaleSelect__;
/*
  keeps track of secondary pre-scale count
*/
static volatile uint8_t __timerPrescaleCount__;

/*
    callback pointer:
*/
static void (*__callBack__)(void) = __place_holder_funct__;

// lookup tables for timer 2 intervals starting with pre-scaler 8:
volatile const uint8_t __timer_2_shiftSelect__[] PROGMEM =
    {
        0,
        1,
        2,
        3,
        4,
        6,

        8,
        10,
        12};
volatile const uint32_t __timer_2_maxIntervals__[] PROGMEM =
    {
        128,
        512,
        1024,
        2048,
        4096,
        16384,

        65536,
        262144,
        1048576};

TimerIntervalClass::TimerIntervalClass()
{
  // presets:
  disableTimer();
  __timerPrescaleSelect__ = 0;
  __timerPrescaleCount__ = 0;
  __callBack__ = __place_holder_funct__;
}

inline void TimerIntervalClass::disableTimer()
{
  // disable all interrupts in Timer 2
  TIMSK2 = 0;
}
inline void TimerIntervalClass::enableTimer()
{
  // enable compare match A interrupt:
  TIMSK2 = (1 << OCIE2A);
}
inline void TimerIntervalClass::setTimer(uint32_t interval_us)
{
  if (!interval_us)
    interval_us++;

  if (interval_us > 1048576)
    interval_us = 1048576;

  volatile uint8_t timerPrescaleSelect;
  volatile const uint8_t prescaleListSize = (sizeof(__timer_2_maxIntervals__) >> 2); // right shift two to divide by 4 to account for 'sizeof(uint32_t)'

  // run through all prescale limits and find the first smallest one that fits 'interval_us':
  for (timerPrescaleSelect = 0; timerPrescaleSelect < prescaleListSize; timerPrescaleSelect++)
    if (interval_us <= pgm_read_dword_near(__timer_2_maxIntervals__ + timerPrescaleSelect))
      break;

  volatile const uint8_t regPrescalerSet = (timerPrescaleSelect < 6 ? timerPrescaleSelect + 2 : B111); // set register pre-scaler
  // if timerPrescaleSelect == 0, we need to left shift by one, otherwise right shift by 'pgm_read_byte_near(__timer_2_shiftSelect__ + timerPrescaleSelect)'
  volatile const uint8_t regCompareMatchSet = ((timerPrescaleSelect ? (interval_us >> pgm_read_byte_near(__timer_2_shiftSelect__ + timerPrescaleSelect)) : (interval_us << 1)) - 1);
  volatile const uint8_t secondaryTimerPrescaleSet = (timerPrescaleSelect < 6 ? 0 : 1 << (pgm_read_byte_near(__timer_2_shiftSelect__ + timerPrescaleSelect) - 6));

  // disable timer while setting registers
  disableTimer();

  __timerPrescaleSelect__ = secondaryTimerPrescaleSet;
  __timerPrescaleCount__ = 0;

  // enable CTC Mode:
  TCCR2A = (1 << WGM21);
  // set prescaler:
  TCCR2B = regPrescalerSet;

  // set compare match value:
  OCR2A = regCompareMatchSet;

  // init counter to 0:
  TCNT2 = 0;
  enableTimer();
}

inline void TimerIntervalClass::setCallback(void (*callBack)(void))
{
  __callBack__ = callBack;
}

// placeholder function to prevent null point error in the event a callback is not set
static inline void __place_holder_funct__(void) {}

ISR(TIMER2_COMPA_vect)
{
  if (__timerPrescaleCount__ < __timerPrescaleSelect__)
    __timerPrescaleCount__++;
  else
  {
    __timerPrescaleCount__ = 0;
    __callBack__();
  }
}

#endif
