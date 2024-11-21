// StepperDriver
//.h
#ifndef StepperDriver_h
#define StepperDriver_h

#include "Arduino.h"
#include "../../TimerInterval/TimerInterval_1.0.0.h"

class StepperDriver
{

  friend void __timerFunct__(void);

public:
  /*
    StepperDriver():
      Input: Step CallBack function

      callback function will be called each time a step needs to be executed
      callback function inputs false if step is in ccw direction. true if cw direction
  */
  StepperDriver(void (*)(bool));

  /*
    moveTo():
      Input: What position to set Stepper too
      will automatically handle acceleration and deceleration
  */
  void moveTo(int32_t);
  /*
    stop()
      decelerates stepper to a stop
  */
  void stop();

  /*
    setSpeed():
      set max speed of steps in steps-per-second
      inputs: float(steps-per-second)(permits steps slower than 1-per-second)
  */
  void setMaxSpeed(float);
  /*
    setAcc():
      set acceleration of stepper in steps-per-second
      inputs: float(smaller number = slow acceleration. larger number = faster acceleration)
  */
  void setAccel(float);

  /*
    step():
      returns current absolute Step Position
  */
  int32_t step();

  /*
    step():
      set absolute position of stepper motor
      stops motor, sets speed and acceleration to 0

      inputs: ( (int32_t)new absolute position of stepper motor )
  */
  void step(int32_t);

private:
  /*
    _computeNewSpeed():
      computes a new step for stepper
      sets all proper constants
      will automatically set new timer interrupt interval
      return: new step interval in microseconds(us)
  */
  unsigned long _computeNewSpeed();

  /*
    function pointer
    called every time a new step is required
    input: true = CW(clockwise), false = CCW(counter-clockwise)
  */
  void (*_stepCallback)(bool);

  /*
    keep track of wether timer is currently running or not
    true = timer tunning and WILL interupt eventually
    false = timer not running/available to set
  */
  bool _timerRunning;

  /*
    current stepper direction
    0(false) = counter-clockwise(CCW)
    1(true) = clockwise(CW)
  */
  bool _direction;
  /*
    current step interval in microseconds(us)
  */
  unsigned long _stepInterval;
  /*
    (current) speed of the stepper in steps-per-second
    positive is clock-wise,
    negative is counter-clockwise

    this value will constantly change  for each step depending on acceleration, deceleration, distant to target, etc.
  */
  float _speed;
  /*
    the maximum speed permitted for the stepper in steps-per-second
    must be > 0
  */
  float _maxSpeed;
  /*
    the acceleration value for the stepper used to acceleration and decelerate the stepper in steps-per-second
    must be > 0
  */
  float _acceleration;
  /*
    last step time in microseconds(us)
  */
  uint32_t _lastStepTime;
  /*
  minimume step width in microseconds(us)
  */
  uint32_t _minStepTime;

  /* The step counter for speed calculations */
  long _n;

  /*
    Initial step size in microseconds (this is the longest possible step size)
  */
  float _c0;

  /*
    Last step size in microseconds
  */
  float _cn;

  /*
    smallest/shortest step size in microseconds based on maxSpeed
  */
  float _cmin; // step size at max speed (us)

  /*
    current (absolute) position of stepper in steps
  */
  int32_t _currentPos;
  /*
    target position of stepper in steps

    the library will use target and current positions to calculate how to move the stepper taking into account speed acceleration and deceleration
  */
  int32_t _targetPos;
};

//.cpp

static void __timerFunct__(void);
static StepperDriver *__timerFunctClassPnt__;

StepperDriver::StepperDriver(void (*stepCallback)(bool))
{
  // set all default values:
  _currentPos = 0;
  _targetPos = 0;
  _speed = 0.0;
  _maxSpeed = 0.0;
  _acceleration = 0.0;
  _stepInterval = 0;
  _lastStepTime = 0;
  _minStepTime = 1; // set a minimume

  // set timer as free:
  _timerRunning = false;

  _stepCallback = stepCallback;
  _direction = 0; // CCW

  _n = 0;
  _c0 = 0.0;
  _cn = 0.0;
  _cmin = 1.0;

  // set minimume defaults to prevent errors:
  // MAKE SURE 'setAccel' IS FIRST BEFORE 'setMaxSpeed'
  setAccel(1);
  setMaxSpeed(1);
  TimerInterval.disableTimer();
  __timerFunctClassPnt__ = this; // set global class pointer
  TimerInterval.setCallback(__timerFunct__);
}

void StepperDriver::moveTo(int32_t step)
{
  if (_targetPos != step)
  {
    _targetPos = step;
    // recompute speed and enable timer only if timer is not running:
    if (!_timerRunning)
    {
      _computeNewSpeed();
      _timerRunning = true;
      TimerInterval.setTimer(_stepInterval);
    }
  }
}
void StepperDriver::stop()
{
  if (_speed != 0.0)
  {
    int32_t stepsToStop = (int32_t)((_speed * _speed) / (2.0 * _acceleration)) + 1;
    if (_speed > 0)
      moveTo(_currentPos + stepsToStop);
    else
      moveTo(_currentPos - stepsToStop);
  }
}

void StepperDriver::setMaxSpeed(float speed)
{
  // check if 'speed' is an acceptable value. Make sure its positive:
  if (speed == 0)
    return;
  if (speed < 0.0)
    speed = -speed;
  if (speed != _maxSpeed)
  {
    float t_cmin = 1000000.0 / speed;

    if (_timerRunning)
      TimerInterval.disableTimer();

    _maxSpeed = speed;
    _cmin = t_cmin;

    if (_timerRunning)
      TimerInterval.enableTimer();

    // recompute '_n':
    if (_n > 0)
    {
      long t_n = (int32_t)((_speed * _speed) / (2.0 * _acceleration));

      if (_timerRunning)
        TimerInterval.disableTimer();

      _n = t_n;

      if (_timerRunning)
        TimerInterval.enableTimer();

      // recompute speed and enable timer only if timer is not running:
      if (!_timerRunning)
      {
        _computeNewSpeed();
        _timerRunning = true;
        TimerInterval.setTimer(_stepInterval);
      }
    }
  }
}
void StepperDriver::setAccel(float accel)
{
  // check if 'accel' is an acceptable value. Make sure its positive:
  if (accel == 0.0)
    return;
  if (accel < 0.0)
    accel = -accel;

  // only change the acceleration if it is a different value:
  if (accel != _acceleration)
  {
    float t_c0 = 0.676 * sqrt(2.0 / accel) * 1000000.0;
    float accelDiff = (_acceleration / accel);

    if (_timerRunning)
      TimerInterval.disableTimer();

    _n = _n * accelDiff;
    _c0 = t_c0;
    _acceleration = accel;

    if (_timerRunning)
      TimerInterval.enableTimer();

    // recompute speed and enable timer only if timer is not running:
    if (!_timerRunning)
    {
      _computeNewSpeed();
      _timerRunning = true;
      TimerInterval.setTimer(_stepInterval);
    }
  }
}

int32_t StepperDriver::step()
{
  return _currentPos;
}

unsigned long StepperDriver::_computeNewSpeed()
{
  int32_t distanceTo = (_targetPos - _currentPos);
  int32_t stepsToStop = (int32_t)((_speed * _speed) / (2.0 * _acceleration));

  if (distanceTo == 0 && stepsToStop <= 1)
  {
    // stop running stepper
    _stepInterval = 0;
    _speed = 0.0;
    _n = 0;
    return _stepInterval;
  }

  /*
    calculate 'n_':
    TODO: test this section:
  */
  if (distanceTo > 0)
  {
    // We are anticlockwise from the target
    // Need to go clockwise from here, maybe decelerate now
    if (_n > 0)
    {
      // Currently accelerating, need to decel now? Or maybe going the wrong way?
      if ((stepsToStop >= distanceTo) || !_direction /*CCW*/)
      {
        _n = -stepsToStop; // Start deceleration
        // Serial.print("CCW, decelerating: ");
        // Serial.println(_n);
      }
    }
    else if (_n < 0)
    {
      // Currently decelerating, need to accel again?
      if ((stepsToStop < distanceTo) && _direction /*CW*/)
      {
        _n = -_n; // Start acceleration
        // Serial.print("CW, accelerating: ");
        // Serial.println(_n);
      }
    }
  }
  else if (distanceTo < 0)
  {
    // We are clockwise from the target
    // Need to go anticlockwise from here, maybe decelerate
    if (_n > 0)
    {
      // Currently accelerating, need to decel now? Or maybe going the wrong way?
      if ((stepsToStop >= -distanceTo) || _direction /*CW*/)
      {
        _n = -stepsToStop; // Start deceleration
        // Serial.print("CW, decelerating: ");
        // Serial.println(_n);
      }
    }
    else if (_n < 0)
    {
      // Currently decelerating, need to accel again?
      if ((stepsToStop < -distanceTo) && !_direction /*CCW*/)
      {
        _n = -_n; // Start acceleration
        // Serial.print("CCW, accelerating: ");
        // Serial.println(_n);
      }
    }
  }

  /*
    calculate '_cn', or stepper interval(us):
  */
  if (_n == 0)
  {
    /*
      if '_n' == 0 then we are not moving, accelerating or decelerating.
      Initialize movement:
    */
    _cn = _c0; // Init with smallest step interval
    _direction = (distanceTo > 0) ? true /*CW*/ : false /*CCW*/;
  }
  else
  {
    _cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1));
    _cn = max(_cn, _cmin);
  }

  _n++;
  _stepInterval = _cn;
  _speed = 1000000.0 / _cn;

  /*
    '_speed' is a positive value right now.
    If the stepper is going CCW, make speed negative:
  */
  if (!_direction /*CCW*/)
    _speed = -_speed;

  // Serial.print("_speed: ");
  // Serial.print(_speed);
  // Serial.print("_direction: ");
  // Serial.println(_direction);
  // Serial.print("_acceleration: ");
  // Serial.println(_acceleration);
  // Serial.print("_cn: ");
  // Serial.println(_cn);
  // Serial.print("_c0: ");
  // Serial.println(_c0);
  // Serial.print("\t_n: ");
  // Serial.println(_n);
  // Serial.print("\t_stepInterval: ");
  // Serial.println(_stepInterval);
  // Serial.print("distanceTo: ");
  // Serial.println(distanceTo);
  // Serial.print("stepsToStop: ");
  // Serial.println(stepsToStop);
  // Serial.println("-----");

  return _stepInterval;
}

static void __timerFunct__()
{
  if (__timerFunctClassPnt__->_direction /*CW*/)
    (__timerFunctClassPnt__->_currentPos)++;
  else /*CCW*/
    (__timerFunctClassPnt__->_currentPos)--;

  __timerFunctClassPnt__->_stepCallback(__timerFunctClassPnt__->_direction);
  __timerFunctClassPnt__->_computeNewSpeed();

  if (__timerFunctClassPnt__->_stepInterval)
    TimerInterval.setTimer(__timerFunctClassPnt__->_stepInterval);
  else
  {
    TimerInterval.disableTimer();
    __timerFunctClassPnt__->_timerRunning = false;
  }
}
#endif
