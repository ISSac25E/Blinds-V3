// file_name
//.h
#ifndef StepperDriver_h
#define StepperDriver_h

#include "Arduino.h"

class StepperDriver
{
public:
  /*
    StepperDriver():
      Input: Step CallBack function

      callback function will be called each time a step needs to be executed
      callback function inputs ((bool)true if even step. false if odd step,
                                (bool)true if step is in cw direction. false if cw direction)

      typically, on an 'even step,' the step pin will be pulled high and on an 'odd step' the pin will be pulled low.
      both adjacent 'even' AND 'odd' steps will have the same exact intervals. A new interval will be calculated only after an odd step is completed
  */
  StepperDriver(void (*)(bool, bool));

  /*
    setSpeed():
      setup speed of stepper. default value is 1.0

      inputs: ( (float)set max speed)(steps-per-second)(permits steps slower than 1-per-second) )

      max speed must be greater than 0.0, if not, method will return without setting anything.
  */
  void setSpeed(float);

  /*
    setAccel():
      set primary acceleration value for stepper motor

      inputs: ( (float)set acceleration(steps-per-second)(smaller number = slow acceleration. larger number = faster acceleration) )

      acceleration value must be greater than 0.0, if not, method will return without setting anything.
  */
  void setAccel(float);

  /*
    setSecondaryAccel():
      set secondary acceleration value for stepper motor used for stopping or changing directions

      inputs: ( (float)set acceleration(steps-per-second)(smaller number = slow acceleration. larger number = faster acceleration) )

      acceleration value must be greater than 0.0, if not, method will return without setting anything.
  */
  void setSecondaryAccel(float);

  /*
    handle():
      run as often as possible.
      will handle step calculation.
  */
  void handle();

  /*
    setStepper():
      set a new 'current' value of the stepper
      the new value will be modified as needed as the stepper moves.
      as soon as this method is called, all previous stepping immediately stop

      inputs: ((int32_t&)new stepper reference value)
  */
  void setStepper(int32_t &);

  /*
    moveTo():
      Input: What position to set Stepper too
      will automatically handle acceleration and deceleration using the 'primary' speed profile
      in the event that the new position is opposite of the stepper's direction of travel,
      the motor will be first slowed to a stop using the 'secondary' profile before continuing the correct direction on the 'primary' profile
  */
  void moveTo(int32_t);
  /*
    stop()
      decelerates stepper to a stop using 'secondary' speed profile
  */
  void stop();

  /*
    ready():
      returns: true = stepper stopped, ready to be transferred to another stepper or for another instruction. false = stepper in motion
  */
  bool ready();

private:
  /*
    _computeNewSpeed():
      computes a new step for stepper
      sets all proper constants
      return: new step interval in microseconds(us)
  */
  unsigned long _computeNewSpeed();
  /*
    _runStep():
      Runs current step and checks if another step is needed
      returns: true if another step required, false otherwise
  */
  bool _runStep();

  /*
    function pointer
    called every time a new step is required
    input: (  true = even step, false = odd step,
              true = CW(clockwise), false = CCW(counter-clockwise)  )
  */
  void (*_stepCallback)(bool, bool);

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
    the current acceleration value for the stepper used to accelerate and decelerate the stepper in steps-per-second
    must be > 0
  */
  float _acceleration;
  /*
    the preset acceleration values for the stepper
    this will be used to set '_acceleration'
    must be > 0
  */
  float _acceleration_s[2];
  /*
    determine which acceleration profile is currently used
    false = primary profile
    true = secondary profile
  */
  bool _profileSelect;
  /*
    last step time in microseconds(us)
  */
  uint32_t _lastStepTime;

  /* the step counter for speed calculations */
  long _n;

  /*
    current initial step size in microseconds
  */
  float _c0;
  /*
    preset initial step size in microseconds for both acceleration profiles
    this will be used to set '_c0'
  */
  float _c0_s[2];

  /*
    last step size in microseconds
  */
  float _cn;

  /*
    min step size in microseconds based on maxSpeed
  */
  float _cmin; // at max speed

  /*
    pointer to the current (absolute) position of stepper in steps
  */
  int32_t *_currentPos;
  /*
    target position of stepper in steps

    the library will use target and current positions to calculate how to move the stepper taking into account speed acceleration and deceleration
  */
  int32_t _targetPos;
};

//.cpp
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
}

void StepperDriver::handle()
{
  if (_runStep())
    _computeNewSpeed();
}

void StepperDriver::moveTo(int32_t step)
{
  if (_targetPos != step)
  {
    _targetPos = step;
    _computeNewSpeed();
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
    _maxSpeed = speed;
    _cmin = 1000000.0 / speed;

    // recompute '_n':
    if (_n > 0)
    {
      _n = (int32_t)((_speed * _speed) / (2.0 * _acceleration));

      // recompute speed again:
      _computeNewSpeed();
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
  if (accel != _acceleration_s[0])
  {
    _c0_s[0] = 0.676 * sqrt(2.0 / accel) * 1000000.0;
    _acceleration_s[0] = accel;

    /*
      check if this profile is currently selected
      If so, change values
    */
    if (!_profileSelect /*primary profile*/)
    {
      _n = _n * (_acceleration / _acceleration_s[0]);
      _c0 = _c0_s[0];
      _acceleration = _acceleration_s[0];

      // recompute speed again:
      _computeNewSpeed();
    }
  }
}
void StepperDriver::setSecondaryAccel(float accel)
{
  // check if 'accel' is an acceptable value. Make sure its positive:
  if (accel == 0.0)
    return;
  if (accel < 0.0)
    accel = -accel;

  // only change the acceleration if it is a different value:
  if (accel != _acceleration_s[1])
  {
    _c0_s[1] = 0.676 * sqrt(2.0 / accel) * 1000000.0;
    _acceleration_s[1] = accel;

    /*
      check if this profile is currently selected
      If so, change values
    */
    if (_profileSelect /*secondary profile*/)
    {
      _n = _n * (_acceleration / _acceleration_s[1]);
      _c0 = _c0_s[1];
      _acceleration = _acceleration_s[1];

      // recompute speed again:
      _computeNewSpeed();
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

  /*
    check if acceleration has to change:
  */
  if (_n == 0)
  {
    
  }

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
  */
  if (distanceTo > 0)
  {
    // We are anticlockwise from the target
    // Need to go clockwise from here, maybe decelerate now
    if (_n > 0)
    {
      // Currently accelerating, need to decel now? Or maybe going the wrong way?
      if ((stepsToStop >= distanceTo) || !_direction /*CCW*/)
        _n = -stepsToStop; // Start deceleration
    }
    else if (_n < 0)
    {
      // Currently decelerating, need to accel again?
      if ((stepsToStop < distanceTo) && _direction /*CW*/)
        _n = -_n; // Start acceleration
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
        _n = -stepsToStop; // Start deceleration
    }
    else if (_n < 0)
    {
      // Currently decelerating, need to accel again?
      if ((stepsToStop < -distanceTo) && !_direction /*CCW*/)
        _n = -_n; // Start acceleration
    }
  }

  /*
    calculate '_cn', or stepper interval(us):
  */
  if (_n == 0)
  {
    _cn = _c0;
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

  Serial.print("_speed: ");
  Serial.print(_speed);
  // Serial.print("_direction: ");
  // Serial.println(_direction);
  // Serial.print("_acceleration: ");
  // Serial.println(_acceleration);
  // Serial.print("_cn: ");
  // Serial.println(_cn);
  // Serial.print("_c0: ");
  // Serial.println(_c0);
  Serial.print("\t_n: ");
  Serial.print(_n);
  Serial.print("\t_stepInterval: ");
  Serial.println(_stepInterval);
  // Serial.print("distanceTo: ");
  // Serial.println(distanceTo);
  // Serial.print("stepsToStop: ");
  // Serial.println(stepsToStop);
  // Serial.println("-----");

  return _stepInterval;
}
bool StepperDriver::_runStep()
{
  if (!_stepInterval)
    return false;

  uint32_t time = micros();
  if (time - _lastStepTime >= _stepInterval)
  {
    if (_direction /*CW*/)
      _currentPos++;
    else /*CCW*/
      _currentPos--;

    _stepCallback(_speed > 0);

    _lastStepTime = time;

    return true;
  }
  else
  {
    return false;
  }
}
#endif
