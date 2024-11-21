// StepperQueue
//.h
#ifndef StepperQueue_h
#define StepperQueue_h

#include "Arduino.h"

// forward declare 'StepperQueue' for 'StepperStruct':
class StepperQueue;

struct StepperStruct
{
  friend class StepperQueue;

private:
  int8_t id = 5;
  int32_t targetPos = 0;
};

class StepperQueue
{
public:
  /*
    StepperQueue():
      initialize queue array

      inputs: ( (StepperStruct*)pointer of 'StepperStruct' array,
                (uint8_t)number of items in the 'StepperStruct' array )

    example:
      StepperStruct stepperStructObj[6];
      StepperQueue stepperQueueObj(stepperStructObj, 6);
  */
  StepperQueue(StepperStruct *, uint8_t);

  /*
    write():
      add a new item to the queue or modify an existing item
      in an item already exists, it will be modified, if not, the new item will be added to the back of the list given enough space

      inputs: ( (uint8_t)id value of item (must be less than number of items in the 'StepperStruct' array),
                (int32_t)value to write to the item )
  */
  void write(uint8_t, int32_t);
  /*
    read():
      read the first item in the list. Return id and corresponding value as through reference variables
      if no items are present in the list, method will return false and reference variable will not be modified
      After reading the item, it will be deleted and the next item in the list(if present) will be read instead on the next call.

      inputs: ( (uint8_t&)id reference value,
                (int32_t&)corresponding value reference value )

      returns: true if an item was present in the list and values were returned via reference values, false otherwise.
  */
  bool read(uint8_t &, int32_t &);

  /*
    available():
      returns: true if an items are available to be read in the list, false otherwise.
  */
  bool available();

private:
  StepperStruct *_stepperStructPnt;
  uint8_t _stepperStructCount;
  uint8_t _stepperStructStart = 0;
  bool _available = false;
};

//.cpp

StepperQueue::StepperQueue(StepperStruct *stepperStructPnt, uint8_t stepperStructCount)
{
  _stepperStructPnt = stepperStructPnt;
  _stepperStructCount = stepperStructCount;

  // initialize all items in array:
  for (uint8_t x = 0; x < _stepperStructCount; x++)
    _stepperStructPnt[x].id = -1;
}

void StepperQueue::write(uint8_t id, int32_t val)
{
  if (id < _stepperStructCount)
    for (uint8_t x = 0; x < _stepperStructCount; x++)
    {
      uint8_t add = (_stepperStructStart + x) % _stepperStructCount;

      if (_stepperStructPnt[add].id == id)
      {
        _stepperStructPnt[add].targetPos = val;
        return;
      }
      if (_stepperStructPnt[add].id == -1)
      {
        _stepperStructPnt[add].id = id;
        _stepperStructPnt[add].targetPos = val;
        _available = true;
        return;
      }
    }
}
bool StepperQueue::read(uint8_t &idRet, int32_t &valRet)
{
  if (_stepperStructPnt[_stepperStructStart].id != -1)
  {
    idRet = _stepperStructPnt[_stepperStructStart].id;
    valRet = _stepperStructPnt[_stepperStructStart].targetPos;
    _stepperStructPnt[_stepperStructStart].id = -1;
    _stepperStructStart++;
    _stepperStructStart %= _stepperStructCount;
    if (_stepperStructPnt[_stepperStructStart].id == -1)
      _available = false;
    return true;
  }
  return false;
}

bool StepperQueue::available()
{
  return _available;
}

#endif
