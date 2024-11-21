// SimpleFilter
//.h
#ifndef SimpleFilter_h
#define SimpleFilter_h

#include "Arduino.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\Utility\bitArray.h"

class RollingAverage_8
{
public:
  /*
    RollingAverage_8:
      set number of data point to store

      inputs: ( (uint16_t) number of data points to average out. Must be greater than 0 )
  */
  RollingAverage_8(uint16_t dataPoints)
  {
    if (!dataPoints)
      dataPoints = 1;
    _dataPoints = dataPoints;
  }

  /*
    avg:
      input new data number to add to the average
      outputs the new average result

      inputs: ( (uint8_t) new data point value )
      outputs: (uint8_t) new average result *(also can be acquired via (uint8_t)avg(void)-method)
  */
  uint8_t avg(uint8_t newVal)
  {
    /*
      subtract current average from total
      equivalent to: _total - _total / _dataPoints
    */
    _total -= _curAvg;

    _total += newVal;
    _curAvg = _total / _dataPoints;

    return _curAvg;
  }

  /*
    avg:
      outputs the most recent average result

      outputs: (uint8_t) most recent average result
  */
  uint8_t avg(void)
  {
    return _curAvg;
  }

  /*
    setDataPoint:
      set new data point range
      will retain current average value

      input: ( (uint16_t) new data points value. Must be greater than 0 )
  */
  void setDataPoint(uint16_t newDataPoints)
  {
    if (!newDataPoints)
      newDataPoints = 1;
    _dataPoints = newDataPoints;
    _total = _curAvg * _dataPoints;
  }

private:
  uint16_t _total = 0; // can *safely hold 256 data points max (2^16 / 2^8 = 256)

  uint16_t _dataPoints;
  uint8_t _curAvg = 0;
};

class RollingAverage_16
{
public:
  /*
    RollingAverage_16:
      set number of data point to store

      inputs: ( (uint32_t) number of data points to average out. Must be greater than 0 )
  */
  RollingAverage_16(uint32_t dataPoints)
  {
    if (!dataPoints)
      dataPoints = 1;
    _dataPoints = dataPoints;
  }

  /*
    avg:
      input new data number to add to the average
      outputs the new average result

      inputs: ( (uint16_t) new data point value )
      outputs: (uint16_t) new average result *(also can be acquired via (uint16_t)avg(void)-method)
  */
  uint16_t avg(uint16_t newVal)
  {
    /*
      subtract current average from total
      equivalent to: _total - _total / _dataPoints
    */
    _total -= _curAvg;

    _total += newVal;
    _curAvg = _total / _dataPoints;

    return _curAvg;
  }

  /*
    avg:
      outputs the most recent average result

      outputs: (uint16_t) most recent average result
  */
  uint16_t avg(void)
  {
    return _curAvg;
  }

  /*
    setDataPoint:
      set new data point range
      will retain current average value

      input: ( (uint32_t) new data points value. Must be greater than 0 )
  */
  void setDataPoint(uint32_t newDataPoints)
  {
    if (!newDataPoints)
      newDataPoints = 1;
    _dataPoints = newDataPoints;
    _total = _curAvg * _dataPoints;
  }

private:
  uint32_t _total = 0; // can *safely hold 65,536 data points max (2^32 / 2^16 = 65,536)

  uint32_t _dataPoints;
  uint16_t _curAvg = 0;
};

#define BufferAverage_Bool_ByteSize(__dataPoints__) ((__dataPoints__ >> 3) + (bool)(__dataPoints__ & B111))

class BufferAverage_Bool
{
public:
  BufferAverage_Bool(uint8_t *buffer, uint16_t dataPoints)
  {
    _dataPoints = dataPoints;
    _buffer = buffer;
    const uint16_t arrayByteLen = BufferAverage_Bool_ByteSize(_dataPoints);

    for (uint16_t x = 0; x < arrayByteLen; x++)
      _buffer[x] = 0;
  }

  uint16_t write(bool newVal)
  {
    _avgPoints -= READ_BIT_16(_buffer, _dataStart);
    WRITE_BIT_16(_buffer, _dataStart, newVal);
    _avgPoints += READ_BIT_16(_buffer, _dataStart);

    _dataStart++;
    if (_dataStart >= _dataPoints)
      _dataStart = 0;

    return _avgPoints;
  }

  uint16_t avg()
  {
    return _avgPoints;
  }

private:
  uint8_t *_buffer;
  uint16_t _dataPoints;
  uint16_t _dataStart = 0;
  uint16_t _avgPoints = 0;
};

class BufferAverage_8
{
public:
  BufferAverage_8(uint8_t *buffer, uint16_t dataPoints)
  {
    _dataPoints = dataPoints;
    _buffer = buffer;

    for (uint16_t x = 0; x < _dataPoints; x++)
      _buffer[x] = 0;
  }

  uint8_t write(bool newVal)
  {
    _total -= _buffer[_dataStart];
    _buffer[_dataStart] = newVal;
    _total += _buffer[_dataStart];

    _avg = _total / _dataPoints;

    _dataStart++;
    if (_dataStart >= _dataPoints)
      _dataStart = 0;

    return _avg;
  }

  uint8_t avg()
  {
    return _avg;
  }

private:
  uint8_t *_buffer;
  uint16_t _dataPoints;
  uint16_t _dataStart = 0;
  uint32_t _total = 0;
  uint8_t _avg = 0;
};

class BufferAverage_16
{
public:
  BufferAverage_16(uint16_t *buffer, uint16_t dataPoints)
  {
    _dataPoints = dataPoints;
    _buffer = buffer;

    for (uint16_t x = 0; x < _dataPoints; x++)
      _buffer[x] = 0;
  }

  uint16_t write(bool newVal)
  {
    _total -= _buffer[_dataStart];
    _buffer[_dataStart] = newVal;
    _total += _buffer[_dataStart];

    _avg = _total / _dataPoints;

    _dataStart++;
    if (_dataStart >= _dataPoints)
      _dataStart = 0;

    return _avg;
  }

  uint16_t avg()
  {
    return _avg;
  }

private:
  uint16_t *_buffer;
  uint16_t _dataPoints;
  uint16_t _dataStart = 0;
  uint32_t _total = 0;
  uint16_t _avg = 0;
};

#endif
