// CircBuffer
//.h
#ifndef CircBuffer_h
#define CircBuffer_h

#include "Arduino.h"

class CircBuffer
{
public:
  /*
    CircBuffer():
      initialize buffer. give array pointer and total available bytes in array:

      inputs: ( (uint8_t*) array pointer,
                (uint8_t) totalBytes available in array )
  */
  CircBuffer(uint8_t *bufferPnt, uint8_t totalBytes);

  /*
    write():
      write an array/packet into the buffer

      inputs: ( (uint8_t*) array to write,
                (uint8_t) bytes to write )

      returns: (bool) write status
                true = write successful
                false = not enough space available / write failed
  */
  bool write(uint8_t *arr, uint8_t bytes);

  /*
    read():
      read next packet / array in buffer

      inputs: ( (uint8_t*) array to return next message into,
                (uint8_t&) bytes of available data )

      returns: (bool) read status
                true = message available
                false = no message available
  */
  bool read(uint8_t *arr, uint8_t &bytes);

  /*
    available():
      returns wether data is available to read

      returns: (bool) read availability
                true = message available to read(used 'read()' to retrieve message)
                false = no message available to read
  */
  bool available();

private:
  uint8_t *_buffer;
  uint8_t _totalBytes;
  uint16_t _start = 0;
  uint16_t _used = 0;
};

//.cpp

CircBuffer::CircBuffer(uint8_t *bufferPnt, uint8_t totalBytes)
{
  _buffer = bufferPnt;
  _totalBytes = totalBytes;
}

bool CircBuffer::write(uint8_t *arr, uint8_t bytes)
{
  if ((_totalBytes - _used) > bytes) // available space needs to be greater by at least one byte
  {
    uint16_t endAdd = _used + _start;

    if (endAdd >= _totalBytes)
      endAdd -= _totalBytes;

    _buffer[endAdd] = bytes;
    for (uint8_t x = 0; x < bytes; x++)
    {
      uint16_t newAdd = (endAdd + (x + 1));
      if (newAdd >= _totalBytes)
        newAdd -= _totalBytes;

      _buffer[newAdd] = arr[x];
    }
    _used += (bytes + 1);
    return true;
  }
  return false;
}

bool CircBuffer::read(uint8_t *arr, uint8_t &bytes)
{
  if (_used)
  {
    uint8_t packLen = _buffer[_start++];
    _used--;

    if (_start >= _totalBytes)
      _start -= _totalBytes;

    for (uint8_t x = 0; x < packLen; x++)
    {
      arr[x] = _buffer[_start++];

      if (_start >= _totalBytes)
        _start -= _totalBytes;

      _used--;
    }
    bytes = packLen;
    return true;
  }
  else
  {
    bytes = 0;
    return false;
  }
}

bool CircBuffer::available()
{
  return (bool)_used;
}
#endif
