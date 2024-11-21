// BitED
//.h
#ifndef BitED_h
#define BitED_h

#include "Arduino.h"
#include "util/bitArray.h"

/*
  BitED_class:
    set and verify the error detection in a packet bit-array
    This class uses a hybrid fo parity bits and checksums to achieve reliable Error detection
*/
class BitED_class
{
public:
  /*
    packet will be modified as well as bitlength adn wil contain only data bits if ed check is successful
  */
  bool checkED(uint8_t *packet, uint8_t &bitLength, uint8_t ED_bitShift)
  {
    /*
      create bit mask for finding remainder when dividing:
      calculate divisor:
    */
    const uint8_t divideMask = ~(0xff << ED_bitShift);
    const uint8_t divisor = (1 << ED_bitShift);

    uint8_t lBit = 0;
    uint8_t lBitLength = 0;
    while (bitLength >> lBitLength)
    {
      lBitLength++;
    }
    for (uint8_t x = 0; x < lBitLength; x++)
    {
      if (READ_BIT(packet, x))
        lBit |= (1 << x);
    }

    /*
      check if lBit matches packet length
    */
    if (lBit != bitLength)
      return false;

    /*
      check if packet length is valid:
    */
    if (bitLength && ((bitLength - lBitLength) % (divisor + 1)))
    {

      const uint8_t dataBits = _dataBits(bitLength, lBitLength, ED_bitShift);

      uint8_t bit_0_count = 0;
      uint8_t bit_1_count = 0;

      for (uint8_t x = 0; x < lBitLength; x++)
      {
        READ_BIT(packet, x) ? bit_1_count++ : bit_0_count++;
      }

      uint8_t bitCount = 0;
      uint8_t parityCount = 0;

      bool oneZeroParity = (dataBits ? READ_BIT(packet, _dataBitAddress(0, lBitLength, ED_bitShift)) : false);

      while (dataBits - bitCount >= divisor)
      {
        bool bitParity = !oneZeroParity;

        /*
          do parity calculations in the for loop and compare:
        */
        for (uint8_t x = 0; x < divisor; x++)
        {
          bool bitHold = READ_BIT(packet, _dataBitAddress(bitCount, lBitLength, ED_bitShift));
          if (bitHold == oneZeroParity)
          {
            bitParity = !bitParity;
          }
          bitHold ? bit_1_count++ : bit_0_count++;
          WRITE_BIT(packet, bitCount, bitHold); // shift all data back to correct position
          bitCount++;
        }
        if (READ_BIT(packet, _parityBitAddress(parityCount, lBitLength, ED_bitShift)) != bitParity)
          return false;
        bitParity ? bit_1_count++ : bit_0_count++;
        oneZeroParity = !oneZeroParity;
        parityCount++;
      }
      while (bitCount < dataBits)
      {
        WRITE_BIT(packet, bitCount, READ_BIT(packet, _dataBitAddress(bitCount, lBitLength, ED_bitShift)));
        READ_BIT(packet, bitCount) ? bit_1_count++ : bit_0_count++;
        bitCount++;
      }

      if (READ_BIT(packet, bitLength - 1) != (oneZeroParity ? bit_1_count & 1 : !(bit_0_count & 1)))
        return false;

      bitLength = dataBits;
      return true;
    }
    return false;
  }
  /*
  packet must be large enough to accommodate all parity bits!
  minimum packet length = (dataBits + (dataBits / ED_bits) + (bool)(dataBits % ED_bits) + 1)
  */
  void setED(uint8_t *packet, uint8_t &dataBits, uint8_t ED_bitShift)
  {
    /*
      create bit mask for finding remainder when dividing:
      calculate divisor:
    */
    const uint8_t divideMask = ~(0xff << ED_bitShift);
    const uint8_t divisor = (1 << ED_bitShift);

    /*
      calculate the raw packet length(without lBits) ahead of time
      calculate lBits length
      calculate full packet length(with lBits)
    */
    const uint8_t rawPacketLength = (dataBits + (dataBits >> ED_bitShift) + 1);
    const uint8_t lBitLength = _lBitLength(rawPacketLength);
    const uint8_t fullPacketLength = (lBitLength + rawPacketLength);
    /*
      rearrange all data bits in packet buffer:
    */
    for (uint8_t x = 0; x < dataBits; x++)
    {
      WRITE_BIT(packet, _dataBitAddress(dataBits - (x + 1), lBitLength, ED_bitShift), READ_BIT(packet, dataBits - (x + 1)));
    }

    uint8_t bit_0_count = 0;
    uint8_t bit_1_count = 0;
    /*
      set packet length check:
    */
    for (uint8_t x = 0; x < lBitLength; x++)
    {
      WRITE_BIT(packet, x, (fullPacketLength & (1 << x)));
      READ_BIT(packet, x) ? bit_1_count++ : bit_0_count++;
    }

    uint8_t bitCount = 0;
    uint8_t parityCount = 0;

    bool oneZeroParity = (dataBits ? READ_BIT(packet, _dataBitAddress(0, lBitLength, ED_bitShift)) : false);

    while (dataBits - bitCount >= divisor)
    {
      bool bitParity = !oneZeroParity;

      /*
        do parity calculations in the for loop:
      */
      for (uint8_t x = 0; x < divisor; x++)
      {
        bool bitHold = READ_BIT(packet, _dataBitAddress(bitCount, lBitLength, ED_bitShift));
        if (bitHold == oneZeroParity)
        {
          bitParity = !bitParity;
        }
        bitHold ? bit_1_count++ : bit_0_count++;
        bitCount++;
      }
      WRITE_BIT(packet, _parityBitAddress(parityCount, lBitLength, ED_bitShift), bitParity);
      bitParity ? bit_1_count++ : bit_0_count++;
      oneZeroParity = !oneZeroParity;
      parityCount++;
    }
    while (bitCount < dataBits)
    {
      READ_BIT(packet, _dataBitAddress(bitCount, lBitLength, ED_bitShift)) ? bit_1_count++ : bit_0_count++;
      bitCount++;
    }

    /*
      set master parity bit:
    */
    WRITE_BIT(packet, fullPacketLength - 1, (oneZeroParity ? bit_1_count & 1 : !(bit_0_count & 1)));
    dataBits = fullPacketLength;
  }

private:
  inline uint8_t _dataBitAddress(uint8_t dataBit, uint8_t lBitLength, uint8_t ED_bitShift) __attribute__((always_inline))
  {
    return (lBitLength + dataBit + (dataBit >> ED_bitShift));
  }
  inline uint8_t _parityBitAddress(uint8_t parityBit, uint8_t lBitLength, uint8_t ED_bitShift) __attribute__((always_inline))
  {
    const uint8_t divisor = (1 << ED_bitShift);
    return (lBitLength + divisor + (parityBit * (divisor + 1)));
  }
  inline uint8_t _dataBits(uint8_t totalBits, uint8_t lBitLength, uint8_t ED_bitShift) __attribute__((always_inline))
  {
    const uint8_t divisor = (1 << ED_bitShift);
    totalBits -= (lBitLength + 1);
    return (totalBits - (((totalBits) / (divisor + 1))));
  }
  inline uint8_t _lBitLength(uint8_t totalBits)
  {
    uint8_t bitCnt = 0;
    while (totalBits >> bitCnt)
    {
      bitCnt++;
      totalBits++;
    }
    return bitCnt;
  }
};
extern BitED_class BitED;

//.cpp

#endif
