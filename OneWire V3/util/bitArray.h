// bitArray
#ifndef bitArray_h
#define bitArray_h
/*
  'READ_BIT' and 'WRITE_BIT' are useful tools only accessible in this file
    'READ_BIT' will target a specific boolean bit from and array of bytes
      e.g: bit 12 = 5th bit in the 2nd byte of a given array
    'WRITE_BIT' will write a boolean value to a targeted bit in a byte-array
*/
inline bool READ_BIT(uint8_t *, uint8_t) __attribute__((always_inline));
bool READ_BIT(uint8_t *arr, uint8_t bit)
{
  return ((arr[bit >> 3] & (1 << (bit & B111))));
}
inline bool READ_BIT(uint8_t *, uint8_t, bool) __attribute__((always_inline));
void WRITE_BIT(uint8_t *arr, uint8_t bit, bool val)
{
  val ? (arr[bit >> 3] |= (1 << (bit & B111))) : (arr[bit >> 3] &= ~(1 << (bit & B111)));
}
#endif