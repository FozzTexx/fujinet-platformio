// FIXME - consolidate with fuji_endian.h

#ifndef ENDIANNESS_H
#define ENDIANNESS_H

#include "fuji_endian.h"

// Returns a uint16 value given two bytes in high-low order
#define UINT16_FROM_HILOBYTES(high, low) ((uint16_t)high << 8 | low)

// Returns the high byte (MSB) of a uint16 value
#define HIBYTE_FROM_UINT16(value) ((uint8_t)((value >> 8) & 0xFF))
// Returns the low byte (LSB) of a uint16 value
#define LOBYTE_FROM_UINT16(value) ((uint8_t)(value & 0xFF))

// Returns a uint16 value from a pointer to two bytes in little-ending order
#define UINT16_FROM_LOHI_BYTEPTR(bytep) ((uint16_t)(*(bytep + 1)) << 8 | (*(bytep + 0)))
// Returns a uint32 value from a pointer to four bytes in little-ending order
#define UINT32_FROM_LOHI_BYTEPTR(bytep) ((uint32_t)(*(bytep + 3)) << 24 | (uint32_t)(*(bytep + 2)) << 16 | (uint32_t)(*(bytep + 1)) << 8 | (*(bytep + 0)))

// Takes UINT32 value and pushes it into 4 consecutive bytes in little-endian order
#define UINT32_TO_LOHI_BYTEPTR(value, bytep) \
    {                                             \
        (bytep)[0] = value & 0xFFUL;              \
        (bytep)[1] = value >> 8 & 0xFFUL;         \
        (bytep)[2] = value >> 16 & 0xFFUL;        \
        (bytep)[3] = value >> 24 & 0xFFUL;        \
    }

#endif /* ENDIANNESS_H */
