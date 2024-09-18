//
// Created by biao on 24-7-1.
//

#ifndef STAND_GO2_CRC32_H
#define STAND_GO2_CRC32_H
#include <cstdint>

static uint32_t crc32_core(const uint32_t *ptr, uint32_t len) {
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else {
                CRC32 <<= 1;
            }

            if (data & xbit) CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

#endif  // STAND_GO2_CRC32_H
