#pragma once

#include <stdint.h>

// ---------------------------------------------------------
// Extract unsigned CAN signal
// ---------------------------------------------------------
inline uint64_t extractUnsigned(
    const uint8_t data[8],
    int startBit,
    int length,
    bool littleEndian)
{
    uint64_t value = 0;

    if (littleEndian)
    {
        // Intel (Little Endian): startBit = LSB, bit numbers increase
        for (int i = 0; i < length; i++)
        {
            int bitPos = startBit + i;
            int byte = bitPos / 8;
            int bit  = bitPos % 8;

            if (data[byte] & (1 << bit))
                value |= (1ULL << i);
        }
    }
    else
    {
        // Motorola (Big Endian): startBit = MSB, DBC sawtooth numbering.
        // Walk MSB->LSB of the signal; within a byte the bit index is
        // (bitPos % 8), and at a byte boundary (bit 0) jump to the next
        // byte's MSB (+15).
        int bitPos = startBit;
        for (int i = 0; i < length; i++)
        {
            int byte = bitPos / 8;
            int bit  = bitPos % 8;

            if (data[byte] & (1 << bit))
                value |= (1ULL << (length - 1 - i));

            if (bit == 0) bitPos += 15;   // cross to next byte, MSB
            else          bitPos -= 1;
        }
    }

    return value;
}

// ---------------------------------------------------------
// Extract signed CAN signal
// ---------------------------------------------------------
inline int64_t extractSigned(
    const uint8_t data[8],
    int startBit,
    int length,
    bool littleEndian)
{
    uint64_t value =
        extractUnsigned(
            data,
            startBit,
            length,
            littleEndian);

    // Sign extension
    if (length < 64)
    {
        uint64_t signBit = 1ULL << (length - 1);

        if (value & signBit)
        {
            value |= (~0ULL << length);
        }
    }

    return (int64_t)value;
}
