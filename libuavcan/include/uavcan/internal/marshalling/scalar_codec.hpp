/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <stdint.h>
#include <limits>
#include <string>
#include <uavcan/internal/util.hpp>
#include <uavcan/internal/marshalling/bit_stream.hpp>

namespace uavcan
{

class ScalarCodec
{
    BitStream& stream_;

    template <int SIZE>
    static void swapByteOrder(uint8_t (&bytes)[SIZE])
    {
        for (int i = 0, j = SIZE - 1; i < j; i++, j--)
        {
            const uint8_t c = bytes[i];
            bytes[i] = bytes[j];
            bytes[j] = c;
        }
    }

    template <int BITLEN, int SIZE>
    static typename EnableIf<(BITLEN > 8)>::Type
    convertByteOrder(uint8_t (&bytes)[SIZE])
    {
#if defined(BYTE_ORDER) && defined(BIG_ENDIAN)
        static const bool big_endian = BYTE_ORDER == BIG_ENDIAN;
#else
        union { long int l; char c[sizeof(long int)]; } u;
        u.l = 1;
        const bool big_endian = u.c[sizeof(long int) - 1] == 1;
#endif
        if (big_endian)
            swapByteOrder(bytes);
    }

    template <int BITLEN, int SIZE>
    static typename EnableIf<(BITLEN <= 8)>::Type
    convertByteOrder(uint8_t (&bytes)[SIZE])
    {
        (void)bytes;
    }

    template <int BITLEN, typename T>
    static typename EnableIf<std::numeric_limits<T>::is_signed && ((sizeof(T) * 8) > BITLEN)>::Type
    fixTwosComplement(T& value)
    {
        StaticAssert<std::numeric_limits<T>::is_integer>::check(); // Not applicable to floating point types

        if (value & (T(1) << (BITLEN - 1)))                        // The most significant bit is set --> negative
            value |= 0xFFFFFFFFFFFFFFFF & ~((T(1) << BITLEN) - 1);
    }

    template <int BITLEN, typename T>
    static typename EnableIf<!std::numeric_limits<T>::is_signed || ((sizeof(T) * 8) == BITLEN)>::Type
    fixTwosComplement(T& value)
    {
        (void)value;
    }

    template <int BITLEN, typename T>
    static typename EnableIf<((sizeof(T) * 8) > BITLEN)>::Type
    clearExtraBits(T& value)
    {
        value &= (1 << BITLEN) - 1;  // Signedness doesn't matter
    }

    template <int BITLEN, typename T>
    static typename EnableIf<((sizeof(T) * 8) == BITLEN)>::Type
    clearExtraBits(T& value)
    {
        (void)value;
    }

    template <int BITLEN, typename T>
    void validate()
    {
        StaticAssert<((sizeof(T) * 8) >= BITLEN)>::check();
        StaticAssert<(BITLEN <= BitStream::MAX_BITS_PER_RW)>::check();
        StaticAssert<std::numeric_limits<T>::is_signed ? (BITLEN > 1) : 1>::check();
    }

public:
    ScalarCodec(BitStream& stream)
    : stream_(stream)
    { }

    template <int BITLEN, typename T>
    int encode(const T value)
    {
        validate<BITLEN, T>();
        union ByteUnion
        {
            T value;
            uint8_t bytes[sizeof(T)];
        } byte_union;
        byte_union.value = value;

        clearExtraBits<BITLEN>(byte_union.value);
        convertByteOrder<BITLEN>(byte_union.bytes);

        // Underlying stream class assumes that more significant bits have lower index, so we need to shift some.
        byte_union.bytes[BITLEN / 8] <<= (8 - (BITLEN % 8)) & 7;

        return stream_.write(byte_union.bytes, BITLEN);
    }

    template <int BITLEN, typename T>
    int decode(T& value)
    {
        validate<BITLEN, T>();
        union ByteUnion
        {
            T value;
            uint8_t bytes[sizeof(T)];
        } byte_union;
        std::fill(byte_union.bytes, byte_union.bytes + sizeof(T), 0);

        const int read_res = stream_.read(byte_union.bytes, BITLEN);
        if (read_res <= 0)
            return read_res;

        byte_union.bytes[BITLEN / 8] >>= (8 - (BITLEN % 8)) & 7;  // As in encode(), vice versa

        convertByteOrder<BITLEN>(byte_union.bytes);
        fixTwosComplement<BITLEN>(byte_union.value);

        value = byte_union.value;
        return read_res;
    }
};

}
