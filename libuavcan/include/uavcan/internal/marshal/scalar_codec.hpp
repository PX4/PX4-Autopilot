/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <stdint.h>
#include <limits>
#include <string>
#include <uavcan/util/compile_time.hpp>
#include <uavcan/internal/marshal/bit_stream.hpp>

namespace uavcan
{

class ScalarCodec
{
    BitStream& stream_;

    template <int Size>
    static void swapByteOrder(uint8_t (&bytes)[Size])
    {
        for (int i = 0, j = Size - 1; i < j; i++, j--)
        {
            const uint8_t c = bytes[i];
            bytes[i] = bytes[j];
            bytes[j] = c;
        }
    }

    template <int BitLen, int Size>
    static typename EnableIf<(BitLen > 8)>::Type
    convertByteOrder(uint8_t (&bytes)[Size])
    {
#if defined(BYTE_ORDER) && defined(BIG_ENDIAN)
        static const bool big_endian = BYTE_ORDER == BIG_ENDIAN;
#else
        union { long int l; char c[sizeof(long int)]; } u;
        u.l = 1;
        const bool big_endian = u.c[sizeof(long int) - 1] == 1;
#endif
        /*
         * I didn't have any big endian machine nearby, so big endian support wasn't tested yet.
         * It is likely to be OK anyway, so feel free to remove this assert() as needed.
         */
        assert(big_endian == false);

        if (big_endian)
            swapByteOrder(bytes);
    }

    template <int BitLen, int Size>
    static typename EnableIf<(BitLen <= 8)>::Type
    convertByteOrder(uint8_t (&bytes)[Size])
    {
        (void)bytes;
    }

    template <int BitLen, typename T>
    static typename EnableIf<std::numeric_limits<T>::is_signed && ((sizeof(T) * 8) > BitLen)>::Type
    fixTwosComplement(T& value)
    {
        StaticAssert<std::numeric_limits<T>::is_integer>::check(); // Not applicable to floating point types

        if (value & (T(1) << (BitLen - 1)))                        // The most significant bit is set --> negative
            value |= 0xFFFFFFFFFFFFFFFF & ~((T(1) << BitLen) - 1);
    }

    template <int BitLen, typename T>
    static typename EnableIf<!std::numeric_limits<T>::is_signed || ((sizeof(T) * 8) == BitLen)>::Type
    fixTwosComplement(T& value)
    {
        (void)value;
    }

    template <int BitLen, typename T>
    static typename EnableIf<((sizeof(T) * 8) > BitLen)>::Type
    clearExtraBits(T& value)
    {
        value &= (T(1) << BitLen) - 1;  // Signedness doesn't matter
    }

    template <int BitLen, typename T>
    static typename EnableIf<((sizeof(T) * 8) == BitLen)>::Type
    clearExtraBits(T& value)
    {
        (void)value;
    }

    template <int BitLen, typename T>
    void validate()
    {
        StaticAssert<((sizeof(T) * 8) >= BitLen)>::check();
        StaticAssert<(BitLen <= BitStream::MaxBitsPerRW)>::check();
        StaticAssert<std::numeric_limits<T>::is_signed ? (BitLen > 1) : 1>::check();
    }

public:
    ScalarCodec(BitStream& stream)
    : stream_(stream)
    { }

    template <int BitLen, typename T>
    int encode(const T value)
    {
        validate<BitLen, T>();
        union ByteUnion
        {
            T value;
            uint8_t bytes[sizeof(T)];
        } byte_union;
        byte_union.value = value;

        clearExtraBits<BitLen>(byte_union.value);
        convertByteOrder<BitLen>(byte_union.bytes);

        // Underlying stream class assumes that more significant bits have lower index, so we need to shift some.
        byte_union.bytes[BitLen / 8] <<= (8 - (BitLen % 8)) & 7;

        return stream_.write(byte_union.bytes, BitLen);
    }

    template <int BitLen, typename T>
    int decode(T& value)
    {
        validate<BitLen, T>();
        union ByteUnion
        {
            T value;
            uint8_t bytes[sizeof(T)];
        } byte_union;
        std::fill(byte_union.bytes, byte_union.bytes + sizeof(T), 0);

        const int read_res = stream_.read(byte_union.bytes, BitLen);
        if (read_res <= 0)
            return read_res;

        byte_union.bytes[BitLen / 8] >>= (8 - (BitLen % 8)) & 7;  // As in encode(), vice versa

        convertByteOrder<BitLen>(byte_union.bytes);
        fixTwosComplement<BitLen>(byte_union.value);

        value = byte_union.value;
        return read_res;
    }
};

}
