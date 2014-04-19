/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/marshal/scalar_codec.hpp>

namespace uavcan
{

void ScalarCodec::swapByteOrder(uint8_t* const bytes, const unsigned len)
{
    assert(bytes);
    for (int i = 0, j = len - 1; i < j; i++, j--)
    {
        const uint8_t c = bytes[i];
        bytes[i] = bytes[j];
        bytes[j] = c;
    }
}

int ScalarCodec::encodeBytesImpl(uint8_t* const bytes, const unsigned bitlen)
{
    assert(bytes);
    // Underlying stream class assumes that more significant bits have lower index, so we need to shift some.
    if (bitlen % 8)
    {
        bytes[bitlen / 8] <<= (8 - (bitlen % 8)) & 7;
    }
    return stream_.write(bytes, bitlen);
}

int ScalarCodec::decodeBytesImpl(uint8_t* const bytes, const unsigned bitlen)
{
    assert(bytes);
    const int read_res = stream_.read(bytes, bitlen);
    if (read_res > 0)
    {
        if (bitlen % 8)
        {
            bytes[bitlen / 8] >>= (8 - (bitlen % 8)) & 7;  // As in encode(), vice versa
        }
    }
    return read_res;
}

}
