/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <uavcan/stdint.hpp>
#include <uavcan/util/compile_time.hpp>
#include <uavcan/impl_constants.hpp>

namespace uavcan
{

class UAVCAN_EXPORT ITransferBuffer;

void bitarrayCopy(const unsigned char* src_org, int src_offset, int src_len, unsigned char* dst_org, int dst_offset);

class UAVCAN_EXPORT BitStream
{
    enum { MaxBytesPerRW = 16 };

    ITransferBuffer& buf_;
    int bit_offset_;
    uint8_t byte_cache_;

    static inline unsigned bitlenToBytelen(unsigned bits) { return (bits + 7) / 8; }

    static inline void copyBitArray(const uint8_t* src_org, int src_offset, int src_len,
                                    uint8_t* dst_org, int dst_offset)
    {
        bitarrayCopy(reinterpret_cast<const unsigned char*>(src_org), src_offset, src_len,
                     reinterpret_cast<unsigned char*>(dst_org), dst_offset);
    }

public:
    enum { MaxBitsPerRW = MaxBytesPerRW * 8 };

    enum
    {
        ResultOutOfBuffer = 0,
        ResultOk          = 1
    };

    BitStream(ITransferBuffer& buf)
        : buf_(buf)
        , bit_offset_(0)
        , byte_cache_(0)
    {
        StaticAssert<sizeof(uint8_t) == 1>::check();
    }

    /**
     * Write/read calls interpret bytes as bit arrays, 8 bits per byte, where the most
     * significant bits have lower index, i.e.:
     *   Hex:     55       2d
     *   Bits:    01010101 00101101
     *   Indices: 0  ..  7 8  ..  15
     * Return values:
     *   Negative - Error
     *   Zero     - Out of buffer space
     *   Positive - OK
     */
    int write(const uint8_t* bytes, const int bitlen);
    int read(uint8_t* bytes, const int bitlen);

#if UAVCAN_TOSTRING
    std::string toString() const;
#endif
};

}
