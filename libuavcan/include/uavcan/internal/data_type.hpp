/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <stdint.h>
#include <algorithm>
#include <uavcan/internal/transport/crc.hpp>

namespace uavcan
{

enum DataTypeKind
{
    DataTypeKindService,
    DataTypeKindMessage,
    NumDataTypeKinds
};

/**
 * CRC-64-WE
 * Description: http://reveng.sourceforge.net/crc-catalogue/17plus.htm#crc.cat-bits.64
 * Initial value: 0xFFFFFFFFFFFFFFFF
 * Poly: 0x42F0E1EBA9EA3693
 * Reverse: no
 * Output xor: 0xFFFFFFFFFFFFFFFF
 * Check: 0x62EC59E3F1A4F00A
 */
class DataTypeSignatureCRC
{
    uint64_t crc_;

public:
    static DataTypeSignatureCRC extend(uint64_t crc)
    {
        DataTypeSignatureCRC ret;
        ret.crc_ = crc ^ 0xFFFFFFFFFFFFFFFF;
        return ret;
    }

    DataTypeSignatureCRC() : crc_(0xFFFFFFFFFFFFFFFF) { }

    void add(uint8_t byte)
    {
        static const uint64_t Poly = 0x42F0E1EBA9EA3693;
        crc_ ^= uint64_t(byte) << 56;
        for (int i = 0; i < 8; i++)
            crc_ = (crc_ & (uint64_t(1) << 63)) ? (crc_ << 1) ^ Poly : crc_ << 1;
    }

    void add(const uint8_t* bytes, unsigned int len)
    {
        assert(bytes);
        while (len--)
            add(*bytes++);
    }

    uint64_t get() const { return crc_ ^ 0xFFFFFFFFFFFFFFFF; }
};


class DataTypeSignature
{
    uint64_t value_;

    void mixin64(uint64_t x);

    DataTypeSignature() : value_(0) { }

public:
    static DataTypeSignature zero() { return DataTypeSignature(); }

    explicit DataTypeSignature(uint64_t value) : value_(value) { }

    void extend(DataTypeSignature dts);

    TransferCRC toTransferCRC() const;

    uint64_t get() const { return value_; }

    bool operator==(DataTypeSignature rhs) const { return value_ == rhs.value_; }
    bool operator!=(DataTypeSignature rhs) const { return !operator==(rhs); }
};


struct DataTypeHash
{
    enum { NumBytes = 16 };
    uint8_t value[NumBytes];

    DataTypeHash()
    {
        std::fill(value, value + NumBytes, 0);
    }

    DataTypeHash(const uint8_t source[NumBytes])
    {
        std::copy(source, source + NumBytes, value);
    }

    bool operator!=(const DataTypeHash& rhs) const { return !operator==(rhs); }
    bool operator==(const DataTypeHash& rhs) const
    {
        return std::equal(value, value + NumBytes, rhs.value);
    }
};


struct DataTypeDescriptor
{
    DataTypeKind kind;
    uint16_t id;
    DataTypeHash hash;

    DataTypeDescriptor()
    : kind(DataTypeKind(0))
    , id(0)
    { }

    DataTypeDescriptor(DataTypeKind kind, uint16_t id, const DataTypeHash& hash)
    : kind(kind)
    , id(id)
    , hash(hash)
    { }

    bool operator!=(const DataTypeDescriptor& rhs) const { return !operator==(rhs); }
    bool operator==(const DataTypeDescriptor& rhs) const
    {
        return (kind == rhs.kind) && (id == rhs.id) && (hash == rhs.hash);
    }
};

}
