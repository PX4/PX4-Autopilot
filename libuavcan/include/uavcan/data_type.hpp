/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <cstring>
#include <string>
#include <algorithm>
#include <uavcan/stdint.hpp>
#include <uavcan/transport/transfer.hpp>

namespace uavcan
{

class TransferCRC;

enum DataTypeKind
{
    DataTypeKindService,
    DataTypeKindMessage,
    NumDataTypeKinds
};


class DataTypeID
{
    uint16_t value_;

public:
    enum { Max = 1023 };

    DataTypeID() : value_(0xFFFF) { }

    DataTypeID(uint16_t id)
    : value_(id)
    {
        assert(isValid());
    }

    bool isValid() const { return value_ <= Max; }

    uint16_t get() const { return value_; }

    bool operator==(DataTypeID rhs) const { return value_ == rhs.value_; }
    bool operator!=(DataTypeID rhs) const { return value_ != rhs.value_; }

    bool operator<(DataTypeID rhs) const { return value_ < rhs.value_; }
    bool operator>(DataTypeID rhs) const { return value_ > rhs.value_; }
    bool operator<=(DataTypeID rhs) const { return value_ <= rhs.value_; }
    bool operator>=(DataTypeID rhs) const { return value_ >= rhs.value_; }
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

public:
    DataTypeSignature() : value_(0) { }
    explicit DataTypeSignature(uint64_t value) : value_(value) { }

    void extend(DataTypeSignature dts);

    TransferCRC toTransferCRC() const;

    uint64_t get() const { return value_; }

    bool operator==(DataTypeSignature rhs) const { return value_ == rhs.value_; }
    bool operator!=(DataTypeSignature rhs) const { return !operator==(rhs); }
};


class DataTypeDescriptor
{
    DataTypeKind kind_;
    DataTypeID id_;
    DataTypeSignature signature_;
    const char* full_name_;

public:
    enum { MaxFullNameLen = 80 };

    DataTypeDescriptor()
    : kind_(DataTypeKind(0))
    , full_name_("")
    { }

    DataTypeDescriptor(DataTypeKind kind, DataTypeID id, const DataTypeSignature& signature, const char* name)
    : kind_(kind)
    , id_(id)
    , signature_(signature)
    , full_name_(name)
    {
        assert(kind < NumDataTypeKinds);
        assert(name);
        assert(std::strlen(name) <= MaxFullNameLen);
    }

    DataTypeKind getKind() const { return kind_; }
    DataTypeID getID() const { return id_; }
    const DataTypeSignature& getSignature() const { return signature_; }
    const char* getFullName() const { return full_name_; }

    bool match(DataTypeKind kind, const char* name) const;
    bool match(DataTypeKind kind, DataTypeID id) const;

    std::string toString() const;

    bool operator!=(const DataTypeDescriptor& rhs) const { return !operator==(rhs); }
    bool operator==(const DataTypeDescriptor& rhs) const;
};

}
