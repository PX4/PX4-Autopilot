/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <cstring>
#include <uavcan/stdint.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/transport/transfer.hpp>

namespace uavcan
{

class UAVCAN_EXPORT TransferCRC;

enum DataTypeKind
{
    DataTypeKindService,
    DataTypeKindMessage,
    NumDataTypeKinds
};


class UAVCAN_EXPORT DataTypeID
{
    uint16_t value_;

public:
    static const uint16_t Max = 1023;

    DataTypeID() : value_(0xFFFF) { }

    DataTypeID(uint16_t id)  // Implicit
        : value_(id)
    {
        UAVCAN_ASSERT(isValid());
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
class UAVCAN_EXPORT DataTypeSignatureCRC
{
    uint64_t crc_;

public:
    static DataTypeSignatureCRC extend(uint64_t crc);

    DataTypeSignatureCRC() : crc_(0xFFFFFFFFFFFFFFFFULL) { }

    void add(uint8_t byte);

    void add(const uint8_t* bytes, unsigned len);

    uint64_t get() const { return crc_ ^ 0xFFFFFFFFFFFFFFFFULL; }
};


class UAVCAN_EXPORT DataTypeSignature
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

/**
 * This class contains complete description of a data type.
 */
class UAVCAN_EXPORT DataTypeDescriptor
{
    DataTypeKind kind_;
    DataTypeID id_;
    DataTypeSignature signature_;
    const char* full_name_;

public:
    static const unsigned MaxFullNameLen = 80;

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
        UAVCAN_ASSERT(kind < NumDataTypeKinds);
        UAVCAN_ASSERT(name);
        UAVCAN_ASSERT(std::strlen(name) <= MaxFullNameLen);
    }

    DataTypeKind getKind() const { return kind_; }
    DataTypeID getID() const { return id_; }
    const DataTypeSignature& getSignature() const { return signature_; }
    const char* getFullName() const { return full_name_; }

    bool match(DataTypeKind kind, const char* name) const;
    bool match(DataTypeKind kind, DataTypeID id) const;

    bool operator!=(const DataTypeDescriptor& rhs) const { return !operator==(rhs); }
    bool operator==(const DataTypeDescriptor& rhs) const;

#if UAVCAN_TOSTRING
    std::string toString() const;
#endif
};

}
