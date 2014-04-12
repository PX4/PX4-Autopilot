/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstring>
#include <cassert>
#include <uavcan/data_type.hpp>
#include <uavcan/transport/crc.hpp>

namespace uavcan
{
/*
 * DataTypeSignatureCRC
 */
DataTypeSignatureCRC DataTypeSignatureCRC::extend(uint64_t crc)
{
    DataTypeSignatureCRC ret;
    ret.crc_ = crc ^ 0xFFFFFFFFFFFFFFFF;
    return ret;
}

void DataTypeSignatureCRC::add(uint8_t byte)
{
    static const uint64_t Poly = 0x42F0E1EBA9EA3693;
    crc_ ^= uint64_t(byte) << 56;
    for (int i = 0; i < 8; i++)
    {
        crc_ = (crc_ & (uint64_t(1) << 63)) ? (crc_ << 1) ^ Poly : crc_ << 1;
    }
}

void DataTypeSignatureCRC::add(const uint8_t* bytes, unsigned len)
{
    assert(bytes);
    while (len--)
    {
        add(*bytes++);
    }
}

/*
 * DataTypeSignature
 */
void DataTypeSignature::mixin64(uint64_t x)
{
    DataTypeSignatureCRC crc = DataTypeSignatureCRC::extend(value_);
    for (int i = 0; i < 64; i += 8)   // LSB first
    {
        crc.add((x >> i) & 0xFF);
    }
    value_ = crc.get();
}

void DataTypeSignature::extend(DataTypeSignature dts)
{
    const uint64_t y = value_;
    mixin64(dts.get());
    mixin64(y);
}

TransferCRC DataTypeSignature::toTransferCRC() const
{
    TransferCRC tcrc;
    for (int i = 0; i < 64; i += 8)    // LSB first
    {
        tcrc.add((value_ >> i) & 0xFF);
    }
    return tcrc;
}

/*
 * DataTypeDescriptor
 */
bool DataTypeDescriptor::match(DataTypeKind kind, const char* name) const
{
    return (kind_ == kind) && !std::strcmp(full_name_, name);
}

bool DataTypeDescriptor::match(DataTypeKind kind, DataTypeID id) const
{
    return (kind_ == kind) && (id_ == id);
}

#if UAVCAN_TOSTRING
std::string DataTypeDescriptor::toString() const
{
    char kindch = '?';
    switch (kind_)
    {
    case DataTypeKindMessage:
    {
        kindch = 'm';
        break;
    }
    case DataTypeKindService:
    {
        kindch = 's';
        break;
    }
    default:
        assert(0);
    }

    using namespace std; // For snprintf()
    char buf[80];
    (void)snprintf(buf, sizeof(buf), "%s:%u%c:%016llx",
                   full_name_, static_cast<unsigned>(id_.get()), kindch,
                   static_cast<unsigned long long>(signature_.get()));
    return std::string(buf);
}
#endif

bool DataTypeDescriptor::operator==(const DataTypeDescriptor& rhs) const
{
    return
        (kind_ == rhs.kind_) &&
        (id_ == rhs.id_) &&
        (signature_ == rhs.signature_) &&
        !std::strcmp(full_name_, rhs.full_name_);
}

}
