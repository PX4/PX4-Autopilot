/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstring>
#include <cstdio>
#include <string>
#include <cassert>
#include <uavcan/data_type.hpp>
#include <uavcan/transport/crc.hpp>

namespace uavcan
{
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

bool DataTypeDescriptor::operator==(const DataTypeDescriptor& rhs) const
{
    return
        (kind_ == rhs.kind_) &&
        (id_ == rhs.id_) &&
        (signature_ == rhs.signature_) &&
        !std::strcmp(full_name_, rhs.full_name_);
}

}
