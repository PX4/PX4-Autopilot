/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <stdint.h>
#include <algorithm>

namespace uavcan
{

enum DataTypeKind
{
    DataTypeKindService,
    DataTypeKindMessage,
    NumDataTypeKinds
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
