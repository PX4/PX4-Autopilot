/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <algorithm>

namespace uavcan
{

enum DataTypeKind
{
    DATA_TYPE_KIND_SERVICE,
    DATA_TYPE_KIND_MESSAGE,
    NUM_DATA_TYPE_KINDS
};


struct DataTypeHash
{
    enum { BYTES = 16 };
    uint8_t value[BYTES];

    DataTypeHash()
    {
        std::fill(value, value + BYTES, 0);
    }

    DataTypeHash(const uint8_t source[BYTES])
    {
        std::copy(source, source + BYTES, value);
    }

    bool operator!=(const DataTypeHash& rhs) const { return !operator==(rhs); }
    bool operator==(const DataTypeHash& rhs) const
    {
        return std::equal(value, value + BYTES, rhs.value);
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
