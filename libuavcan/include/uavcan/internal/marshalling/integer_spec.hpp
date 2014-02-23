/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <stdint.h>
#include <limits>
#include <uavcan/internal/util.hpp>
#include <uavcan/internal/marshalling/scalar_codec.hpp>
#include <uavcan/internal/marshalling/type_util.hpp>

namespace uavcan
{

enum Signedness { SignednessUnsigned, SignednessSigned };

template <unsigned int BitLen_, Signedness Signedness, CastMode CastMode>
class IntegerSpec
{
    enum { IsSigned = Signedness == SignednessSigned };

    struct ErrorNoSuchInteger;

public:
    enum { BitLen = BitLen_ };
    enum { MinBitLen = BitLen };

    typedef typename StaticIf<(BitLen <= 8),  typename StaticIf<IsSigned, int8_t,  uint8_t>::Result,
            typename StaticIf<(BitLen <= 16), typename StaticIf<IsSigned, int16_t, uint16_t>::Result,
            typename StaticIf<(BitLen <= 32), typename StaticIf<IsSigned, int32_t, uint32_t>::Result,
            typename StaticIf<(BitLen <= 64), typename StaticIf<IsSigned, int64_t, uint64_t>::Result,
                              ErrorNoSuchInteger>::Result>::Result>::Result>::Result StorageType;

private:
    typedef typename IntegerSpec<BitLen, SignednessUnsigned, CastMode>::StorageType UnsignedStorageType;

    IntegerSpec();

    struct ExactSizeLimits
    {
        static StorageType max() { return std::numeric_limits<StorageType>::max(); }
        static StorageType min() { return std::numeric_limits<StorageType>::min(); }
        static UnsignedStorageType mask() { return ~UnsignedStorageType(0); }
    };

    struct NonExactSizeLimits
    {
        static StorageType max()
        {
            return IsSigned ? ((StorageType(1) << (BitLen - 1)) - 1) : ((StorageType(1) << BitLen) - 1);
        }
        static StorageType min() { return IsSigned ? -(StorageType(1) << (BitLen - 1)) : 0; }
        static UnsignedStorageType mask() { return (UnsignedStorageType(1) << BitLen) - 1; }
    };

    enum { IsExactSize = (BitLen == 8) || (BitLen == 16) || (BitLen == 32) || (BitLen == 64) };

    typedef typename StaticIf<IsExactSize, ExactSizeLimits, NonExactSizeLimits>::Result Limits;

    static void saturate(StorageType& value)
    {
        if (value > max())
            value = max();
        else if (value <= min()) // 'Less or Equal' allows to suppress compiler warning on unsigned types
            value = min();
    }

    static void truncate(StorageType& value)
    {
        // cppcheck-suppress duplicateExpression
        value &= Limits::mask();
    }

public:
    // cppcheck-suppress duplicateExpression
    static StorageType max() { return Limits::max(); }
    // cppcheck-suppress duplicateExpression
    static StorageType min() { return Limits::min(); }

    static int encode(StorageType value, ScalarCodec& codec)
    {
        // cppcheck-suppress duplicateExpression
        if (CastMode == CastModeSaturate)
            saturate(value);
        else
            truncate(value);
        return codec.encode<BitLen>(value);
    }

    static int decode(StorageType& out_value, ScalarCodec& codec)
    {
        return codec.decode<BitLen>(out_value);
    }
};

template <CastMode CastMode>
class IntegerSpec<1, SignednessSigned, CastMode>;   // Invalid instantiation

}
