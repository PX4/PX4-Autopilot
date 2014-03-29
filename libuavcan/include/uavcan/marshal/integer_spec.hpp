/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <limits>
#include <uavcan/stdint.hpp>
#include <uavcan/data_type.hpp>
#include <uavcan/util/compile_time.hpp>
#include <uavcan/marshal/scalar_codec.hpp>
#include <uavcan/marshal/type_util.hpp>

namespace uavcan
{

enum Signedness { SignednessUnsigned, SignednessSigned };

template <unsigned BitLen_, Signedness Signedness, CastMode CastMode>
class IntegerSpec
{
    struct ErrorNoSuchInteger;

public:
    enum { IsSigned = Signedness == SignednessSigned };
    enum { BitLen = BitLen_ };
    enum { MinBitLen = BitLen };
    enum { MaxBitLen = BitLen };
    enum { IsPrimitive = 1 };

    typedef typename Select<(BitLen <= 8),  typename Select<IsSigned, int8_t,  uint8_t>::Result,
            typename Select<(BitLen <= 16), typename Select<IsSigned, int16_t, uint16_t>::Result,
            typename Select<(BitLen <= 32), typename Select<IsSigned, int32_t, uint32_t>::Result,
            typename Select<(BitLen <= 64), typename Select<IsSigned, int64_t, uint64_t>::Result,
                              ErrorNoSuchInteger>::Result>::Result>::Result>::Result StorageType;

    typedef typename IntegerSpec<BitLen, SignednessUnsigned, CastMode>::StorageType UnsignedStorageType;

private:
    IntegerSpec();

    struct LimitsImplGeneric
    {
        static StorageType max()
        {
            StaticAssert<(sizeof(uintmax_t) >= 8)>::check();
            return StorageType(IsSigned ? ((uintmax_t(1) << (BitLen - 1)) - 1) : ((uintmax_t(1) << BitLen) - 1));
        }
        static UnsignedStorageType mask()
        {
            StaticAssert<(sizeof(uintmax_t) >= 8)>::check();
            return UnsignedStorageType((uintmax_t(1) << BitLen) - 1);
        }
    };

    struct LimitsImpl64
    {
        static StorageType max() { return StorageType(IsSigned ? 0x7FFFFFFFFFFFFFFF : 0xFFFFFFFFFFFFFFFF); }
        static UnsignedStorageType mask() { return 0xFFFFFFFFFFFFFFFF; }
    };

    typedef typename Select<(BitLen == 64), LimitsImpl64, LimitsImplGeneric>::Result Limits;

    static void saturate(StorageType& value)
    {
        if (value > max())
        {
            value = max();
        }
        else if (value <= min()) // 'Less or Equal' allows to suppress compiler warning on unsigned types
        {
            value = min();
        }
    }

    static void truncate(StorageType& value) { value &= mask(); }

    static void validate()
    {
        StaticAssert<(BitLen <= (sizeof(StorageType) * 8))>::check();
        assert(max() <= std::numeric_limits<StorageType>::max());
        assert(min() >= std::numeric_limits<StorageType>::min());
    }

public:
    static StorageType max() { return Limits::max(); }
    static StorageType min() { return IsSigned ? StorageType(-max() - 1) : 0; }
    static UnsignedStorageType mask() { return Limits::mask(); }

    static int encode(StorageType value, ScalarCodec& codec, TailArrayOptimizationMode)
    {
        validate();
        // cppcheck-suppress duplicateExpression
        if (CastMode == CastModeSaturate)
        {
            saturate(value);
        }
        else
        {
            truncate(value);
        }
        return codec.encode<BitLen>(value);
    }

    static int decode(StorageType& out_value, ScalarCodec& codec, TailArrayOptimizationMode)
    {
        validate();
        return codec.decode<BitLen>(out_value);
    }

    static void extendDataTypeSignature(DataTypeSignature&) { }
};

template <CastMode CastMode>
class IntegerSpec<1, SignednessSigned, CastMode>;   // Invalid instantiation

template <Signedness Signedness, CastMode CastMode>
class IntegerSpec<0, Signedness, CastMode>;         // Invalid instantiation


template <typename T>
struct IsIntegerSpec
{
    enum { Result = 0 };
};

template <unsigned BitLen, Signedness Signedness, CastMode CastMode>
struct IsIntegerSpec<IntegerSpec<BitLen, Signedness, CastMode> >
{
    enum { Result = 1 };
};


template <unsigned BitLen, Signedness Signedness, CastMode CastMode>
struct YamlStreamer<IntegerSpec<BitLen, Signedness, CastMode> >
{
    typedef IntegerSpec<BitLen, Signedness, CastMode> RawType;
    typedef typename RawType::StorageType StorageType;

    template <typename Stream>
    static void stream(Stream& s, const StorageType value, int)
    {
        // Get rid of character types - we want its integer representation, not ASCII code
        typedef typename Select<(sizeof(StorageType) >= sizeof(int)), StorageType,
                                typename Select<RawType::IsSigned, int, unsigned>::Result >::Result TempType;
        s << TempType(value);
    }
};

}
