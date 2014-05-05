/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/util/templates.hpp>
#include <uavcan/impl_constants.hpp>

namespace uavcan
{

enum CastMode { CastModeSaturate, CastModeTruncate };

enum TailArrayOptimizationMode { TailArrayOptDisabled, TailArrayOptEnabled };


template <unsigned long Num>
struct IntegerBitLen
{
    enum { Result = 1 + IntegerBitLen<(Num >> 1)>::Result };
};
template <>
struct IntegerBitLen<0>
{
    enum { Result = 0 };
};


template <unsigned long BitLen>
struct BitLenToByteLen
{
    enum { Result = (BitLen + 7) / 8 };
};


template <typename T, typename Enable = void>
struct StorageType
{
    typedef T Type;
};

template <typename T>
struct StorageType<T, typename EnableIfType<typename T::StorageType>::Type>
{
    typedef typename T::StorageType Type;
};


template <typename T>
class IsPrimitiveType
{
    typedef char Yes;
    struct No { Yes dummy[8]; };

    template <typename U>
    static typename EnableIf<U::IsPrimitive, Yes>::Type test(int);

    template <typename>
    static No test(...);

public:
    enum { Result = sizeof(test<T>(0)) == sizeof(Yes) };
};


template <typename T>
class UAVCAN_EXPORT YamlStreamer;

}
