/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/internal/util.hpp>

namespace uavcan
{

enum CastMode { CastModeSaturate, CastModeTruncate };


template <unsigned long Num>
struct IntegerBitLen    { enum { Result = 1 + IntegerBitLen<(Num >> 1)>::Result }; };
template <>
struct IntegerBitLen<0> { enum { Result = 0 }; };


// TODO: fix
template <typename T, typename Enable = void>
struct StorageTypeImpl { typedef T Type; };

template <typename T>
struct StorageTypeImpl<T, typename EnableIfType<typename T::StorageType>::Type>
{
    typedef typename T::StorageType Type;
};

template <typename T>
class StorageType : public T
{
    StorageType();
public:
    typedef typename StorageTypeImpl<T>::Type Type;
};

}
