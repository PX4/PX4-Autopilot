/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/internal/util.hpp>
#include <uavcan/internal/marshalling/integer_spec.hpp>
#include <uavcan/internal/marshalling/float_spec.hpp>

namespace uavcan
{

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
