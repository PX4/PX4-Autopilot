/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/internal/util.hpp>
#include <uavcan/internal/marshalling/scalar_codec.hpp>

namespace uavcan
{

enum CastMode { CastModeSaturate, CastModeTruncate };

template <typename T, typename Enable = void>
struct AddStorageTypeImpl { typedef T StorageType; };

template <typename T>
struct AddStorageTypeImpl<T, typename EnableIfType<typename T::StorageType>::Type> { };

template <typename T>
struct AddStorageType : public T, public AddStorageTypeImpl<T> { };

}
