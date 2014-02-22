/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <algorithm>
#include <stdexcept>
#include <uavcan/internal/impl_constants.hpp>
#include <uavcan/internal/util.hpp>
#include <uavcan/internal/marshalling/types.hpp>

namespace uavcan
{

template <typename T, unsigned int Size_>
class StaticArray
{
    typedef StaticArray<T, Size_> SelfType;

public:
    enum { IsDynamic = 0 };
    enum { Size = Size_ };

    typedef T RawValueType;
    typedef typename StorageType<T>::Type ValueType;
    typedef unsigned int SizeType;

private:
    ValueType data_[Size];

    void checkOrAdjustRange(SizeType& pos) const
    {
        if (pos < Size)
            return;
#if UAVCAN_EXCEPTIONS
        throw std::out_of_range(typeid(*this).name());
#else
        assert(0);
        pos = Size - 1;  // Ha ha
#endif
    }

    template <typename U>
    typename EnableIf<sizeof(U(0) == U())>::Type initialize(int) { std::fill(begin(), end(), U()); }
    template <typename> void initialize(...) { }

public:
    StaticArray() { initialize<ValueType>(0); }

    static int encode(const SelfType& array, ScalarCodec& codec)
    {
        for (SizeType i = 0; i < Size; i++)
        {
            const int res = RawValueType::encode(array.data_[i], codec);
            if (res <= 0)
                return res;
        }
        return 1;
    }

    static int decode(SelfType& array, ScalarCodec& codec)
    {
        for (SizeType i = 0; i < Size; i++)
        {
            const int res = RawValueType::decode(array.data_[i], codec);
            if (res <= 0)
                return res;
        }
        return 1;
    }

    template <typename R>
    bool operator==(const R& rhs) const
    {
        return (size() == rhs.size()) && std::equal(begin(), end(), rhs.begin());
    }
    template <typename R> bool operator!=(const R& rhs) const { return !operator==(rhs); }

    template <typename R>
    bool operator<(const R& rhs) const
    {
        return std::lexicographical_compare(begin(), end(), rhs.begin(), rhs.end());
    }

    ValueType& at(SizeType pos)
    {
        checkOrAdjustRange(pos);
        return data_[pos];
    }
    const ValueType& at(SizeType pos) const
    {
        checkOrAdjustRange(pos);
        return data_[pos];
    }

    ValueType& operator[](SizeType pos) { return at(pos); }
    const ValueType& operator[](SizeType pos) const { return at(pos); }

    ValueType* begin() { return data_; }
    ValueType* end() { return data_ + Size; }
    const ValueType* begin() const { return data_; }
    const ValueType* end() const { return data_ + Size; }

    SizeType size() const { return Size; }
    ValueType* data() { return data_; }
    const ValueType* data() const { return data_; }

    /**
     * STL compatibility
     */
    typedef ValueType value_type;
    typedef SizeType size_type;
    typedef ValueType* iterator;
    typedef const ValueType* const_iterator;
};

template <typename T> class StaticArray<T, 0>;  // Invalid instantiation

}
