/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <bitset>
#include <algorithm>
#include <stdexcept>
#include <typeinfo>
#include <uavcan/internal/impl_constants.hpp>
#include <uavcan/internal/util.hpp>
#include <uavcan/internal/marshal/type_util.hpp>

namespace uavcan
{

enum ArrayMode { ArrayModeStatic, ArrayModeDynamic };


template <unsigned int Size>
class StaticArrayBase
{
public:
    enum { SizeBitLen = 0 };
    typedef unsigned int SizeType;
    SizeType size()     const { return Size; }
    SizeType capacity() const { return Size; }

protected:
    StaticArrayBase() { }
    ~StaticArrayBase() { }

    SizeType validateRange(SizeType pos) const
    {
        if (pos < Size)
            return pos;
#if UAVCAN_EXCEPTIONS
        throw std::out_of_range(typeid(*this).name());
#else
        assert(0);
        return Size - 1;  // Ha ha
#endif
    }
};


template <unsigned int MaxSize>
class DynamicArrayBase
{
protected:
    typedef IntegerSpec<IntegerBitLen<MaxSize>::Result, SignednessUnsigned, CastModeSaturate> RawSizeType;
public:
    typedef typename StorageType<RawSizeType>::Type SizeType;

private:
    SizeType size_;

protected:
    DynamicArrayBase() : size_(0) { }
    ~DynamicArrayBase() { }

    SizeType validateRange(SizeType pos) const
    {
        if (pos < size_)
            return pos;
#if UAVCAN_EXCEPTIONS
        throw std::out_of_range(typeid(*this).name());
#else
        assert(0);
        return (size_ == 0) ? 0 : (size_ - 1);
#endif
    }

    void grow()
    {
        if (size_ >= MaxSize)
            validateRange(MaxSize);  // Will throw, assert() or do nothing
        else
            size_++;
    }

    void shrink()
    {
        if (size_ > 0)
            size_--;
    }

public:
    enum { SizeBitLen = RawSizeType::BitLen };

    SizeType size() const
    {
        assert(size_ ? ((size_ - 1u) <= (MaxSize - 1u)) : 1); // -Werror=type-limits
        return size_;
    }

    SizeType capacity() const { return MaxSize; }

    void clear() { size_ = 0; }
};

/**
 * Statically allocated array with optional dynamic-like behavior
 */
template <typename T, ArrayMode ArrayMode, unsigned int MaxSize>
class ArrayImpl : public StaticIf<ArrayMode == ArrayModeDynamic,
                                  DynamicArrayBase<MaxSize>,
                                  StaticArrayBase<MaxSize> >::Result
{
    typedef ArrayImpl<T, ArrayMode, MaxSize> SelfType;
    typedef typename StaticIf<ArrayMode == ArrayModeDynamic,
                              DynamicArrayBase<MaxSize>,
                              StaticArrayBase<MaxSize> >::Result Base;

    typename StorageType<T>::Type data_[MaxSize];

    template <typename U>
    typename EnableIf<sizeof(U(0) == U())>::Type initialize(int) { std::fill(data_, data_ + MaxSize, U()); }
    template <typename> void initialize(...) { }

public:
    typedef typename StorageType<T>::Type ValueType;
    typedef typename Base::SizeType SizeType;

    using Base::size;
    using Base::capacity;

    ArrayImpl() { initialize<ValueType>(0); }

    ValueType& at(SizeType pos)             { return data_[validateRange(pos)]; }
    const ValueType& at(SizeType pos) const { return data_[Base::validateRange(pos)]; }

    ValueType& operator[](SizeType pos)             { return at(pos); }
    const ValueType& operator[](SizeType pos) const { return at(pos); }

    ValueType* begin()             { return data_; }
    const ValueType* begin() const { return data_; }
    ValueType* end()               { return data_ + Base::size(); }
    const ValueType* end()   const { return data_ + Base::size(); }

    ValueType& front()             { return at(0); }
    const ValueType& front() const { return at(0); }
    ValueType& back()              { return at(Base::size() - 1); }
    const ValueType& back()  const { return at(Base::size() - 1); }

    template <typename R>
    bool operator<(const R& rhs) const
    {
        return std::lexicographical_compare(begin(), end(), rhs.begin(), rhs.end());
    }

    typedef ValueType* iterator;
    typedef const ValueType* const_iterator;
};

/**
 * Bit array specialization
 */
template <unsigned int MaxSize, ArrayMode ArrayMode, CastMode CastMode>
class ArrayImpl<IntegerSpec<1, SignednessUnsigned, CastMode>, ArrayMode, MaxSize>
    : public std::bitset<MaxSize>
    , public StaticIf<ArrayMode == ArrayModeDynamic, DynamicArrayBase<MaxSize>, StaticArrayBase<MaxSize> >::Result
{
    typedef typename StaticIf<ArrayMode == ArrayModeDynamic,
                              DynamicArrayBase<MaxSize>,
                              StaticArrayBase<MaxSize> >::Result ArrayBase;

public:
    typedef typename std::bitset<MaxSize>::reference Reference;
    typedef typename ArrayBase::SizeType SizeType;

    using ArrayBase::size;
    using ArrayBase::capacity;

    Reference at(SizeType pos)  { return std::bitset<MaxSize>::operator[](ArrayBase::validateRange(pos)); }
    bool at(SizeType pos) const { return std::bitset<MaxSize>::operator[](ArrayBase::validateRange(pos)); }

    Reference operator[](SizeType pos)  { return at(pos); }
    bool operator[](SizeType pos) const { return at(pos); }
};

/**
 * Zero length arrays are not allowed
 */
template <typename T, ArrayMode ArrayMode> class ArrayImpl<T, ArrayMode, 0>;


template <typename T, ArrayMode ArrayMode, unsigned int MaxSize_>
class Array : public ArrayImpl<T, ArrayMode, MaxSize_>
{
    typedef ArrayImpl<T, ArrayMode, MaxSize_> Base;
    typedef Array<T, ArrayMode, MaxSize_> SelfType;

    static bool isOptimizedTailArray(TailArrayOptimizationMode tao_mode)
    {
        return (T::MinBitLen >= 8) && (tao_mode == TailArrayOptEnabled);
    }

    int encodeImpl(ScalarCodec& codec, const TailArrayOptimizationMode tao_mode, FalseType) const  /// Static
    {
        assert(size() > 0);
        for (SizeType i = 0; i < size(); i++)
        {
            const bool last_item = i == (size() - 1);
            const int res = RawValueType::encode(Base::at(i), codec, last_item ? tao_mode : TailArrayOptDisabled);
            if (res <= 0)
                return res;
        }
        return 1;
    }

    int encodeImpl(ScalarCodec& codec, const TailArrayOptimizationMode tao_mode, TrueType) const   /// Dynamic
    {
        StaticAssert<IsDynamic>::check();
        const bool self_tao_enabled = isOptimizedTailArray(tao_mode);
        if (!self_tao_enabled)
        {
            const int res_sz = Base::RawSizeType::encode(size(), codec, TailArrayOptDisabled);
            if (res_sz <= 0)
                return res_sz;
        }
        if (size() == 0)
            return 1;
        return encodeImpl(codec, self_tao_enabled ? TailArrayOptDisabled : tao_mode, FalseType());
    }

    int decodeImpl(ScalarCodec& codec, const TailArrayOptimizationMode tao_mode, FalseType)  /// Static
    {
        assert(size() > 0);
        for (SizeType i = 0; i < size(); i++)
        {
            const bool last_item = i == (size() - 1);
            ValueType value;                          // TODO: avoid extra copy
            const int res = RawValueType::decode(value, codec, last_item ? tao_mode : TailArrayOptDisabled);
            if (res <= 0)
                return res;
            Base::at(i) = value;
        }
        return 1;
    }

    int decodeImpl(ScalarCodec& codec, const TailArrayOptimizationMode tao_mode, TrueType)   /// Dynamic
    {
        StaticAssert<IsDynamic>::check();
        Base::clear();
        if (isOptimizedTailArray(tao_mode))
        {
            while (true)
            {
                ValueType value;
                const int res = RawValueType::decode(value, codec, TailArrayOptDisabled);
                if (res < 0)
                    return res;
                if (res == 0)             // Success: End of stream reached (even if zero items were read)
                    return 1;
                if (size() == MaxSize_)   // Error: Max array length reached, but the end of stream is not
                    return -1;
                push_back(value);
            }
        }
        else
        {
            typename StorageType<typename Base::RawSizeType>::Type sz = 0;
            const int res_sz = Base::RawSizeType::decode(sz, codec, TailArrayOptDisabled);
            if (res_sz <= 0)
                return res_sz;
            if ((sz > 0) && ((sz - 1u) > (MaxSize_ - 1u))) // -Werror=type-limits
                return -1;
            resize(sz);
            if (sz == 0)
                return 1;
            return decodeImpl(codec, tao_mode, FalseType());
        }
        assert(0); // Unreachable
        return -1;
    }

public:
    typedef T RawValueType;
    typedef typename StorageType<T>::Type ValueType;
    typedef typename Base::SizeType SizeType;

    using Base::size;

    enum { IsDynamic = ArrayMode == ArrayModeDynamic };
    enum { MaxSize = MaxSize_ };
    enum { MinBitLen = IsDynamic ? 0 : (RawValueType::MinBitLen * MaxSize) };
    enum { MaxBitLen = Base::SizeBitLen + RawValueType::MaxBitLen * MaxSize };

    static int encode(const SelfType& array, ScalarCodec& codec, const TailArrayOptimizationMode tao_mode)
    {
        return array.encodeImpl(codec, tao_mode, BooleanType<IsDynamic>());
    }

    static int decode(SelfType& array, ScalarCodec& codec, const TailArrayOptimizationMode tao_mode)
    {
        return array.decodeImpl(codec, tao_mode, BooleanType<IsDynamic>());
    }

    bool empty() const { return size() == 0; }

    void pop_back() { Base::shrink(); }
    void push_back(const ValueType& value)
    {
        Base::grow();
        Base::at(size() - 1) = value;
    }

    void resize(SizeType new_size, ValueType filler = ValueType())
    {
        if (new_size > size())
        {
            unsigned int cnt = new_size - size();
            while (cnt--)
                push_back(filler);
        }
        else if (new_size < size())
        {
            unsigned int cnt = size() - new_size;
            while (cnt--)
                pop_back();
        }
    }

    template <typename R>
    bool operator==(const R& rhs) const
    {
        if (size() != rhs.size())
            return false;
        for (SizeType i = 0; i < size(); i++)  // Bitset does not have iterators
            if (!(Base::at(i) == rhs[i]))
                return false;
        return true;
    }
    template <typename R> bool operator!=(const R& rhs) const { return !operator==(rhs); }

    typedef ValueType value_type;
    typedef SizeType size_type;
};

}
