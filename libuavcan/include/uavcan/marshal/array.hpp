/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <bitset>
#include <cstdio>
#include <algorithm>
#include <stdexcept>
#include <cstring>
#include <uavcan/error.hpp>
#include <uavcan/util/compile_time.hpp>
#include <uavcan/impl_constants.hpp>
#include <uavcan/marshal/type_util.hpp>
#include <uavcan/marshal/integer_spec.hpp>

#ifndef UAVCAN_EXCEPTIONS
# error UAVCAN_EXCEPTIONS
#endif

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
        {
            return pos;
        }
#if UAVCAN_EXCEPTIONS
        throw std::out_of_range("uavcan::Array");
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
        {
            return pos;
        }
#if UAVCAN_EXCEPTIONS
        throw std::out_of_range("uavcan::Array");
#else
        assert(0);
        return (size_ == 0) ? 0 : (size_ - 1);
#endif
    }

    void grow()
    {
        if (size_ >= MaxSize)
        {
            validateRange(MaxSize);  // Will throw, assert() or do nothing
        }
        else
        {
            size_++;
        }
    }

    void shrink()
    {
        if (size_ > 0)
        {
            size_--;
        }
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
class ArrayImpl : public Select<ArrayMode == ArrayModeDynamic,
                                DynamicArrayBase<MaxSize>, StaticArrayBase<MaxSize> >::Result
{
    typedef ArrayImpl<T, ArrayMode, MaxSize> SelfType;
    typedef typename Select<ArrayMode == ArrayModeDynamic,
                            DynamicArrayBase<MaxSize>, StaticArrayBase<MaxSize> >::Result Base;

public:
    enum
    {
        IsStringLike = IsIntegerSpec<T>::Result && (T::MaxBitLen == 8 || T::MaxBitLen == 7) &&
                       (ArrayMode == ArrayModeDynamic)
    };

private:
    typedef typename StorageType<T>::Type BufferType[MaxSize + (IsStringLike ? 1 : 0)];
    BufferType data_;

    template <typename U>
    typename EnableIf<sizeof(U(0) == U())>::Type initialize(int)
    {
        if (ArrayMode != ArrayModeDynamic)
        {
            std::fill(data_, data_ + MaxSize, U());
        }
    }
    template <typename> void initialize(...) { }

public:
    typedef typename StorageType<T>::Type ValueType;
    typedef typename Base::SizeType SizeType;

    using Base::size;
    using Base::capacity;

    ArrayImpl() { initialize<ValueType>(0); }

    const char* c_str() const
    {
        StaticAssert<IsStringLike>::check();
        assert(size() < (MaxSize + 1));
        const_cast<BufferType&>(data_)[size()] = 0;  // Ad-hoc string termination
        return reinterpret_cast<const char*>(data_);
    }

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
    , public Select<ArrayMode == ArrayModeDynamic, DynamicArrayBase<MaxSize>, StaticArrayBase<MaxSize> >::Result
{
    typedef typename Select<ArrayMode == ArrayModeDynamic,
                            DynamicArrayBase<MaxSize>, StaticArrayBase<MaxSize> >::Result ArrayBase;

public:
    enum { IsStringLike = 0 };

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
            {
                return res;
            }
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
            {
                return res_sz;
            }
        }
        if (size() == 0)
        {
            return 1;
        }
        return encodeImpl(codec, self_tao_enabled ? TailArrayOptDisabled : tao_mode, FalseType());
    }

    int decodeImpl(ScalarCodec& codec, const TailArrayOptimizationMode tao_mode, FalseType)  /// Static
    {
        assert(size() > 0);
        for (SizeType i = 0; i < size(); i++)
        {
            const bool last_item = i == (size() - 1);
            ValueType value = ValueType();                          // TODO: avoid extra copy
            const int res = RawValueType::decode(value, codec, last_item ? tao_mode : TailArrayOptDisabled);
            if (res <= 0)
            {
                return res;
            }
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
                ValueType value = ValueType();
                const int res = RawValueType::decode(value, codec, TailArrayOptDisabled);
                if (res < 0)
                {
                    return res;
                }
                if (res == 0)             // Success: End of stream reached (even if zero items were read)
                {
                    return 1;
                }
                if (size() == MaxSize_)   // Error: Max array length reached, but the end of stream is not
                {
                    return -ErrInvalidMarshalData;
                }
                push_back(value);
            }
        }
        else
        {
            typename StorageType<typename Base::RawSizeType>::Type sz = 0;
            const int res_sz = Base::RawSizeType::decode(sz, codec, TailArrayOptDisabled);
            if (res_sz <= 0)
            {
                return res_sz;
            }
            if ((sz > 0) && ((sz - 1u) > (MaxSize_ - 1u))) // -Werror=type-limits
            {
                return -ErrInvalidMarshalData;
            }
            resize(sz);
            if (sz == 0)
            {
                return 1;
            }
            return decodeImpl(codec, tao_mode, FalseType());
        }
        assert(0); // Unreachable
        return -ErrLogic;
    }

public:
    typedef T RawValueType;
    typedef typename StorageType<T>::Type ValueType;
    typedef typename Base::SizeType SizeType;

    using Base::size;
    using Base::capacity;

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

    static void extendDataTypeSignature(DataTypeSignature& signature)
    {
        RawValueType::extendDataTypeSignature(signature);
    }

    bool empty() const { return size() == 0; }

    void pop_back() { Base::shrink(); }
    void push_back(const ValueType& value)
    {
        Base::grow();
        Base::at(size() - 1) = value;
    }

    void resize(SizeType new_size, const ValueType& filler)
    {
        if (new_size > size())
        {
            unsigned int cnt = new_size - size();
            while (cnt--)
            {
                push_back(filler);
            }
        }
        else if (new_size < size())
        {
            unsigned int cnt = size() - new_size;
            while (cnt--)
            {
                pop_back();
            }
        }
    }

    void resize(SizeType new_size)
    {
        resize(new_size, ValueType());
    }

    /*
     * Comparison operators
     */
    template <typename R>
    typename EnableIf<sizeof(((const R*)(0U))->size()) && sizeof((*((const R*)(0U)))[0]), bool>::Type
    operator==(const R& rhs) const
    {
        if (size() != rhs.size())
        {
            return false;
        }
        for (SizeType i = 0; i < size(); i++)  // Bitset does not have iterators
        {
            if (!(Base::at(i) == rhs[i]))
            {
                return false;
            }
        }
        return true;
    }

    bool operator==(const char* ch) const
    {
        if (ch == NULL)
        {
            return false;
        }
        return std::strcmp(Base::c_str(), ch) == 0;
    }

    template <typename R> bool operator!=(const R& rhs) const { return !operator==(rhs); }

    /*
     * Assign/append operators
     */
    SelfType& operator=(const char* ch)
    {
        StaticAssert<Base::IsStringLike>::check();
        StaticAssert<IsDynamic>::check();
        Base::clear();
        if (ch == NULL)
        {
            handleFatalError("Null pointer in Array<>::operator=(const char*)");
        }
        while (*ch)
        {
            push_back(*ch++);
        }
        return *this;
    }

    SelfType& operator+=(const char* ch)
    {
        StaticAssert<Base::IsStringLike>::check();
        StaticAssert<IsDynamic>::check();
        if (ch == NULL)
        {
            handleFatalError("Null pointer in Array<>::operator+=(const char*)");
        }
        while (*ch)
        {
            push_back(*ch++);
        }
        return *this;
    }

    template <uavcan::ArrayMode RhsArrayMode, unsigned int RhsMaxSize>
    SelfType& operator+=(const Array<T, RhsArrayMode, RhsMaxSize>& rhs)
    {
        typedef Array<T, RhsArrayMode, RhsMaxSize> Rhs;
        typedef typename Select<(sizeof(SizeType) > sizeof(typename Rhs::SizeType)),
                                SizeType, typename Rhs::SizeType>::Result CommonSizeType;
        StaticAssert<IsDynamic>::check();
        for (CommonSizeType i = 0; i < rhs.size(); i++)
        {
            push_back(rhs[i]);
        }
        return *this;
    }

    /*
     * Formatting appender.
     * This method doesn't raise an overflow error; instead it silently truncates the data to fit the array capacity.
     */
    template <typename A>
    void appendFormatted(const char* const format, const A value)
    {
        StaticAssert<Base::IsStringLike>::check();
        StaticAssert<IsDynamic>::check();

        StaticAssert<sizeof(A() == A(0))>::check();             // This check allows to weed out most non-trivial types
        StaticAssert<sizeof(A) <= sizeof(long double)>::check(); // Another stupid check to catch non-trivial types

        if (!format)
        {
            assert(0);
            return;
        }
        // Add some hardcore runtime checks for the format string correctness?

        ValueType* const ptr = Base::end();
        assert(capacity() >= size());
        const SizeType max_size = capacity() - size();

        // We have one extra byte for the null terminator, hence +1
        const int ret = std::snprintf(reinterpret_cast<char*>(ptr), max_size + 1, format, value);

        for (int i = 0; i < std::min(ret, int(max_size)); i++)
        {
            Base::grow();
        }
        if (ret < 0)
        {
            assert(0);           // Likely an invalid format string
            (*this) += format;   // So we print it as is in release builds
        }
    }

    typedef ValueType value_type;
    typedef SizeType size_type;
};

template <typename R, typename T, ArrayMode ArrayMode, unsigned int MaxSize>
inline bool operator==(const R& rhs, const Array<T, ArrayMode, MaxSize>& lhs)
{
    return lhs.operator==(rhs);
}

template <typename R, typename T, ArrayMode ArrayMode, unsigned int MaxSize>
inline bool operator!=(const R& rhs, const Array<T, ArrayMode, MaxSize>& lhs)
{
    return lhs.operator!=(rhs);
}


template <typename T, ArrayMode ArrayMode, unsigned int MaxSize>
class YamlStreamer<Array<T, ArrayMode, MaxSize> >
{
    typedef Array<T, ArrayMode, MaxSize> ArrayType;

    static bool isNiceCharacter(int c)
    {
        if (c >= 32 && c <= 126)
        {
            return true;
        }
        static const char Good[] = {'\n', '\r', '\t'};
        for (unsigned int i = 0; i < sizeof(Good) / sizeof(Good[0]); i++)
        {
            if (Good[i] == c)
            {
                return true;
            }
        }
        return false;
    }

    template <typename Stream>
    static void streamPrimitives(Stream& s, const ArrayType& array)
    {
        s << '[';
        for (std::size_t i = 0; i < array.size(); i++)
        {
            YamlStreamer<T>::stream(s, array.at(i), 0);
            if ((i + 1) < array.size())
            {
                s << ", ";
            }
        }
        s << ']';
    }

    template <typename Stream>
    static void streamCharacters(Stream& s, const ArrayType& array)
    {
        s << '"';
        for (std::size_t i = 0; i < array.size(); i++)
        {
            const int c = array.at(i);
            if (c < 32 || c > 126)
            {
                char nibbles[2] = {char((c >> 4) & 0xF), char(c & 0xF)};
                for (int i = 0; i < 2; i++)
                {
                    nibbles[i] += '0';
                    if (nibbles[i] > '9')
                    {
                        nibbles[i] += 'A' - '9' - 1;
                    }
                }
                s << "\\x" << nibbles[0] << nibbles[1];
            }
            else
            {
                if (c == '"' || c == '\\')      // YAML requires to escape these two
                {
                    s << '\\';
                }
                s << char(c);
            }
        }
        s << '"';
    }

    struct SelectorStringLike { };
    struct SelectorPrimitives { };
    struct SelectorObjects { };

    template <typename Stream>
    static void genericStreamImpl(Stream& s, const ArrayType& array, int, SelectorStringLike)
    {
        bool printable_only = true;
        for (int i = 0; i < array.size(); i++)
        {
            if (!isNiceCharacter(array[i]))
            {
                printable_only = false;
                break;
            }
        }
        if (printable_only)
        {
            streamCharacters(s, array);
        }
        else
        {
            streamPrimitives(s, array);
            s << " # ";
            streamCharacters(s, array);
        }
    }

    template <typename Stream>
    static void genericStreamImpl(Stream& s, const ArrayType& array, int, SelectorPrimitives)
    {
        streamPrimitives(s, array);
    }

    template <typename Stream>
    static void genericStreamImpl(Stream& s, const ArrayType& array, int level, SelectorObjects)
    {
        if (array.empty())
        {
            s << "[]";
            return;
        }
        for (std::size_t i = 0; i < array.size(); i++)
        {
            s << '\n';
            for (int pos = 0; pos < level; pos++)
            {
                s << "  ";
            }
            s << "- ";
            YamlStreamer<T>::stream(s, array.at(i), level + 1);
        }
    }

public:
    template <typename Stream>
    static void stream(Stream& s, const ArrayType& array, int level)
    {
        typedef typename Select<ArrayType::IsStringLike, SelectorStringLike,
                                typename Select<IsPrimitiveType<typename ArrayType::RawValueType>::Result,
                                                SelectorPrimitives,
                                                SelectorObjects>::Result >::Result Type;
        genericStreamImpl(s, array, level, Type());
    }
};

}
