/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <uavcan/error.hpp>
#include <uavcan/util/bitset.hpp>
#include <uavcan/util/templates.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/marshal/type_util.hpp>
#include <uavcan/marshal/integer_spec.hpp>

#ifndef UAVCAN_CPP_VERSION
# error UAVCAN_CPP_VERSION
#endif

#ifndef UAVCAN_EXCEPTIONS
# error UAVCAN_EXCEPTIONS
#endif

#if UAVCAN_EXCEPTIONS
# include <stdexcept>
#endif

namespace uavcan
{

enum ArrayMode { ArrayModeStatic, ArrayModeDynamic };


template <unsigned Size>
class UAVCAN_EXPORT StaticArrayBase
{
protected:
    typedef IntegerSpec<IntegerBitLen<Size>::Result, SignednessUnsigned, CastModeSaturate> RawSizeType;

public:
    enum { SizeBitLen = 0 };

    typedef typename StorageType<RawSizeType>::Type SizeType;

    SizeType size()     const { return SizeType(Size); }
    SizeType capacity() const { return SizeType(Size); }

protected:
    StaticArrayBase() { }
    ~StaticArrayBase() { }

    SizeType validateRange(SizeType pos) const
    {
        if (pos < SizeType(Size))
        {
            return pos;
        }
#if UAVCAN_EXCEPTIONS
        throw std::out_of_range("uavcan::Array");
#else
        UAVCAN_ASSERT(0);
        return SizeType(Size - 1U);  // Ha ha
#endif
    }
};


template <unsigned MaxSize>
class UAVCAN_EXPORT DynamicArrayBase
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
        UAVCAN_ASSERT(0);
        return SizeType((size_ == 0U) ? 0U : (size_ - 1U));
#endif
    }

    void grow()
    {
        if (size_ >= MaxSize)
        {
            (void)validateRange(MaxSize);  // Will throw, UAVCAN_ASSERT() or do nothing
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
        UAVCAN_ASSERT(size_ ? ((size_ - 1u) <= (MaxSize - 1u)) : 1); // -Werror=type-limits
        return size_;
    }

    SizeType capacity() const { return MaxSize; }

    void clear() { size_ = 0; }
};

/**
 * Common functionality for both static and dynamic arrays.
 * Static arrays are of fixed size; methods that can alter the size (e.g. push_back() and such) will fail to compile.
 * Dynamic arrays contain a fixed-size buffer (it's size is enough to fit maximum number of elements) plus the
 * currently allocated number of elements.
 */
template <typename T, ArrayMode ArrayMode, unsigned MaxSize>
class UAVCAN_EXPORT ArrayImpl : public Select<ArrayMode == ArrayModeDynamic,
                                              DynamicArrayBase<MaxSize>, StaticArrayBase<MaxSize> >::Result
{
    typedef ArrayImpl<T, ArrayMode, MaxSize> SelfType;
    typedef typename Select<ArrayMode == ArrayModeDynamic,
                            DynamicArrayBase<MaxSize>, StaticArrayBase<MaxSize> >::Result Base;

public:
    enum
    {
        /// True if the array contents can be interpreted as a 8-bit string (ASCII or UTF8).
        IsStringLike = IsIntegerSpec<T>::Result && (T::MaxBitLen == 8 || T::MaxBitLen == 7) &&
                       (ArrayMode == ArrayModeDynamic)
    };

private:
    typedef typename StorageType<T>::Type BufferType[MaxSize + (IsStringLike ? 1 : 0)];
    BufferType data_;

    template <typename U>
    typename EnableIf<sizeof(U(0) >= U())>::Type initialize(int)
    {
        if (ArrayMode != ArrayModeDynamic)
        {
            ::uavcan::fill(data_, data_ + MaxSize, U());
        }
    }
    template <typename> void initialize(...) { }

protected:
    ~ArrayImpl() { }

public:
    typedef typename StorageType<T>::Type ValueType;
    typedef typename Base::SizeType SizeType;

    using Base::size;
    using Base::capacity;

    ArrayImpl() { initialize<ValueType>(0); }

    /**
     * Returns zero-terminated string, same as std::string::c_str().
     * This method will compile only if the array can be interpreted as 8-bit string (ASCII of UTF8).
     */
    const char* c_str() const
    {
        StaticAssert<IsStringLike>::check();
        UAVCAN_ASSERT(size() < (MaxSize + 1));
        const_cast<BufferType&>(data_)[size()] = 0;  // Ad-hoc string termination
        return reinterpret_cast<const char*>(data_);
    }

    /**
     * Range-checking subscript.
     * If the index is out of range:
     * - if exceptions are enabled, std::out_of_range will be thrown.
     * - if exceptions are disabled and UAVCAN_ASSERT() is enabled, execution will be aborted.
     * - if exceptions are disabled and UAVCAN_ASSERT() is disabled, index will be constrained to
     *   the closest valid value.
     */
    ValueType& at(SizeType pos)             { return data_[Base::validateRange(pos)]; }
    const ValueType& at(SizeType pos) const { return data_[Base::validateRange(pos)]; }

    /**
     * Range-checking subscript. @ref at()
     */
    ValueType& operator[](SizeType pos)             { return at(pos); }
    const ValueType& operator[](SizeType pos) const { return at(pos); }

    /**
     * Standard container methods. Applicable to both dynamic and static arrays.
     */
    ValueType* begin()             { return data_; }
    const ValueType* begin() const { return data_; }
    ValueType* end()               { return data_ + Base::size(); }
    const ValueType* end()   const { return data_ + Base::size(); }
    ValueType& front()             { return at(0U); }
    const ValueType& front() const { return at(0U); }
    ValueType& back()              { return at(SizeType(Base::size() - 1U)); }
    const ValueType& back()  const { return at(SizeType(Base::size() - 1U)); }

    /**
     * Performs standard lexicographical compare of the elements.
     */
    template <typename R>
    bool operator<(const R& rhs) const
    {
        return ::uavcan::lexicographical_compare(begin(), end(), rhs.begin(), rhs.end());
    }

    /**
     * Aliases for compatibility with standard containers.
     */
    typedef ValueType* iterator;
    typedef const ValueType* const_iterator;
};

/**
 * Memory-efficient specialization for bit arrays (each element maps to a single bit rather than single byte).
 * This should be compatible with std::bitset.
 */
template <unsigned MaxSize, ArrayMode ArrayMode, CastMode CastMode>
class UAVCAN_EXPORT ArrayImpl<IntegerSpec<1, SignednessUnsigned, CastMode>, ArrayMode, MaxSize>
    : public BitSet<MaxSize>
    , public Select<ArrayMode == ArrayModeDynamic, DynamicArrayBase<MaxSize>, StaticArrayBase<MaxSize> >::Result
{
    typedef typename Select<ArrayMode == ArrayModeDynamic,
                            DynamicArrayBase<MaxSize>, StaticArrayBase<MaxSize> >::Result ArrayBase;

public:
    enum { IsStringLike = 0 };

    typedef typename BitSet<MaxSize>::Reference Reference;
    typedef typename ArrayBase::SizeType SizeType;

    using ArrayBase::size;
    using ArrayBase::capacity;

    /**
     * Range-checking subscript. Throws if enabled; UAVCAN_ASSERT() if enabled; else constraints the position.
     */
    Reference at(SizeType pos)  { return BitSet<MaxSize>::operator[](ArrayBase::validateRange(pos)); }
    bool at(SizeType pos) const { return BitSet<MaxSize>::operator[](ArrayBase::validateRange(pos)); }

    /**
     * @ref at()
     */
    Reference operator[](SizeType pos)  { return at(pos); }
    bool operator[](SizeType pos) const { return at(pos); }
};

/**
 * Zero length arrays are not allowed
 */
template <typename T, ArrayMode ArrayMode> class ArrayImpl<T, ArrayMode, 0>;

/**
 * Generic array implementation.
 * This class is compatible with most standard library functions operating on containers (e.g. std::sort(),
 * std::lexicographical_compare(), etc.).
 * No dynamic memory is used.
 * All functions that can modify the array or access elements are range checking. If the range error occurs:
 * - if exceptions are enabled, std::out_of_range will be thrown;
 * - if UAVCAN_ASSERT() is enabled, program will be terminated on UAVCAN_ASSERT(0);
 * - otherwise the index value will be constrained to the closest valid value.
 */
template <typename T, ArrayMode ArrayMode, unsigned MaxSize_>
class UAVCAN_EXPORT Array : public ArrayImpl<T, ArrayMode, MaxSize_>
{
    typedef ArrayImpl<T, ArrayMode, MaxSize_> Base;
    typedef Array<T, ArrayMode, MaxSize_> SelfType;

    static bool isOptimizedTailArray(TailArrayOptimizationMode tao_mode)
    {
        return (T::MinBitLen >= 8) && (tao_mode == TailArrayOptEnabled);
    }

    int encodeImpl(ScalarCodec& codec, const TailArrayOptimizationMode tao_mode, FalseType) const  /// Static
    {
        UAVCAN_ASSERT(size() > 0);
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
        UAVCAN_ASSERT(size() > 0);
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
        UAVCAN_ASSERT(0); // Unreachable
        return -ErrLogic;
    }

    template <typename ScalarType, typename InputIter>
    void packSquareMatrixImpl(const InputIter src_row_major)
    {
        StaticAssert<IsDynamic>::check();
        const unsigned Width = CompileTimeIntSqrt<MaxSize>::Result;

        bool all_nans = true;
        bool scalar_matrix = true;
        bool diagonal_matrix = true;
        /*
         * Detecting how the matrix can be compressed:
         * - Matrix that consists only of NANs will be eliminated completely;
         * - Scalar matrix will be reduced to one value;
         * - Diagonal matrix will be reduced to array of length Width.
         */
        {
            unsigned index = 0;
            for (InputIter it = src_row_major; index < MaxSize; ++it, ++index)
            {
                const bool on_diagonal = (index / Width) == (index % Width);
                const bool nan = isNaN(*it);
                if (!nan)
                {
                    all_nans = false;
                }
                if (!on_diagonal && !isCloseToZero(*it))
                {
                    scalar_matrix = false;  // This matrix cannot be compressed.
                    diagonal_matrix = false;
                    break;
                }
                if (on_diagonal && !areClose(*it, *src_row_major))
                {
                    scalar_matrix = false;
                }
            }
        }
        /*
         * Actual packing is performed here.
         */
        this->clear();
        if (!all_nans)
        {
            unsigned index = 0;
            for (InputIter it = src_row_major; index < MaxSize; ++it, ++index)
            {
                const bool on_diagonal = (index / Width) == (index % Width);
                if (diagonal_matrix && !on_diagonal)
                {
                    continue;
                }
                this->push_back(ValueType(*it));
                if (scalar_matrix)
                {
                    break;
                }
            }
        }
    }

    template <typename ScalarType, typename OutputIter>
    void unpackSquareMatrixImpl(OutputIter it) const
    {
        StaticAssert<IsDynamic>::check();
        const unsigned Width = CompileTimeIntSqrt<MaxSize>::Result;
        /*
         * Unpacking as follows:
         * - Array of length 1 will be unpacked to scalar matrix
         * - Array of length Width will be unpacked to diagonal matrix
         * - Array of length MaxSize will be unpacked to full matrix
         * - All other length values will yield zero matrix
         */
        if (this->size() == Width || this->size() == 1)
        {
            for (unsigned index = 0; index < MaxSize; index++)
            {
                const bool on_diagonal = (index / Width) == (index % Width);
                if (on_diagonal)
                {
                    const SizeType source_index = SizeType((this->size() == 1) ? 0 : (index / Width));
                    *it++ = ScalarType(this->at(source_index));
                }
                else
                {
                    *it++ = ScalarType(0);
                }
            }
        }
        else if (this->size() == MaxSize)
        {
            for (SizeType index = 0; index < MaxSize; index++)
            {
                *it++ = ScalarType(this->at(index));
            }
        }
        else
        {
            // coverity[suspicious_sizeof : FALSE]
            ::uavcan::fill_n(it, MaxSize, ScalarType(0));
        }
    }

public:
    typedef T RawValueType;                           ///< This may be not the same as the element type.
    typedef typename StorageType<T>::Type ValueType;  ///< This is the actual stored element type.
    typedef typename Base::SizeType SizeType;         ///< Minimal width size type.

    using Base::size;
    using Base::capacity;

    enum { IsDynamic = ArrayMode == ArrayModeDynamic };
    enum { MaxSize = MaxSize_ };
    enum
    {
        MinBitLen = (IsDynamic == 0)
                    ? (static_cast<unsigned>(RawValueType::MinBitLen) * static_cast<unsigned>(MaxSize))
                    : 0
    };
    enum
    {
        MaxBitLen = static_cast<unsigned>(Base::SizeBitLen) +
                    static_cast<unsigned>(RawValueType::MaxBitLen) * static_cast<unsigned>(MaxSize)
    };

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

    /**
     * Only for dynamic arrays. Range checking.
     */
    void pop_back() { Base::shrink(); }
    void push_back(const ValueType& value)
    {
        Base::grow();
        Base::at(SizeType(size() - 1)) = value;
    }

    /**
     * Only for dynamic arrays. Range checking.
     */
    void resize(SizeType new_size, const ValueType& filler)
    {
        if (new_size > size())
        {
            SizeType cnt = SizeType(new_size - size());
            while (cnt-- > 0)
            {
                push_back(filler);
            }
        }
        else if (new_size < size())
        {
            SizeType cnt = SizeType(size() - new_size);
            while (cnt-- > 0)
            {
                pop_back();
            }
        }
        else
        {
            ; // Exact size
        }
    }

    /**
     * Only for dynamic arrays. Range checking.
     */
    void resize(SizeType new_size)
    {
        resize(new_size, ValueType());
    }

    /**
     * This operator accepts any container with size() and [].
     * Members must be comparable via operator ==.
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

    /**
     * This method compares two arrays using @ref areClose(), which ensures proper comparison of
     * floating point values, or DSDL data structures which contain floating point fields at any depth.
     * Please refer to the documentation of @ref areClose() to learn more about how it works and how to
     * define custom fuzzy comparison behavior.
     * Any container with size() and [] is acceptable.
     */
    template <typename R>
    typename EnableIf<sizeof(((const R*)(0U))->size()) && sizeof((*((const R*)(0U)))[0]), bool>::Type
    isClose(const R& rhs) const
    {
        if (size() != rhs.size())
        {
            return false;
        }
        for (SizeType i = 0; i < size(); i++)  // Bitset does not have iterators
        {
            if (!areClose(Base::at(i), rhs[i]))
            {
                return false;
            }
        }
        return true;
    }

    /**
     * This operator can only be used with string-like arrays; otherwise it will fail to compile.
     * @ref c_str()
     */
    bool operator==(const char* ch) const
    {
        if (ch == NULL)
        {
            return false;
        }
        return std::strncmp(Base::c_str(), ch, MaxSize) == 0;
    }

    /**
     * @ref operator==()
     */
    template <typename R> bool operator!=(const R& rhs) const { return !operator==(rhs); }

    /**
     * This operator can only be used with string-like arrays; otherwise it will fail to compile.
     * @ref c_str()
     */
    SelfType& operator=(const char* ch)
    {
        StaticAssert<Base::IsStringLike>::check();
        StaticAssert<IsDynamic>::check();
        Base::clear();
        if (ch == NULL)
        {
            handleFatalError("Array::operator=(const char*)");
        }
        while (*ch)
        {
            push_back(ValueType(*ch++));  // Value type is likely to be unsigned char, so conversion may be required.
        }
        return *this;
    }

    /**
     * This operator can only be used with string-like arrays; otherwise it will fail to compile.
     * @ref c_str()
     */
    SelfType& operator+=(const char* ch)
    {
        StaticAssert<Base::IsStringLike>::check();
        StaticAssert<IsDynamic>::check();
        if (ch == NULL)
        {
            handleFatalError("Array::operator+=(const char*)");
        }
        while (*ch)
        {
            push_back(ValueType(*ch++));
        }
        return *this;
    }

    /**
     * Appends another Array<> with the same element type. Mode and max size can be different.
     */
    template <uavcan::ArrayMode RhsArrayMode, unsigned RhsMaxSize>
    SelfType& operator+=(const Array<T, RhsArrayMode, RhsMaxSize>& rhs)
    {
        typedef Array<T, RhsArrayMode, RhsMaxSize> Rhs;
        StaticAssert<IsDynamic>::check();
        for (typename Rhs::SizeType i = 0; i < rhs.size(); i++)
        {
            push_back(rhs[i]);
        }
        return *this;
    }

    /**
     * Formatting appender.
     * This method doesn't raise an overflow error; instead it silently truncates the data to fit the array capacity.
     * Works only with string-like arrays, otherwise fails to compile.
     * @param format    Format string for std::snprintf(), e.g. "%08x", "%f"
     * @param value     Arbitrary value of a primitive type (should fail to compile if there's a non-primitive type)
     */
    template <typename A>
    void appendFormatted(const char* const format, const A value)
    {
        StaticAssert<Base::IsStringLike>::check();
        StaticAssert<IsDynamic>::check();

        StaticAssert<sizeof(A() >= A(0))>::check();              // This check allows to weed out most compound types
        StaticAssert<sizeof(A) <= sizeof(long double)>::check(); // Another stupid check to catch non-primitive types

        if (!format)
        {
            UAVCAN_ASSERT(0);
            return;
        }
        // Add some hardcore runtime checks for the format string correctness?

        ValueType* const ptr = Base::end();
        UAVCAN_ASSERT(capacity() >= size());
        const SizeType max_size = SizeType(capacity() - size());

        // We have one extra byte for the null terminator, hence +1
        using namespace std; // For snprintf()
        const int ret = snprintf(reinterpret_cast<char*>(ptr), SizeType(max_size + 1U), format, value);

        for (int i = 0; i < min(ret, int(max_size)); i++)
        {
            Base::grow();
        }
        if (ret < 0)
        {
            UAVCAN_ASSERT(0);    // Likely an invalid format string
            (*this) += format;   // So we print it as is in release builds
        }
    }

    /**
     * Fills this array as a packed square matrix from a static array.
     * Please refer to the specification to learn more about matrix packing.
     * Note that matrix packing code uses @ref areClose() for comparison.
     */
    template <typename ScalarType>
    void packSquareMatrix(const ScalarType (&src_row_major)[MaxSize])
    {
        packSquareMatrixImpl<ScalarType, const ScalarType*>(src_row_major);
    }

    /**
     * Fills this array as a packed square matrix in place.
     * Please refer to the specification to learn more about matrix packing.
     * Note that matrix packing code uses @ref areClose() for comparison.
     */
    void packSquareMatrix()
    {
        if (this->size() == MaxSize)
        {
            ValueType matrix[MaxSize];
            for (SizeType i = 0; i < MaxSize; i++)
            {
                matrix[i] = this->at(i);
            }
            packSquareMatrix(matrix);
        }
        else if (this->size() == 0)
        {
            ; // Nothing to do - leave the matrix empty
        }
        else
        {
#if UAVCAN_EXCEPTIONS
            throw std::out_of_range("uavcan::Array::packSquareMatrix()");
#else
            UAVCAN_ASSERT(0);
            this->clear();
#endif
        }

    }

    /**
     * Fills this array as a packed square matrix from any container that has the following public entities:
     *  - method begin()
     *  - method size()
     *  - only for C++03: type value_type
     * Please refer to the specification to learn more about matrix packing.
     * Note that matrix packing code uses @ref areClose() for comparison.
     */
    template <typename R>
    typename EnableIf<sizeof(((const R*)(0U))->begin()) && sizeof(((const R*)(0U))->size())>::Type
    packSquareMatrix(const R& src_row_major)
    {
        if (src_row_major.size() == MaxSize)
        {
#if UAVCAN_CPP_VERSION > UAVCAN_CPP03
            typedef typename RemoveReference<decltype(*src_row_major.begin())>::Type RhsValueType;
            packSquareMatrixImpl<RhsValueType>(src_row_major.begin());
#else
            packSquareMatrixImpl<typename R::value_type>(src_row_major.begin());
#endif
        }
        else if (src_row_major.size() == 0)
        {
            this->clear();
        }
        else
        {
#if UAVCAN_EXCEPTIONS
            throw std::out_of_range("uavcan::Array::packSquareMatrix()");
#else
            UAVCAN_ASSERT(0);
            this->clear();
#endif
        }
    }

    /**
     * Reconstructs full matrix, result will be saved into a static array.
     * Please refer to the specification to learn more about matrix packing.
     */
    template <typename ScalarType>
    void unpackSquareMatrix(ScalarType (&dst_row_major)[MaxSize]) const
    {
        unpackSquareMatrixImpl<ScalarType, ScalarType*>(dst_row_major);
    }

    /**
     * Reconstructs full matrix in place.
     * Please refer to the specification to learn more about matrix packing.
     */
    void unpackSquareMatrix()
    {
        ValueType matrix[MaxSize];
        unpackSquareMatrix(matrix);

        this->clear();
        for (unsigned i = 0; i < MaxSize; i++)
        {
            this->push_back(matrix[i]);
        }
    }

    /**
     * Reconstructs full matrix, result will be saved into container that has the following public entities:
     *  - method begin()
     *  - method size()
     *  - only for C++03: type value_type
     * Please refer to the specification to learn more about matrix packing.
     */
    template <typename R>
    typename EnableIf<sizeof(((const R*)(0U))->begin()) && sizeof(((const R*)(0U))->size())>::Type
    unpackSquareMatrix(R& dst_row_major) const
    {
        if (dst_row_major.size() == MaxSize)
        {
#if UAVCAN_CPP_VERSION > UAVCAN_CPP03
            typedef typename RemoveReference<decltype(*dst_row_major.begin())>::Type RhsValueType;
            unpackSquareMatrixImpl<RhsValueType>(dst_row_major.begin());
#else
            unpackSquareMatrixImpl<typename R::value_type>(dst_row_major.begin());
#endif
        }
        else
        {
#if UAVCAN_EXCEPTIONS
            throw std::out_of_range("uavcan::Array::unpackSquareMatrix()");
#else
            UAVCAN_ASSERT(0);
#endif
        }
    }

    /**
     * Aliases for compatibility with standard containers.
     */
    typedef ValueType value_type;
    typedef SizeType size_type;
};

template <typename R, typename T, ArrayMode ArrayMode, unsigned MaxSize>
UAVCAN_EXPORT
inline bool operator==(const R& rhs, const Array<T, ArrayMode, MaxSize>& lhs)
{
    return lhs.operator==(rhs);
}

template <typename R, typename T, ArrayMode ArrayMode, unsigned MaxSize>
UAVCAN_EXPORT
inline bool operator!=(const R& rhs, const Array<T, ArrayMode, MaxSize>& lhs)
{
    return lhs.operator!=(rhs);
}

/**
 * YAML streamer specification for any Array<>
 */
template <typename T, ArrayMode ArrayMode, unsigned MaxSize>
class UAVCAN_EXPORT YamlStreamer<Array<T, ArrayMode, MaxSize> >
{
    typedef Array<T, ArrayMode, MaxSize> ArrayType;

    static bool isNiceCharacter(int c)
    {
        if (c >= 32 && c <= 126)
        {
            return true;
        }
        static const char Good[] = {'\n', '\r', '\t'};
        for (unsigned i = 0; i < sizeof(Good) / sizeof(Good[0]); i++)
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
        for (typename ArrayType::SizeType i = 0; i < array.size(); i++)
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
        for (typename ArrayType::SizeType i = 0; i < array.size(); i++)
        {
            const int c = array.at(i);
            if (c < 32 || c > 126)
            {
                char nibbles[2] = {char((c >> 4) & 0xF), char(c & 0xF)};
                for (int k = 0; k < 2; k++)
                {
                    nibbles[k] = char(nibbles[k] + '0');
                    if (nibbles[k] > '9')
                    {
                        nibbles[k] = char(nibbles[k] + 'A' - '9' - 1);
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
        for (typename ArrayType::SizeType i = 0; i < array.size(); i++)
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
        for (typename ArrayType::SizeType i = 0; i < array.size(); i++)
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
    /**
     * Prints Array<> into the stream in YAML format.
     */
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
