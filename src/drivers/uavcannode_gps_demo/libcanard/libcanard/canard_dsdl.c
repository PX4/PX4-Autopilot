/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2016-2020 UAVCAN Development Team.
/// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "canard_dsdl.h"
#include <assert.h>
#include <float.h>
#include <string.h>

// --------------------------------------------- BUILD CONFIGURATION ---------------------------------------------

/// There are two implementations of the primitive (de-)serialization algorithms: a generic one, which is invariant
/// to the native byte order (and therefore compatible with any platform), and the optimized one which is compatible
/// with little-endian platforms only. By default, the slow generic algorithm is used.
/// If the target platform is little-endian, the user can enable this option to use the optimized algorithm.
#ifndef CANARD_DSDL_CONFIG_LITTLE_ENDIAN
#    define CANARD_DSDL_CONFIG_LITTLE_ENDIAN false
#endif

/// By default, this macro resolves to the standard assert(). The user can redefine this if necessary.
/// To disable assertion checks completely, make it expand into `(void)(0)`.
#ifndef CANARD_ASSERT
// Intentional violation of MISRA: assertion macro cannot be replaced with a function definition.
#    define CANARD_ASSERT(x) assert(x)  // NOSONAR
#endif

/// This macro is needed only for testing and for library development. Do not redefine this in production.
#ifndef CANARD_PRIVATE
#    define CANARD_PRIVATE static
#endif

#if !defined(__STDC_VERSION__) || (__STDC_VERSION__ < 199901L)
#    error "Unsupported language: ISO C99 or a newer version is required."
#endif

/// In general, _Static_assert is not present on C99 compilers, except for gnu99
#if !defined(static_assert)
// Intentional violation of MISRA: static assertion macro cannot be replaced with a function definition.
#    define static_assert(x, ...) typedef char _static_assert_gl(_static_assertion_, __LINE__)[(x) ? 1 : -1]  // NOSONAR
#    define _static_assert_gl(a, b) _static_assert_gl_impl(a, b)                                              // NOSONAR
// Intentional violation of MISRA: the paste operator ## cannot be avoided in this context.
#    define _static_assert_gl_impl(a, b) a##b  // NOSONAR
#endif

/// Detect whether the target platform is compatible with IEEE 754.
#define CANARD_DSDL_PLATFORM_IEEE754_FLOAT \
    ((FLT_RADIX == 2) && (FLT_MANT_DIG == 24) && (FLT_MIN_EXP == -125) && (FLT_MAX_EXP == 128))
#define CANARD_DSDL_PLATFORM_IEEE754_DOUBLE \
    ((FLT_RADIX == 2) && (DBL_MANT_DIG == 53) && (DBL_MIN_EXP == -1021) && (DBL_MAX_EXP == 1024))

// --------------------------------------------- COMMON ITEMS ---------------------------------------------

/// Per the DSDL specification, 1 byte = 8 bits.
#define BYTE_WIDTH 8U
#define BYTE_MAX 0xFFU

#define WIDTH16 16U
#define WIDTH32 32U
#define WIDTH64 64U

// --------------------------------------------- PRIMITIVE SERIALIZATION ---------------------------------------------

CANARD_PRIVATE size_t chooseMin(size_t a, size_t b)
{
    return (a < b) ? a : b;
}

CANARD_PRIVATE size_t getBitCopySize(const size_t  buf_size_bytes,
                                     const size_t  offset_bit,
                                     const size_t  requested_length_bit,
                                     const uint8_t value_length_bit)
{
    const size_t buf_size_bit  = buf_size_bytes * BYTE_WIDTH;
    const size_t remaining_bit = buf_size_bit - chooseMin(buf_size_bit, offset_bit);
    return chooseMin(remaining_bit, chooseMin(requested_length_bit, value_length_bit));
}

// --------------------------------------------- PUBLIC API - BIT ARRAY ---------------------------------------------

void canardDSDLCopyBits(const size_t      length_bit,
                        const size_t      src_offset_bit,
                        const size_t      dst_offset_bit,
                        const void* const src,
                        void* const       dst)
{
    CANARD_ASSERT((src != NULL) && (dst != NULL) && (src != dst));
    if ((0U == (src_offset_bit % BYTE_WIDTH)) && (0U == (dst_offset_bit % BYTE_WIDTH)))
    {
        const size_t length_bytes = (size_t)(length_bit / BYTE_WIDTH);
        // Intentional violation of MISRA: Pointer arithmetics. This is done to remove the API constraint that
        // offsets be under 8 bits. Fewer constraints reduce the chance of API misuse.
        const uint8_t* const psrc = (src_offset_bit / BYTE_WIDTH) + (const uint8_t*) src;  // NOSONAR NOLINT
        uint8_t* const       pdst = (dst_offset_bit / BYTE_WIDTH) + (uint8_t*) dst;        // NOSONAR NOLINT
        // Clang-Tidy raises an error recommending the use of memcpy_s() instead.
        // We ignore it because the safe functions are poorly supported; reliance on them may limit the portability.
        (void) memcpy(pdst, psrc, length_bytes);  // NOLINT
        const uint8_t length_mod = (uint8_t)(length_bit % BYTE_WIDTH);
        if (0U != length_mod)  // If the length is unaligned, the last byte requires special treatment.
        {
            // Intentional violation of MISRA: Pointer arithmetics. It is unavoidable in this context.
            const uint8_t* const last_src = psrc + length_bytes;  // NOLINT NOSONAR
            uint8_t* const       last_dst = pdst + length_bytes;  // NOLINT NOSONAR
            CANARD_ASSERT(length_mod < BYTE_WIDTH);
            const uint8_t mask = (uint8_t)((1U << length_mod) - 1U);
            *last_dst          = (uint8_t)(*last_dst & (uint8_t) ~mask) | (uint8_t)(*last_src & mask);
        }
    }
    else
    {
        // The algorithm was originally designed by Ben Dyer for Libuavcan v0:
        // https://github.com/UAVCAN/libuavcan/blob/legacy-v0/libuavcan/src/marshal/uc_bit_array_copy.cpp#L12-L58
        // This version is modified for v1 where the bit order is the opposite.
        const uint8_t* const psrc     = (const uint8_t*) src;
        uint8_t* const       pdst     = (uint8_t*) dst;
        size_t               src_off  = src_offset_bit;
        size_t               dst_off  = dst_offset_bit;
        const size_t         last_bit = src_off + length_bit;
        while (last_bit > src_off)
        {
            const uint8_t src_mod = (uint8_t)(src_off % BYTE_WIDTH);
            const uint8_t dst_mod = (uint8_t)(dst_off % BYTE_WIDTH);
            const uint8_t max_mod = (src_mod > dst_mod) ? src_mod : dst_mod;

            const uint8_t size = (uint8_t) chooseMin(BYTE_WIDTH - max_mod, last_bit - src_off);
            CANARD_ASSERT((size > 0U) && (size <= BYTE_WIDTH));

            // Suppress a false warning from Clang-Tidy & Sonar that size is being over-shifted. It's not.
            const uint8_t mask = (uint8_t)((((1U << size) - 1U) << dst_mod) & BYTE_MAX);  // NOLINT NOSONAR
            CANARD_ASSERT(mask > 0U);

            // Intentional violation of MISRA: indexing on a pointer.
            // This simplifies the implementation greatly and avoids pointer arithmetics.
            const uint8_t in =
                (uint8_t)((uint8_t)(psrc[src_off / BYTE_WIDTH] >> src_mod) << dst_mod) & BYTE_MAX;  // NOSONAR

            // Intentional violation of MISRA: indexing on a pointer.
            // This simplifies the implementation greatly and avoids pointer arithmetics.
            const uint8_t a = pdst[dst_off / BYTE_WIDTH] & ((uint8_t) ~mask);  // NOSONAR
            const uint8_t b = in & mask;

            // Intentional violation of MISRA: indexing on a pointer.
            // This simplifies the implementation greatly and avoids pointer arithmetics.
            pdst[dst_off / BYTE_WIDTH] = a | b;  // NOSONAR

            src_off += size;
            dst_off += size;
        }
        CANARD_ASSERT(last_bit == src_off);
    }
}

// --------------------------------------------- PUBLIC API - INTEGER ---------------------------------------------

void canardDSDLSetBit(uint8_t* const buf, const size_t off_bit, const bool value)
{
    CANARD_ASSERT(buf != NULL);
    const uint8_t val = value ? 1U : 0U;
    canardDSDLCopyBits(1U, 0U, off_bit, &val, buf);
}

static_assert(WIDTH64 == (sizeof(uint64_t) * BYTE_WIDTH), "Unexpected size of uint64_t");

void canardDSDLSetUxx(uint8_t* const buf, const size_t off_bit, const uint64_t value, const uint8_t len_bit)
{
    CANARD_ASSERT(buf != NULL);
    const size_t saturated_len_bit = chooseMin(len_bit, WIDTH64);
#if CANARD_DSDL_CONFIG_LITTLE_ENDIAN
    canardDSDLCopyBits(saturated_len_bit, 0U, off_bit, (const uint8_t*) &value, buf);
#else
    const uint8_t tmp[sizeof(uint64_t)] = {
        (uint8_t)((value >> 0U) & BYTE_MAX),   // Suppress warnings about the magic numbers. Their purpose is clear.
        (uint8_t)((value >> 8U) & BYTE_MAX),   // NOLINT NOSONAR
        (uint8_t)((value >> 16U) & BYTE_MAX),  // NOLINT NOSONAR
        (uint8_t)((value >> 24U) & BYTE_MAX),  // NOLINT NOSONAR
        (uint8_t)((value >> 32U) & BYTE_MAX),  // NOLINT NOSONAR
        (uint8_t)((value >> 40U) & BYTE_MAX),  // NOLINT NOSONAR
        (uint8_t)((value >> 48U) & BYTE_MAX),  // NOLINT NOSONAR
        (uint8_t)((value >> 56U) & BYTE_MAX),  // NOLINT NOSONAR
    };
    canardDSDLCopyBits(saturated_len_bit, 0U, off_bit, &tmp[0], buf);
#endif
}

void canardDSDLSetIxx(uint8_t* const buf, const size_t off_bit, const int64_t value, const uint8_t len_bit)
{
    // The naive sign conversion is safe and portable according to the C standard:
    // 6.3.1.3.3: if the new type is unsigned, the value is converted by repeatedly adding or subtracting one more
    // than the maximum value that can be represented in the new type until the value is in the range of the new type.
    canardDSDLSetUxx(buf, off_bit, (uint64_t) value, len_bit);
}

bool canardDSDLGetBit(const uint8_t* const buf, const size_t buf_size, const size_t off_bit)
{
    return 1U == canardDSDLGetU8(buf, buf_size, off_bit, 1U);
}

uint8_t canardDSDLGetU8(const uint8_t* const buf, const size_t buf_size, const size_t off_bit, const uint8_t len_bit)
{
    CANARD_ASSERT(buf != NULL);
    const size_t copy_size = getBitCopySize(buf_size, off_bit, len_bit, BYTE_WIDTH);
    CANARD_ASSERT(copy_size <= (sizeof(uint8_t) * BYTE_WIDTH));
    uint8_t val = 0;
    canardDSDLCopyBits(copy_size, off_bit, 0U, buf, &val);
    return val;
}

uint16_t canardDSDLGetU16(const uint8_t* const buf, const size_t buf_size, const size_t off_bit, const uint8_t len_bit)
{
    CANARD_ASSERT(buf != NULL);
    const size_t copy_size = getBitCopySize(buf_size, off_bit, len_bit, WIDTH16);
    CANARD_ASSERT(copy_size <= (sizeof(uint16_t) * BYTE_WIDTH));
#if CANARD_DSDL_CONFIG_LITTLE_ENDIAN
    uint16_t val = 0U;
    canardDSDLCopyBits(copy_size, off_bit, 0U, buf, (uint8_t*) &val);
    return val;
#else
    uint8_t tmp[sizeof(uint16_t)] = {0};
    canardDSDLCopyBits(copy_size, off_bit, 0U, buf, &tmp[0]);
    return (uint16_t)(tmp[0] | (uint16_t)(((uint16_t) tmp[1]) << BYTE_WIDTH));
#endif
}

uint32_t canardDSDLGetU32(const uint8_t* const buf, const size_t buf_size, const size_t off_bit, const uint8_t len_bit)
{
    CANARD_ASSERT(buf != NULL);
    const size_t copy_size = getBitCopySize(buf_size, off_bit, len_bit, WIDTH32);
    CANARD_ASSERT(copy_size <= (sizeof(uint32_t) * BYTE_WIDTH));
#if CANARD_DSDL_CONFIG_LITTLE_ENDIAN
    uint32_t val = 0U;
    canardDSDLCopyBits(copy_size, off_bit, 0U, buf, (uint8_t*) &val);
    return val;
#else
    uint8_t tmp[sizeof(uint32_t)] = {0};
    canardDSDLCopyBits(copy_size, off_bit, 0U, buf, &tmp[0]);
    return (uint32_t)(tmp[0] |                      // Suppress warnings about the magic numbers.
                      ((uint32_t) tmp[1] << 8U) |   // NOLINT NOSONAR
                      ((uint32_t) tmp[2] << 16U) |  // NOLINT NOSONAR
                      ((uint32_t) tmp[3] << 24U));  // NOLINT NOSONAR
#endif
}

uint64_t canardDSDLGetU64(const uint8_t* const buf, const size_t buf_size, const size_t off_bit, const uint8_t len_bit)
{
    CANARD_ASSERT(buf != NULL);
    const size_t copy_size = getBitCopySize(buf_size, off_bit, len_bit, WIDTH64);
    CANARD_ASSERT(copy_size <= (sizeof(uint64_t) * BYTE_WIDTH));
#if CANARD_DSDL_CONFIG_LITTLE_ENDIAN
    uint64_t val = 0U;
    canardDSDLCopyBits(copy_size, off_bit, 0U, buf, (uint8_t*) &val);
    return val;
#else
    uint8_t tmp[sizeof(uint64_t)] = {0};
    canardDSDLCopyBits(copy_size, off_bit, 0U, buf, &tmp[0]);
    return (uint64_t)(tmp[0] |                      // Suppress warnings about the magic numbers.
                      ((uint64_t) tmp[1] << 8U) |   // NOLINT NOSONAR
                      ((uint64_t) tmp[2] << 16U) |  // NOLINT NOSONAR
                      ((uint64_t) tmp[3] << 24U) |  // NOLINT NOSONAR
                      ((uint64_t) tmp[4] << 32U) |  // NOLINT NOSONAR
                      ((uint64_t) tmp[5] << 40U) |  // NOLINT NOSONAR
                      ((uint64_t) tmp[6] << 48U) |  // NOLINT NOSONAR
                      ((uint64_t) tmp[7] << 56U));  // NOLINT NOSONAR
#endif
}

int8_t canardDSDLGetI8(const uint8_t* const buf, const size_t buf_size, const size_t off_bit, const uint8_t len_bit)
{
    const uint8_t sat = (uint8_t) chooseMin(len_bit, BYTE_WIDTH);
    uint8_t       val = canardDSDLGetU8(buf, buf_size, off_bit, sat);
    const bool    neg = (sat > 0U) && ((val & (1ULL << (sat - 1U))) != 0U);
    val               = ((sat < BYTE_WIDTH) && neg) ? (uint8_t)(val | ~((1U << sat) - 1U)) : val;  // Sign extension
    return neg ? (int8_t)((-(int8_t)(uint8_t) ~val) - 1) : (int8_t) val;
}

int16_t canardDSDLGetI16(const uint8_t* const buf, const size_t buf_size, const size_t off_bit, const uint8_t len_bit)
{
    const uint8_t sat = (uint8_t) chooseMin(len_bit, WIDTH16);
    uint16_t      val = canardDSDLGetU16(buf, buf_size, off_bit, sat);
    const bool    neg = (sat > 0U) && ((val & (1ULL << (sat - 1U))) != 0U);
    val               = ((sat < WIDTH16) && neg) ? (uint16_t)(val | ~((1U << sat) - 1U)) : val;  // Sign extension
    return neg ? (int16_t)((-(int16_t)(uint16_t) ~val) - 1) : (int16_t) val;
}

int32_t canardDSDLGetI32(const uint8_t* const buf, const size_t buf_size, const size_t off_bit, const uint8_t len_bit)
{
    const uint8_t sat = (uint8_t) chooseMin(len_bit, WIDTH32);
    uint32_t      val = canardDSDLGetU32(buf, buf_size, off_bit, sat);
    const bool    neg = (sat > 0U) && ((val & (1ULL << (sat - 1U))) != 0U);
    val               = ((sat < WIDTH32) && neg) ? (uint32_t)(val | ~((1UL << sat) - 1U)) : val;  // Sign extension
    return neg ? (int32_t)((-(int32_t) ~val) - 1) : (int32_t) val;
}

int64_t canardDSDLGetI64(const uint8_t* const buf, const size_t buf_size, const size_t off_bit, const uint8_t len_bit)
{
    const uint8_t sat = (uint8_t) chooseMin(len_bit, WIDTH64);
    uint64_t      val = canardDSDLGetU64(buf, buf_size, off_bit, sat);
    const bool    neg = (sat > 0U) && ((val & (1ULL << (sat - 1U))) != 0U);
    val               = ((sat < WIDTH64) && neg) ? (uint64_t)(val | ~((1ULL << sat) - 1U)) : val;  // Sign extension
    return neg ? (int64_t)((-(int64_t) ~val) - 1) : (int64_t) val;
}

// --------------------------------------------- PUBLIC API - FLOAT16 ---------------------------------------------

#if CANARD_DSDL_PLATFORM_IEEE754_FLOAT

static_assert(WIDTH32 == (sizeof(CanardDSDLFloat32) * BYTE_WIDTH), "Unsupported floating point model");

// Intentional violation of MISRA: we need this union because the alternative is far more error prone.
// We have to rely on low-level data representation details to do the conversion; unions are helpful.
typedef union  // NOSONAR
{
    uint32_t          bits;
    CanardDSDLFloat32 real;
} Float32Bits;

CANARD_PRIVATE uint16_t float16Pack(const CanardDSDLFloat32 value)
{
    // The no-lint statements suppress the warnings about magic numbers.
    // The no-lint statements suppress the warning about the use of union. This is required for low-level bit access.
    const uint32_t    round_mask = ~(uint32_t) 0x0FFFU;                 // NOLINT NOSONAR
    const Float32Bits f32inf     = {.bits = ((uint32_t) 255U) << 23U};  // NOLINT NOSONAR
    const Float32Bits f16inf     = {.bits = ((uint32_t) 31U) << 23U};   // NOLINT NOSONAR
    const Float32Bits magic      = {.bits = ((uint32_t) 15U) << 23U};   // NOLINT NOSONAR
    Float32Bits       in         = {.real = value};                     // NOSONAR
    const uint32_t    sign       = in.bits & (((uint32_t) 1U) << 31U);  // NOLINT NOSONAR
    in.bits ^= sign;
    uint16_t out = 0;
    if (in.bits >= f32inf.bits)
    {
        // The no-lint statements suppress the warnings about magic numbers.
        if ((in.bits & 0x7FFFFFUL) != 0)  // NOLINT NOSONAR
        {
            out = 0x7E00U;  // NOLINT NOSONAR
        }
        else
        {
            out = (in.bits > f32inf.bits) ? (uint16_t) 0x7FFFU : (uint16_t) 0x7C00U;  // NOLINT NOSONAR
        }
    }
    else
    {
        in.bits &= round_mask;
        in.real *= magic.real;
        in.bits -= round_mask;
        if (in.bits > f16inf.bits)
        {
            in.bits = f16inf.bits;
        }
        out = (uint16_t)(in.bits >> 13U);  // NOLINT NOSONAR
    }
    out |= (uint16_t)(sign >> 16U);  // NOLINT NOSONAR
    return out;
}

CANARD_PRIVATE CanardDSDLFloat32 float16Unpack(const uint16_t value)
{
    // The no-lint statements suppress the warnings about magic numbers.
    // The no-lint statements suppress the warning about the use of union. This is required for low-level bit access.
    const Float32Bits magic   = {.bits = ((uint32_t) 0xEFU) << 23U};             // NOLINT NOSONAR
    const Float32Bits inf_nan = {.bits = ((uint32_t) 0x8FU) << 23U};             // NOLINT NOSONAR
    Float32Bits       out     = {.bits = ((uint32_t)(value & 0x7FFFU)) << 13U};  // NOLINT NOSONAR
    out.real *= magic.real;
    if (out.real >= inf_nan.real)
    {
        out.bits |= ((uint32_t) 0xFFU) << 23U;  // NOLINT NOSONAR
    }
    out.bits |= ((uint32_t)(value & 0x8000U)) << 16U;  // NOLINT NOSONAR
    return out.real;
}

void canardDSDLSetF16(uint8_t* const buf, const size_t off_bit, const CanardDSDLFloat32 value)
{
    canardDSDLSetUxx(buf, off_bit, float16Pack(value), WIDTH16);
}

CanardDSDLFloat32 canardDSDLGetF16(const uint8_t* const buf, const size_t buf_size, const size_t off_bit)
{
    return float16Unpack(canardDSDLGetU16(buf, buf_size, off_bit, WIDTH16));
}

#endif  // CANARD_DSDL_PLATFORM_IEEE754_FLOAT

// --------------------------------------------- PUBLIC API - FLOAT32 ---------------------------------------------

#if CANARD_DSDL_PLATFORM_IEEE754_FLOAT

static_assert(WIDTH32 == (sizeof(CanardDSDLFloat32) * BYTE_WIDTH), "Unsupported floating point model");

void canardDSDLSetF32(uint8_t* const buf, const size_t off_bit, const CanardDSDLFloat32 value)
{
    // Intentional violation of MISRA: use union to perform fast conversion from an IEEE 754-compatible native
    // representation into a serializable integer. The assumptions about the target platform properties are made
    // clear. In the future we may add a more generic conversion that is platform-invariant.
    union  // NOSONAR
    {
        CanardDSDLFloat32 fl;
        uint32_t          in;
    } const tmp = {value};  // NOSONAR
    canardDSDLSetUxx(buf, off_bit, tmp.in, sizeof(tmp) * BYTE_WIDTH);
}

CanardDSDLFloat32 canardDSDLGetF32(const uint8_t* const buf, const size_t buf_size, const size_t off_bit)
{
    // Intentional violation of MISRA: use union to perform fast conversion to an IEEE 754-compatible native
    // representation into a serializable integer. The assumptions about the target platform properties are made
    // clear. In the future we may add a more generic conversion that is platform-invariant.
    union  // NOSONAR
    {
        uint32_t          in;
        CanardDSDLFloat32 fl;
    } const tmp = {canardDSDLGetU32(buf, buf_size, off_bit, WIDTH32)};  // NOSONAR
    return tmp.fl;
}

#endif  // CANARD_DSDL_PLATFORM_IEEE754_FLOAT

// --------------------------------------------- PUBLIC API - FLOAT64 ---------------------------------------------

#if CANARD_DSDL_PLATFORM_IEEE754_DOUBLE

static_assert(WIDTH64 == (sizeof(CanardDSDLFloat64) * BYTE_WIDTH), "Unsupported floating point model");

CanardDSDLFloat64 canardDSDLGetF64(const uint8_t* const buf, const size_t buf_size, const size_t off_bit)
{
    // Intentional violation of MISRA: use union to perform fast conversion to an IEEE 754-compatible native
    // representation into a serializable integer. The assumptions about the target platform properties are made
    // clear. In the future we may add a more generic conversion that is platform-invariant.
    union  // NOSONAR
    {
        uint64_t          in;
        CanardDSDLFloat64 fl;
    } const tmp = {canardDSDLGetU64(buf, buf_size, off_bit, WIDTH64)};  // NOSONAR
    return tmp.fl;
}

void canardDSDLSetF64(uint8_t* const buf, const size_t off_bit, const CanardDSDLFloat64 value)
{
    // Intentional violation of MISRA: use union to perform fast conversion from an IEEE 754-compatible native
    // representation into a serializable integer. The assumptions about the target platform properties are made
    // clear. In the future we may add a more generic conversion that is platform-invariant.
    union  // NOSONAR
    {
        CanardDSDLFloat64 fl;
        uint64_t          in;
    } const tmp = {value};  // NOSONAR
    canardDSDLSetUxx(buf, off_bit, tmp.in, sizeof(tmp) * BYTE_WIDTH);
}

#endif  // CANARD_DSDL_PLATFORM_IEEE754_DOUBLE
