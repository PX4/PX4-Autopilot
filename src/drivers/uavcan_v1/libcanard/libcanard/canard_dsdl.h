///                         __   __   _______   __   __   _______   _______   __   __
///                        |  | |  | /   _   ` |  | |  | /   ____| /   _   ` |  ` |  |
///                        |  | |  | |  |_|  | |  | |  | |  |      |  |_|  | |   `|  |
///                        |  |_|  | |   _   | `  `_/  / |  |____  |   _   | |  |`   |
///                        `_______/ |__| |__|  `_____/  `_______| |__| |__| |__| `__|
///                            |      |            |         |      |         |
///                        ----o------o------------o---------o------o---------o-------
///
/// This is a DSDL serialization helper for libcanard -- a trivial optional extension library that contains basic
/// DSDL field serialization routines. It is intended for use in unconventional applications where auto-generated
/// DSDL serialization routines are not available. Most applications are not expected to need this; instead, they are
/// recommended to automatically transpile DSDL into C using Nunavut: https://github.com/UAVCAN/nunavut.
///
/// This library is designed to be compatible with any instruction set architecture (including big endian platforms)
/// but the floating point functionality will be automatically disabled at compile time if the target platform does not
/// use an IEEE 754-compatible floating point model. Support for other floating point models may be implemented later.
/// If your application requires non-IEEE-754 floats, please reach out to the maintainers via https://forum.uavcan.org.
///
/// To use the library, copy the files canard_dsdl.c and canard_dsdl.h into the source tree of the application.
/// No special compilation options are required. There are optional build configuration options defined near the top
/// of canard_dsdl.c; they may be used to fine-tune the library for the target platform (but it is not necessary).
///
/// Some high-integrity systems may prefer to avoid this extension because it relies on unsafe memory operations.
///
/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2016-2020 UAVCAN Development Team.
/// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#ifndef CANARD_DSDL_H_INCLUDED
#define CANARD_DSDL_H_INCLUDED

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef float  CanardDSDLFloat32;
typedef double CanardDSDLFloat64;

/// Copy the specified number of bits from the source buffer into the destination buffer in accordance with the
/// DSDL bit-level serialization specification. The offsets may be arbitrary (may exceed 8 bits).
/// If both offsets are byte-aligned, the algorithm degenerates to memcpy() (the last byte may be copied separately).
/// If the source and the destination overlap, the behavior is undefined.
/// If either source or destination pointers are NULL, the behavior is undefined.
/// Arguments:
///     length_bit      The number of bits to copy. Both source and destination shall be large enough.
///     src_offset_bit  Offset in bits from the source pointer. May exceed 8.
///     dst_offset_bit  Offset in bits from the destination pointer. May exceed 8.
///     src             Source buffer. Shall be at least ceil(length_bit/8) bytes large.
///     dst             Destination buffer. Shall be at least ceil(length_bit/8) bytes large.
void canardDSDLCopyBits(const size_t      length_bit,
                        const size_t      src_offset_bit,
                        const size_t      dst_offset_bit,
                        const void* const src,
                        void* const       dst);

/// Serialize a DSDL field value at the specified bit offset from the beginning of the destination buffer.
/// The behavior is undefined if the input pointer is NULL. The time complexity is linear of the bit length.
/// One-bit-wide signed integers are processed without raising an error but the result is unspecified.
/// The floating point functions are only available if the target platform has an IEEE 754-compatible float model.
///
/// Arguments:
///     buf     Destination buffer where the result will be stored.
///     off_bit Offset, in bits, from the beginning of the buffer. May exceed one byte.
///     value   The value itself (in case of integers it is promoted to 64-bit for unification).
///     len_bit Length of the serialized representation, in bits. Zero has no effect. Values above 64 bit are saturated.
void canardDSDLSetBit(uint8_t* const buf, const size_t off_bit, const bool value);
void canardDSDLSetUxx(uint8_t* const buf, const size_t off_bit, const uint64_t value, const uint8_t len_bit);
void canardDSDLSetIxx(uint8_t* const buf, const size_t off_bit, const int64_t value, const uint8_t len_bit);
void canardDSDLSetF16(uint8_t* const buf, const size_t off_bit, const CanardDSDLFloat32 value);
void canardDSDLSetF32(uint8_t* const buf, const size_t off_bit, const CanardDSDLFloat32 value);
void canardDSDLSetF64(uint8_t* const buf, const size_t off_bit, const CanardDSDLFloat64 value);

/// Deserialize a DSDL field value located at the specified bit offset from the beginning of the source buffer.
/// If the deserialized value extends beyond the end of the buffer, the missing bits are taken as zero, as required
/// by the DSDL specification (see Implicit Zero Extension Rule, IZER).
/// The floating point functions are only available if the target platform has an IEEE 754-compatible float model.
///
/// If len_bit is greater than the return type, extra bits will be truncated per standard narrowing conversion rules.
/// If len_bit is shorter than the return type, missing bits will be zero per standard integer promotion rules.
/// Essentially, for integers, it would be enough to have 64-bit versions only; narrower variants exist only to avoid
/// narrowing type conversions of the result and for some performance gains.
///
/// The behavior is undefined if the input pointer is NULL. The time complexity is linear of the bit length.
/// One-bit-wide signed integers are processed without raising an error but the result is unspecified.
///
/// Arguments:
///     buf      Source buffer where the serialized representation will be read from.
///     buf_size The size of the source buffer, in bytes. Reads past this limit will be assumed to return zero bits.
///     off_bit  Offset, in bits, from the beginning of the buffer. May exceed one byte.
///     len_bit  Length of the serialized representation, in bits. Zero returns zero. Out-of-range values are saturated.
bool     canardDSDLGetBit(const uint8_t* const buf, const size_t buf_size, const size_t off_bit);
uint8_t  canardDSDLGetU8(const uint8_t* const buf, const size_t buf_size, const size_t off_bit, const uint8_t len_bit);
uint16_t canardDSDLGetU16(const uint8_t* const buf, const size_t buf_size, const size_t off_bit, const uint8_t len_bit);
uint32_t canardDSDLGetU32(const uint8_t* const buf, const size_t buf_size, const size_t off_bit, const uint8_t len_bit);
uint64_t canardDSDLGetU64(const uint8_t* const buf, const size_t buf_size, const size_t off_bit, const uint8_t len_bit);
int8_t   canardDSDLGetI8(const uint8_t* const buf, const size_t buf_size, const size_t off_bit, const uint8_t len_bit);
int16_t  canardDSDLGetI16(const uint8_t* const buf, const size_t buf_size, const size_t off_bit, const uint8_t len_bit);
int32_t  canardDSDLGetI32(const uint8_t* const buf, const size_t buf_size, const size_t off_bit, const uint8_t len_bit);
int64_t  canardDSDLGetI64(const uint8_t* const buf, const size_t buf_size, const size_t off_bit, const uint8_t len_bit);
CanardDSDLFloat32 canardDSDLGetF16(const uint8_t* const buf, const size_t buf_size, const size_t off_bit);
CanardDSDLFloat32 canardDSDLGetF32(const uint8_t* const buf, const size_t buf_size, const size_t off_bit);
CanardDSDLFloat64 canardDSDLGetF64(const uint8_t* const buf, const size_t buf_size, const size_t off_bit);

#ifdef __cplusplus
}
#endif
#endif  // CANARD_DSDL_H_INCLUDED
