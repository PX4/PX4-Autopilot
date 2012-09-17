/****************************************************************************
 * arch/rgmp/include/stdint.h
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_RGMP_INCLUDE_STDINTL_H
#define __ARCH_RGMP_INCLUDE_STDINTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <arch/types.h>
#include <limits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Limits of exact-width integer types */

#define INT8_MIN            0x80
#define INT8_MAX            0x7f
#define UINT8_MAX           0xff

#define INT16_MIN           0x8000
#define INT16_MAX           0x7fff
#define UINT16_MAX          0xffff

#ifdef __INT64_DEFINED
#  define INT24_MIN         0x800000
#  define INT24_MAX         0x7fffff
#  define UINT24_MAX        0xffffff
#endif

#define INT32_MIN           0x80000000
#define INT32_MAX           0x7fffffff
#define UINT32_MAX          0xffffffff

#ifdef __INT64_DEFINED
#  define INT64_MIN         0x8000000000000000
#  define INT64_MAX         0x7fffffffffffffff
#  define UINT64_MAX        0xffffffffffffffff
#endif

/* Limits of minimum-width integer types */

#define INT8_LEASTN_MIN     0x80
#define INT8_LEASTN_MAX     0x7f
#define UINT8_LEASTN_MAX    0xff

#define INT16_LEASTN_MIN    0x8000
#define INT16_LEASTN_MAX    0x7fff
#define UINT16_LEASTN_MAX   0xffff

#ifdef __INT64_DEFINED
#  define INT24_LEASTN_MIN  0x800000
#  define INT24_LEASTN_MAX  0x7fffff
#  define UINT24_LEASTN_MAX 0xffffff
#endif

#define INT32_LEASTN_MIN    0x80000000
#define INT32_LEASTN_MAX    0x7fffffff
#define UINT32_LEASTN_MAX   0xffffffff

#ifdef __INT64_DEFINED
#  define INT64_LEASTN_MIN  0x8000000000000000
#  define INT64_LEASTN_MAX  0x7fffffffffffffff
#  define UINT64_LEASTN_MAX 0xffffffffffffffff
#endif

/* Limits of fastest minimum-width integer types */

#define INT8_FASTN_MIN      0x80
#define INT8_FASTN_MAX      0x7f
#define UINT8_FASTN_MAX     0xff

#define INT16_FASTN_MIN     0x8000
#define INT16_FASTN_MAX     0x7fff
#define UINT16_FASTN_MAX    0xffff

#ifdef __INT64_DEFINED
#  define INT24_FASTN_MIN   0x800000
#  define INT24_FASTN_MAX   0x7fffff
#  define UINT24_FASTN_MAX  0xffffff
#endif

#define INT32_FASTN_MIN     0x80000000
#define INT32_FASTN_MAX     0x7fffffff
#define UINT32_FASTN_MAX    0xffffffff

#ifdef __INT64_DEFINED
#  define INT64_FASTN_MIN   0x8000000000000000
#  define INT64_FASTN_MAX   0x7fffffffffffffff
#  define UINT64_FASTN_MAX  0xffffffffffffffff
#endif

/* Limits of integer types capable of holding object pointers */

#define INTPTR_MIN          PTR_MIN
#define INTPTR_MAX          PTR_MIN
#define UINTPTR_MAX         UPTR_MAX

/* Limits of greatest-width integer types */

#ifdef __INT64_DEFINED
#  define INTMAX_MIN        INT64_MIN
#  define INTMAX_MAX        INT64_MAX

#  define UINTMAX_MIN       UINT64_MIN
#  define UINTMAX_MAX       UINT64_MAX
#else
#  define INTMAX_MIN        INT32_MIN
#  define INTMAX_MAX        INT32_MAX

#  define UINTMAX_MIN       UINT32_MIN
#  define UINTMAX_MAX       UINT32_MAX
#endif

/* Macros for minimum-width integer constant expressions */

#if 0 /* REVISIT: Depends on architecture specific implementation */
#define INT8_C(x) x
#define INT16_C(x) x
#define INT32_C(x) x ## L
#define INT64_C(x) x ## LL

#define UINT8_C(x) x
#define UINT16_C(x) x
#define UINT32_C(x) x ## UL
#define UINT64_C(x) x ## ULL
#endif

/* Macros for greatest-width integer constant expressions

#ifdef CONFIG_HAVE_LONG_LONG
#  define INTMAX_C(x) x ## LL
#  define UINTMAX_C(x) x ## ULL
#else
#  define INTMAX_C(x) x ## L
#  define UINTMAX_C(x) x ## UL
#endif

/* Limits of Other Integer Types */

#if 0
#  define PTRDIFF_MIN
#  define PTRDIFF_MAX
#endif

#ifdef CONFIG_SMALL_MEMORY
#  define SIZE_MAX 0xffff
#else
#  define SIZE_MAX 0xffffffff
#endif

#if 0
#  define WCHAR_MIN
#  define WCHAR_MAX

#  define WINT_MIN
#  define WINT_MAX
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Exact-width integer types.  NOTE that these types are defined in
 * architecture-specific logic with leading underscore character. This file
 * typedef's these to the final name without the underscore character.  This
 * roundabout way of doings things allows the stdint.h to be removed from the
 * include/ directory in the event that the user prefers to use the definitions
 * provided by their toolchain header files.
 */

#include <rgmp/types.h>

/* Minimum-width integer types */

typedef _int8_t      int_least8_t;
typedef _uint8_t     uint_least8_t;

typedef _int16_t     int_least16_t;
typedef _uint16_t    uint_least16_t;

#ifdef __INT24_DEFINED
typedef _int24_t     int_least24_t;
typedef _uint24_t    uint_least24_t;
#else
typedef _int32_t     int_least24_t;
typedef _uint32_t    uint_least24_t;
#endif

typedef _int32_t     int_least32_t;
typedef _uint32_t    uint_least32_t;

#ifdef __INT64_DEFINED
typedef _int64_t     int_least64_t;
typedef _uint64_t    uint_least64_t;
#endif

/* Fastest minimum-width integer types */

typedef _int8_t      int_fast8_t;
typedef _uint8_t     uint_fast8_t;

typedef int          int_fast16_t;
typedef unsigned int uint_fast16_t;

#ifdef __INT24_DEFINED
typedef _int24_t     int_fast24_t;
typedef _uint24_t    uint_fast24_t;
#else
typedef _int32_t     int_fast24_t;
typedef _uint32_t    uint_fast24_t;
#endif

typedef _int32_t     int_fast32_t;
typedef _uint32_t    uint_fast32_t;

#ifdef __INT64_DEFINED
typedef _int64_t     int_fast64_t;
typedef _uint64_t    uint_fast64_t;
#endif

/* Integer types capable of holding object pointers */

#ifndef CONFIG_ARCH_RGMP
typedef _intptr_t    intptr_t;
typedef _uintptr_t   uintptr_t;
#endif

/* Greatest-width integer types */

#ifdef __INT64_DEFINED
typedef _int64_t     intmax_t;
typedef _uint64_t    uintmax_t;
#else
typedef _int32_t     intmax_t;
typedef _uint32_t    uintmax_t;
#endif

#endif /* __ARCH_RGMP_INCLUDE_STDINTL_H */
