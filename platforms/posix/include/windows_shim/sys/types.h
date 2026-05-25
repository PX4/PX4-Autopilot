/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
/**
 * @file sys/types.h
 *
 * MinGW's <sys/types.h> omits the BSD-style unprefixed size aliases
 * (uint, ushort, ulong) that glibc exports under _GNU_SOURCE. PX4 code
 * uses `uint` directly in a couple of places - forward to the real
 * header and add the aliases.
 */
#pragma once

#if defined(_MSC_VER)
#if defined(__has_include)
#  if __has_include(<../ucrt/sys/types.h>)
#    include <../ucrt/sys/types.h>
#  endif
#endif
#include <stdint.h>
#include <stddef.h>
#include <basetsd.h>
#ifndef _PID_T_
/** @brief Process id type for native MSVC builds. */
typedef int pid_t;
#define _PID_T_
#endif
#ifndef _MODE_T_
/** @brief POSIX file mode bitmask type for native MSVC builds. */
typedef int mode_t;
#define _MODE_T_
#endif
#ifndef _OFF_T_DEFINED
/** @brief File offset type for native MSVC builds. */
typedef long off_t;
#define _OFF_T_DEFINED
#endif
#ifndef _SSIZE_T_DEFINED
/** @brief Signed size type matching Windows SSIZE_T. */
typedef SSIZE_T ssize_t;
#define _SSIZE_T_DEFINED
#endif
#ifndef _USECONDS_T_DEFINED
/** @brief Microsecond interval type used by usleep(). */
typedef unsigned int useconds_t;
#define _USECONDS_T_DEFINED
#endif
#else
#include_next <sys/types.h>
#endif

#ifdef _WIN32
#ifndef _PX4_SYS_TYPES_ALIASES_DEFINED
#define _PX4_SYS_TYPES_ALIASES_DEFINED
/** @name BSD/GNU scalar aliases
 *
 * glibc exposes these names in PX4's POSIX build modes. Windows CRT headers do
 * not, so the shim defines them once for shared code that uses the shorter
 * spellings.
 *
 * @{
 */
typedef unsigned char  u_char;
typedef unsigned short u_short;
typedef unsigned int   u_int;
typedef unsigned long  u_long;
typedef unsigned int   uint;
typedef unsigned short ushort;
typedef unsigned long  ulong;
typedef long long      quad_t;
typedef unsigned long long u_quad_t;
/** @} */
#endif
#endif /* _WIN32 */
