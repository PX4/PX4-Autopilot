/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name APL nor the names of its contributors may be
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
 * @file ecl.h
 * Adapter / shim layer for system calls needed by ECL
 *
 */
#pragma once

#if defined(__PX4_POSIX) || defined(__PX4_NUTTX)

#include <drivers/drv_hrt.h>
#include <px4_log.h>

#define ecl_absolute_time hrt_absolute_time
#define ecl_elapsed_time hrt_elapsed_time
using ecl_abstime = hrt_abstime;

#if defined(__PX4_NUTTX)
#  define ECL_INFO PX4_DEBUG
#  define ECL_WARN PX4_DEBUG
#  define ECL_ERR  PX4_DEBUG
#else
#  define ECL_INFO PX4_INFO
#  define ECL_WARN PX4_WARN
#  define ECL_ERR  PX4_ERR
#endif

#elif defined(__PAPARAZZI)

#include "std.h"

#define ecl_absolute_time() (0)
#define ecl_elapsed_time(t) (*t * 0UL) // TODO: add simple time functions

using ecl_abstime = uint64_t;

#define ECL_INFO(...)
#define ECL_WARN(...)
#define ECL_ERR(...)

#else

#include <cstdio>
#include <cstdint>

#define ecl_absolute_time() (0)
#define ecl_elapsed_time(t) (*t * 0UL) // TODO: add simple time functions

using ecl_abstime = uint64_t;

#define ECL_INFO(X, ...) printf(X "\n", ##__VA_ARGS__)
#define ECL_WARN(X, ...) fprintf(stderr, X "\n", ##__VA_ARGS__)
#define ECL_ERR(X, ...) fprintf(stderr, X "\n", ##__VA_ARGS__)

#endif /* PX4_POSIX || PX4_NUTTX */

#include <math.h>
#define ISFINITE(x) __builtin_isfinite(x)
