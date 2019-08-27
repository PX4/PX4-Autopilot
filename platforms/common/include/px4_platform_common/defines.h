/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file px4_defines.h
 *
 * Generally used magic defines
 */

#pragma once

#include <px4_platform_common/log.h>

#if defined(__PX4_NUTTX) && !defined(CONFIG_ARCH_MATH_H)
#error CONFIG_ARCH_MATH_H is required to use math definitions and functions
#endif

/****************************************************************************
 * Defines for all platforms.
 ****************************************************************************/

/* Get the name of the default value fiven the param name */
#define PX4_PARAM_DEFAULT_VALUE_NAME(_name) PARAM_##_name##_DEFAULT

/* Shortcuts to define parameters when the default value is defined according to PX4_PARAM_DEFAULT_VALUE_NAME */
#define PX4_PARAM_DEFINE_INT32(_name) PARAM_DEFINE_INT32(_name, PX4_PARAM_DEFAULT_VALUE_NAME(_name))
#define PX4_PARAM_DEFINE_FLOAT(_name) PARAM_DEFINE_FLOAT(_name, PX4_PARAM_DEFAULT_VALUE_NAME(_name))

#define PX4_ERROR (-1)
#define PX4_OK 0

/* Wrapper for 2d matrices */
#define PX4_ARRAY2D(_array, _ncols, _x, _y) (_array[_x * _ncols + _y])

/* Wrapper for rotation matrices stored in arrays */
#define PX4_R(_array, _x, _y) PX4_ARRAY2D(_array, 3, _x, _y)

/* Define PX4_ISFINITE */
#ifdef __cplusplus
constexpr bool PX4_ISFINITE(float x) { return __builtin_isfinite(x); }
constexpr bool PX4_ISFINITE(double x) { return __builtin_isfinite(x); }
#endif /* __cplusplus */

#if defined(__PX4_NUTTX) || defined(__PX4_POSIX)
/****************************************************************************
 * Building for NuttX or POSIX.
 ****************************************************************************/

/* Main entry point */
#define PX4_MAIN_FUNCTION(_prefix) int _prefix##_task_main(int argc, char *argv[])

/* Parameter handle datatype */
#include <parameters/param.h>
typedef param_t px4_param_t;

/* Get value of parameter by name */
#define PX4_PARAM_GET_BYNAME(_name, _destpt) param_get(param_find(_name), _destpt)

#else // defined(__PX4_NUTTX) || defined(__PX4_POSIX)
/****************************************************************************/
#error "No target OS defined"
#endif

#if defined(__PX4_NUTTX)
/****************************************************************************
 * NuttX specific defines.
 ****************************************************************************/

#define PX4_ROOTFSDIR ""
#define PX4_STORAGEDIR PX4_ROOTFSDIR "/fs/microsd"
#define _PX4_IOC(x,y) _IOC(x,y)

// mode for open with O_CREAT
#define PX4_O_MODE_777 0777
#define PX4_O_MODE_666 0666
#define PX4_O_MODE_600 0600

#ifndef PRIu64
#  define PRIu64 "llu"
#endif
#ifndef PRId64
#  define PRId64 "lld"
#endif

#ifndef offsetof
#  define offsetof(TYPE, MEMBER) __builtin_offsetof (TYPE, MEMBER)
#endif

#elif defined(__PX4_POSIX)
/****************************************************************************
 * POSIX Specific defines
 ****************************************************************************/

// Flag is meaningless on Linux
#ifndef O_BINARY
#define O_BINARY 0
#endif

// mode for open with O_CREAT
#define PX4_O_MODE_777 (S_IRWXU | S_IRWXG | S_IRWXO)
#define PX4_O_MODE_666 (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH )
#define PX4_O_MODE_600 (S_IRUSR | S_IWUSR)

// NuttX _IOC is equivalent to Linux _IO
#define _PX4_IOC(x,y) _IO(x,y)

/* FIXME - Used to satisfy build */
#define getreg32(a)    (*(volatile uint32_t *)(a))

#define USEC_PER_TICK (1000000/PX4_TICKS_PER_SEC)
#define USEC2TICK(x) (((x)+(USEC_PER_TICK/2))/USEC_PER_TICK)

#ifdef __PX4_QURT

// QURT specific
#  include "dspal_math.h"
#  define PX4_ROOTFSDIR "."
#  define PX4_TICKS_PER_SEC 1000L
#  define SIOCDEVPRIVATE 999999

#else // __PX4_QURT

// All POSIX except QURT.

__BEGIN_DECLS
extern long PX4_TICKS_PER_SEC;
__END_DECLS

#  if defined(__PX4_POSIX_EAGLE) || defined(__PX4_POSIX_EXCELSIOR)
#    define PX4_ROOTFSDIR "/home/linaro"
#  elif defined(__PX4_POSIX_BEBOP)
#    define PX4_ROOTFSDIR "/data/ftp/internal_000/px4"
#  else
#    define PX4_ROOTFSDIR "."
#  endif

#endif // __PX4_QURT

#define PX4_STORAGEDIR PX4_ROOTFSDIR
#endif // __PX4_POSIX

#if defined(__PX4_POSIX)
/****************************************************************************
 * Defines for POSIX and ROS
 ****************************************************************************/

#define OK 0
#define ERROR -1
#define MAX_RAND 32767

/* Math macro's for float literals. Do not use M_PI et al as they aren't
 * defined (neither C nor the C++ standard define math constants) */
#define M_E_F			2.71828183f
#define M_LOG2E_F		1.44269504f
#define M_LOG10E_F		0.43429448f
#define M_LN2_F			0.69314718f
#define M_LN10_F		2.30258509f
#define M_PI_F			3.14159265f
#define M_TWOPI_F		6.28318531f
#define M_PI_2_F		1.57079632f
#define M_PI_4_F		0.78539816f
#define M_3PI_4_F		2.35619449f
#define M_SQRTPI_F		1.77245385f
#define M_1_PI_F		0.31830989f
#define M_2_PI_F		0.63661977f
#define M_2_SQRTPI_F		1.12837917f
#define M_DEG_TO_RAD_F		0.0174532925f
#define M_RAD_TO_DEG_F		57.2957795f
#define M_SQRT2_F		1.41421356f
#define M_SQRT1_2_F		0.70710678f
#define M_LN2LO_F		1.90821484E-10f
#define M_LN2HI_F		0.69314718f
#define M_SQRT3_F		1.73205081f
#define M_IVLN10_F		0.43429448f	// 1 / log(10)
#define M_LOG2_E_F		0.69314718f
#define M_INVLN2_F		1.44269504f	// 1 / log(2)

#define M_DEG_TO_RAD 		0.017453292519943295
#define M_RAD_TO_DEG 		57.295779513082323

#endif // defined(__PX4_POSIX)
