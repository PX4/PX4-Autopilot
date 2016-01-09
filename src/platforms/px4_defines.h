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

#include <px4_log.h>
#include <math.h>

/* Get the name of the default value fiven the param name */
#define PX4_PARAM_DEFAULT_VALUE_NAME(_name) PARAM_##_name##_DEFAULT

/* Shortcuts to define parameters when the default value is defined according to PX4_PARAM_DEFAULT_VALUE_NAME */
#define PX4_PARAM_DEFINE_INT32(_name) PARAM_DEFINE_INT32(_name, PX4_PARAM_DEFAULT_VALUE_NAME(_name))
#define PX4_PARAM_DEFINE_FLOAT(_name) PARAM_DEFINE_FLOAT(_name, PX4_PARAM_DEFAULT_VALUE_NAME(_name))

#define PX4_ERROR (-1)
#define PX4_OK 0

#if defined(__PX4_ROS)
/*
 * Building for running within the ROS environment
 */
#define noreturn_function
#ifdef __cplusplus
#include "ros/ros.h"
#endif

/* Main entry point */
#define PX4_MAIN_FUNCTION(_prefix) int main(int argc, char **argv)

/* Get value of parameter by name, which is equal to the handle for ros */
#define PX4_PARAM_GET_BYNAME(_name, _destpt) ros::param::get(_name, *_destpt)

#define PX4_ISFINITE(x) std::isfinite(x)

#elif defined(__PX4_NUTTX) || defined(__PX4_POSIX)
/*
 * Building for NuttX or POSIX
 */
#include <platforms/px4_includes.h>
/* Main entry point */
#define PX4_MAIN_FUNCTION(_prefix) int _prefix##_task_main(int argc, char *argv[])

/* Parameter handle datatype */
#include <systemlib/param/param.h>
typedef param_t px4_param_t;

/* Get value of parameter by name */
#define PX4_PARAM_GET_BYNAME(_name, _destpt) param_get(param_find(_name), _destpt)

#else
#error "No target OS defined"
#endif

/*
 * NuttX Specific defines
 */
#if defined(__PX4_NUTTX)

#define PX4_ROOTFSDIR

/* XXX this is a hack to resolve conflicts with NuttX headers */
#if !defined(__PX4_TESTS)
#define isspace(c) \
	((c) == ' '  || (c) == '\t' || (c) == '\n' || \
	 (c) == '\r' || (c) == '\f' || c== '\v')
#endif

#define _PX4_IOC(x,y) _IOC(x,y)

#define px4_statfs_buf_f_bavail_t int

#define PX4_ISFINITE(x) isfinite(x)

// mode for open with O_CREAT
#define PX4_O_MODE_777 0777
#define PX4_O_MODE_666 0666
#define PX4_O_MODE_600 0600

#ifndef PRIu64
#define PRIu64 "llu"
#endif
#ifndef PRId64
#define PRId64 "lld"
#endif

/*
 * POSIX Specific defines
 */
#elif defined(__PX4_POSIX)

// Flag is meaningless on Linux
#define O_BINARY 0

// mode for open with O_CREAT
#define PX4_O_MODE_777 (S_IRWXU | S_IRWXG | S_IRWXO)
#define PX4_O_MODE_666 (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH )
#define PX4_O_MODE_600 (S_IRUSR | S_IWUSR)


// NuttX _IOC is equivalent to Linux _IO
#define _PX4_IOC(x,y) _IO(x,y)

/* FIXME - Used to satisfy build */
//STM DocID018909 Rev 8 Sect 39.1 (Unique device ID Register)
#define UNIQUE_ID       0x1FFF7A10
#define STM32_SYSMEM_UID "SIMULATIONID"

/* FIXME - Used to satisfy build */
#define getreg32(a)    (*(volatile uint32_t *)(a))

#ifdef __PX4_QURT
#define PX4_TICKS_PER_SEC 1000L
#else
__BEGIN_DECLS
extern long PX4_TICKS_PER_SEC;
__END_DECLS
#endif

#define USEC_PER_TICK (1000000UL/PX4_TICKS_PER_SEC)
#define USEC2TICK(x) (((x)+(USEC_PER_TICK/2))/USEC_PER_TICK)

#define px4_statfs_buf_f_bavail_t unsigned long

#if defined(__PX4_QURT)
#define PX4_ROOTFSDIR
#else
#define PX4_ROOTFSDIR "rootfs"
#endif

#endif


/*
 * Defines for ROS and Linux
 */
#if defined(__PX4_ROS) || defined(__PX4_POSIX)
#define OK 0
#define ERROR -1

#define MAX_RAND 32767

#if defined(__PX4_QURT)
#include "dspal_math.h"
__BEGIN_DECLS
#include <math.h>
__END_DECLS
#else
#include <math.h>
#endif

/* Float defines of the standard double length constants  */
#define M_E_F			(float)M_E
#define M_LOG2E_F		(float)M_LOG2E
#define M_LOG10E_F		(float)M_LOG10E
#define M_LN2_F			(float)M_LN2
#define M_LN10_F		(float)M_LN10
#define M_PI_F			(float)M_PI
#define M_TWOPI_F       	(M_PI_F * 2.0f)
#define M_PI_2_F		(float)M_PI_2
#define M_PI_4_F		(float)M_PI_4
#define M_3PI_4_F		(float)2.3561944901923448370E0f
#define M_SQRTPI_F      	(float)1.77245385090551602792981f
#define M_1_PI_F		(float)M_1_PI
#define M_2_PI_F		(float)M_2_PI
#define M_2_SQRTPI_F		1.12837916709551257390f
#define M_DEG_TO_RAD_F 		0.01745329251994f
#define M_RAD_TO_DEG_F 		57.2957795130823f
#define M_SQRT2_F		(float)M_SQRT2
#define M_SQRT1_2_F		(float)M_SQRT1_2
#define M_LN2LO_F       	1.9082149292705877000E-10f
#define M_LN2HI_F       	6.9314718036912381649E-1f
#define M_SQRT3_F		1.73205080756887719000f
#define M_IVLN10_F      	0.43429448190325182765f /* 1 / log(10) */
#define M_LOG2_E_F      	_M_LN2_F
#define M_INVLN2_F      	1.4426950408889633870E0f/* 1 / log(2)  */
#define M_DEG_TO_RAD 		0.01745329251994
#define M_RAD_TO_DEG 		57.2957795130823

#ifndef __PX4_QURT

#if defined(__cplusplus)
#include <cmath>
#define PX4_ISFINITE(x) std::isfinite(x)
#else
#define PX4_ISFINITE(x) isfinite(x)
#endif
#endif

#endif

#if defined(__PX4_QURT)

#define PX4_ROOTFSDIR
#define DEFAULT_PARAM_FILE "/fs/eeprom/parameters"

#define SIOCDEVPRIVATE 999999

// Missing math.h defines
#define PX4_ISFINITE(x) __builtin_isfinite(x)

#endif

/*
 *Defines for all platforms
 */

/* wrapper for 2d matrices */
#define PX4_ARRAY2D(_array, _ncols, _x, _y) (_array[_x * _ncols + _y])

/* wrapper for rotation matrices stored in arrays */
#define PX4_R(_array, _x, _y) PX4_ARRAY2D(_array, 3, _x, _y)
