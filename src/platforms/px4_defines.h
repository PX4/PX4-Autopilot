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
/* Get the name of the default value fiven the param name */
#define PX4_PARAM_DEFAULT_VALUE_NAME(_name) PARAM_##_name##_DEFAULT

/* Shortcuts to define parameters when the default value is defined according to PX4_PARAM_DEFAULT_VALUE_NAME */
#define PX4_PARAM_DEFINE_INT32(_name) PARAM_DEFINE_INT32(_name, PX4_PARAM_DEFAULT_VALUE_NAME(_name))
#define PX4_PARAM_DEFINE_FLOAT(_name) PARAM_DEFINE_FLOAT(_name, PX4_PARAM_DEFAULT_VALUE_NAME(_name))


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

/* Print/output wrappers */
#define PX4_WARN ROS_WARN
#define PX4_INFO ROS_INFO

/* Get value of parameter by name, which is equal to the handle for ros */
#define PX4_PARAM_GET_BYNAME(_name, _destpt) ros::param::get(_name, *_destpt)

#define OK 0
#define ERROR -1

//XXX hack to be able to use isfinte from math.h, -D_GLIBCXX_USE_C99_MATH seems not to work
#define isfinite(_value) std::isfinite(_value)

/* Useful constants.  */
#define M_E_F			2.7182818284590452354f
#define M_LOG2E_F		1.4426950408889634074f
#define M_LOG10E_F		0.43429448190325182765f
#define M_LN2_F			_M_LN2_F
#define M_LN10_F		2.30258509299404568402f
#define M_PI_F			3.14159265358979323846f
#define M_TWOPI_F       (M_PI_F * 2.0f)
#define M_PI_2_F		1.57079632679489661923f
#define M_PI_4_F		0.78539816339744830962f
#define M_3PI_4_F		2.3561944901923448370E0f
#define M_SQRTPI_F      1.77245385090551602792981f
#define M_1_PI_F		0.31830988618379067154f
#define M_2_PI_F		0.63661977236758134308f
#define M_2_SQRTPI_F	1.12837916709551257390f
#define M_DEG_TO_RAD_F 	0.01745329251994f
#define M_RAD_TO_DEG_F 	57.2957795130823f
#define M_SQRT2_F		1.41421356237309504880f
#define M_SQRT1_2_F		0.70710678118654752440f
#define M_LN2LO_F       1.9082149292705877000E-10f
#define M_LN2HI_F       6.9314718036912381649E-1f
#define M_SQRT3_F		1.73205080756887719000f
#define M_IVLN10_F      0.43429448190325182765f /* 1 / log(10) */
#define M_LOG2_E_F      _M_LN2_F
#define M_INVLN2_F      1.4426950408889633870E0f  /* 1 / log(2) */
#define M_DEG_TO_RAD 	0.01745329251994
#define M_RAD_TO_DEG 	57.2957795130823

#else
/*
 * Building for NuttX
 */
#include <platforms/px4_includes.h>
/* Main entry point */
#define PX4_MAIN_FUNCTION(_prefix) int _prefix##_task_main(int argc, char *argv[])

/* Print/output wrappers */
#define PX4_WARN warnx
#define PX4_INFO warnx

/* Parameter handle datatype */
#include <systemlib/param/param.h>
typedef param_t px4_param_t;

/* Get value of parameter by name */
#define PX4_PARAM_GET_BYNAME(_name, _destpt) param_get(param_find(_name), _destpt)

/* XXX this is a hack to resolve conflicts with NuttX headers */
#if !defined(__PX4_TESTS)
#define isspace(c) \
	((c) == ' '  || (c) == '\t' || (c) == '\n' || \
	 (c) == '\r' || (c) == '\f' || c== '\v')
#endif

#endif

/* Defines for all platforms */

/* wrapper for 2d matrices */
#define PX4_ARRAY2D(_array, _ncols, _x, _y) (_array[_x * _ncols + _y])

/* wrapper for rotation matrices stored in arrays */
#define PX4_R(_array, _x, _y) PX4_ARRAY2D(_array, 3, _x, _y)
