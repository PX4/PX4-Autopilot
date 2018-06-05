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
 * @file px4_includes.h
 *
 * Includes headers depending on the build target
 */

#pragma once

#if defined(__PX4_ROS)
/*
 * Building for running within the ROS environment
 */

#elif defined(__PX4_NUTTX)
/*
 * Building for NuttX
 */
#include <nuttx/config.h>
#include <uORB/uORB.h>

#include <systemlib/err.h>
#include <parameters/param.h>
#include <systemlib/systemlib.h>

#elif defined(__PX4_POSIX) && !defined(__PX4_QURT)
/*
 * Building for Posix
 */
#include <string.h>
#include <assert.h>
#include <uORB/uORB.h>

#define ASSERT(x) assert(x)

#include <systemlib/err.h>
#include <parameters/param.h>
#include <systemlib/systemlib.h>

#elif defined(__PX4_QURT)
/*
 * Building for QuRT
 */
#include <string.h>
#include <assert.h>
#include <uORB/uORB.h>

#define ASSERT(x) assert(x)

#include <systemlib/err.h>
#include <parameters/param.h>
#include <systemlib/systemlib.h>

#else
#error "No target platform defined"
#endif
