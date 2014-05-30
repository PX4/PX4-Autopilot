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
 * @file drv_io_expander.h
 *
 * IO expander device API
 */

#pragma once

#include <stdint.h>
#include <sys/ioctl.h>

/*
 * ioctl() definitions
 */

#define _IOXIOCBASE		(0x2800)
#define _IOXIOC(_n)		(_IOC(_IOXIOCBASE, _n))

/** set a bitmask (non-blocking) */
#define IOX_SET_MASK		_IOXIOC(1)

/** get a bitmask (blocking) */
#define IOX_GET_MASK		_IOXIOC(2)

/** set device mode (non-blocking) */
#define IOX_SET_MODE		_IOXIOC(3)

/** set constant values (non-blocking) */
#define IOX_SET_VALUE		_IOXIOC(4)

/* ... to IOX_SET_VALUE + 8 */

/* enum passed to RGBLED_SET_MODE ioctl()*/
enum IOX_MODE {
	IOX_MODE_OFF,
	IOX_MODE_ON,
	IOX_MODE_TEST_OUT
};
