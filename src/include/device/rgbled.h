/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
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
 * @file rgbled.h
 *
 * RGB led device API
 */

#pragma once

#include <stdint.h>
#include <sys/ioctl.h>

/* more devices will be 1, 2, etc */
#define RGBLED_DEVICE_PATH "/dev/rgbled0"

/*
 * ioctl() definitions
 */

#define _RGBLEDIOCBASE		(0x2900)
#define _RGBLEDIOC(_n)		(_IOC(_RGBLEDIOCBASE, _n))

/** play the named script in *(char *)arg, repeating forever */
#define RGBLED_PLAY_SCRIPT_NAMED	_RGBLEDIOC(1)

/** play the numbered script in (arg), repeating forever */
#define RGBLED_PLAY_SCRIPT		_RGBLEDIOC(2)

/** 
 * Set the user script; (arg) is a pointer to an array of script lines,
 * where each line is an array of four bytes giving <duration>, <command>, arg[0-2]
 *
 * The script is terminated by a zero command.
 */
#define RGBLED_SET_USER_SCRIPT		_RGBLEDIOC(3)

/** set constant RGB values */
#define RGBLED_SET			_RGBLEDIOC(4)

/* 
  structure passed to RGBLED_SET ioctl()
  Note that the driver scales the brightness to 0 to 255, regardless
  of the hardware scaling
 */
struct RGBLEDSet {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
};
