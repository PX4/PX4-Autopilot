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
 * @file drv_rgbled.h
 *
 * RGB led device API
 */

#pragma once

#include <stdint.h>
#include <sys/ioctl.h>

/* more devices will be 1, 2, etc */
#define RGBLED0_DEVICE_PATH "/dev/rgbled0"

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
#define RGBLED_SET_RGB			_RGBLEDIOC(4)

/** set color */
#define RGBLED_SET_COLOR		_RGBLEDIOC(5)

/** set blink speed */
#define RGBLED_SET_MODE			_RGBLEDIOC(6)

/** set pattern */
#define RGBLED_SET_PATTERN		_RGBLEDIOC(7)


/* 
  structure passed to RGBLED_SET_RGB ioctl()
  Note that the driver scales the brightness to 0 to 255, regardless
  of the hardware scaling
 */
typedef struct {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} rgbled_rgbset_t;

/* enum passed to RGBLED_SET_COLOR ioctl()*/
typedef enum {
	RGBLED_COLOR_OFF,
	RGBLED_COLOR_RED,
	RGBLED_COLOR_YELLOW,
	RGBLED_COLOR_PURPLE,
	RGBLED_COLOR_GREEN,
	RGBLED_COLOR_BLUE,
	RGBLED_COLOR_WHITE,
	RGBLED_COLOR_AMBER,
	RGBLED_COLOR_DIM_RED,
	RGBLED_COLOR_DIM_YELLOW,
	RGBLED_COLOR_DIM_PURPLE,
	RGBLED_COLOR_DIM_GREEN,
	RGBLED_COLOR_DIM_BLUE,
	RGBLED_COLOR_DIM_WHITE,
	RGBLED_COLOR_DIM_AMBER
} rgbled_color_t;

/* enum passed to RGBLED_SET_MODE ioctl()*/
typedef enum {
	RGBLED_MODE_OFF,
	RGBLED_MODE_ON,
	RGBLED_MODE_BLINK_SLOW,
	RGBLED_MODE_BLINK_NORMAL,
	RGBLED_MODE_BLINK_FAST,
	RGBLED_MODE_BREATHE,
	RGBLED_MODE_PATTERN
} rgbled_mode_t;

#define RGBLED_PATTERN_LENGTH 20

typedef struct {
	rgbled_color_t color[RGBLED_PATTERN_LENGTH];
	unsigned duration[RGBLED_PATTERN_LENGTH];
} rgbled_pattern_t;
