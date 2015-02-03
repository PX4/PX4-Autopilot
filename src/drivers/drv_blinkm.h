/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file drv_blinkm.h
 *
 * BlinkM driver API
 *
 * This could probably become a more generalised API for multi-colour LED
 * driver systems, or be merged with the generic LED driver.
 */

#pragma once

#include <stdint.h>
#include <sys/ioctl.h>

#define BLINKM0_DEVICE_PATH	"/dev/blinkm0"

/*
 * ioctl() definitions
 */

#define _BLINKMIOCBASE		(0x2900)
#define _BLINKMIOC(_n)		(_IOC(_BLINKMIOCBASE, _n))

/** play the named script in *(char *)arg, repeating forever */
#define BLINKM_PLAY_SCRIPT_NAMED	_BLINKMIOC(1)

/** play the numbered script in (arg), repeating forever */
#define BLINKM_PLAY_SCRIPT		_BLINKMIOC(2)

/** 
 * Set the user script; (arg) is a pointer to an array of script lines,
 * where each line is an array of four bytes giving <duration>, <command>, arg[0-2]
 *
 * The script is terminated by a zero command.
 */
#define BLINKM_SET_USER_SCRIPT		_BLINKMIOC(3)
