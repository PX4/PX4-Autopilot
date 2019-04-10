/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file drv_tone_alarm.h
 *
 * Driver for the PX4 audio alarm port, /dev/tone_alarm.
 *
 * The tone_alarm driver supports a set of predefined "alarm"
 * patterns and one user-supplied pattern.  Patterns are ordered by
 * priority, with a higher-priority pattern interrupting any
 * lower-priority pattern that might be playing.
 *
 * The TONE_SET_ALARM ioctl can be used to select a predefined
 * alarm pattern, from 1 - <TBD>.  Selecting pattern zero silences
 * the alarm.
 *
 * To supply a custom pattern, write an array of 1 - <TBD> tone_note
 * structures to /dev/tone_alarm.  The custom pattern has a priority
 * of zero.
 *
 * Patterns will normally play once and then silence (if a pattern
 * was overridden it will not resume).  A pattern may be made to
 * repeat by inserting a note with the duration set to
 * DURATION_REPEAT.  This pattern will loop until either a
 * higher-priority pattern is started or pattern zero is requested
 * via the ioctl.
 */

#pragma once

#include <lib/tunes/tune_definition.h>
#include <uORB/topics/tune_control.h>

#define CBRK_BUZZER_KEY 782097
#define CBRK_OFF        0
#define CBRK_ON         1
#define CBRK_UNINIT     2

#define _TONE_ALARM_BASE        0x7400
#define TONE_SET_ALARM          _PX4_IOC(_TONE_ALARM_BASE, 1)
#define TONE_ALARM0_DEVICE_PATH "/dev/tone_alarm0"

// TODO: remove this once the tone_alarm driver is changed to the new tunelib
enum {
	TONE_STOP_TUNE = 0,
	TONE_STARTUP_TUNE,
	TONE_ERROR_TUNE,
	TONE_NOTIFY_POSITIVE_TUNE,
	TONE_NOTIFY_NEUTRAL_TUNE,
	TONE_NOTIFY_NEGATIVE_TUNE,
	TONE_ARMING_WARNING_TUNE,
	TONE_BATTERY_WARNING_SLOW_TUNE,
	TONE_BATTERY_WARNING_FAST_TUNE,
	TONE_GPS_WARNING_TUNE,
	TONE_ARMING_FAILURE_TUNE,
	TONE_PARACHUTE_RELEASE_TUNE,
	TONE_EKF_WARNING_TUNE,
	TONE_BARO_WARNING_TUNE,
	TONE_SINGLE_BEEP_TUNE,
	TONE_HOME_SET,
	TONE_NUMBER_OF_TUNES
};
