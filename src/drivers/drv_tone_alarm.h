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

#ifndef DRV_TONE_ALARM_H_
#define DRV_TONE_ALARM_H_

#include <sys/ioctl.h>

#define TONEALARM0_DEVICE_PATH "/dev/tone_alarm0"

#define _TONE_ALARM_BASE	0x7400
#define TONE_SET_ALARM		_IOC(_TONE_ALARM_BASE, 1)

/* structure describing one note in a tone pattern */
struct tone_note {
	uint8_t		pitch;
	uint8_t		duration;	/* duration in multiples of 10ms */
#define DURATION_END		0	/* ends the pattern */
#define DURATION_REPEAT		255	/* resets the note counter to zero */
};

enum tone_pitch {
	TONE_NOTE_E4,   /* E4 */
	TONE_NOTE_F4,   /* F4 */
	TONE_NOTE_F4S,  /* F#4/Gb4 */
	TONE_NOTE_G4,   /* G4 */
	TONE_NOTE_G4S,  /* G#4/Ab4 */
	TONE_NOTE_A4,   /* A4 */
	TONE_NOTE_A4S,  /* A#4/Bb4 */
	TONE_NOTE_B4,   /* B4 */
	TONE_NOTE_C5,   /* C5 */
	TONE_NOTE_C5S,  /* C#5/Db5 */
	TONE_NOTE_D5,   /* D5 */
	TONE_NOTE_D5S,  /* D#5/Eb5 */
	TONE_NOTE_E5,   /* E5 */
	TONE_NOTE_F5,   /* F5 */
	TONE_NOTE_F5S,  /* F#5/Gb5 */
	TONE_NOTE_G5,   /* G5 */
	TONE_NOTE_G5S,  /* G#5/Ab5 */
	TONE_NOTE_A5,   /* A5 */
	TONE_NOTE_A5S,  /* A#5/Bb5 */
	TONE_NOTE_B5,   /* B5 */
	TONE_NOTE_C6,   /* C6 */
	TONE_NOTE_C6S,  /* C#6/Db6 */
	TONE_NOTE_D6,   /* D6 */
	TONE_NOTE_D6S,  /* D#6/Eb6 */
	TONE_NOTE_E6,   /* E6 */
	TONE_NOTE_F6,   /* F6 */
	TONE_NOTE_F6S,  /* F#6/Gb6 */
	TONE_NOTE_G6,   /* G6 */
	TONE_NOTE_G6S,  /* G#6/Ab6 */
	TONE_NOTE_A6,   /* A6 */
	TONE_NOTE_A6S,  /* A#6/Bb6 */
	TONE_NOTE_B6,   /* B6 */
	TONE_NOTE_C7,   /* C7 */
	TONE_NOTE_C7S,  /* C#7/Db7 */
	TONE_NOTE_D7,   /* D7 */
	TONE_NOTE_D7S,  /* D#7/Eb7 */
	TONE_NOTE_E7,   /* E7 */
	TONE_NOTE_F7,   /* F7 */
	TONE_NOTE_F7S,  /* F#7/Gb7 */
	TONE_NOTE_G7,   /* G7 */
	TONE_NOTE_G7S,  /* G#7/Ab7 */
	TONE_NOTE_A7,   /* A7 */
	TONE_NOTE_A7S,  /* A#7/Bb7 */
	TONE_NOTE_B7,   /* B7 */
	TONE_NOTE_C8,   /* C8 */
	TONE_NOTE_C8S,  /* C#8/Db8 */
	TONE_NOTE_D8,   /* D8 */
	TONE_NOTE_D8S,  /* D#8/Eb8 */

	TONE_NOTE_SILENCE,
	TONE_NOTE_MAX
};

enum {
	TONE_STOP_TUNE = 0,
	TONE_STARTUP_TUNE,
	TONE_ERROR_TUNE,
	TONE_NOTIFY_POSITIVE_TUNE,
	TONE_NOTIFY_NEUTRAL_TUNE,
	TONE_NOTIFY_NEGATIVE_TUNE,
	/* Do not include these unused tunes
	TONE_CHARGE_TUNE,
	TONE_DIXIE_TUNE,
	TONE_CUCURACHA_TUNE,
	TONE_YANKEE_TUNE,
	TONE_DAISY_TUNE,
	TONE_WILLIAM_TELL_TUNE, */
	TONE_ARMING_WARNING_TUNE,
	TONE_BATTERY_WARNING_SLOW_TUNE,
	TONE_BATTERY_WARNING_FAST_TUNE,
	TONE_GPS_WARNING_TUNE,
	TONE_ARMING_FAILURE_TUNE,
	TONE_PARACHUTE_RELEASE_TUNE,
	TONE_EKF_WARNING_TUNE,
	TONE_BARO_WARNING_TUNE,
	TONE_NUMBER_OF_TUNES
};

#endif /* DRV_TONE_ALARM_H_ */
