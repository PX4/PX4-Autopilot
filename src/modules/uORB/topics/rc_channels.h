/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file rc_channels.h
 * Definition of the rc_channels uORB topic.
 *
 * @deprecated DO NOT USE FOR NEW CODE
 */

#ifndef RC_CHANNELS_H_
#define RC_CHANNELS_H_

#include <stdint.h>
#include "../uORB.h"

/**
 * This defines the mapping of the RC functions.
 * The value assigned to the specific function corresponds to the entry of
 * the channel array channels[].
 */
enum RC_CHANNELS_FUNCTION {
	THROTTLE = 0,
	ROLL,
	PITCH,
	YAW,
	MODE,
	RETURN,
	POSCTL,
	LOITER,
	OFFBOARD,
	ACRO,
	FLAPS,
	AUX_1,
	AUX_2,
	AUX_3,
	AUX_4,
	AUX_5
};

// MAXIMUM FUNCTIONS IS != MAXIMUM RC INPUT CHANNELS

#define RC_CHANNELS_FUNCTION_MAX 18

/**
 * @addtogroup topics
 * @{
 */
struct rc_channels_s {
	uint64_t timestamp;									/**< Timestamp in microseconds since boot time */
	uint64_t timestamp_last_valid;						/**< Timestamp of last valid RC signal */
	float channels[RC_CHANNELS_FUNCTION_MAX];			/**< Scaled to -1..1 (throttle: 0..1) */
	uint8_t channel_count;								/**< Number of valid channels */
	int8_t function[RC_CHANNELS_FUNCTION_MAX];			/**< Functions mapping */
	uint8_t rssi;										/**< Receive signal strength index */
	bool signal_lost;									/**< Control signal lost, should be checked together with topic timeout */
}; /**< radio control channels. */

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(rc_channels);

#endif
