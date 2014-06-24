/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 */

#ifndef RC_CHANNELS_H_
#define RC_CHANNELS_H_

#include <stdint.h>
#include "../uORB.h"

/**
 * The number of RC channel inputs supported.
 * Current (Q4/2013) radios support up to 18 channels,
 * leaving at a sane value of 16.
 * This number can be greater then number of RC channels,
 * because single RC channel can be mapped to multiple
 * functions, e.g. for various mode switches.
 */
#define RC_CHANNELS_MAPPED_MAX   16

/**
 * This defines the mapping of the RC functions.
 * The value assigned to the specific function corresponds to the entry of
 * the channel array chan[].
 */
enum RC_CHANNELS_FUNCTION {
	THROTTLE = 0,
	ROLL     = 1,
	PITCH    = 2,
	YAW      = 3,
	MODE = 4,
	RETURN = 5,
	POSCTL = 6,
	LOITER = 7,
	OFFBOARD_MODE = 8,
	ACRO    = 9,
	FLAPS   = 10,
	AUX_1   = 11,
	AUX_2   = 12,
	AUX_3   = 13,
	AUX_4   = 14,
	AUX_5   = 15,
	RC_CHANNELS_FUNCTION_MAX /**< indicates the number of functions. There can be more functions than RC channels. */
};

/**
 * @addtogroup topics
 * @{
 */

struct rc_channels_s {

	uint64_t timestamp;                 /**< In microseconds since boot time. */
	uint64_t timestamp_last_valid;      /**< timestamp of last valid RC signal. */
	struct {
		float scaled;                     /**< Scaled to -1..1 (throttle: 0..1) */
	} chan[RC_CHANNELS_MAPPED_MAX];
	uint8_t chan_count;                 /**< number of valid channels */

	/*String array to store the names of the functions*/
	char function_name[RC_CHANNELS_FUNCTION_MAX][20];
	int8_t function[RC_CHANNELS_FUNCTION_MAX];
	uint8_t rssi;                       /**< Overall receive signal strength */
	bool signal_lost;		/**< control signal lost, should be checked together with topic timeout */
}; /**< radio control channels. */

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(rc_channels);

#endif
