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
 * @file drv_rc_input.h
 *
 * R/C input interface.
 */

#ifndef _DRV_RC_INPUT_H
#define _DRV_RC_INPUT_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_orb_dev.h"

/**
 * Path for the default R/C input device.
 *
 * Note that on systems with more than one R/C input path (e.g.
 * PX4FMU with PX4IO connected) there may be other devices that
 * respond to this protocol.
 *
 * Input data may be obtained by subscribing to the input_rc
 * object, or by poll/reading from the device.
 */
#define RC_INPUT_DEVICE_PATH	"/dev/input_rc"

/**
 * Maximum number of R/C input channels in the system. S.Bus has up to 18 channels.
 */
#define RC_INPUT_MAX_CHANNELS	18

/**
 * Maximum RSSI value
 */
#define RC_INPUT_RSSI_MAX	255

/**
 * @addtogroup topics
 * @{
 */

/**
 * Input signal type, value is a control position from zero to 100
 * percent.
 */
typedef uint16_t		rc_input_t;

enum RC_INPUT_SOURCE {
	RC_INPUT_SOURCE_UNKNOWN = 0,
	RC_INPUT_SOURCE_PX4FMU_PPM,
	RC_INPUT_SOURCE_PX4IO_PPM,
	RC_INPUT_SOURCE_PX4IO_SPEKTRUM,
	RC_INPUT_SOURCE_PX4IO_SBUS,
	RC_INPUT_SOURCE_PX4IO_ST24
};

/**
 * R/C input status structure.
 *
 * Published to input_rc, may also be published to other names depending
 * on the board involved.
 */
struct rc_input_values {
	/** publication time */
	uint64_t		timestamp_publication;

	/** last valid reception time */
	uint64_t		timestamp_last_signal;

	/** number of channels actually being seen */
	uint32_t		channel_count;

	/** receive signal strength indicator (RSSI): < 0: Undefined, 0: no signal, 255: full reception */
	int32_t			rssi;

	/**
	 * explicit failsafe flag: true on TX failure or TX out of range , false otherwise.
	 * Only the true state is reliable, as there are some (PPM) receivers on the market going
	 * into failsafe without telling us explicitly.
	 * */
	bool			rc_failsafe;

	/**
	 * RC receiver connection status: True,if no frame has arrived in the expected time, false otherwise.
	 * True usally means that the receiver has been disconnected, but can also indicate a radio link loss on "stupid" systems.
	 * Will remain false, if a RX with failsafe option continues to transmit frames after a link loss.
	 * */
	bool			rc_lost;

	/**
	 * Number of lost RC frames.
	 * Note: intended purpose: observe the radio link quality if RSSI is not available
	 * This value must not be used to trigger any failsafe-alike funtionality.
	 * */
	uint16_t		rc_lost_frame_count;

	/**
	 * Number of total RC frames.
	 * Note: intended purpose: observe the radio link quality if RSSI is not available
	 * This value must not be used to trigger any failsafe-alike funtionality.
	 * */
	uint16_t		rc_total_frame_count;

	/**
	 * Length of a single PPM frame.
	 * Zero for non-PPM systems
	 */
	uint16_t		rc_ppm_frame_length;

	/** Input source */
	enum RC_INPUT_SOURCE 	input_source;

	/** measured pulse widths for each of the supported channels */
	rc_input_t		values[RC_INPUT_MAX_CHANNELS];
};

/**
 * @}
 */

/*
 * ObjDev tag for R/C inputs.
 */
ORB_DECLARE(input_rc);

#define _RC_INPUT_BASE		0x2b00

/** Fetch R/C input values into (rc_input_values *)arg */
#define RC_INPUT_GET			_IOC(_RC_INPUT_BASE, 0)

/** Enable RSSI input via ADC */
#define RC_INPUT_ENABLE_RSSI_ANALOG	_IOC(_RC_INPUT_BASE, 1)

/** Enable RSSI input via PWM signal */
#define RC_INPUT_ENABLE_RSSI_PWM	_IOC(_RC_INPUT_BASE, 2)

#endif /* _DRV_RC_INPUT_H */
