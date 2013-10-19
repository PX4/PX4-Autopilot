/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
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
 * @file missionlib.h
 * MAVLink missionlib components
 */

// XXX trim includes
#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <stdlib.h>

#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <mavlink/mavlink_log.h>

#include <mavlink/orb_topics.h>
#include <mavlink/mavlink_bridge_header.h>
#include <navigator/waypoints.h>
#include <mavlink/missionlib.h>
#include <mavlink/util.h>

static uint8_t missionlib_msg_buf[MAVLINK_MAX_PACKET_LEN];

__EXPORT int
missionlib_send_mavlink_message(mavlink_message_t *msg)
{
	uint16_t len = mavlink_msg_to_send_buffer(missionlib_msg_buf, msg);

	mavlink_send_uart_bytes(chan, missionlib_msg_buf, len);
	return 0;
}

__EXPORT int
missionlib_send_mavlink_gcs_string(const char *string)
{
	const int len = MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN;
	mavlink_statustext_t statustext;
	int i = 0;

	while (i < len - 1) {
		statustext.text[i] = string[i];

		if (string[i] == '\0')
			break;

		i++;
	}

	if (i > 1) {
		/* Enforce null termination */
		statustext.text[i] = '\0';
		mavlink_message_t msg;

		mavlink_msg_statustext_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &statustext);
		return missionlib_send_mavlink_message(&msg);

	} else {
		return 1;
	}
}

/**
 * Get system time since boot in microseconds
 *
 * @return the system time since boot in microseconds
 */
__EXPORT uint64_t missionlib_get_system_timestamp()
{
	return hrt_absolute_time();
}

