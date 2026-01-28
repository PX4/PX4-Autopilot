/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file mavlink_log.cpp
 * MAVLink text logging.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <drivers/drv_hrt.h>
#include <px4_platform_common/posix.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#include <uORB/topics/mavlink_log.h>
#include "mavlink_log.h"

__EXPORT void mavlink_vasprintf(int severity, orb_advert_t *mavlink_log_pub, const char *fmt, ...)
{
	// TODO: add compile check for maxlen

	if (!fmt) {
		return;
	}

	if (mavlink_log_pub == nullptr) {
		return;
	}

	mavlink_log_s log_msg;
	log_msg.severity = severity;
	log_msg.timestamp = hrt_absolute_time();

	va_list ap;
	va_start(ap, fmt);
	vsnprintf((char *)log_msg.text, sizeof(log_msg.text), fmt, ap);
	va_end(ap);

	if (*mavlink_log_pub != nullptr) {
		orb_publish(ORB_ID(mavlink_log), *mavlink_log_pub, &log_msg);

	} else {
		*mavlink_log_pub = orb_advertise(ORB_ID(mavlink_log), &log_msg);
	}
}
