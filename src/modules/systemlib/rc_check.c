/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file rc_check.c
 *
 * RC calibration check
 */

#include <nuttx/config.h>

#include <stdio.h>
#include <fcntl.h>

#include <systemlib/err.h>
#include <systemlib/rc_check.h>
#include <systemlib/param/param.h>
#include <mavlink/mavlink_log.h>
#include <drivers/drv_rc_input.h>

int rc_calibration_check(int mavlink_fd) {

	char nbuf[20];
	param_t _parameter_handles_min, _parameter_handles_trim, _parameter_handles_max,
	_parameter_handles_rev, _parameter_handles_dz;

	float param_min, param_max, param_trim, param_rev, param_dz;

	/* first check channel mappings */
			/* check which map param applies */
		// if (map_by_channel[i] >= MAX_CONTROL_CHANNELS) {
		// 	mavlink_log_critical(mavlink_fd, "ERR: RC_%d_MAP >= # CHANS", i+1);
		// 	/* give system time to flush error message in case there are more */
		// 	usleep(100000);
		// 	count++;
		// }

	int channel_fail_count = 0;

	for (int i = 0; i < RC_INPUT_MAX_CHANNELS; i++) {
		/* should the channel be enabled? */
		uint8_t count = 0;

		/* min values */
		sprintf(nbuf, "RC%d_MIN", i + 1);
		_parameter_handles_min = param_find(nbuf);
		param_get(_parameter_handles_min, &param_min);

		/* trim values */
		sprintf(nbuf, "RC%d_TRIM", i + 1);
		_parameter_handles_trim = param_find(nbuf);
		param_get(_parameter_handles_trim, &param_trim);

		/* max values */
		sprintf(nbuf, "RC%d_MAX", i + 1);
		_parameter_handles_max = param_find(nbuf);
		param_get(_parameter_handles_max, &param_max);

		/* channel reverse */
		sprintf(nbuf, "RC%d_REV", i + 1);
		_parameter_handles_rev = param_find(nbuf);
		param_get(_parameter_handles_rev, &param_rev);

		/* channel deadzone */
		sprintf(nbuf, "RC%d_DZ", i + 1);
		_parameter_handles_dz = param_find(nbuf);
		param_get(_parameter_handles_dz, &param_dz);

		/* assert min..center..max ordering */
		if (param_min < 500) {
			count++;
			mavlink_log_critical(mavlink_fd, "#audio ERR: RC_%d_MIN < 500", i+1);
			/* give system time to flush error message in case there are more */
			usleep(100000);
		}
		if (param_max > 2500) {
			count++;
			mavlink_log_critical(mavlink_fd, "#audio ERR: RC_%d_MAX > 2500", i+1);
			/* give system time to flush error message in case there are more */
			usleep(100000);
		}
		if (param_trim < param_min) {
			count++;
			mavlink_log_critical(mavlink_fd, "#audio ERR: RC_%d_TRIM < MIN (%d/%d)", i+1, (int)param_trim, (int)param_min);
			/* give system time to flush error message in case there are more */
			usleep(100000);
		}
		if (param_trim > param_max) {
			count++;
			mavlink_log_critical(mavlink_fd, "#audio ERR: RC_%d_TRIM > MAX (%d/%d)", i+1, (int)param_trim, (int)param_max);
			/* give system time to flush error message in case there are more */
			usleep(100000);
		}

		/* assert deadzone is sane */
		if (param_dz > 500) {
			mavlink_log_critical(mavlink_fd, "#audio ERR: RC_%d_DZ > 500", i+1);
			/* give system time to flush error message in case there are more */
			usleep(100000);
			count++;
		}

		/* check which map param applies */
		// if (map_by_channel[i] >= MAX_CONTROL_CHANNELS) {
		// 	mavlink_log_critical(mavlink_fd, "ERR: RC_%d_MAP >= # CHANS", i+1);
		// 	/* give system time to flush error message in case there are more */
		// 	usleep(100000);
		// 	count++;
		// }

		/* sanity checks pass, enable channel */
		if (count) {
			mavlink_log_critical(mavlink_fd, "#audio ERROR: %d config error(s) for RC channel %d.", count, (i + 1));
			warnx("ERROR: %d config error(s) for RC channel %d.", count, (i + 1));
			usleep(100000);
		}

		channel_fail_count += count;
	}

	return channel_fail_count;
}
