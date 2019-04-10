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

#include "rc_check.h"

#include <px4_config.h>
#include <px4_time.h>

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

#include <systemlib/err.h>

#include <parameters/param.h>
#include <systemlib/mavlink_log.h>
#include <drivers/drv_rc_input.h>

#define RC_INPUT_MAP_UNMAPPED 0

int rc_calibration_check(orb_advert_t *mavlink_log_pub, bool report_fail, bool isVTOL)
{
	char nbuf[20];
	param_t _parameter_handles_min, _parameter_handles_trim, _parameter_handles_max,
		_parameter_handles_rev, _parameter_handles_dz;

	unsigned map_fail_count = 0;

	const char *rc_map_mandatory[] = {	/*"RC_MAP_MODE_SW",*/
		/* needs discussion if this should be mandatory "RC_MAP_POSCTL_SW"*/
		nullptr /* end marker */
	};

	unsigned j = 0;

	/* if VTOL, check transition switch mapping */
	if (isVTOL) {
		param_t trans_parm = param_find("RC_MAP_TRANS_SW");

		if (trans_parm == PARAM_INVALID) {
			if (report_fail) { mavlink_log_critical(mavlink_log_pub, "RC_MAP_TRANS_SW PARAMETER MISSING."); }

			/* give system time to flush error message in case there are more */
			px4_usleep(100000);
			map_fail_count++;

		} else {
			int32_t transition_switch;
			param_get(trans_parm, &transition_switch);

			if (transition_switch < 1) {
				if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Transition switch RC_MAP_TRANS_SW not set."); }

				map_fail_count++;
			}
		}
	}


	/* first check channel mappings */
	while (rc_map_mandatory[j] != nullptr) {

		param_t map_parm = param_find(rc_map_mandatory[j]);

		if (map_parm == PARAM_INVALID) {
			if (report_fail) { mavlink_log_critical(mavlink_log_pub, "RC ERROR: PARAM %s MISSING.", rc_map_mandatory[j]); }

			/* give system time to flush error message in case there are more */
			px4_usleep(100000);
			map_fail_count++;
			j++;
			continue;
		}

		int32_t mapping;
		param_get(map_parm, &mapping);

		if (mapping > input_rc_s::RC_INPUT_MAX_CHANNELS) {
			if (report_fail) { mavlink_log_critical(mavlink_log_pub, "RC ERROR: %s >= NUMBER OF CHANNELS.", rc_map_mandatory[j]); }

			/* give system time to flush error message in case there are more */
			px4_usleep(100000);
			map_fail_count++;
		}

		if (mapping == 0) {
			if (report_fail) { mavlink_log_critical(mavlink_log_pub, "RC ERROR: mandatory %s is unmapped.", rc_map_mandatory[j]); }

			/* give system time to flush error message in case there are more */
			px4_usleep(100000);
			map_fail_count++;
		}

		j++;
	}

	unsigned total_fail_count = 0;
	unsigned channels_failed = 0;

	for (unsigned i = 0; i < input_rc_s::RC_INPUT_MAX_CHANNELS; i++) {
		/* should the channel be enabled? */
		uint8_t count = 0;

		/* initialize values to values failing the check */
		float param_min = 0.0f;
		float param_max = 0.0f;
		float param_trim = 0.0f;
		float param_rev = 0.0f;
		float param_dz = RC_INPUT_MAX_DEADZONE_US * 2.0f;

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
		if (param_min < RC_INPUT_LOWEST_MIN_US) {
			count++;

			if (report_fail) { mavlink_log_critical(mavlink_log_pub, "RC ERROR: RC%d_MIN < %u.", i + 1, RC_INPUT_LOWEST_MIN_US); }

			/* give system time to flush error message in case there are more */
			px4_usleep(100000);
		}

		if (param_max > RC_INPUT_HIGHEST_MAX_US) {
			count++;

			if (report_fail) { mavlink_log_critical(mavlink_log_pub, "RC ERROR: RC%d_MAX > %u.", i + 1, RC_INPUT_HIGHEST_MAX_US); }

			/* give system time to flush error message in case there are more */
			px4_usleep(100000);
		}

		if (param_trim < param_min) {
			count++;

			if (report_fail) { mavlink_log_critical(mavlink_log_pub, "RC ERROR: RC%d_TRIM < MIN (%d/%d).", i + 1, (int)param_trim, (int)param_min); }

			/* give system time to flush error message in case there are more */
			px4_usleep(100000);
		}

		if (param_trim > param_max) {
			count++;

			if (report_fail) { mavlink_log_critical(mavlink_log_pub, "RC ERROR: RC%d_TRIM > MAX (%d/%d).", i + 1, (int)param_trim, (int)param_max); }

			/* give system time to flush error message in case there are more */
			px4_usleep(100000);
		}

		/* assert deadzone is sane */
		if (param_dz > RC_INPUT_MAX_DEADZONE_US) {
			if (report_fail) { mavlink_log_critical(mavlink_log_pub, "RC ERROR: RC%d_DZ > %u.", i + 1, RC_INPUT_MAX_DEADZONE_US); }

			/* give system time to flush error message in case there are more */
			px4_usleep(100000);
			count++;
		}

		total_fail_count += count;

		if (count) {
			channels_failed++;
		}
	}

	if (channels_failed) {
		px4_sleep(2);

		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "%d config error%s for %d RC channel%s.",
					     total_fail_count,
					     (total_fail_count > 1) ? "s" : "", channels_failed, (channels_failed > 1) ? "s" : "");
		}

		px4_usleep(100000);
	}

	return total_fail_count + map_fail_count;
}
