/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "../PreFlightCheck.hpp"
#include <lib/parameters/param.h>
#include <systemlib/mavlink_log.h>
#include <stdio.h>
#include <string.h>
#ifdef __PX4_DARWIN
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif

/* Error codes
 *
 * -1 = File does not exists
 * -2 = FMU card missing
 *  0 = Parsing failure
 *
 */

static constexpr char ext_component_path[] = PX4_STORAGEDIR "/ext_component_updated";

bool PreFlightCheck::externalUpdateCheck(orb_advert_t *mavlink_log_pub, const bool report_fail)
{
	static bool update_succeeded = false;
	static bool already_executed = false;
	static int error_code = -1;
	int32_t enable_check = -1;
	struct statfs statfs_buf;
	uint64_t total_bytes = 0;
	param_get(param_find("COM_EXT_COMP_EN"), &enable_check);

	if (enable_check == 0) {
		return true;
	}

	if (!already_executed) {
		already_executed = true;

		if (statfs(ext_component_path, &statfs_buf) == 0) {
			total_bytes = (uint64_t)statfs_buf.f_blocks * statfs_buf.f_bsize;
		}

		if (total_bytes == 0) { // on NuttX we get 0 total bytes if no SD card is inserted
			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "Update Check Fail: Missing FMU SD Card");
			}

			error_code = -2;
			return true; // Still allow arming when the SD card is missing
		}

		FILE *fp = fopen(ext_component_path, "r");

		if (fp == nullptr) {
			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "Update Fail: External update file not found");
			}

			return false;
		}

		char reported_status[64];
		int reported_code = 0;

		if (fscanf(fp, "%s %d", reported_status, &reported_code) == 2) {

			if (!strcmp(reported_status, "SUCCESS")) {
				update_succeeded = true;

			} else if (!strcmp(reported_status, "FAIL")) {
				error_code = reported_code;
			}

		} else {
			error_code = 0;
		}
	}

	bool result = update_succeeded;

	if (report_fail && !update_succeeded) {
		if (error_code == -2) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Software update check failed.");
			result = true; // Still allow arming when the SD card is missing

		} else {
			mavlink_log_critical(mavlink_log_pub, "Update Fail: Software update failed, error code %d.",
					     error_code);
		}
	}

	return result;
}
