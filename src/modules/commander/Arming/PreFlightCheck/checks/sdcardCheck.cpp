/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
#ifdef __PX4_DARWIN
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif

bool PreFlightCheck::sdcardCheck(orb_advert_t *mavlink_log_pub, const bool report_fail)
{
	const char *microsd_dir = PX4_STORAGEDIR;
	bool success = true;
	struct statfs statfs_buf;
	uint64_t total_bytes = 0;

	int32_t reporting_enabled = 0;
	param_get(param_find("COM_ARM_SDCARD"), &reporting_enabled);

	if (reporting_enabled == 0) {
		return success;
	}

	if (statfs(microsd_dir, &statfs_buf) == 0) {
		total_bytes = (uint64_t)statfs_buf.f_blocks * statfs_buf.f_bsize;
	}

	if (total_bytes == 0) { // on NuttX we get 0 total bytes if no SD card is inserted

		if (reporting_enabled == 2) {
			success = false;
		}

		if (report_fail && reporting_enabled > 0) {
			mavlink_log_critical(mavlink_log_pub, "Warning! Missing FMU SD Card.");
		}
	}

	return success;
}
