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
#include <dirent.h>
#include <string.h>

#ifdef __PX4_DARWIN
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif

bool PreFlightCheck::sdcardCheck(orb_advert_t *mavlink_log_pub, bool &sd_card_detected_once,
				 const bool report_fail)
{
	bool success = true;

	int32_t param_com_arm_sdcard{0};
	param_get(param_find("COM_ARM_SDCARD"), &param_com_arm_sdcard);

	if (param_com_arm_sdcard > 0) {
		struct statfs statfs_buf;

		if (!sd_card_detected_once && statfs(PX4_STORAGEDIR, &statfs_buf) == 0) {
			// on NuttX we get a data block count f_blocks and byte count per block f_bsize if an SD card is inserted
			sd_card_detected_once = (statfs_buf.f_blocks > 0) && (statfs_buf.f_bsize > 0);
		}

		if (!sd_card_detected_once) {
			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "Warning! Missing FMU SD Card.");
			}

			if (param_com_arm_sdcard == 2) {
				// disallow arming without sd card
				success = false;
			}
		}
	}

#ifdef __PX4_NUTTX
	// Check for hardfault files
	static bool hardfault_checked_once = false; // TODO: make this a class member when porting to newer PX4
	static bool hardfault_file_present = false;

	if (!hardfault_checked_once) {
		hardfault_checked_once = true;

		int32_t param_com_arm_hardfault_check{0};
		param_get(param_find("COM_ARM_HFLT_CHK"), &param_com_arm_hardfault_check);

		if (param_com_arm_hardfault_check > 0) {

			DIR *dp = opendir(PX4_STORAGEDIR);

			if (dp != nullptr) {

				struct dirent *result;

				while ((result = readdir(dp)) && !hardfault_file_present) {

					// Check for pattern fault_*.log
					if (strncmp("fault_", result->d_name, 6) == 0 && strcmp(result->d_name + strlen(result->d_name) - 4, ".log") == 0) {
						hardfault_file_present = true;
					}
				}

				closedir(dp);
			}
		}
	}

	if (hardfault_file_present) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Crash dumps present on SD, vehicle needs service");
		}

		success = false;
	}

#endif

	return success;
}
