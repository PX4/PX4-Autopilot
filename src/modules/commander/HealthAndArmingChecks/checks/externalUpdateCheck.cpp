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

#include "externalUpdateCheck.hpp"
#include <stdio.h>
#include <string.h>
#ifdef __PX4_DARWIN
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif
#include <errno.h>

static constexpr char ext_component_path[] = PX4_STORAGEDIR "/ext_component_updated";

void ExternalUpdateChecks::checkAndReport(const Context &context, Report &reporter)
{
#ifdef PX4_STORAGEDIR

	if (_param_com_ext_comp_en.get() == 0) {
		return;
	}

	if (!_already_executed) {
		_already_executed = true;

		struct statfs statfs_buf;
		uint64_t total_bytes = 0;

		if (statfs(ext_component_path, &statfs_buf) == 0) {
			total_bytes = (uint64_t)statfs_buf.f_blocks * statfs_buf.f_bsize;
		}

		if (total_bytes == 0) { // on NuttX we get 0 total bytes if no SD card is inserted
			_update_succeeded = true; // Still allow arming (there's a separate sdcard check)

		} else {
			FILE *fp = fopen(ext_component_path, "r");

			if (fp == nullptr) {
				PX4_WARN("Failed to open external update file (%i)", errno);

			} else {

				char reported_status[64];
				int reported_code = 0;

				if (fscanf(fp, "%s %d", reported_status, &reported_code) == 2) {

					if (!strcmp(reported_status, "SUCCESS")) {
						_update_succeeded = true;

					} else if (!strcmp(reported_status, "FAIL")) {
						_error_code = reported_code;
					}
				}

				fclose(fp);
			}
		}
	}

	if (!_update_succeeded) {
		if (_error_code == -1) {
			/* EVENT
			 * @description
			 * <profile name="dev">
			 * This check can be configured via <param>COM_EXT_COMP_EN</param> parameter.
			 * </profile>
			 */
			reporter.armingCheckFailure(NavModes::All, health_component_t::system,
						    events::ID("check_ext_update_check_fail"),
						    events::Log::Error, "Software update check failed");

		} else {
			/* EVENT
			 * @description
			 * External software component update failed with error code {1}.
			 *
			 * <profile name="dev">
			 * This check can be configured via <param>COM_EXT_COMP_EN</param> parameter.
			 * </profile>
			 */
			reporter.armingCheckFailure<uint32_t>(NavModes::All, health_component_t::system,
							      events::ID("check_ext_update_err"),
							      events::Log::Error, "Software update failed", _error_code);
		}
	}

#endif
}
