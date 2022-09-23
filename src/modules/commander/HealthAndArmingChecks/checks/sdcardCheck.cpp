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

#include "sdcardCheck.hpp"

#ifdef __PX4_DARWIN
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif

void SdCardChecks::checkAndReport(const Context &context, Report &reporter)
{
	if (_param_com_arm_sdcard.get() <= 0) {
		return;
	}

	struct statfs statfs_buf;

	if (!_sdcard_detected && statfs(PX4_STORAGEDIR, &statfs_buf) == 0) {
		// on NuttX we get a data block count f_blocks and byte count per block f_bsize if an SD card is inserted
		_sdcard_detected = (statfs_buf.f_blocks > 0) && (statfs_buf.f_bsize > 0);
	}

	if (!_sdcard_detected) {
		NavModes affected_modes{NavModes::None};

		if (_param_com_arm_sdcard.get() == 2) {
			// disallow arming without sd card
			affected_modes = NavModes::All;
		}

		/* EVENT
		 * @description
		 * Insert an SD Card to the autopilot and reboot the system.
		 *
		 * <profile name="dev">
		 * This check can be configured via <param>COM_ARM_SDCARD</param> parameter.
		 * </profile>
		 */
		reporter.armingCheckFailure(affected_modes, health_component_t::system, events::ID("check_missing_fmu_sdcard"),
					    events::Log::Error, "Missing FMU SD Card");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Missing FMU SD Card");
		}
	}
}
