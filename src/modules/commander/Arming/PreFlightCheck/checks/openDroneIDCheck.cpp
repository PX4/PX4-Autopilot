/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/events.h>

bool PreFlightCheck::openDroneIDCheck(orb_advert_t *mavlink_log_pub, const bool report_fail,
				      const vehicle_status_flags_s &status_flags)
{
	bool success = true;

	int32_t param_com_prearm_odid = false;
	param_get(param_find("COM_PREARM_ODID"), &param_com_prearm_odid);
	const bool open_drone_id_required = param_com_prearm_odid != 0;

	if (open_drone_id_required) {
		if (!status_flags.open_drone_id_system_present) {
			success = false;

			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "Fail: OpenDroneID system missing");
				events::send(events::ID("commander_open_drone_id_missing"), events::Log::Critical, "OpenDroneID system missing");
			}

		} else if (!status_flags.open_drone_id_system_healthy) {
			success = false;

			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "Fail: OpenDroneID system not ready");
				events::send(events::ID("commander_open_drone_id_system_not_ready"), events::Log::Critical,
					     "OpenDroneID system not ready");
			}
		}
	}

	return success;
}
