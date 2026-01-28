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

#ifndef CURRENT_MODE_HPP
#define CURRENT_MODE_HPP

#include <uORB/topics/vehicle_status.h>
#include <lib/modes/standard_modes.hpp>

class MavlinkStreamCurrentMode : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamCurrentMode(mavlink); }

	static constexpr const char *get_name_static() { return "CURRENT_MODE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_CURRENT_MODE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_CURRENT_MODE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamCurrentMode(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	bool send() override
	{
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.update(&vehicle_status)) {
			mavlink_current_mode_t current_mode{};
			current_mode.custom_mode = get_px4_custom_mode(vehicle_status.nav_state).data;
			current_mode.intended_custom_mode = get_px4_custom_mode(vehicle_status.nav_state_user_intention).data;
			current_mode.standard_mode = (uint8_t) mode_util::getStandardModeFromNavState(vehicle_status.nav_state,
						     vehicle_status.vehicle_type, vehicle_status.is_vtol);
			mavlink_msg_current_mode_send_struct(_mavlink->get_channel(), &current_mode);
			return true;
		}

		return false;
	}
};

#endif // CURRENT_MODE_HPP
