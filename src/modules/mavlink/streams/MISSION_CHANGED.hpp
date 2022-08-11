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

#ifndef MISSION_CHANGED_HPP
#define MISSION_CHANGED_HPP

#include <uORB/topics/mission.h>

class MavlinkStreamMissionChanged : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMissionChanged(mavlink); }

	static constexpr const char *get_name_static() { return "MISSION_CHANGED"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_MISSION_CHANGED; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_mission_sub.advertised()) {
			return MAVLINK_MSG_ID_MISSION_CHANGED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamMissionChanged(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _mission_sub{ORB_ID(mission)};

	bool send() override
	{
		mission_s mission;

		if (_mission_sub.update(&mission)) {

			if (mission.mission_changed) {
				mavlink_mission_changed_t msg{};
				msg.start_index = -1; // All items
				msg.end_index = -1; // Ignored
				msg.origin_sysid = mission.origin_sysid;
				msg.origin_compid = mission.origin_compid;
				msg.mission_type = MAV_MISSION_TYPE_MISSION;

				mavlink_msg_mission_changed_send_struct(_mavlink->get_channel(), &msg);
				return true;

			} else {
				return false;
			}
		}

		return false;
	}
};

#endif // MISSION_CHANGED_HPP
