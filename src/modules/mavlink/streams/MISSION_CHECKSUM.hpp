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

#ifndef MISSION_CHECKSUM_HPP
#define MISSION_CHECKSUM_HPP

#include <uORB/topics/mission_checksum.h>

class MavlinkStreamMissionChecksum : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMissionChecksum(mavlink); }

	static constexpr const char *get_name_static() { return "MISSION_CHECKSUM"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_MISSION_CHECKSUM; }

	const char *get_name() const override { return MavlinkStreamMissionChecksum::get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _mission_checksum_sub.advertised() ? (MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

	bool request_message(float param2, float param3, float param4, float param5, float param6, float param7) override
	{
		return send(static_cast<uint8_t>(param2));
	}

private:
	explicit MavlinkStreamMissionChecksum(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _mission_checksum_sub{ORB_ID(mission_checksum)};

	bool send() override
	{
		return send(MAV_MISSION_TYPE_ALL);
	}

	bool send(uint8_t mission_type)
	{
		mission_checksum_s csum;

		if (_mission_checksum_sub.advertised() && _mission_checksum_sub.copy(&csum)) {
			mavlink_mission_checksum_t msg{};

			if (mission_type == MAV_MISSION_TYPE_MISSION) {
				msg.checksum = csum.mission_checksum;

			} else if (mission_type == MAV_MISSION_TYPE_FENCE) {
				msg.checksum = csum.fence_checksum;

			} else if (mission_type == MAV_MISSION_TYPE_RALLY) {
				msg.checksum = csum.rally_checksum;

			} else if (mission_type == MAV_MISSION_TYPE_ALL) {
				msg.checksum = csum.all_checksum;

			} else {
				return false;
			}

			msg.mission_type = mission_type;
			mavlink_msg_mission_checksum_send_struct(_mavlink->get_channel(), &msg);
		}

		return true;
	}
};

#endif // MISSION_CHECKSUM_HPP
