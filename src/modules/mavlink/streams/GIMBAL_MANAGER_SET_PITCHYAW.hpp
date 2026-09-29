/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#ifndef GIMBAL_MANAGER_SET_PITCHYAW_HPP
#define GIMBAL_MANAGER_SET_PITCHYAW_HPP

#include <uORB/topics/gimbal_manager_set_pitchyaw.h>

class MavlinkStreamGimbalManagerSetPitchyaw : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamGimbalManagerSetPitchyaw(mavlink); }

	static constexpr const char *get_name_static() { return "GIMBAL_MANAGER_SET_PITCHYAW"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_gimbal_manager_set_pitchyaw_sub.advertised()) {
			return (MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
		}

		return 0;
	}

private:
	explicit MavlinkStreamGimbalManagerSetPitchyaw(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _gimbal_manager_set_pitchyaw_sub{ORB_ID(gimbal_manager_set_pitchyaw)};

	bool send() override
	{
		gimbal_manager_set_pitchyaw_s set_pitchyaw;

		if (_gimbal_manager_set_pitchyaw_sub.update(&set_pitchyaw)) {
			mavlink_gimbal_manager_set_pitchyaw_t msg{};

			msg.target_system = set_pitchyaw.target_system;
			msg.target_component = set_pitchyaw.target_component;
			msg.gimbal_device_id = set_pitchyaw.gimbal_device_id;
			msg.flags = set_pitchyaw.flags;
			msg.pitch = set_pitchyaw.pitch;
			msg.yaw = set_pitchyaw.yaw;
			msg.pitch_rate = set_pitchyaw.pitch_rate;
			msg.yaw_rate = set_pitchyaw.yaw_rate;

			mavlink_msg_gimbal_manager_set_pitchyaw_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // GIMBAL_MANAGER_SET_PITCHYAW_HPP
