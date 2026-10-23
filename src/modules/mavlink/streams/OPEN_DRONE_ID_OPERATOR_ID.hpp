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

#ifndef OPEN_DRONE_ID_OPERATOR_ID_HPP
#define OPEN_DRONE_ID_OPERATOR_ID_HPP

#include <uORB/topics/open_drone_id_operator_id.h>
#include <modules/mavlink/open_drone_id_bridge.hpp>

class MavlinkStreamOpenDroneIdOperatorId : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamOpenDroneIdOperatorId(mavlink); }
	static constexpr const char *get_name_static() { return "OPEN_DRONE_ID_OPERATOR_ID"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID; }
	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _configuration_sub.advertised() ? MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamOpenDroneIdOperatorId(Mavlink *mavlink) : MavlinkStream(mavlink) {}
	uORB::Subscription _configuration_sub{ORB_ID(open_drone_id_operator_id)};

	bool send() override
	{
		open_drone_id_operator_id_s configuration{};

		if (_configuration_sub.copy(&configuration)
		    && open_drone_id_bridge::send_on_channel(configuration.mavlink_channel, _mavlink->get_channel())) {
			mavlink_open_drone_id_operator_id_t msg{};
			open_drone_id_bridge::operator_id(msg, configuration);
			mavlink_msg_open_drone_id_operator_id_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // OPEN_DRONE_ID_OPERATOR_ID_HPP
