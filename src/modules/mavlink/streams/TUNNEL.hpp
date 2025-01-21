/****************************************************************************
 *
 *   Copyright (c) 2025 Technology Innovation Institute. All rights reserved.
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

#ifndef TUNNEL_HPP
#define TUNNEL_HPP

#include <uORB/topics/mavlink_tunnel.h>

class MavlinkStreamTunnel : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamTunnel(mavlink); }

	static constexpr const char *get_name_static() { return "TUNNEL"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_TUNNEL; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_TUNNEL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamTunnel(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _tunnel_sub{ORB_ID(mavlink_tunnel)};

	bool send() override
	{
		mavlink_tunnel_s tunnel;

		if (_tunnel_sub.update(&tunnel)) {
			if (tunnel.target_system == _mavlink->get_system_id()) {
				// Skip messages sent by self
				return false;
			}

			mavlink_tunnel_t msg{};
			msg.payload_type =  tunnel.payload_type;
			msg.target_system =  tunnel.target_system;
			msg.target_component =  tunnel.target_component;
			msg.payload_length =  tunnel.payload_length;

			if (msg.payload_length > MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN) {
				PX4_ERR("tunnel payload too big: %u", msg.payload_length);
				return false;
			}

			memcpy(msg.payload, tunnel.payload, msg.payload_length);

			mavlink_msg_tunnel_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // TUNNEL_HPP
