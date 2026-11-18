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

/**
 * @file MavlinkStreamExtensionExample.hpp
 *
 * The stream of the px4_add_mavlink_extension() example: a MEMORY_VECT whose first four value
 * bytes hold, little-endian, the count of MEMORY_VECT messages MavlinkExtensionExampleHandler
 * received.
 */

#ifndef MAVLINK_STREAM_EXTENSION_EXAMPLE_HPP
#define MAVLINK_STREAM_EXTENSION_EXAMPLE_HPP

#include "MavlinkExtensionExampleHandler.hpp"

class MavlinkStreamExtensionExample : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamExtensionExample(mavlink); }

	static constexpr const char *get_name_static() { return "MEMORY_VECT"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_MEMORY_VECT; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_MEMORY_VECT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamExtensionExample(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	bool send() override
	{
		const uint32_t received = MavlinkExtensionExampleHandler::received();

		mavlink_memory_vect_t msg{};
		msg.ver = 1;  // the type codes of version 1
		msg.type = 1; // 16 x uint16_t: the count's low word, then its high word
		memcpy(msg.value, &received, sizeof(received)); // little-endian on all PX4 targets

		mavlink_msg_memory_vect_send_struct(_mavlink->get_channel(), &msg);
		return true;
	}
};

#endif // MAVLINK_STREAM_EXTENSION_EXAMPLE_HPP
