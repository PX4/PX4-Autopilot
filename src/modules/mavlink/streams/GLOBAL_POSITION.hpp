/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#ifndef GLOBAL_POSITION_HPP
#define GLOBAL_POSITION_HPP

#include <stdint.h>

#include <uORB/topics/vehicle_global_position.h>

class MavlinkStreamGLobalPosition : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamGLobalPosition(mavlink); }

	static constexpr const char *get_name_static() { return "GLOBAL_POSITION"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_GLOBAL_POSITION; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _aux_global_position_sub.advertised() ? (MAVLINK_MSG_ID_GLOBAL_POSITION_LEN +
				MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamGLobalPosition(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _aux_global_position_sub{ORB_ID(aux_global_position)};

	bool send() override
	{
		vehicle_global_position_s pos{};

		if (_aux_global_position_sub.update(&pos)) {
			mavlink_global_position_t msg{};

			msg.id = UINT8_C(1);
			msg.time_usec = pos.timestamp;
			msg.source = GLOBAL_POSITION_UNKNOWN;
			msg.flags = 0;

			if (PX4_ISFINITE(pos.lat)) {
				msg.lat = static_cast<int32_t>(pos.lat * 1e7);

			} else {
				msg.lat = INT32_MAX;
			}

			if (PX4_ISFINITE(pos.lon)) {
				msg.lon = static_cast<int32_t>(pos.lon * 1e7);

			} else {
				msg.lon = INT32_MAX;
			}

			msg.alt = pos.alt;
			msg.alt_ellipsoid = pos.alt_ellipsoid;

			msg.eph = pos.eph;
			msg.epv = pos.epv;


			mavlink_msg_global_position_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // GLOBAL_POSITION_HPP
