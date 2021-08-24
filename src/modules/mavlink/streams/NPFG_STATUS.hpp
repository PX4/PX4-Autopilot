/****************************************************************************
 *
 *   Copyright (c) 2021 Autonomous Systems Lab, ETH Zurich. All rights reserved.
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

#ifndef NPFG_STATUS_HPP
#define NPFG_STATUS_HPP

#include <uORB/topics/npfg_status.h>

class MavlinkStreamNPFGStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamNPFGStatus(mavlink); }

	static constexpr const char *get_name_static() { return "NPFG_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_NPFG_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_npfg_status_sub.advertised()) {
			return MAVLINK_MSG_ID_NPFG_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamNPFGStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _npfg_status_sub{ORB_ID(npfg_status)};

	bool send() override
	{
		struct npfg_status_s npfg_status;

		if (_npfg_status_sub.update(&npfg_status)) {
			mavlink_npfg_status_t msg{};

			msg.timestamp = npfg_status.timestamp;
			msg.lat_accel = npfg_status.lat_accel;
			msg.lat_accel_ff = npfg_status.lat_accel_ff;
			msg.bearing_feas = npfg_status.bearing_feas;
			msg.bearing_feas_on_track = npfg_status.bearing_feas_on_track;
			msg.signed_track_error = npfg_status.signed_track_error;
			msg.track_error_bound = npfg_status.track_error_bound;
			msg.airspeed_ref = npfg_status.airspeed_ref;
			msg.bearing = math::degrees(npfg_status.bearing);
			msg.heading_ref = math::degrees(npfg_status.heading_ref);
			msg.min_ground_speed_ref = npfg_status.min_ground_speed_ref;
			msg.adapted_period = npfg_status.adapted_period;
			msg.p_gain = npfg_status.p_gain;
			msg.time_const = npfg_status.time_const;

			mavlink_msg_npfg_status_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // NPFG_STATUS
