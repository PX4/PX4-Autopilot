/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef OPTICAL_FLOW_RAD_HPP
#define OPTICAL_FLOW_RAD_HPP

#include <uORB/topics/vehicle_optical_flow.h>

class MavlinkStreamOpticalFlowRad : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamOpticalFlowRad(mavlink); }

	static constexpr const char *get_name_static() { return "OPTICAL_FLOW_RAD"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_OPTICAL_FLOW_RAD; }

	const char *get_name() const override { return MavlinkStreamOpticalFlowRad::get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _vehicle_optical_flow_sub.advertised() ? (MAVLINK_MSG_ID_OPTICAL_FLOW_RAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) :
		       0;
	}

private:
	explicit MavlinkStreamOpticalFlowRad(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _vehicle_optical_flow_sub{ORB_ID(vehicle_optical_flow)};

	bool send() override
	{
		vehicle_optical_flow_s flow;

		if (_vehicle_optical_flow_sub.update(&flow)) {
			mavlink_optical_flow_rad_t msg{};

			msg.time_usec = flow.timestamp_sample;
			msg.sensor_id = _vehicle_optical_flow_sub.get_instance();
			msg.integration_time_us = flow.integration_timespan_us;
			msg.integrated_x = flow.pixel_flow[0];
			msg.integrated_y = flow.pixel_flow[1];
			msg.integrated_xgyro = flow.delta_angle[0];
			msg.integrated_ygyro = flow.delta_angle[1];
			msg.integrated_zgyro = flow.delta_angle[2];
			// msg.temperature = 0;
			msg.quality = flow.quality;
			// msg.time_delta_distance_us = 0;

			if (PX4_ISFINITE(flow.distance_m)) {
				msg.distance = flow.distance_m;

			} else {
				msg.distance = -1;
			}

			mavlink_msg_optical_flow_rad_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // OPTICAL_FLOW_RAD_HPP
