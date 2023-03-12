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

#ifndef VEHICLE_ATTITUDE_HPP
#define VEHICLE_ATTITUDE_HPP

#include <uORB/topics/vehicle_attitude.h>

class MavlinkStreamVehicleAttitude : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamVehicleAttitude(mavlink); }

	static constexpr const char *get_name_static() { return "VEHICLE_ATTITUDE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_VEHICLE_ATTITUDE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_vehicle_attitude_sub.advertised()) {
			return MAVLINK_MSG_ID_VEHICLE_ATTITUDE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamVehicleAttitude(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	bool send() override
	{
		if (_vehicle_attitude_sub.updated()) {

			vehicle_attitude_s vehicle_attitude{};
			_vehicle_attitude_sub.copy(&vehicle_attitude);

			mavlink_vehicle_attitude_t msg{};
			msg.q_w = vehicle_attitude.q[0];
			msg.q_x = vehicle_attitude.q[1];
			msg.q_y = vehicle_attitude.q[2];
			msg.q_z = vehicle_attitude.q[3];

			msg.delta_q_w = vehicle_attitude.delta_q_reset[0];
			msg.delta_q_x = vehicle_attitude.delta_q_reset[1];
			msg.delta_q_y = vehicle_attitude.delta_q_reset[2];
			msg.delta_q_z = vehicle_attitude.delta_q_reset[3];

			mavlink_msg_vehicle_attitude_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}
		return false;
	}
};

#endif // VEHICLE_ATTITUDE_HPP
