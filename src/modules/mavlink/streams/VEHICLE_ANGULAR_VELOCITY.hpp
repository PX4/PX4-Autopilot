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

#ifndef VEHICLE_ANGULAR_VELOCITY_HPP
#define VEHICLE_ANGULAR_VELOCITY_HPP

#include <uORB/topics/vehicle_angular_velocity.h>

class MavlinkStreamVehicleAngularVelocity : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamVehicleAngularVelocity(mavlink); }

	static constexpr const char *get_name_static() { return "VEHICLE_ANGULAR_VELOCITY"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_VEHICLE_ANGULAR_VELOCITY; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_vehicle_angular_velocity_sub.advertised()) {
			return MAVLINK_MSG_ID_VEHICLE_ANGULAR_VELOCITY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamVehicleAngularVelocity(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};

	bool send() override
	{
		if (_vehicle_angular_velocity_sub.updated()) {

			vehicle_angular_velocity_s vehicle_angular_velocity{};
			_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);

			mavlink_vehicle_angular_velocity_t msg{};
			msg.angular_velocity_x = vehicle_angular_velocity.xyz[0];
			msg.angular_velocity_y = vehicle_angular_velocity.xyz[1];
			msg.angular_velocity_z = vehicle_angular_velocity.xyz[2];

			mavlink_msg_vehicle_angular_velocity_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}
		return false;
	}
};

#endif // VEHICLE_ANGULAR_VELOCITY_HPP
