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

#ifndef GIMBAL_DEVICE_ATTITUDE_STATUS_HPP
#define GIMBAL_DEVICE_ATTITUDE_STATUS_HPP

#include <uORB/topics/gimbal_device_attitude_status.h>

class MavlinkStreamGimbalDeviceAttitudeStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamGimbalDeviceAttitudeStatus(mavlink); }

	static constexpr const char *get_name_static() { return "GIMBAL_DEVICE_ATTITUDE_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_gimbal_device_attitude_status_sub.advertised()) {
			return MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamGimbalDeviceAttitudeStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _gimbal_device_attitude_status_sub{ORB_ID(gimbal_device_attitude_status)};

	bool send() override
	{
		gimbal_device_attitude_status_s gimbal_device_attitude_status{};

		if (_gimbal_device_attitude_status_sub.update(&gimbal_device_attitude_status)) {

			if (gimbal_device_attitude_status.received_from_mavlink) {
				// If we have already received the gimbal device's attitude via
				// mavlink it is already forwarded directly and we don't need
				// to re-publish it here.
				return false;
			}

			mavlink_gimbal_device_attitude_status_t msg{};

			msg.target_system = gimbal_device_attitude_status.target_system;
			msg.target_component = gimbal_device_attitude_status.target_component;

			msg.time_boot_ms = gimbal_device_attitude_status.timestamp / 1000;

			msg.flags = gimbal_device_attitude_status.device_flags;

			msg.q[0] = gimbal_device_attitude_status.q[0];
			msg.q[1] = gimbal_device_attitude_status.q[1];
			msg.q[2] = gimbal_device_attitude_status.q[2];
			msg.q[3] = gimbal_device_attitude_status.q[3];

			msg.angular_velocity_x = gimbal_device_attitude_status.angular_velocity_x;
			msg.angular_velocity_y = gimbal_device_attitude_status.angular_velocity_y;
			msg.angular_velocity_z = gimbal_device_attitude_status.angular_velocity_z;

			msg.failure_flags = gimbal_device_attitude_status.failure_flags;

			mavlink_msg_gimbal_device_attitude_status_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // GIMBAL_DEVICE_ATTITUDE_STATUS
