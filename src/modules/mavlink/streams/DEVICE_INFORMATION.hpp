/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#ifndef DEVICE_INFORMATION_HPP
#define DEVICE_INFORMATION_HPP

#include <cstdlib>
#include <uORB/topics/device_information.h>

class MavlinkStreamDeviceInformation : public MavlinkStream
{
public:
	static constexpr const char *get_name_static() { return "DEVICE_INFORMATION"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_DEVICE_INFORMATION; }

	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamDeviceInformation(mavlink); }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_device_information_sub.advertised()) {
			return MAVLINK_MSG_ID_DEVICE_INFORMATION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamDeviceInformation(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _device_information_sub{ORB_ID(device_information)};

	bool send() override
	{
		device_information_s device_information;

		if (_device_information_sub.advertised() && _device_information_sub.copy(&device_information)) {
			mavlink_device_information_t msg{};

			msg.device_type = device_information.device_type;
			memcpy(msg.vendor_name, device_information.vendor_name, sizeof(msg.vendor_name));
			memcpy(msg.model_name, device_information.model_name, sizeof(msg.model_name));
			memcpy(msg.firmware_version, device_information.firmware_version, sizeof(msg.firmware_version));
			memcpy(msg.hardware_version, device_information.hardware_version, sizeof(msg.hardware_version));
			memcpy(msg.serial_number, device_information.serial_number, sizeof(msg.serial_number));

			mavlink_msg_device_information_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // DEVICE_INFORMATION_HPP
