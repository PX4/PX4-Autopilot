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

#ifndef ESC_EEPROM_HPP
#define ESC_EEPROM_HPP

#include <uORB/topics/esc_eeprom.h>

class MavlinkStreamESCEeprom : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamESCEeprom(mavlink); }

	static constexpr const char *get_name_static() { return "ESC_EEPROM"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ESC_EEPROM; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _esc_eeprom_sub.advertised() ? MAVLINK_MSG_ID_ESC_EEPROM_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamESCEeprom(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _esc_eeprom_sub{ORB_ID(esc_eeprom)};

	bool send() override
	{
		esc_eeprom_s esc_eeprom;

		if (_esc_eeprom_sub.update(&esc_eeprom)) {
			mavlink_esc_eeprom_t msg{};

			// ESC_EEPROM
			// index 			: Motor Index (0 - 4)
			// data[128]		: Raw data
			// length			: Length of data in the buffer
			//
			// The only ESCs that can output params are DShot and CAN.
			// Assumes ESC is DShot. CAN ESCs would use DroneCAN parameters.

			msg.index = esc_eeprom.index;
			memcpy(msg.data, esc_eeprom.data, esc_eeprom.length);
			msg.length = esc_eeprom.length;

			PX4_INFO("sending ESC_EEPROM %d", _mavlink->get_channel());
			mavlink_msg_esc_eeprom_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // ESC_EEPROM_HPP
