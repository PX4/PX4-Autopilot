/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#ifndef RADIO_STATUS_HPP
#define RADIO_STATUS_HPP

#include <uORB/topics/radio_status.h>

class MavlinkStreamRadioStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamRadioStatus(mavlink); }

	static constexpr const char *get_name_static() { return "RADIO_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_RADIO_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _radio_status_sub.advertised() ? (MAVLINK_MSG_ID_RADIO_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamRadioStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _radio_status_sub{ORB_ID(radio_status), 0};

	bool send() override
	{
		radio_status_s radio_status;
		bool sent = false;

		while ((_mavlink->get_free_tx_buf() >= get_size()) && _radio_status_sub.update(&radio_status)) {
			mavlink_radio_status_t msg{};

			msg.rssi = radio_status.rssi;
			msg.remrssi = radio_status.remote_rssi;
			msg.txbuf = radio_status.txbuf;
			msg.noise = radio_status.noise;
			msg.remnoise = radio_status.remote_noise;
			msg.rxerrors = radio_status.rxerrors;
			msg.fixed = radio_status.fix;

			mavlink_msg_radio_status_send_struct(_mavlink->get_channel(), &msg);

			sent = true;
		}

		return sent;
	}
};

#endif // RADIO_STATUS_HPP
