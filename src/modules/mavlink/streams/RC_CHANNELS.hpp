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

#ifndef RC_CHANNELS_HPP
#define RC_CHANNELS_HPP

#include <uORB/topics/input_rc.h>

class MavlinkStreamRCChannels : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamRCChannels(mavlink); }

	static constexpr const char *get_name_static() { return "RC_CHANNELS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_RC_CHANNELS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _input_rc_sub.advertised() ? (MAVLINK_MSG_ID_RC_CHANNELS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamRCChannels(Mavlink *mavlink) : MavlinkStream(mavlink)
	{
		param_get(param_find("MAV_RC_BRIDGE"), (int32_t*) &_rc_bridge);
		param_get(param_find("MAV_RC_FM1"), &_vfc_fm1_chan);
		param_get(param_find("MAV_RC_FM2"), &_vfc_fm2_chan);
		param_get(param_find("MAV_RC_FM3"), &_vfc_fm3_chan);
	}

	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _manual_control_input_sub{ORB_ID(manual_control_input)};

	bool _rc_bridge{false};
	int32_t _vfc_fm1_chan{0};
	int32_t _vfc_fm2_chan{0};
	int32_t _vfc_fm3_chan{0};

	bool send() override
	{
		bool send_msg = false;
		mavlink_rc_channels_t msg{};

		input_rc_s rc;
		if (_input_rc_sub.update(&rc)) {
			// send RC channel data and RSSI
			msg.time_boot_ms = rc.timestamp / 1000;
			msg.chancount = rc.channel_count;
			msg.chan1_raw  = (rc.channel_count > 0)  ? rc.values[0]  : UINT16_MAX;
			msg.chan2_raw  = (rc.channel_count > 1)  ? rc.values[1]  : UINT16_MAX;
			msg.chan3_raw  = (rc.channel_count > 2)  ? rc.values[2]  : UINT16_MAX;
			msg.chan4_raw  = (rc.channel_count > 3)  ? rc.values[3]  : UINT16_MAX;
			msg.chan5_raw  = (rc.channel_count > 4)  ? rc.values[4]  : UINT16_MAX;
			msg.chan6_raw  = (rc.channel_count > 5)  ? rc.values[5]  : UINT16_MAX;
			msg.chan7_raw  = (rc.channel_count > 6)  ? rc.values[6]  : UINT16_MAX;
			msg.chan8_raw  = (rc.channel_count > 7)  ? rc.values[7]  : UINT16_MAX;
			msg.chan9_raw  = (rc.channel_count > 8)  ? rc.values[8]  : UINT16_MAX;
			msg.chan10_raw = (rc.channel_count > 9)  ? rc.values[9]  : UINT16_MAX;
			msg.chan11_raw = (rc.channel_count > 10) ? rc.values[10] : UINT16_MAX;
			msg.chan12_raw = (rc.channel_count > 11) ? rc.values[11] : UINT16_MAX;
			msg.chan13_raw = (rc.channel_count > 12) ? rc.values[12] : UINT16_MAX;
			msg.chan14_raw = (rc.channel_count > 13) ? rc.values[13] : UINT16_MAX;
			msg.chan15_raw = (rc.channel_count > 14) ? rc.values[14] : UINT16_MAX;
			msg.chan16_raw = (rc.channel_count > 15) ? rc.values[15] : UINT16_MAX;
			msg.chan17_raw = (rc.channel_count > 16) ? rc.values[16] : UINT16_MAX;
			msg.chan18_raw = (rc.channel_count > 17) ? rc.values[17] : UINT16_MAX;
			msg.rssi = (rc.channel_count > 0) ? rc.rssi : 0;
			send_msg = true;
		} else if (_rc_bridge) {
			manual_control_setpoint_s manual_control_input{};
			if (_manual_control_input_sub.update(&manual_control_input)) {
				msg.time_boot_ms = hrt_absolute_time() / 1000;
				msg.chancount = 8;

				// Hardcoded mappings for now. Could be parameterized later
				msg.chan1_raw  = (uint16_t) ((manual_control_input.roll * 500.0f) + 1500.0f);
				msg.chan2_raw  = (uint16_t) ((manual_control_input.pitch * 500.0f) + 1500.0f);
				msg.chan3_raw  = (uint16_t) ((manual_control_input.throttle * 500.0f) + 1500.0f);
				msg.chan4_raw  = (uint16_t) ((manual_control_input.yaw * 500.0f) + 1500.0f);

				// Map aux channels to next 4 channels for VFC use
				msg.chan5_raw  = manual_control_input.aux4;
				msg.chan6_raw  = manual_control_input.aux4;
				msg.chan7_raw  = manual_control_input.aux3;
				msg.chan8_raw  = manual_control_input.aux3;

				msg.chan9_raw  = UINT16_MAX;
				msg.chan10_raw = UINT16_MAX;
				msg.chan11_raw = UINT16_MAX;
				msg.chan12_raw = UINT16_MAX;
				msg.chan13_raw = UINT16_MAX;
				msg.chan14_raw = UINT16_MAX;
				msg.chan15_raw = UINT16_MAX;
				msg.chan16_raw = UINT16_MAX;
				msg.chan17_raw = UINT16_MAX;
				msg.chan18_raw = UINT16_MAX;
				msg.rssi = UINT8_MAX - 1;
				send_msg = true;
			}
		}

		if (send_msg) {
			mavlink_msg_rc_channels_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // RC_CHANNELS_HPP
