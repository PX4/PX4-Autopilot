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

#ifndef ESC_STATUS_HPP
#define ESC_STATUS_HPP

#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/esc_status.h>
#include <mathlib/mathlib.h>

class MavlinkStreamESCStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamESCStatus(mavlink); }

	static constexpr const char *get_name_static() { return "ESC_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ESC_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		static constexpr unsigned message_size = MAVLINK_MSG_ID_ESC_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		return _esc_status_subs.advertised_count() * message_size;
	}

private:
	explicit MavlinkStreamESCStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::SubscriptionMultiArray<esc_status_s> _esc_status_subs{ORB_ID::esc_status};

	static constexpr uint8_t MAX_ESC_OUTPUTS = 12; // See output_functions.hpp
	static constexpr uint8_t ESCS_PER_MSG = MAVLINK_MSG_ESC_STATUS_FIELD_RPM_LEN;
	static constexpr uint8_t MAX_NUM_MSGS = MAX_ESC_OUTPUTS / ESCS_PER_MSG;
	static constexpr hrt_abstime ESC_TIMEOUT = 100000;

	struct EscStatus {
		hrt_abstime timestamp;
		int32_t rpm;
		float voltage;
		float current;
	};

	EscStatus _escs[MAX_ESC_OUTPUTS] = {};

	void update_data() override
	{
		int subscriber_count = math::min(_esc_status_subs.size(), MAX_NUM_MSGS);

		for (int i = 0; i < subscriber_count; i++) {
			esc_status_s esc = {};

			if (_esc_status_subs[i].update(&esc)) {
				for (int j = 0; j < esc_status_s::CONNECTED_ESC_MAX; j++) {

					bool is_motor = ((int)esc.esc[j].actuator_function >= esc_report_s::ACTUATOR_FUNCTION_MOTOR1) &&
							((int)esc.esc[j].actuator_function <= esc_report_s::ACTUATOR_FUNCTION_MOTOR12);

					if (is_motor) {
						// Map OutputFunction number to index
						int index = (int)esc.esc[j].actuator_function - esc_report_s::ACTUATOR_FUNCTION_MOTOR1;
						_escs[index].timestamp = esc.esc[j].timestamp;
						_escs[index].rpm = esc.esc[j].esc_rpm;
						_escs[index].voltage = esc.esc[j].esc_voltage;
						_escs[index].current = esc.esc[j].esc_current;
					}
				}
			}
		}
	}

	bool send() override
	{
		bool updated = false;

		for (int i = 0; i < MAX_NUM_MSGS; i++) {

			hrt_abstime now = hrt_absolute_time();

			mavlink_esc_status_t msg = {};
			msg.index = i * ESCS_PER_MSG;
			msg.time_usec = now;

			bool atleast_one_esc_updated = false;

			for (int j = 0; j < ESCS_PER_MSG; j++) {

				EscStatus &esc = _escs[i * ESCS_PER_MSG + j];

				if ((esc.timestamp != 0) && (esc.timestamp + ESC_TIMEOUT) > now) {
					msg.rpm[j] = esc.rpm;
					msg.voltage[j] = esc.voltage;
					msg.current[j] = esc.current;
					atleast_one_esc_updated = true;
				}
			}

			if (atleast_one_esc_updated) {
				mavlink_msg_esc_status_send_struct(_mavlink->get_channel(), &msg);
				updated = true;
			}
		}

		return updated;
	}
};

#endif // ESC_STATUS_HPP
