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

#ifndef HYGROMETER_SENSOR_HPP
#define HYGROMETER_SENSOR_HPP

#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/sensor_hygrometer.h>

class MavlinkStreamHygrometerSensor : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamHygrometerSensor(mavlink); }

	static constexpr const char *get_name_static() { return "HYGROMETER_SENSOR"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_HYGROMETER_SENSOR; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sensor_hygrometer_subs.advertised_count() * (MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN +
				MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}

private:
	explicit MavlinkStreamHygrometerSensor(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::SubscriptionMultiArray<sensor_hygrometer_s> _sensor_hygrometer_subs{ORB_ID::sensor_hygrometer};

	bool send() override
	{
		bool updated = false;

		for (int i = 0; i < _sensor_hygrometer_subs.size(); i++) {
			sensor_hygrometer_s sensor_hygrometer;

			if (_sensor_hygrometer_subs[i].update(&sensor_hygrometer)) {
				mavlink_hygrometer_sensor_t msg{};
				msg.id          = i; // uint8_t Hygrometer ID
				msg.temperature = roundf(sensor_hygrometer.temperature * 100.f); // degrees to centidegrees (int16_t)
				msg.humidity    = roundf(sensor_hygrometer.humidity);            // % (uint16_t)

				mavlink_msg_hygrometer_sensor_send_struct(_mavlink->get_channel(), &msg);

				updated = true;
			}
		}

		return updated;
	}
};
#endif // HYGROMETER_SENSOR_HPP
