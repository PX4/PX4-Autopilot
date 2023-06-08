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

#ifndef GPS_INPUT_HPP
#define GPS_INPUT_HPP

#include <uORB/topics/sensor_gps.h>

class MavlinkStreamGPSInput : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamGPSInput(mavlink); }

	static constexpr const char *get_name_static() { return "GPS_INPUT"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_GPS_INPUT; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sensor_gps_subs.advertised_count() * (MAVLINK_MSG_ID_GPS_INPUT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}

private:
	explicit MavlinkStreamGPSInput(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::SubscriptionMultiArray<sensor_gps_s> _sensor_gps_subs{ORB_ID::sensor_gps};

	bool send() override
	{
		bool updated = false;

		for (int i = 0; i < _sensor_gps_subs.size(); i++) {
			sensor_gps_s gps;

			if (_sensor_gps_subs[i].update(&gps)) {
				mavlink_gps_input_t msg{};

				msg.time_usec = gps.timestamp;
				msg.time_utc_usec = gps.time_utc_usec;
				msg.gps_id = i;
				msg.fix_type = gps.fix_type;
				msg.lat = gps.lat;
				msg.lon = gps.lon;
				msg.alt = gps.alt * 1e-3f;
				msg.hdop = gps.hdop;
				msg.vdop = gps.vdop;
				msg.vn = gps.vel_n_m_s;
				msg.ve = gps.vel_e_m_s;
				msg.vd = gps.vel_d_m_s;
				msg.horiz_accuracy = gps.eph;
				msg.vert_accuracy = gps.epv;

				if (PX4_ISFINITE(gps.heading)) {
					if (fabsf(gps.heading) < FLT_EPSILON) {
						msg.yaw = 36000; // Use 36000 for north.

					} else {
						msg.yaw = math::degrees(matrix::wrap_2pi(gps.heading)) * 100.0f; // centidegrees
					}
				}

				mavlink_msg_gps_input_send_struct(_mavlink->get_channel(), &msg);

				updated = true;
			}
		}

		return updated;
	}
};

#endif // GPS_INPUT





