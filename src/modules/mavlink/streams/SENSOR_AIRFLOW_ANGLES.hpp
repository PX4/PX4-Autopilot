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

#ifndef SENSOR_AIRFLOW_ANGLES_HPP
#define SENSOR_AIRFLOW_ANGLES_HPP

#include <uORB/topics/airflow_aoa.h>
#include <uORB/topics/airflow_slip.h>

class MavlinkStreamSensorAirflowAngles : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamSensorAirflowAngles(mavlink); }

	static constexpr const char *get_name_static() { return "SENSOR_AIRFLOW_ANGLES"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
    if (_airflow_aoa_sub.advertised() || _airflow_slip_sub.advertised()) {
      return MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

    return 0;
	}

private:
	explicit MavlinkStreamSensorAirflowAngles(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _airflow_aoa_sub{ORB_ID(airflow_aoa)};
	uORB::Subscription _airflow_slip_sub{ORB_ID(airflow_slip)};

	bool send() override
	{
		if (_airflow_aoa_sub.updated() || _airflow_slip_sub.updated()) {
		  mavlink_sensor_airflow_angles_t msg{};

	    struct airflow_aoa_s airflow_aoa;
      if (_airflow_aoa_sub.copy(&airflow_aoa)) {
        msg.timestamp = airflow_aoa.timestamp / 1000;
        msg.angleofattack = math::degrees(airflow_aoa.aoa_rad);
        msg.angleofattack_valid = airflow_aoa.valid;
      } else {
        msg.timestamp = 0;
        msg.angleofattack = 0.0;
        msg.angleofattack_valid = false;
      }

      struct airflow_slip_s airflow_slip;
      if (_airflow_slip_sub.copy(&airflow_slip)) {
        const uint64_t timestamp = airflow_slip.timestamp / 1000;
        if (timestamp > msg.timestamp) {
          msg.timestamp = timestamp;
        }
        msg.sideslip = math::degrees(airflow_slip.slip_rad);
        msg.sideslip_valid = airflow_slip.valid;
      } else {
        msg.timestamp = 0;
        msg.sideslip = 0.0;
        msg.sideslip_valid = false;
      }

      mavlink_msg_sensor_airflow_angles_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // SENSOR_AIRFLOW_ANGLES
