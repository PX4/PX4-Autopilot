/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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


#include "ekf.h"
#include <aid_sources/gnss/gnss.hpp>

#if defined(CONFIG_EKF2_GNSS) && defined(MODULE_NAME)

using matrix::Vector3f;

void Gnss::initParameters(Ekf &ekf)
{
	for (uint8_t i = 0; i < MAX_GNSS_INSTANCES; i++) {
		char param_name[20] {};
		snprintf(param_name, sizeof(param_name), "EKF2_GPS%d_CTRL", i);
		_slots[i].ctrl_handle = param_find(param_name);
	}

	updateParameters(ekf);
}

void Gnss::updateParameters(Ekf &ekf)
{
	for (uint8_t i = 0; i < MAX_GNSS_INSTANCES; i++) {
		if (_slots[i].ctrl_handle != PARAM_INVALID) {
			param_get(_slots[i].ctrl_handle, &ekf.gnssSource(i).params.ctrl);
		}
	}
}

bool Gnss::anySlotEnabled(const Ekf &ekf, const int32_t bit) const
{
	for (uint8_t i = 0; i < MAX_GNSS_INSTANCES; i++) {
		const int32_t ctrl = ekf.gnssSource(i).params.ctrl;

		if ((bit == 0) ? (ctrl != 0) : (ctrl & bit)) {
			return true;
		}
	}

	return false;
}

void Gnss::advertiseEnabledPublications(const Ekf &ekf)
{
	for (uint8_t i = 0; i < MAX_GNSS_INSTANCES; i++) {
		const GnssSource &src = ekf.gnssSource(i);

		if (src.ctrl(GnssCtrl::HPOS)) {
			_slots[i].aid_src_pos_pub.advertise();
		}

		if (src.ctrl(GnssCtrl::VEL)) {
			_slots[i].aid_src_vel_pub.advertise();
		}
	}
}

void Gnss::updateSamples(Ekf &ekf, const float yaw_offset_deg)
{
	for (uint8_t instance = 0; instance < MAX_GNSS_INSTANCES; instance++) {
		sensor_gps_s vehicle_gps_position;

		if (!_slots[instance].sub.update(&vehicle_gps_position)) {
			continue;
		}

		if (!vehicle_gps_position.vel_ned_valid) {
			continue; //TODO: change and set to NAN
		}

		const Vector3f vel_ned(vehicle_gps_position.vel_n_m_s,
				       vehicle_gps_position.vel_e_m_s,
				       vehicle_gps_position.vel_d_m_s);

		if (fabsf(yaw_offset_deg) > 0.f) {
			if (!PX4_ISFINITE(vehicle_gps_position.heading_offset) && PX4_ISFINITE(vehicle_gps_position.heading)) {
				// Apply offset
				float yaw_offset = matrix::wrap_pi(math::radians(yaw_offset_deg));
				vehicle_gps_position.heading_offset = yaw_offset;
				vehicle_gps_position.heading = matrix::wrap_pi(vehicle_gps_position.heading - yaw_offset);
			}
		}

		const float altitude_amsl = static_cast<float>(vehicle_gps_position.altitude_msl_m);
		const float altitude_ellipsoid = static_cast<float>(vehicle_gps_position.altitude_ellipsoid_m);

		// timestamp_sample is corrected by the sensors module (per-receiver delay or PPS)
		const bool timestamp_corrected = vehicle_gps_position.timestamp_sample > 0
						 && vehicle_gps_position.timestamp_sample != vehicle_gps_position.timestamp;

		gnssSample gnss_sample{
			.time_us = timestamp_corrected ? vehicle_gps_position.timestamp_sample : vehicle_gps_position.timestamp,
			.lat = vehicle_gps_position.latitude_deg,
			.lon = vehicle_gps_position.longitude_deg,
			.alt = altitude_amsl,
			.vel = vel_ned,
			.hacc = vehicle_gps_position.eph,
			.vacc = vehicle_gps_position.epv,
			.sacc = vehicle_gps_position.s_variance_m_s,
			.fix_type = vehicle_gps_position.fix_type,
			.nsats = vehicle_gps_position.satellites_used,
			.pdop = sqrtf(vehicle_gps_position.hdop *vehicle_gps_position.hdop
				      + vehicle_gps_position.vdop * vehicle_gps_position.vdop),
			.yaw = vehicle_gps_position.heading, //TODO: move to different message
			.yaw_acc = vehicle_gps_position.heading_accuracy,
			.yaw_offset = vehicle_gps_position.heading_offset,
			.spoofed = vehicle_gps_position.spoofing_state == sensor_gps_s::SPOOFING_STATE_DETECTED,
			.jammed = vehicle_gps_position.jamming_state == sensor_gps_s::JAMMING_STATE_DETECTED,
			.pos_body = Vector3f(vehicle_gps_position.antenna_offset_x,
					     vehicle_gps_position.antenna_offset_y,
					     vehicle_gps_position.antenna_offset_z),
			.device_id = vehicle_gps_position.device_id,
		};

		ekf.setGpsData(gnss_sample, instance);

		const float geoid_height = altitude_ellipsoid - altitude_amsl;

		if (_last_geoid_height_update_us == 0) {
			_geoid_height_lpf.reset(geoid_height);
			_last_geoid_height_update_us = gnss_sample.time_us;

		} else if (gnss_sample.time_us > _last_geoid_height_update_us) {
			_geoid_height_lpf.setParameters(gnss_sample.time_us - _last_geoid_height_update_us,
							kGeoidHeightLpfTimeConstant);
			_geoid_height_lpf.update(geoid_height);
			_last_geoid_height_update_us = gnss_sample.time_us;
		}
	}
}

void Gnss::publishAidSourceStatus(const Ekf &ekf, const hrt_abstime &timestamp, const uint8_t estimator_instance,
				  const bool replay_mode)
{
	for (uint8_t i = 0; i < MAX_GNSS_INSTANCES; i++) {
		const estimator_aid_source2d_s &pos = ekf.aid_src_gnss_pos(i);

		if (pos.timestamp_sample > _slots[i].pos_pub_last) {
			estimator_aid_source2d_s status_out{pos};
			status_out.estimator_instance = estimator_instance;
			status_out.timestamp = replay_mode ? timestamp : hrt_absolute_time();
			_slots[i].aid_src_pos_pub.publish(status_out);
			_slots[i].pos_pub_last = pos.timestamp_sample;
		}

		const estimator_aid_source3d_s &vel = ekf.aid_src_gnss_vel(i);

		if (vel.timestamp_sample > _slots[i].vel_pub_last) {
			estimator_aid_source3d_s status_out{vel};
			status_out.estimator_instance = estimator_instance;
			status_out.timestamp = replay_mode ? timestamp : hrt_absolute_time();
			_slots[i].aid_src_vel_pub.publish(status_out);
			_slots[i].vel_pub_last = vel.timestamp_sample;
		}
	}
}

#endif // CONFIG_EKF2_GNSS && MODULE_NAME
