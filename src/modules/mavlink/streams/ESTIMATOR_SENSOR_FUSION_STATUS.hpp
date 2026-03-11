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

#ifndef ESTIMATOR_SENSOR_FUSION_STATUS_HPP
#define ESTIMATOR_SENSOR_FUSION_STATUS_HPP

#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/estimator_fusion_control.h>

/**
 * Array index = ESTIMATOR_SENSOR_FUSION_SOURCE - 1:
 *   [0] GPS    [1] OF     [2] EV    [3] AGP
 *   [4] BARO   [5] RNG    [6] DRAG  [7] MAG   [8] IMU
 *
 * Each element is a per-instance bitmask (bit 0 = instance 0, etc.).
 */

class MavlinkStreamEstimatorSensorFusionStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamEstimatorSensorFusionStatus(mavlink); }

	static constexpr const char *get_name_static() { return "ESTIMATOR_SENSOR_FUSION_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ESTIMATOR_SENSOR_FUSION_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _estimator_status_flags_sub.advertised() ? MAVLINK_MSG_ID_ESTIMATOR_SENSOR_FUSION_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	// Array indices matching ESTIMATOR_SENSOR_FUSION_SOURCE - 1
	static constexpr uint8_t IDX_GPS  = 0;
	static constexpr uint8_t IDX_OF   = 1;
	static constexpr uint8_t IDX_EV   = 2;
	static constexpr uint8_t IDX_AGP  = 3;
	static constexpr uint8_t IDX_BARO = 4;
	static constexpr uint8_t IDX_RNG  = 5;
	static constexpr uint8_t IDX_DRAG = 6;
	static constexpr uint8_t IDX_MAG  = 7;
	static constexpr uint8_t IDX_IMU  = 8;

	explicit MavlinkStreamEstimatorSensorFusionStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _estimator_status_flags_sub{ORB_ID(estimator_status_flags)};
	uORB::Subscription _estimator_fusion_control_sub{ORB_ID(estimator_fusion_control)};

	bool send() override
	{
		estimator_status_flags_s flags;

		if (_estimator_status_flags_sub.update(&flags)) {
			mavlink_estimator_sensor_fusion_status_t msg{};

			// --- intended: effective CTRL values with runtime overrides ---
			estimator_fusion_control_s fc{};
			_estimator_fusion_control_sub.copy(&fc);

			if (fc.gps_intended  != 0) { msg.intended[IDX_GPS]  = 1; }

			if (fc.of_intended   != 0) { msg.intended[IDX_OF]   = 1; }

			if (fc.ev_intended   != 0) { msg.intended[IDX_EV]   = 1; }

			if (fc.baro_intended != 0) { msg.intended[IDX_BARO] = 1; }

			if (fc.rng_intended  != 0) { msg.intended[IDX_RNG]  = 1; }

			if (fc.drag_intended != 0) { msg.intended[IDX_DRAG] = 1; }

			if (fc.mag_intended  != 0) { msg.intended[IDX_MAG]  = 1; }

			if (fc.imu_intended  != 0) { msg.intended[IDX_IMU]  = 1; }

			for (int i = 0; i < 4; i++) {
				if (fc.agp_intended[i] != 0) { msg.intended[IDX_AGP] |= (1u << i); }
			}

			// --- active: estimator is actually fusing data from this source ---
			if (flags.cs_gnss_pos || flags.cs_gps_hgt || flags.cs_gnss_vel || flags.cs_gnss_yaw) {
				msg.active[IDX_GPS] = 1;
			}

			if (flags.cs_opt_flow) {
				msg.active[IDX_OF] = 1;
			}

			if (flags.cs_ev_pos || flags.cs_ev_hgt || flags.cs_ev_vel || flags.cs_ev_yaw) {
				msg.active[IDX_EV] = 1;
			}

			msg.active[IDX_AGP] = fc.agp_active;

			if (flags.cs_baro_hgt) {
				msg.active[IDX_BARO] = 1;
			}

			if (flags.cs_rng_hgt) {
				msg.active[IDX_RNG] = 1;
			}

			// Note: no cs_drag or cs_imu flags in estimator_status_flags

			if (flags.cs_mag) {
				msg.active[IDX_MAG] = 1;
			}

			mavlink_msg_estimator_sensor_fusion_status_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // ESTIMATOR_SENSOR_FUSION_STATUS_HPP
