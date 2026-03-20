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

#include <uORB/topics/estimator_fusion_control.h>
#include <cmath>

/**
 * Array index = ESTIMATOR_SENSOR_FUSION_SOURCE - 1:
 *   [0] GPS    [1] OF     [2] EV    [3] AGP
 *   [4] BARO   [5] RNG    [6] MAG   [7] ASPD  [8] RNGBCN
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
		return _estimator_fusion_control_sub.advertised() ? MAVLINK_MSG_ID_ESTIMATOR_SENSOR_FUSION_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	// Array indices matching ESTIMATOR_SENSOR_FUSION_SOURCE - 1
	static constexpr uint8_t IDX_GPS    = 0;
	static constexpr uint8_t IDX_OF     = 1;
	static constexpr uint8_t IDX_EV     = 2;
	static constexpr uint8_t IDX_AGP    = 3;
	static constexpr uint8_t IDX_BARO   = 4;
	static constexpr uint8_t IDX_RNG    = 5;
	static constexpr uint8_t IDX_MAG    = 6;
	static constexpr uint8_t IDX_ASPD   = 7;
	static constexpr uint8_t IDX_RNGBCN = 8;

	explicit MavlinkStreamEstimatorSensorFusionStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _estimator_fusion_control_sub{ORB_ID(estimator_fusion_control)};

	bool send() override
	{
		estimator_fusion_control_s fc;

		if (_estimator_fusion_control_sub.update(&fc)) {
			mavlink_estimator_sensor_fusion_status_t msg{};

			for (int i = 0; i < 9; i++) { msg.test_ratio[i] = NAN; }

			// intended: sensor enabled by user AND CTRL param not disabled
			for (int i = 0; i < 2; i++) {
				if (fc.gps_intended[i]) { msg.intended[IDX_GPS] |= (1u << i); }
			}

			msg.intended[IDX_OF]     = fc.of_intended;
			msg.intended[IDX_EV]     = fc.ev_intended;
			msg.intended[IDX_BARO]   = fc.baro_intended;
			msg.intended[IDX_RNG]    = fc.rng_intended;
			msg.intended[IDX_MAG]    = fc.mag_intended;
			msg.intended[IDX_ASPD]   = fc.aspd_intended;
			msg.intended[IDX_RNGBCN] = fc.rngbcn_intended;

			for (int i = 0; i < 4; i++) {
				if (fc.agp_intended[i]) { msg.intended[IDX_AGP] |= (1u << i); }
			}

			// active: estimator is actually fusing data from this source
			for (int i = 0; i < 2; i++) {
				if (fc.gps_active[i]) { msg.active[IDX_GPS] |= (1u << i); }
			}

			msg.active[IDX_OF]   = fc.of_active;
			msg.active[IDX_EV]   = fc.ev_active;

			for (int i = 0; i < 4; i++) {
				if (fc.agp_active[i]) { msg.active[IDX_AGP] |= (1u << i); }
			}

			msg.active[IDX_BARO] = fc.baro_active;
			msg.active[IDX_RNG]  = fc.rng_active;
			msg.active[IDX_MAG]  = fc.mag_active;
			msg.active[IDX_ASPD]   = fc.aspd_active;
			msg.active[IDX_RNGBCN] = fc.rngbcn_active;

			mavlink_msg_estimator_sensor_fusion_status_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // ESTIMATOR_SENSOR_FUSION_STATUS_HPP
