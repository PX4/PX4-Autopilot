/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4. All rights reserved.
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

/**
 * @file gps_fusion.cpp
 * Function for fusing gps measurements
 */

/* #include <mathlib/mathlib.h> */
#include "ekf.h"

void Ekf::updateGpsYaw(const gpsSample &gps_sample)
{
	auto &gps_yaw = _aid_src_gnss_yaw;
	resetEstimatorAidStatus(gps_yaw);

	if (PX4_ISFINITE(gps_sample.yaw)) {
		// initially populate for estimator_aid_src_gnss_yaw logging

		// calculate the observed yaw angle of antenna array, converting a from body to antenna yaw measurement
		const float measured_hdg = wrap_pi(gps_sample.yaw + _gps_yaw_offset);

		gps_yaw.observation = measured_hdg;
		gps_yaw.observation_variance = sq(fmaxf(_params.gps_heading_noise, 1.0e-2f));

		// define the predicted antenna array vector and rotate into earth frame
		const Vector3f ant_vec_bf = {cosf(_gps_yaw_offset), sinf(_gps_yaw_offset), 0.0f};
		const Vector3f ant_vec_ef = _R_to_earth * ant_vec_bf;
		const float predicted_hdg = atan2f(ant_vec_ef(1), ant_vec_ef(0));
		gps_yaw.innovation = wrap_pi(predicted_hdg - gps_yaw.observation);

		gps_yaw.fusion_enabled = _control_status.flags.gps_yaw;

		gps_yaw.timestamp_sample = gps_sample.time_us;
	}
}
