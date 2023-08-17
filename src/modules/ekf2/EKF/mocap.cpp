/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "mocap.hpp"

#if defined(CONFIG_EKF2_MOCAP) && defined(MODULE_NAME)

void Mocap::update(Ekf &ekf, const estimator::imuSample &imu_delayed)
{

#if defined(MODULE_NAME)

	if (_mocap_sub.updated()) {

		vehicle_odometry_s mocap{};
		_mocap_sub.copy(&mocap);

		const int64_t time_us = mocap.timestamp_sample - static_cast<int64_t>(_param_ekf2_mocap_delay.get() * 1000);

		MocapSample sample{};
		sample.time_us = time_us;
		sample.pos = Vector3f{mocap.position};
		_mocap_buffer.push(sample);
	}

#endif // MODULE_NAME


	MocapSample mocap_sample;

	if (_mocap_buffer.pop_first_older_than(imu_delayed.time_us, &mocap_sample)) {

		if (!ekf.global_origin_valid()) {
			return;
		}

		// position
		{
			estimator_aid_source2d_s aid_src{};

			const Vector2f pos_obs_var{
				math::max(sq(mocap_sample.position_var(0)), sq(_param_ekf2_mocap_noise.get())),
				math::max(sq(mocap_sample.position_var(1)), sq(_param_ekf2_mocap_noise.get()))
			};

			ekf.updateHorizontalPositionAidSrcStatus(mocap_sample.time_us,
					mocap_sample.pos.xy(),                         // observation
					pos_obs_var,                                   // observation variance
					math::max(_param_ekf2_mocap_gate.get(), 1.f),  // innovation gate
					aid_src);
			aid_src.fusion_enabled = _param_ekf2_mocap_ctrl.get() & 0b001; // bit 0 Horizontal position

			// TODO: check frame
			if (ekf.control_status_flags().yaw_align) {
				ekf.fuseHorizontalPosition(aid_src);
			}

#if defined(MODULE_NAME)
			aid_src.timestamp = hrt_absolute_time();
			_estimator_aid_src_mocap_xy_pub.publish(aid_src);
#endif // MODULE_NAME
		}

	}
}

#endif // CONFIG_EKF2_MOCAP
