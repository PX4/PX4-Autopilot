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

/**
 * @file hover_thrust_estimator.cpp
 *
 * @author Mathieu Bresciani 	<brescianimathieu@gmail.com>
 */

#include "hover_thrust_estimator.hpp"

void HoverThrustEstimator::reset()
{
	_thrust = 0.f;
	_acc_z = 0.f;
	_hover_thrust_ekf.setHoverThrustStdDev(_param_hte_ht_err_init.get());
	_hover_thrust_ekf.resetAccelNoise();

}

void HoverThrustEstimator::updateParams()
{
	const float ht_err_init_prev = _param_hte_ht_err_init.get();
	ModuleParams::updateParams();

	_hover_thrust_ekf.setProcessNoiseStdDev(_param_hte_ht_noise.get());

	if (fabsf(_param_hte_ht_err_init.get() - ht_err_init_prev) > FLT_EPSILON) {
		_hover_thrust_ekf.setHoverThrustStdDev(_param_hte_ht_err_init.get());
	}

	_hover_thrust_ekf.setAccelInnovGate(_param_hte_acc_gate.get());
}

void HoverThrustEstimator::update(const float dt)
{
	_hover_thrust_ekf.predict(dt);

	ZeroOrderHoverThrustEkf::status status{};
	_hover_thrust_ekf.fuseAccZ(_acc_z, _thrust, status);

	publishStatus(status);
}

void HoverThrustEstimator::publishStatus(ZeroOrderHoverThrustEkf::status &status)
{
	hover_thrust_estimate_s status_msg{};
	status_msg.timestamp = hrt_absolute_time();
	status_msg.hover_thrust = status.hover_thrust;
	status_msg.hover_thrust_var = status.hover_thrust_var;
	status_msg.accel_innov = status.innov;
	status_msg.accel_innov_var = status.innov_var;
	status_msg.accel_innov_test_ratio = status.innov_test_ratio;
	status_msg.accel_noise_var = status.accel_noise_var;
	_hover_thrust_ekf_pub.publish(status_msg);
}
