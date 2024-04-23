/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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
 * @file AttitudeControlMath.hpp
 */

#pragma once

#include <matrix/matrix/math.hpp>

namespace AttitudeControlMath
{
/**
 * Rotate a tilt quaternion (without yaw rotation) such that when rotated by a yaw setpoint
 * the resulting tilt is the same as if it was rotated by the current yaw of the vehicle
 * @param q_sp_tilt pure tilt quaternion (yaw = 0) that needs to be corrected
 * @param q_att current attitude of the vehicle
 * @param q_sp_yaw pure yaw quaternion of the desired yaw setpoint
 */
void inline correctTiltSetpointForYawError(matrix::Quatf &q_sp_tilt, const matrix::Quatf &q_att,
		const matrix::Quatf &q_sp_yaw)
{
	const matrix::Vector3f z_unit(0.f, 0.f, 1.f);

	// Extract yaw from the current attitude
	const matrix::Vector3f att_z = q_att.dcm_z();
	const matrix::Quatf q_tilt(z_unit, att_z);
	const matrix::Quatf q_yaw = q_tilt.inversed() * q_att; // This is not euler yaw

	// Find the quaternion that creates a tilt aligned with the body frame
	// when rotated by the yaw setpoint
	// To do so, solve q_yaw * q_tilt_ne = q_sp_yaw * q_sp_rp_compensated
	const matrix::Quatf q_sp_rp_compensated = q_sp_yaw.inversed() * q_yaw * q_sp_tilt;

	// Extract the corrected tilt
	const matrix::Vector3f att_sp_z = q_sp_rp_compensated.dcm_z();
	q_sp_tilt = matrix::Quatf(z_unit, att_sp_z);
}
}
