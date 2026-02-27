/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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
 * @file rate_control.cpp
 */

#include "fw_indi_control.hpp"
#include <px4_platform_common/defines.h>

using namespace matrix;

Quatf
FixedWingIndiControl::get_flat_attitude(Vector3f vel, Vector3f f)
{

	const Vector3f vel_air = vel - wind_estimate_;

	// compute force component projected onto lift axis
	const Vector3f vel_normalized = vel_air.normalized();

	Vector3f f_drag = (f * vel_normalized) * vel_normalized;
	Vector3f f_lift = f - f_drag;

	const Vector3f lift_normalized = f_lift.normalized();
	const Vector3f wing_normalized = -vel_normalized.cross(lift_normalized);
	// compute rotation matrix between ENU and FRD frame
	Dcmf R_bi;
	R_bi(0, 0) = vel_normalized(0);
	R_bi(0, 1) = vel_normalized(1);
	R_bi(0, 2) = vel_normalized(2);
	R_bi(1, 0) = wing_normalized(0);
	R_bi(1, 1) = wing_normalized(1);
	R_bi(1, 2) = wing_normalized(2);
	R_bi(2, 0) = lift_normalized(0);
	R_bi(2, 1) = lift_normalized(1);
	R_bi(2, 2) = lift_normalized(2);
	R_bi.renormalize();

	float rho_corrected;

	if (_cal_airspeed >= _stall_speed) {
		rho_corrected = _rho * powf(_cal_airspeed / _true_airspeed, 2);

	} else {
		rho_corrected = _rho;
	}

	// compute required AoA
	Vector3f f_phi = R_bi * f_lift;
	float AoA = ((2.f * f_phi(2)) / (rho_corrected * _area * (vel_air * vel_air) + 0.001f) - _C_L0) / _C_L1 - _aoa_offset;
	// compute final rotation matrix
	Eulerf e(0.f, AoA, 0.f);
	Dcmf R_pitch(e);
	Dcmf Rotation(R_pitch * R_bi);

	Quatf q(Rotation);
	return q;
}
