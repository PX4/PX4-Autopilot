/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file AdmittanceControl.hpp
 *
 * PMEN - Defines admittance controller. Modifies setpoints received for offboard control.
 *
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <matrix/integration.hpp>
#include <uORB/topics/vehicle_local_position_setpoint.h>

using namespace matrix;


struct AdmittanceParameters {
	float M[4];
	float C[4];
	float K[4];
	float Fd[4];
};

struct BellParameters {
	float K_max[4];
	float K_min[4];
	float M_max[4];
	float M_min[4];
	float A[4];
	float B1[4];
	float B2[4];
	float B3[4];
	float lpf_sat_factor;
};

class AdmittanceControl
{
public:

	AdmittanceControl() = default;
	~AdmittanceControl() = default;

	void initialize(BellParameters &bell_params);
	void reset(const vehicle_local_position_setpoint_s &setpoint);
	void update(const float &dt, const Vector<float, 4> &We, const Vector<float, 8> &u, const float &target_dist,
			const Quatf &q, const vehicle_local_position_setpoint_s &setpoint);

	vehicle_local_position_setpoint_s getSetpoints() const {return _admittance_sp_ned;}
	AdmittanceParameters getAdmittanceParameters() const {return _params;}
	float getSaturationFactor() const {return _sat_factor;}

private:

	Vector<float, 8> _integrate();
	int _integrate_rk4(Vector<float, 8> (*f)(float, const Matrix<float, 8, 1> &x, const Matrix<float, 4, 1> &u,
			const AdmittanceParameters &params),
			const Matrix<float, 8, 1> &y0,
			const Matrix<float, 4, 1> &u,
			float t0,
			float tf,
			float h0,
			Matrix<float, 8, 1> &y1,
			const AdmittanceParameters &params
			);

	float _dt{0.01f}; //< Sampling Time
	Vector<float, 8> _y{};
	Vector<float, 4> _We{};
	Vector3f _acc_sp{};
	vehicle_local_position_setpoint_s _admittance_sp_ned{};

	AdmittanceParameters _params{};
	BellParameters _params_bell{};

	float _sat_factor{0.f};
};
