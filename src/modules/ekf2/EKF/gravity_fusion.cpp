/****************************************************************************
 *
 *   Copyright (c) 2018 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file gravity_fusion.cpp
 * Fuse observations from the gravity vector to constrain roll
 * and pitch (a la complementary filter).
 * 
 * @author Daniel M. Sahu <danielmohansahu@gmail.com>
 */

#include "ekf.h"
#include "python/ekf_derivation/generated/compute_gravity_innov_var_and_k_and_h.h"

#include <mathlib/mathlib.h>

void Ekf::fuseGravity()
{
	// get raw accelerometer reading at delayed horizon and expected measurement noise (gaussian)
	const imuSample imu = get_imu_sample_delayed();
	const Vector3f measurement = imu.delta_vel / imu.delta_vel_dt - getAccelBias();
	const float acc_measurement_noise = sq(_params.gravity_noise);

	// initialize fusion variables
	Vector24f Kx, Ky, Kz; // Kalman gain vectors
	Vector3f innovation_variance;

	// calculate kalman gains and innovation variances
	sym::ComputeGravityInnovVarAndKAndH(
		getStateAtFusionHorizonAsVector(), P, measurement, acc_measurement_noise, FLT_EPSILON,
		&_gravity_innov, &innovation_variance, &Kx, &Ky, &Kz);

	// perform fusion for each axis
	if (!measurementUpdate(Kx, innovation_variance(0), _gravity_innov(0))) {
		PX4_ERR("Gravity fusion (X axis) unsuccessful.");
	}
	if (!measurementUpdate(Ky, innovation_variance(1), _gravity_innov(1))) {
		PX4_ERR("Gravity fusion (y axis) unsuccessful.");
	}
	if (!measurementUpdate(Kz, innovation_variance(2), _gravity_innov(2))) {
		PX4_ERR("Gravity fusion (Z axis) unsuccessful.");
	}
}
