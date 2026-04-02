/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * Test code for BaroThrustCfRls
 * Run: make tests TESTFILTER=baro_thrust_cf_rls
 */

#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>
#include <random>
#include <cmath>

#include "baro_thrust_cf_rls.hpp"

using namespace matrix;

class BaroThrustCfRlsTest : public ::testing::Test
{
protected:
	BaroThrustCfRls _estimator{};
	static constexpr float _dt = 0.02f; // 50 Hz

	std::default_random_engine _rng{42};
	std::normal_distribution<float> _normal{0.f, 1.f};
};

// ---------------------------------------------------------------------------
// computeAccelUp — gravity handling
// ---------------------------------------------------------------------------

TEST_F(BaroThrustCfRlsTest, AccelUpAtRestLevel)
{
	// At rest, level: specific force in FRD body = {0, 0, -g}
	const Vector3f accel_body{0.f, 0.f, -CONSTANTS_ONE_G};
	const Quatf identity{};

	const float accel_up = BaroThrustCfRls::computeAccelUp(accel_body, identity);

	EXPECT_NEAR(accel_up, 0.f, 1e-4f);
}

TEST_F(BaroThrustCfRlsTest, AccelUpAscending)
{
	const float a_up_true = 2.f;
	const Vector3f accel_body{0.f, 0.f, -(CONSTANTS_ONE_G + a_up_true)};
	const Quatf identity{};

	const float accel_up = BaroThrustCfRls::computeAccelUp(accel_body, identity);

	EXPECT_NEAR(accel_up, a_up_true, 1e-4f);
}

TEST_F(BaroThrustCfRlsTest, AccelUpTilted45Hover)
{
	const float theta = 45.f * 3.14159265f / 180.f;
	const Vector3f accel_body{CONSTANTS_ONE_G * sinf(theta), 0.f, -CONSTANTS_ONE_G * cosf(theta)};
	const Quatf attitude{Eulerf{0.f, theta, 0.f}};

	const float accel_up = BaroThrustCfRls::computeAccelUp(accel_body, attitude);

	EXPECT_NEAR(accel_up, 0.f, 1e-3f);
}

// ---------------------------------------------------------------------------
// CF residual
// ---------------------------------------------------------------------------

TEST_F(BaroThrustCfRlsTest, CfResidualConstantAltitude)
{
	_estimator.reset();

	const float baro_alt = 100.f;
	const float accel_up = 0.f;
	float residual = 0.f;

	for (int i = 0; i < 500; i++) { // 10 seconds
		residual = _estimator.updateCf(baro_alt, accel_up, _dt);
	}

	EXPECT_NEAR(residual, 0.f, 1e-3f);
}

TEST_F(BaroThrustCfRlsTest, CfResidualCapturesPropwash)
{
	_estimator.reset();

	const float true_alt = 100.f;

	// Settle CF
	for (int i = 0; i < 500; i++) {
		_estimator.updateCf(true_alt, 0.f, _dt);
	}

	// Apply a +3 m propwash error to baro
	const float propwash_error = 3.f;
	float residual = _estimator.updateCf(true_alt + propwash_error, 0.f, _dt);

	EXPECT_NEAR(residual, propwash_error, 0.5f);
}

// ---------------------------------------------------------------------------
// RLS convergence
// ---------------------------------------------------------------------------

TEST_F(BaroThrustCfRlsTest, RlsConvergenceNoiseless)
{
	// Feed residual = K_true * thrust + bias, verify K estimate converges.
	_estimator.reset();

	const float K_true = -5.f;
	const float bias_true = 0.3f;

	for (float t = 0.f; t < 30.f; t += _dt) {
		const float thrust = 0.5f + 0.15f * sinf(2.f * 3.14159f * t / 5.f);
		const float residual = K_true * thrust + bias_true;
		_estimator.updateEstimator(residual, thrust, _dt);
	}

	EXPECT_NEAR(_estimator.kEstimate(), K_true, 0.1f);
}

TEST_F(BaroThrustCfRlsTest, RlsConvergenceWithNoise)
{
	_estimator.reset();

	const float K_true = -3.f;
	const float bias_true = 0.5f;
	const float noise_std = 0.3f;

	for (float t = 0.f; t < 60.f; t += _dt) {
		const float thrust = 0.5f + 0.15f * sinf(2.f * 3.14159f * t / 5.f);
		const float residual = K_true * thrust + bias_true + noise_std * _normal(_rng);
		_estimator.updateEstimator(residual, thrust, _dt);
	}

	EXPECT_NEAR(_estimator.kEstimate(), K_true, 0.5f);
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

TEST_F(BaroThrustCfRlsTest, ResetClearsConvergence)
{
	_estimator.reset();

	const float K_true = -3.f;
	const float bias_true = 0.1f;

	// Drive to convergence
	for (float t = 0.f; t < 80.f; t += _dt) {
		const float thrust = 0.5f + 0.15f * sinf(2.f * 3.14159f * t / 5.f);
		const float residual = K_true * thrust + bias_true;
		_estimator.updateEstimator(residual, thrust, _dt);
		_estimator.checkConvergence(t, _dt);
	}

	EXPECT_TRUE(_estimator.convergedLocked());
	EXPECT_NEAR(_estimator.kEstimate(), K_true, 0.2f);

	// Reset should clear all convergence state
	_estimator.reset();
	EXPECT_FALSE(_estimator.converged());
	EXPECT_FALSE(_estimator.convergedLocked());
	EXPECT_NEAR(_estimator.kEstimate(), 0.f, 1e-6f);
	EXPECT_NEAR(_estimator.kEstimateVar(), BaroThrustCfRls::RLS_P_INIT, 1e-6f);
}

// ---------------------------------------------------------------------------
// Convergence gating
// ---------------------------------------------------------------------------

TEST_F(BaroThrustCfRlsTest, ConvergenceRequiresMinTime)
{
	_estimator.reset();

	const float K_true = -3.f;

	for (float t = 0.f; t < 25.f; t += _dt) { // < 30s minimum
		const float thrust = 0.5f + 0.15f * sinf(2.f * 3.14159f * t / 5.f);
		const float residual = K_true * thrust;
		_estimator.updateEstimator(residual, thrust, _dt);
		_estimator.checkConvergence(t, _dt);
	}

	EXPECT_FALSE(_estimator.convergedLocked());
}

TEST_F(BaroThrustCfRlsTest, NoExcitationDoesNotConverge)
{
	_estimator.reset();

	// Constant thrust — no excitation. RLS can fit K but the excitation
	// gate should prevent convergence since we can't trust the estimate.
	for (float t = 0.f; t < 80.f; t += _dt) {
		const float thrust = 0.5f;
		const float residual = -3.f * thrust;
		_estimator.updateEstimator(residual, thrust, _dt);
		_estimator.checkConvergence(t, _dt);
	}

	EXPECT_FALSE(_estimator.convergedLocked());
}

TEST_F(BaroThrustCfRlsTest, ConvergenceLocksAfterHoldTime)
{
	_estimator.reset();

	const float K_true = -3.f;
	const float bias_true = 0.1f;

	for (float t = 0.f; t < 80.f; t += _dt) {
		const float thrust = 0.5f + 0.15f * sinf(2.f * 3.14159f * t / 5.f);
		const float residual = K_true * thrust + bias_true;
		_estimator.updateEstimator(residual, thrust, _dt);
		_estimator.checkConvergence(t, _dt);
	}

	EXPECT_TRUE(_estimator.convergedLocked());
	EXPECT_NEAR(_estimator.kEstimate(), K_true, 0.2f);
}
