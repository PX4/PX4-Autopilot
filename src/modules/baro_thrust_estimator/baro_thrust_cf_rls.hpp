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
 * @file baro_thrust_cf_rls.hpp
 *
 * Complementary Filter + Recursive Least Squares (CF+RLS) estimator
 * for barometer thrust compensation.  Pure math — no uORB, no PX4
 * module infrastructure.
 *
 * Problem: propwash creates a static pressure error at the baro sensor
 * proportional to motor output.  We want to identify the gain K in:
 *
 *     baro_error = K * mean_motor_output + bias
 *
 * But we can't directly observe baro_error in flight — the vehicle is
 * actually moving, so baro altitude changes for real reasons too.
 *
 * Solution — two stages:
 *
 * 1. Complementary Filter (CF) — isolate the baro error signal.
 *    Fuses barometer altitude with double-integrated accelerometer to
 *    produce an altitude estimate.  The CF uses a very low crossover
 *    frequency (0.05 Hz), trusting the accelerometer for fast changes
 *    and the baro for DC.  The residual (baro minus CF prediction)
 *    contains the thrust-correlated pressure error plus noise, with
 *    real vehicle motion removed.
 *
 * 2. Recursive Least Squares (RLS) — identify K from the residual.
 *    Fits the linear model: residual = K * thrust + bias, updating
 *    the estimate with each new sample.  RLS is chosen over batch
 *    least-squares because it runs online with O(1) memory, adapts
 *    to changing conditions, and provides a covariance estimate (P)
 *    used for convergence detection.  The forgetting factor (lambda)
 *    down-weights old data to track slow parameter drift.
 *
 * Convergence requires: low parameter variance (P[0][0]), acceptable
 * prediction error (absolute OR relative to explained variance),
 * sufficient thrust excitation, stable K estimate for 10s, and a 10s
 * hold period.  The relative error path allows first-flight calibration
 * (PCOEF=0) where absolute residual is high but the model explains
 * most of the variance.  Once locked, K is saved to SENS_BARO_PCOEF
 * on disarm.
 */

#pragma once

#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <matrix/math.hpp>
#include <geo/geo.h>

#include <cmath>
#include <float.h>

class BaroThrustCfRls
{
public:
	// --- RLS tuning ---
	static constexpr float RLS_LAMBDA = 0.998f;     ///< forgetting factor (1.0 = never forget, lower = adapt faster)
	static constexpr float RLS_P_INIT = 100.f;      ///< initial covariance diagonal (high = uncertain, learns fast)

	// --- Convergence criteria ---
	static constexpr float CONVERGENCE_VAR_THR = 3.f;       ///< max K variance (P[0][0]) to consider converged
	static constexpr float CONVERGENCE_ERR_THR = 0.5f;      ///< max prediction error variance (absolute) [m^2]
	static constexpr float CONVERGENCE_ERR_REL_THR = 0.4f;  ///< max error/total variance ratio (model explains >60%)
	static constexpr float CONVERGENCE_ERR_MAX_THR = 4.0f;   ///< absolute error cap for relative path [m^2]
	static constexpr float MIN_THRUST_EXCITATION = 0.05f;    ///< min thrust std dev to trust the estimate
	static constexpr float MIN_EXCITATION_TIME_S = 5.f;      ///< cumulative seconds above excitation threshold
	static constexpr float MIN_ESTIMATION_TIME_S = 30.f;     ///< min flight time before convergence allowed
	static constexpr float K_STABILITY_TIME_S = 10.f;        ///< K must be stable within threshold for this long
	static constexpr float CONVERGENCE_HOLD_TIME_S = 10.f;   ///< must stay converged this long before locking
	static constexpr float K_STABILITY_DIFF_THR = 0.5f;      ///< max |K_smoothed - K_raw| for stability [m]

	// --- Complementary filter ---
	static constexpr float CF_BANDWIDTH_HZ_DEFAULT = 0.05f; ///< default crossover frequency [Hz]
	static constexpr float ERROR_VAR_INIT = 10.f;   ///< initial prediction error variance

	/**
	 * 2-state RLS estimator: fits residual = theta[0]*thrust + theta[1]
	 * where theta[0] = K (the gain we want) and theta[1] = bias.
	 * P is the 2x2 parameter covariance matrix.
	 */
	struct RlsEstimator {
		float theta[2] {};      ///< [K, bias] parameter estimates
		float P[2][2] {};       ///< 2x2 parameter covariance
		float error_var{ERROR_VAR_INIT}; ///< exponentially-weighted prediction error variance

		void reset(float p_init);
		void update(float residual, float thrust, float dt, float lambda);
	};

	void reset();

	/**
	 * Set CF crossover frequency.  Call before first updateCf() or after reset().
	 */
	void setCfBandwidth(float hz) { _cf_bandwidth_hz = hz; }

	/**
	 * Convert body-frame specific force (including gravity, FRD) to
	 * altitude-up linear acceleration.
	 */
	static float computeAccelUp(const matrix::Vector3f &accel_body, const matrix::Quatf &attitude);

	/**
	 * Update complementary filter and return residual (baro - accel prediction).
	 */
	float updateCf(float baro_alt, float accel_up, float dt);

	/**
	 * Update the RLS estimator with the CF residual and motor thrust.
	 */
	void updateEstimator(float residual, float thrust, float dt);

	void checkConvergence(float elapsed_since_start_s, float dt);

	bool converged() const { return _converged; }
	bool convergedLocked() const { return _converged_locked; }
	float kEstimate() const { return _rls.theta[0]; }
	float kEstimateVar() const { return _rls.P[0][0]; }
	float errorVar() const { return _rls.error_var; }
	float thrustStd() const { return sqrtf(fmaxf(_thrust_var.getState(), 0.f)); }

private:
	RlsEstimator _rls{};

	// CF state: 2nd-order (position + velocity) integrator corrected by baro
	float _cf_bandwidth_hz{CF_BANDWIDTH_HZ_DEFAULT}; ///< crossover frequency [Hz]
	float _cf_alt{0.f};             ///< CF altitude estimate [m]
	float _cf_vel{0.f};             ///< CF vertical velocity estimate [m/s]
	bool _cf_initialized{false};

	// Convergence tracking
	AlphaFilter<float> _k_est_smoothed{};  ///< low-pass filtered K for stability check
	float _k_stable_elapsed_s{0.f};        ///< time K has been within stability threshold
	bool _converged{false};                ///< all convergence criteria currently met
	bool _converged_locked{false};         ///< converged and held long enough — ready to save
	float _converged_elapsed_s{0.f};       ///< accumulated hold time while converged

	float _excitation_elapsed_s{0.f};      ///< cumulative time with sufficient thrust excitation

	// Thrust excitation tracking (need variation in thrust to observe K)
	AlphaFilter<float> _thrust_mean{};     ///< low-pass mean thrust
	AlphaFilter<float> _thrust_var{};      ///< low-pass thrust variance
};
