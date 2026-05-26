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

/**
 * @file KF_position.h
 * @brief Per-axis Kalman Filter for VTEST position states.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include <matrix/math.hpp>
#include <vtest_derivation/generated/state.h>

#include "../VTEOosm.h"
#include "../common.h"

namespace vision_target_estimator
{

class KF_position
{
public:
	static constexpr size_t kStateSize = vtest::State::size;
	using VectorState = matrix::Vector<float, kStateSize>;
	using SquareMatrixState = matrix::SquareMatrix<float, kStateSize>;

	friend class OOSMManager<KF_position, kStateSize, float>;

	KF_position() = default;
	~KF_position() = default;
	static constexpr float kMinVar = 1e-9f;

	struct ScalarMeas {
		uint64_t time_us;
		float val;
		float unc;
		VectorState H;
	};

	void predict(float dt, float acc_uav);

	// Primary Fusion Interface (history-consistent OOSM)
	// Projected correction OOSM approximation:
	// - Most accurate with bounded delays and when fusions are processed in timestamp order (oldest -> newest).
	// - Does not replay intermediate measurement updates (unlike EKF2's delayed fusion horizon).
	// Returns FusionResult describing the result for logging.
	FusionResult fuseScalarAtTime(const ScalarMeas &meas, uint64_t now_us, float nis_threshold);

	// History Management
	void pushHistory(const uint64_t time_us);
	void resetHistory();

	// Setters / Getters
	void setState(const VectorState &state) { _state = state; }
	void setStateCovarianceDiag(const VectorState &var)
	{
		const SquareMatrixState var_mat = diag(var);
		_state_covariance = var_mat;
	};

	const VectorState &getState() const { return _state; }
	const SquareMatrixState &getStateCovariance() const { return _state_covariance; }
	VectorState getStateCovarianceDiag() const { return _state_covariance.diag(); }

	void setInputVar(float var) { _input_var = var; }
	void setBiasVar(float var) { _bias_var = var; }
	void setTargetAccVar(float var) { _acc_var = var; }

private:
	SquareMatrixState getTransitionMatrix(float dt) const;

	void predictState(float dt, float acc,
			  const VectorState &x_in,
			  const SquareMatrixState &P_in,
			  VectorState &x_out,
			  SquareMatrixState &P_out);

	void computeInnovation(const VectorState &state,
			       const SquareMatrixState &cov, const ScalarMeas &meas,
			       float &innov, float &innov_var) const;

	void applyCorrection(VectorState &state,
			     SquareMatrixState &cov,
			     const VectorState &K,
			     float innov, float S);

	VectorState _state{};
	SquareMatrixState _state_covariance{};

	float _bias_var{0.f};  // bias random-walk spectral density (m^2 / s)
	float _acc_var{0.f};   // target acceleration random-walk spectral density ((m/s^2)^2 / s)
	float _input_var{0.f}; // white UAV acceleration noise PSD (m^2 / s^3, or (m/s^2)^2 / Hz)

	float _last_acc{0.f}; // last UAV acceleration input (also used as OOSM fallback input)

	// OOSM history buffer:
	// 0.5 s window @ 50 Hz predict rate = 25 samples.
	// Note that the 0.5 s window is enforced with kOosmMaxTimeUs = 500_ms.
	OOSMManager<KF_position, kStateSize, float, kOosmHistorySize> _oosm;
};
} // namespace vision_target_estimator
