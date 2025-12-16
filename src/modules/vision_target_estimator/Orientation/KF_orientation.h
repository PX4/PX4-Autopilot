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
 * @file KF_orientation.h
 * @brief Filter to estimate the orientation of static and moving targets. State: [yaw, yaw_rate]
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <matrix/Matrix.hpp>
#include <matrix/Vector.hpp>

#pragma once

#include "../common.h"

namespace vision_target_estimator
{

namespace State
{
static constexpr uint8_t yaw{0};
static constexpr uint8_t yaw_rate{1};
static constexpr uint8_t size{2};
};

class KF_orientation
{
public:
	KF_orientation() = default;
	~KF_orientation() = default;

	struct ScalarMeas {
		uint64_t time_us;
		float val;
		float unc;
		matrix::Vector<float, State::size> H;
	};

	void predictState(float dt);
	void predictCov(float dt);

	// Primary Fusion Interface (history-consistent OOSM)
	// Projected correction OOSM approximation:
	// - Most accurate with bounded delays and when fusions are processed in timestamp order (oldest -> newest).
	// - Does not replay intermediate measurement updates (unlike EKF2's delayed fusion horizon).
	// Returns FusionResult describing the result for logging.
	FusionResult fuseScalarAtTime(const ScalarMeas &meas, uint64_t now_us, float nis_threshold);

	// History Management
	void pushHistory(const uint64_t time_us);
	void resetHistory();

	// Backwards state prediciton
	void setH(const matrix::Vector<float, State::size> &h_meas) { _meas_matrix_row_vect = h_meas; }
	void setState(const matrix::Vector<float, State::size> &state) { _state = state; }
	void setStateCovarianceDiag(const matrix::Vector<float, State::size> &var)
	{
		const matrix::SquareMatrix<float, State::size> var_mat = diag(var);
		_state_covariance = var_mat;
	};

	const matrix::Vector<float, State::size> &getState() const { return _state; }
	const matrix::SquareMatrix<float, State::size> &getStateCovariance() const { return _state_covariance; }
	matrix::Vector<float, State::size> getStateCovarianceDiag() const { return _state_covariance.diag(); }

	void setNisThreshold(float nis_threshold) { _nis_threshold = nis_threshold; }

	float getTestRatio() const
	{
		if (fabsf(_innov_cov) < 1e-6f || _nis_threshold <= 0.f) {
			return -1.f;
		}

		const float nis = math::sq(_innov) / _innov_cov;
		return nis / _nis_threshold;
	}

private:
	struct StateSample {
		uint64_t time_us{0};
		matrix::Vector<float, State::size> state{};
		matrix::SquareMatrix<float, State::size> cov{};
	};

	void applyCorrection(matrix::Vector<float, State::size> &state,
			     matrix::SquareMatrix<float, State::size> &cov,
			     const matrix::Vector<float, State::size> &K,
			     float innov, float S);

	bool computeFusionGain(const matrix::Vector<float, State::size> &state,
			       const matrix::SquareMatrix<float, State::size> &cov, const ScalarMeas &meas, float nis_threshold,
			       FusionResult &out_res,
			       matrix::Vector<float, State::size> &out_K);

	matrix::SquareMatrix<float, State::size> getTransitionMatrix(float dt)
	{
		float data[State::size * State::size] = {
			1, dt,
			0, 1
		};

		return matrix::SquareMatrix<float, State::size>(data);
	}

	matrix::Vector<float, State::size> _state;
	matrix::Vector<float, State::size> _sync_state;
	matrix::Vector<float, State::size> _meas_matrix_row_vect;
	matrix::SquareMatrix<float, State::size> _state_covariance;
	float _innov{0.0f}; // residual of last measurement update
	float _innov_cov{0.0f}; // innovation covariance of last measurement update
	float _nis_threshold{0.0f}; // Normalized innovation squared test threshold

	// History Buffer (fixed-size)
	// 0.5s window @ 50Hz predict rate = 25 samples.
	// Note that the 0.5s window is enforced with kOosmMaxTimeUs = 500_ms
	static constexpr uint8_t kHistorySize = 25;
	StateSample _history[kHistorySize] {};
	uint8_t _history_head{0};
	bool _history_valid{false};
};
} // namespace vision_target_estimator
