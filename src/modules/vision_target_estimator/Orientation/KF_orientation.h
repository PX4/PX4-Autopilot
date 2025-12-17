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

#include "../VTEOosm.h"
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
	friend class OOSMManager<KF_orientation, State::size, EmptyInput>;

	KF_orientation() = default;
	~KF_orientation() = default;

	struct ScalarMeas {
		uint64_t time_us;
		float val;
		float unc;
		matrix::Vector<float, State::size> H;
	};

	void predict(float dt);

	// Primary Fusion Interface (history-consistent OOSM)
	// Projected correction OOSM approximation:
	// - Most accurate with bounded delays and when fusions are processed in timestamp order (oldest -> newest).
	// - Does not replay intermediate measurement updates (unlike EKF2's delayed fusion horizon).
	// Returns FusionResult describing the result for logging.
	FusionResult fuseScalarAtTime(const ScalarMeas &meas, uint64_t now_us, float nis_threshold);

	// History Management
	void pushHistory(const uint64_t time_us);
	void resetHistory();

	// State
	void setState(const matrix::Vector<float, State::size> &state) { _state = state; }
	void setStateCovarianceDiag(const matrix::Vector<float, State::size> &var)
	{
		const matrix::SquareMatrix<float, State::size> var_mat = diag(var);
		_state_covariance = var_mat;
	};

	const matrix::Vector<float, State::size> &getState() const { return _state; }
	matrix::Vector<float, State::size> getStateCovarianceDiag() const { return _state_covariance.diag(); }

private:
	void getTransitionMatrix(float dt, matrix::SquareMatrix<float, State::size> &phi) const;

	void predictState(float dt, const EmptyInput &input,
			  const matrix::Vector<float, State::size> &x_in,
			  const matrix::SquareMatrix<float, State::size> &P_in,
			  matrix::Vector<float, State::size> &x_out,
			  matrix::SquareMatrix<float, State::size> &P_out);

	void computeInnovation(const matrix::Vector<float, State::size> &state,
			       const matrix::SquareMatrix<float, State::size> &cov, const ScalarMeas &meas,
			       float &innov, float &innov_var) const;

	void applyCorrection(matrix::Vector<float, State::size> &state,
			     matrix::SquareMatrix<float, State::size> &cov,
			     const matrix::Vector<float, State::size> &K,
			     float innov, float S);

	matrix::Vector<float, State::size> _state;
	matrix::SquareMatrix<float, State::size> _state_covariance;

	// OOSM history buffer:
	// 0.5s window @ 50Hz predict rate = 25 samples.
	// Note that the 0.5s window is enforced with kOosmMaxTimeUs = 500_ms.
	OOSMManager<KF_orientation, State::size, EmptyInput> _oosm;
};
} // namespace vision_target_estimator
