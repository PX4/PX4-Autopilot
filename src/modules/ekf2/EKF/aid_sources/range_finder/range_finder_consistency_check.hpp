/****************************************************************************
 *
 *   Copyright (c) 2022 PX4. All rights reserved.
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
 * @file range_finder_consistency_check.hpp
 * @brief Compute statistical tests of the range finder data
 * using the estimated velocity as a reference in order to detect sensor faults
 */

#ifndef EKF_RANGE_FINDER_CONSISTENCY_CHECK_HPP
#define EKF_RANGE_FINDER_CONSISTENCY_CHECK_HPP

#include <mathlib/math/filter/AlphaFilter.hpp>
#include <ekf_derivation/generated/state.h>


class RangeFinderConsistencyCheck final
{
public:
	RangeFinderConsistencyCheck() = default;
	~RangeFinderConsistencyCheck() = default;

	enum class KinematicState : int {
		INCONSISTENT = 0,
		CONSISTENT = 1,
		UNKNOWN = 2
	};

	float getTestRatioLpf() const { return _initialized ? _test_ratio_lpf.getState() : 0.f; }
	float getInnov() const { return _initialized ? _innov : 0.f; }
	float getInnovVar() const { return _initialized ? _innov_var : 0.f; }
	bool isKinematicallyConsistent() const { return _state == KinematicState::CONSISTENT; }
	bool isNotKinematicallyInconsistent() const { return _state != KinematicState::INCONSISTENT; }
	void setGate(const float gate) { _gate = gate; }
	void run(const float &z, const float &vz, const matrix::SquareMatrix<float, estimator::State::size> &P,
		 const float &dist_bottom, const float &dist_bottom_var, uint64_t time_us);
	void reset()
	{
		_state = (_initialized && _state == KinematicState::CONSISTENT) ? KinematicState::UNKNOWN : _state;
		_initialized = false;
	}

	uint8_t current_posD_reset_count{0};
	float terrain_process_noise{0.0f};
	bool horizontal_motion{false};

private:

	void update(const float &z, const float &z_var, const float &vz, const float &vz_var, const float &dist_bottom,
		    const float &dist_bottom_var, const uint64_t &time_us);
	void init(const float &z, const float &z_var, const float &dist_bottom, const float &dist_bottom_var);
	matrix::SquareMatrix<float, 2> _P{};
	matrix::SquareMatrix<float, 2> _H{};
	matrix::Vector2f _x{};
	bool _initialized{false};
	float _innov{0.f};
	float _innov_var{0.f};
	uint64_t _time_last_update_us{0};
	AlphaFilter<float> _test_ratio_lpf{};
	float _gate{1.f};
	KinematicState _state{KinematicState::UNKNOWN};
	float _t_since_first_sample{0.f};
	uint8_t _last_posD_reset_count{0};
};

namespace RangeFilter
{
static constexpr estimator::IdxDof z{0, 1};
static constexpr estimator::IdxDof terrain{1, 1};
static constexpr uint8_t size{2};
};

#endif // !EKF_RANGE_FINDER_CONSISTENCY_CHECK_HPP
