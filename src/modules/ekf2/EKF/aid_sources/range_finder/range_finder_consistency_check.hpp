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

	float getTestRatioLpf() const { return _test_ratio_lpf.getState(); }
	float getInnov() const { return _innov; }
	float getInnovVar() const { return _innov_var; }

	bool isKinematicallyConsistent() const { return _state == KinematicState::CONSISTENT; }
	bool isNotKinematicallyInconsistent() const { return _state != KinematicState::INCONSISTENT || _fixed_wing; }
	void UpdateMiniKF(float z, float z_var, float vz, float vz_var, float dist_bottom, float dist_bottom_var,
			  uint64_t time_us);
	void initMiniKF(float var_z, float var_terrain, float z, float dist_bottom);
	void stopMiniKF()
	{
		if (_initialized) {
			if (_state == KinematicState::CONSISTENT) {
				_state = KinematicState::UNKNOWN;
			}

			_initialized = false;
		}
	}
	int getConsistencyState() const { return static_cast<int>(_state); }

	bool isRunning() { return _initialized; }

	void setNotMoving()
	{
		if (_state == KinematicState::CONSISTENT) {
			_state = KinematicState::UNKNOWN;
		}
	}

	void setFixedWing(bool is_fixed_wing, float gate)
	{
		_fixed_wing = is_fixed_wing;
		_gate = gate;
	}

	void run(const float z, const float vz, const matrix::SquareMatrix<float, estimator::State::size> P,
		 const float dist_bottom, const float dist_bottom_var, uint64_t time_us);

	uint8_t current_posD_reset_count{0};

private:

	matrix::SquareMatrix<float, 2> _R{};
	matrix::SquareMatrix<float, 2> _P{};
	matrix::SquareMatrix<float, 2> _A{};
	matrix::SquareMatrix<float, 2> _H{};
	matrix::Vector2f _x{};
	bool _initialized{false};
	float _innov{};
	float _innov_var{};
	uint64_t _time_last_update_us{0};
	float _dist_bottom_prev{};
	AlphaFilter<float> _test_ratio_lpf{0.3};
	float _gate{1.f};
	int _sample_count{0};
	KinematicState _state{KinematicState::UNKNOWN};
	int _min_nr_of_samples{0};
	bool _fixed_wing{false};
	uint8_t _last_posD_reset_count{0};
};

#endif // !EKF_RANGE_FINDER_CONSISTENCY_CHECK_HPP
