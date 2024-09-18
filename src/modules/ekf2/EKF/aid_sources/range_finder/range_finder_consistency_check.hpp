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

class RangeFinderConsistencyCheck final
{
public:
	RangeFinderConsistencyCheck() = default;
	~RangeFinderConsistencyCheck() = default;

	float getTestRatioLpf() const { return _test_ratio_lpf.getState(); }
	float getInnov() const { return _innov; }
	float getInnovVar() const { return _innov_var; }

	bool isKinematicallyConsistent() const { return _is_kinematically_consistent; }
	void UpdateMiniKF(float z, float z_var, float vz, float vz_var, float dist_bottom, float dist_bottom_var,
			  uint64_t time_us);
	void initMiniKF(float p1, float p2, float x1, float x2);
	void stopMiniKF() { _initialized = false; }
	bool isRunning() { return _initialized; }

	matrix::SquareMatrix<float, 2> _R{};
	matrix::SquareMatrix<float, 2> _P{};
	matrix::SquareMatrix<float, 2> _A{};
	matrix::SquareMatrix<float, 2> _H{};
	matrix::Vector2f _x{};
	bool _initialized{false};

private:
	float _innov{};
	float _innov_var{};
	uint64_t _time_last_update_us{0};
	float _dist_bottom_prev{};
	AlphaFilter<float> _test_ratio_lpf{0.3}; // average signed test ratio used to detect a bias in the data
	float _gate{1.f};
	bool _is_kinematically_consistent{true};
	int _sample_count{0};
};

#endif // !EKF_RANGE_FINDER_CONSISTENCY_CHECK_HPP
