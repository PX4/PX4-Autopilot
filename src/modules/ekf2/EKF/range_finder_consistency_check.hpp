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

#pragma once

#include <mathlib/math/filter/AlphaFilter.hpp>

class RangeFinderConsistencyCheck final
{
public:
	RangeFinderConsistencyCheck() = default;
	~RangeFinderConsistencyCheck() = default;

	void update(float dist_bottom, float dist_bottom_var, float vz, float vz_var, uint64_t time_us);

	void setGate(float gate) { _gate = gate; }

	float getTestRatio() const { return _test_ratio; }
	float getSignedTestRatioLpf() const { return _signed_test_ratio_lpf.getState(); }
	float getInnov() const { return _innov; }
	float getInnovVar() const { return _innov_var; }
	bool isKinematicallyConsistent() const { return _is_kinematically_consistent; }

private:
	void updateConsistency(float vz, uint64_t time_us);

	uint64_t _time_last_update_us{};
	float _dist_bottom_prev{};

	float _test_ratio{};
	AlphaFilter<float> _signed_test_ratio_lpf{}; // average signed test ratio used to detect a bias in the data
	float _gate{.2f};
	float _innov{};
	float _innov_var{};

	bool _is_kinematically_consistent{true};
	uint64_t _time_last_inconsistent_us{};

	static constexpr float _signed_test_ratio_tau = 2.f;

	static constexpr float _min_vz_for_valid_consistency = .5f;
	static constexpr uint64_t _consistency_hyst_time_us = 1e6;
};
