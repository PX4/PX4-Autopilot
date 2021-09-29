/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file system_identification.hpp
 *
 * @author Mathieu Bresciani <mathieu@auterion.com>
 */

#pragma once

#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <px4_platform_common/defines.h>

#include "arx_rls.hpp"

class SystemIdentification final
{
public:
	SystemIdentification() = default;
	~SystemIdentification() = default;

	void reset(const matrix::Vector<float, 5> &id_state_init = {});
	void update(float u, float y); // update filters and model
	void update(); // update model only (to be called after updateFilters)
	void updateFilters(float u, float y);
	bool areFiltersInitialized() const { return _are_filters_initialized; }
	void updateFitness();
	const matrix::Vector<float, 5> &getCoefficients() const { return _rls.getCoefficients(); }
	const matrix::Vector<float, 5> getVariances() const { return _rls.getVariances(); }
	const matrix::Vector<float, 5> &getDiffEstimate() const { return _rls.getDiffEstimate(); }
	float getFitness() const { return _fitness_lpf.getState(); }
	float getInnovation() const { return _rls.getInnovation(); }

	void setLpfCutoffFrequency(float sample_freq, float cutoff)
	{
		_u_lpf.set_cutoff_frequency(sample_freq, cutoff);
		_y_lpf.set_cutoff_frequency(sample_freq, cutoff);
	}
	void setHpfCutoffFrequency(float sample_freq, float cutoff) { _alpha_hpf = sample_freq / (sample_freq + 2.f * M_PI_F * cutoff); }

	void setForgettingFactor(float time_constant, float dt) { _rls.setForgettingFactor(time_constant, dt); }
	void setFitnessLpfTimeConstant(float time_constant, float dt)
	{
		_fitness_lpf.setParameters(dt, time_constant);
		_dt = dt;
	}

	float getFilteredInputData() const { return _u_hpf; }
	float getFilteredOutputData() const { return _y_hpf; }

private:
	ArxRls<2, 2, 1> _rls;
	math::LowPassFilter2p<float> _u_lpf{400.f, 30.f};
	math::LowPassFilter2p<float> _y_lpf{400.f, 30.f};

	//TODO: replace by HighPassFilter class
	float _alpha_hpf{0.f};
	float _u_hpf{0.f};
	float _y_hpf{0.f};

	float _u_prev{0.f};
	float _y_prev{0.f};

	bool _are_filters_initialized{false};

	AlphaFilter<float> _fitness_lpf;
	float _dt{0.1f};
};
