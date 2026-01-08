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

/* Description:
 * This class produces a multiplicative gain that can be used on the output of a controller
 * to dynamically reduce the loop gain when oscillations are detected.
 *
 * Algorithm based on
 * Orr, Jeb, and Tannen Van Zwieten. "Robust, practical adaptive control for launch vehicles."
 * AIAA Guidance, Navigation, and Control Conference. 2012.
 * https://ntrs.nasa.gov/api/citations/20120015662/downloads/20120015662.pdf
 */

#pragma once

// PX4 includes
#include <px4_platform_common/module_params.h>

// Libraries
#include <math.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/topics/gain_compression.h>

class GainCompression
{
public:
	void reset() { _compression_gain = 1.f; }
	float update(float input, float dt);

	void setLpfCutoffFrequency(float sample_freq, float cutoff)
	{
		_lpf.setCutoffFreq(sample_freq, cutoff);
	}
	void setHpfCutoffFrequency(float sample_freq, float cutoff) { _alpha_hpf = sample_freq / (sample_freq + 2.f * M_PI_F * cutoff); }

	float getSpectralDamperHpf() const { return _hpf * _hpf; }
	float getSpectralDamperLpf() const { return _lpf.getState(); }
	void setCompressionGainMin(float gain_min) { _compression_gain_min = gain_min; }

private:
	float _compression_gain{1.f};
	float _compression_gain_min{0.3f};

	float _alpha_hpf{0.f};
	float _hpf{0.f};

	float _input_prev{0.f};

	AlphaFilter<float> _lpf;

	static constexpr float _kSpectralDamperGain{200.f}; // tuned based on flight test data to obtain a quick response
	static constexpr float _kLeakageGain{0.1f}; // 1/time_constant, slow enough to not interfere with the controller

};

class GainCompression3d : public ModuleParams
{
public:
	GainCompression3d(ModuleParams *parent);
	~GainCompression3d() = default;

	void reset();
	void update(const matrix::Vector3f &input, float dt);
	const matrix::Vector3f &getGains() const { return _gains; };


protected:
	void updateParams() override;

private:
	// uORB publications
	uORB::Publication<gain_compression_s> _gain_compression_pub{ORB_ID(gain_compression)};

	GainCompression _compression_gains[3];
	matrix::Vector3f _gains{1.f, 1.f, 1.f};

	hrt_abstime _time_last_publication{0};

	static constexpr float _kLpfCutoffFrequency{5.f}; // Just above the control bandwidth of most UAVs
	static constexpr float _kHpfCutoffFrequency{2.f * _kLpfCutoffFrequency}; // 1 Octave above LPF cutoff, as recommended by the reference paper

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::FW_GC_EN>) _param_fw_gc_en,
		(ParamFloat<px4::params::FW_GC_GAIN_MIN>) _param_fw_gc_gain_min
	)
};
