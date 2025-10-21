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

#include "gain_compression.hpp"

using matrix::Vector3f;
using namespace time_literals;

GainCompression3d::GainCompression3d(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
	_gain_compression_pub.advertise();
}

void GainCompression3d::reset()
{
	for (unsigned i = 0; i < 3; i++) {
		_compression_gains[i].reset();
	}
}

void GainCompression3d::updateParams()
{
	ModuleParams::updateParams();

	for (unsigned i = 0; i < 3; i++) {
		_compression_gains[i].setCompressionGainMin(_param_fw_gc_gain_min.get());
	}
}

void GainCompression3d::update(const Vector3f &input, const float dt)
{
	if (!_param_fw_gc_en.get()) {
		reset();
		_gains.setOne();
		return;
	}

	Vector3f hpf;
	Vector3f lpf;
	const float sample_freq = 1.f / math::constrain(dt, 1e-3f, 100e-3f);

	for (unsigned i = 0; i < 3; i++) {
		_compression_gains[i].setLpfCutoffFrequency(sample_freq, _kLpfCutoffFrequency);
		_compression_gains[i].setHpfCutoffFrequency(sample_freq, _kHpfCutoffFrequency);

		_gains(i) = _compression_gains[i].update(input(i), dt);

		hpf(i) = _compression_gains[i].getSpectralDamperHpf();
		lpf(i) = _compression_gains[i].getSpectralDamperLpf();
	}

	const hrt_abstime now = hrt_absolute_time();

	if ((now - _time_last_publication) > 100_ms) {
		gain_compression_s msg;
		msg.timestamp = now;
		_gains.copyTo(msg.compression_gains);
		hpf.copyTo(msg.spectral_damper_hpf);
		lpf.copyTo(msg.spectral_damper_out);
		input.copyTo(msg.input);
		_gain_compression_pub.publish(msg);

		_time_last_publication = now;
	}
}

float GainCompression::update(const float input, const float dt)
{
	_hpf = _alpha_hpf * _hpf + _alpha_hpf * (input - _input_prev);
	_lpf.update(_hpf * _hpf);

	_input_prev = input;

	const float ka = fmaxf(_compression_gain - _compression_gain_min, 0.f);
	const float spectral_damping = -_kSpectralDamperGain * ka * _lpf.getState();

	const float leakage = _kLeakageGain * (1.f - _compression_gain);

	const float ka_dot = spectral_damping + leakage;
	_compression_gain = math::constrain(_compression_gain + ka_dot * dt, _compression_gain_min, 1.f);

	return _compression_gain;
}
