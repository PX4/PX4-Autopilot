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

#include "MotorFailureDetector.hpp"

#include <cmath>

using namespace time_literals;

void MotorFailureDetector::configure(const Config &cfg)
{
	_cfg = cfg;

	// An unset undercurrent band mirrors the overcurrent one, so update() only ever runs two bands.
	if (_cfg.undercurrent_threshold_a <= 0.f) {
		_cfg.undercurrent_threshold_a = _cfg.overcurrent_threshold_a;
		_cfg.undercurrent_persistence_s = _cfg.overcurrent_persistence_s;
	}

	reset();
}

void MotorFailureDetector::reset()
{
	const hrt_abstime overcurrent_persistence_us = (hrt_abstime)(_cfg.overcurrent_persistence_s * 1_s);
	const hrt_abstime undercurrent_persistence_us = (hrt_abstime)(fmaxf(_cfg.undercurrent_persistence_s, 0.f) * 1_s);

	for (int i = 0; i < kMaxMotors; ++i) {
		_status[i] = MotorStatus{};
		_residual_lpf[i].reset(0.f);
		_hyst_over[i] = systemlib::Hysteresis();
		_hyst_over[i].set_hysteresis_time_from(false, overcurrent_persistence_us);  // false = rising-edge debounce time
		_hyst_under[i] = systemlib::Hysteresis();
		_hyst_under[i].set_hysteresis_time_from(false, undercurrent_persistence_us);
	}

	_last_us = 0;
}

void MotorFailureDetector::update(int num_motors, hrt_abstime now_us,
				  const float command[], const float current[],
				  const bool reversible[])
{
	// disable the current-residual check
	if (_cfg.overcurrent_threshold_a <= 0.f) {
		return;
	}

	const float dt = static_cast<float>(now_us - _last_us) * 1e-6f;
	_last_us = now_us;

	// dt > kMaxGap (also the first call) is a gap: reset filters + debounce, skip (latch kept).
	if (dt > kMaxGap) {
		for (int i = 0; i < num_motors; ++i) {
			_status[i].residual_lpf = 0.f;
			_residual_lpf[i].reset(0.f);
			_hyst_over[i].set_state_and_update(false, now_us);  // cancel any pending trip across the gap
			_hyst_under[i].set_state_and_update(false, now_us);
		}

		return;
	}

	for (int i = 0; i < num_motors; ++i) {
		MotorStatus &s = _status[i];
		const float u = command[i];

		// Cannot evaluate -> exclude and reset (re-entry starts clean; latch kept).
		s.excluded = std::isnan(u) || reversible[i];

		if (s.excluded) {
			s.residual_lpf = 0.f;
			_residual_lpf[i].reset(0.f);
			_hyst_over[i].set_state_and_update(false, now_us);  // cancel pending; keep any latch
			_hyst_under[i].set_state_and_update(false, now_us);
			continue;
		}

		// Command-only, so valid even on a dropout.
		const float i_expected = _cfg.current_slope_a * u + _cfg.current_idle_a;

		if (std::isfinite(current[i])) {
			// A motor cannot draw less than 0 A, so a residual below -I_exp is regeneration, not a fault.
			s.residual = fmaxf(current[i] - i_expected, -i_expected);
			_residual_lpf[i].setParameters(dt, _cfg.residual_lpf_tau_s);
			_residual_lpf[i].update(s.residual);
			s.residual_lpf = _residual_lpf[i].getState();
		}

		// else: dropout -- residual held; the debounce keeps running on it (fault + dropout still latches).

		// Each band must hold for its persistence time (Hysteresis); the decision is then latched.
		_hyst_over[i].set_state_and_update(s.residual_lpf >= _cfg.overcurrent_threshold_a, now_us);
		_hyst_under[i].set_state_and_update(-s.residual_lpf >= _cfg.undercurrent_threshold_a, now_us);

		if (_hyst_over[i].get_state() || _hyst_under[i].get_state()) {
			s.failed = true;
		}
	}
}

bool MotorFailureDetector::anyFailed() const
{
	for (int i = 0; i < kMaxMotors; ++i) {
		if (_status[i].failed) {
			return true;
		}
	}

	return false;
}

int MotorFailureDetector::firstFailed() const
{
	for (int i = 0; i < kMaxMotors; ++i) {
		if (_status[i].failed) {
			return i;
		}
	}

	return -1;
}
