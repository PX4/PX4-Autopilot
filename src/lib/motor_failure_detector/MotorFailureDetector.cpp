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

	for (int i = 0; i < kMaxMotors; ++i) {
		_hyst_over[i].set_hysteresis_time_from(false, _cfg.overcurrent_trigger_time_s * 1_s); // false = rising-edge debounce time
		_hyst_under[i].set_hysteresis_time_from(false, _cfg.undercurrent_trigger_time_s * 1_s);
	}
}

void MotorFailureDetector::reset()
{
	for (int i = 0; i < kMaxMotors; ++i) {
		_failure[i] = Failure::None;
		_residual_lpf[i].reset(0.f);
		_hyst_over[i].set_state_and_update(false, 0);  // no falling-edge delay: clears immediately, keeps the debounce times
		_hyst_under[i].set_state_and_update(false, 0);
		_last_us[i] = 0;
	}
}

void MotorFailureDetector::update(int i, hrt_abstime now_us, float command, float current)
{
	if (!validIndex(i)) {
		return;
	}

	// Same sample as last time (the ESC has not reported since): nothing new to evaluate.
	if (now_us == _last_us[i]) {
		return;
	}

	const float dt = static_cast<float>(now_us - _last_us[i]) * 1e-6f;
	_last_us[i] = now_us;

	// Cannot evaluate (non-finite command or current, or dt > kMaxGap incl. the first call)
	// -> reset filter + debounce so re-entry starts clean; the latch is kept.
	if (!std::isfinite(command) || !std::isfinite(current) || dt > kMaxGap) {
		_residual_lpf[i].reset(0.f);
		_hyst_over[i].set_state_and_update(false, now_us); // cancel any pending trip
		_hyst_under[i].set_state_and_update(false, now_us);
		return;
	}

	const float i_expected = _cfg.current_slope_a * command + _cfg.current_idle_a;

	// A motor cannot draw negative current: regeneration counts as 0 A, not as a fault.
	_residual_lpf[i].setParameters(dt, kResidualLpfTau);
	_residual_lpf[i].update(fmaxf(current, 0.f) - i_expected);

	// Each band (a threshold <= 0 disables it) must hold for its trigger time (Hysteresis); the decision is then latched.
	const float residual_lpf = _residual_lpf[i].getState();
	const bool over = (_cfg.overcurrent_threshold_a > 0.f) && (residual_lpf >= _cfg.overcurrent_threshold_a);
	const bool under = (_cfg.undercurrent_threshold_a > 0.f) && (-residual_lpf >= _cfg.undercurrent_threshold_a);
	_hyst_over[i].set_state_and_update(over, now_us);
	_hyst_under[i].set_state_and_update(under, now_us);

	if (_failure[i] == Failure::None) {
		if (_hyst_under[i].get_state()) {
			_failure[i] = Failure::Undercurrent;

		} else if (_hyst_over[i].get_state()) {
			_failure[i] = Failure::Overcurrent;
		}
	}
}
