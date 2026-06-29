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

void MotorFailureDetector::reset()
{
	const hrt_abstime persistence_us = (hrt_abstime)(_cfg.persistence_s * 1e6f);

	for (int i = 0; i < kMaxMotors; ++i) {
		_status[i] = MotorStatus{};
		_residual_lpf[i].reset(0.f);
		_hyst[i] = systemlib::Hysteresis();                        // clear state + pending request
		_hyst[i].set_hysteresis_time_from(false, persistence_us);  // rising debounce == persistence
	}

	_last_us = 0;
}

void MotorFailureDetector::update(int num_motors, hrt_abstime now_us,
				  const float command[], const float current[],
				  const bool reversible[], const bool enabled[])
{
	const float dt = static_cast<float>(now_us - _last_us) * 1e-6f;
	_last_us = now_us;

	// Data gap -> reset filters + debounce and skip (latched failures kept). Also absorbs the first
	// call (_last_us inits 0, so dt is the whole timestamp) and a backwards clock (uint64 subtraction
	// wraps to a huge dt). A duplicate timestamp (dt == 0) is harmless: the LPF alpha is then 0.
	if (dt > kMaxGap) {
		for (int i = 0; i < num_motors; ++i) {
			_status[i].residual_lpf = 0.f;
			_residual_lpf[i].reset(0.f);
			_hyst[i].set_state_and_update(false, now_us);  // cancel any pending trip across the gap
		}

		return;
	}

	// Non-positive threshold/persistence (e.g. a default Config) = monitor-only: residuals
	// are still computed but a failure is never latched.
	const bool detection_enabled = (_cfg.threshold_a > 0.f) && (_cfg.persistence_s > 0.f);

	for (int i = 0; i < num_motors; ++i) {
		MotorStatus &s = _status[i];

		const float u = command[i];

		// Cannot evaluate this motor -> exclude and reset (re-entry starts clean; latch kept).
		const bool excluded = !enabled[i]
				      || std::isnan(u)
				      || reversible[i];

		s.excluded = excluded;

		if (excluded) {
			s.residual_lpf = 0.f;
			_residual_lpf[i].reset(0.f);
			_hyst[i].set_state_and_update(false, now_us);  // cancel pending; keep any latch
			continue;
		}

		// Expected current is command-only, so it is valid even on a telemetry dropout and also
		// sets the throttle-relative part of the threshold below.
		const float i_expected = _cfg.model_a * u * u + _cfg.model_b * u + _cfg.model_c;

		if (std::isfinite(current[i])) {
			s.residual = current[i] - i_expected;
			_residual_lpf[i].setParameters(dt, _cfg.residual_lpf_tau_s);
			_residual_lpf[i].update(s.residual);
			s.residual_lpf = _residual_lpf[i].getState();
		}

		// else: telemetry dropout -- HOLD residual / residual_lpf; the persistence test below
		// keeps running on the held value so a fault that also drops telemetry still latches.

		// Throttle-relative trip band (threshold_rel = 0 => flat); tracks the floor's growth with throttle.
		const float threshold = _cfg.threshold_a + _cfg.threshold_rel * i_expected;

		// Debounce + latch: the over-threshold condition must hold for persistence_s (Hysteresis);
		// the failure decision is then latched (sticky -- Hysteresis itself is not).
		const bool over = detection_enabled && (std::fabs(s.residual_lpf) >= threshold);
		_hyst[i].set_state_and_update(over, now_us);

		if (_hyst[i].get_state()) {
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
