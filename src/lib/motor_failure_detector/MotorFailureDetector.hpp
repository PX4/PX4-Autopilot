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

/**
 * @file MotorFailureDetector.hpp
 *
 * Per-motor current-residual motor-failure detector: expected current from the
 * commanded signal (linear, I = slope*u + idle), fault latched when the low-passed
 * residual holds past a trip band for the persistence time.
 *
 * Band and persistence are per direction: benign transients are much larger on the
 * overcurrent side, so one shared band makes the undercurrent side -- where a stopped motor
 * or a lost prop shows up -- inherit a limit the fault itself never reaches.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/hysteresis/hysteresis.h>
#include <mathlib/math/filter/AlphaFilter.hpp>

class MotorFailureDetector
{
public:
	static constexpr int kMaxMotors = 12;

	// Step longer than this [s] => data gap (resets filters + debounce).
	// Between the ~10 Hz stream and the 400 ms ESC-offline timeout.
	static constexpr float kMaxGap = 0.3f;

	// One model for all motors
	struct Config {
		float current_slope_a;             ///< expected current per unit thrust command [A]
		float current_idle_a;              ///< expected current at zero command [A]
		float residual_lpf_tau_s;          ///< residual LPF time constant [s]
		float overcurrent_threshold_a;     ///< trip band above expected current [A]; <= 0 => check disabled
		float overcurrent_persistence_s;   ///< time above the overcurrent band before latching [s]
		float undercurrent_threshold_a;    ///< same, below expected current; <= 0 => one symmetric band
		float undercurrent_persistence_s;  ///< same, below expected current
	};

	struct MotorStatus {
		float residual;        ///< signed residual I - I_expected [A], floored at -I_expected
		float residual_lpf;    ///< low-pass filtered residual [A]
		bool  failed;          ///< latched failure decision
		bool  excluded;        ///< not evaluated this step (reversible or NaN command)
	};

	MotorFailureDetector() { reset(); }

	void configure(const Config &cfg) { _cfg = cfg; reset(); }

	/** Reset all per-motor state (filters, debounce, latch). */
	void reset();

	/**
	 * Advance to timestamp now_us [us] (dt derived internally). NaN or reversible
	 * commands exclude a motor; a non-finite current is a held dropout (the debounce
	 * keeps running, so a fault that also drops telemetry still latches); a step >
	 * kMaxGap resets. configure() first; overcurrent_threshold_a <= 0 disables the check.
	 */
	void update(int num_motors, hrt_abstime now_us,
		    const float command[], const float current[],
		    const bool reversible[]);

	const MotorStatus &status(int i) const { return _status[i]; }
	bool anyFailed() const;
	int  firstFailed() const; ///< index of first latched-failed motor, or -1

private:
	Config                _cfg{};
	MotorStatus           _status[kMaxMotors] {};
	AlphaFilter<float>    _residual_lpf[kMaxMotors];
	systemlib::Hysteresis _hyst_over[kMaxMotors];
	systemlib::Hysteresis _hyst_under[kMaxMotors];   // only used in the asymmetric case
	hrt_abstime           _last_us{0};
};
