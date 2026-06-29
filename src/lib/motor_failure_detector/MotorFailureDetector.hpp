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
 * commanded signal (linear, I = slope*u + idle), fault latched when |LPF(residual)|
 * holds above the threshold for the persistence time.
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
		float model_b;                  ///< current-vs-command slope [A]
		float model_c;                  ///< idle current offset [A]
		float residual_lpf_tau_s;       ///< residual LPF time constant [s]
		float threshold_a;              ///< trip threshold [A]; <= 0 => check disabled
		float persistence_s;            ///< time over threshold before latching [s]
	};

	struct MotorStatus {
		float residual;        ///< raw signed residual I - I_expected [A]
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
	 * kMaxGap resets. configure() first; threshold_a <= 0 disables the check.
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
	systemlib::Hysteresis _hyst[kMaxMotors];
	hrt_abstime           _last_us{0};
};
