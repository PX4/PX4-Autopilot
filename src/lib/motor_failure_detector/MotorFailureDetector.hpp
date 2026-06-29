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
 * Per-motor current-residual motor-failure detector. Expected current is
 * predicted from the commanded signal u (not measured RPM, which the fault
 * corrupts): I_expected = a*u^2 + b*u + c. A fault latches when the low-pass-filtered
 * residual |LPF(I - I_expected)| stays >= threshold for >= persistence. The
 * residual filter is a mathlib AlphaFilter and the persistence debounce is a
 * systemlib::Hysteresis.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/hysteresis/hysteresis.h>
#include <mathlib/math/filter/AlphaFilter.hpp>

class MotorFailureDetector
{
public:
	static constexpr int kMaxMotors = 12;

	// A step longer than this [s] is a data gap -> filters + debounce reset. It is an outage detector
	// keyed off the sample rate, NOT the filter tuning: well above normal ESC sample jitter (~5 ms at
	// 184 Hz) so jitter never trips it, yet short enough to catch a real telemetry outage. Deliberately
	// independent of residual_lpf_tau_s (a fixed 0.1 s is safe across the usable tau range).
	static constexpr float kMaxGap = 0.1f;

	// One global current model for all motors (per-motor steady bias under a fleet fit was < 0.25 A).
	struct Config {
		float model_a;                  ///< u^2 coefficient [A] (steady)
		float model_b;                  ///< u coefficient [A] (linear term; 0 => pure quadratic, model_a=0 => linear)
		float model_c;                  ///< no-load / idle current offset [A]
		float residual_lpf_tau_s;       ///< LPF time constant on the residual [s]
		float threshold_a;              ///< constant trip threshold on |LPF(residual)| [A]; <= 0 => monitor-only
		float threshold_rel;            ///< throttle-relative add: trips at threshold_a + threshold_rel*I_expected (0 => flat)
		float persistence_s;            ///< how long |LPF(residual)| must stay >= threshold to latch [s]
	};

	struct MotorStatus {
		float residual;        ///< raw signed residual I - I_expected [A]
		float residual_lpf;    ///< low-pass filtered residual [A]
		bool  failed;          ///< latched failure decision
		bool  excluded;        ///< not evaluated this step (disabled, reversible, or NaN command)
	};

	MotorFailureDetector() { reset(); }

	void configure(const Config &cfg) { _cfg = cfg; reset(); }

	/** Reset all per-motor state (filters, debounce, latched decisions); re-applies the persistence time. */
	void reset();

	/**
	 * Advance the detector to timestamp now_us [us]; dt is derived internally
	 * (esc_status.timestamp in flight, the log timestamp in replay). Arrays hold
	 * num_motors entries (num_motors <= kMaxMotors; matches actuator_motors.control[]).
	 * command = actuator_motors.control[i]; a NaN or reversible command excludes that
	 * motor. A non-finite current is a telemetry dropout: the residual is held and the
	 * debounce keeps running on it (so a fault that also drops telemetry still latches).
	 * A step > kMaxGap is a data gap (filters + debounce reset). configure() first;
	 * a zero Config is monitor-only.
	 */
	void update(int num_motors, hrt_abstime now_us,
		    const float command[], const float current[],
		    const bool reversible[], const bool enabled[]);

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
