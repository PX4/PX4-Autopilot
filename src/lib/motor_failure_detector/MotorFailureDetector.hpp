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
 * residual holds past a trip band for the trigger time.
 *
 * Band and trigger time are per direction: benign transients are much larger on the
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
	// Same as esc_status CONNECTED_ESC_MAX, hardcoded to keep the library uORB-free.
	static constexpr int kMaxMotors = 12;

	// Step longer than this [s] => data gap (resets filters + debounce).
	// Equal to the ESC-offline timeout: every ESC that is still online gets evaluated.
	static constexpr float kMaxGap = 0.4f;

	// Time constant of the residual low-pass filter [s]
	static constexpr float kResidualLpfTau = 0.2f;

	// One model for all motors
	struct Config {
		float current_slope_a;             ///< expected current per unit thrust command [A]
		float current_idle_a;              ///< expected current at zero command [A]
		float overcurrent_threshold_a;     ///< trip band above expected current [A]; <= 0 disables this band
		float overcurrent_trigger_time_s;  ///< time above the overcurrent band before latching [s]
		float undercurrent_threshold_a;    ///< trip band below expected current [A]; <= 0 disables this band
		float undercurrent_trigger_time_s; ///< time below the undercurrent band before latching [s]
	};

	enum class Failure : uint8_t { None, Undercurrent, Overcurrent };

	MotorFailureDetector() { reset(); }

	/** Set the model, bands and trigger times. Keeps filter, debounce and latch state. */
	void configure(const Config &cfg);

	/** Reset all per-motor state (filters, debounce, latch). */
	void reset();

	/**
	 * Advance motor i to its sample timestamp now_us [us] (dt derived per motor; a repeated
	 * timestamp is the same sample again and is ignored). A non-finite command or current is not
	 * evaluated (the model only describes forward thrust, so pass NAN for a reversible motor);
	 * neither is a step > kMaxGap, e.g. after the motor was not updated for a while. Both reset
	 * filter and debounce, the latch is kept. configure() first.
	 */
	void update(int i, hrt_abstime now_us, float command, float current);

	/** Failure of motor i, latched at the first trip and kept until reset(). */
	Failure failure(int i) const { return validIndex(i) ? _failure[i] : Failure::None; }

	/** Low-pass filtered residual I - I_expected of motor i [A]. */
	float residualFiltered(int i) const { return validIndex(i) ? _residual_lpf[i].getState() : 0.f; }

private:
	static bool validIndex(int i) { return 0 <= i && i < kMaxMotors; }

	Config                _cfg{};
	Failure               _failure[kMaxMotors] {};
	AlphaFilter<float>    _residual_lpf[kMaxMotors];
	systemlib::Hysteresis _hyst_over[kMaxMotors];
	systemlib::Hysteresis _hyst_under[kMaxMotors];
	hrt_abstime           _last_us[kMaxMotors] {};
};
