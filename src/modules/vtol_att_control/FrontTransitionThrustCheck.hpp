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
 * @file FrontTransitionThrustCheck.hpp
 *
 * Checks at the start of a front transition, where the drag is still low, that the forward thrust accelerates the
 * vehicle. Example: with a failed pusher, the forward acceleration stays about zero at the transition throttle.
 */

#pragma once

#include <drivers/drv_hrt.h>

class FrontTransitionThrustCheck
{
public:
	/**
	 * Restart for a new front transition. Not checked if the forward thrust at the start (e.g. pusher assist) is
	 * half of the transition thrust or more: what is added is too small to tell from gusts.
	 *
	 * @param start_share forward thrust at the start, as a share of the transition thrust
	 */
	void start(float start_share);

	/** No check until the next start() */
	void skip() { _checking = false; }

	bool isChecking() const { return _checking; }

	/**
	 * Fits the forward acceleration to the thrust added since the start (least squares, so each sample counts with
	 * its thrust) and decides once the samples weigh as much as 1 s at the transition thrust.
	 *
	 * @param timestamp_sample [us] samples 100 ms or more apart are not integrated
	 * @param thrust_share forward thrust as a share of the transition thrust, 1 at the transition throttle
	 * @param forward_acceleration [m/s^2] accelerometer along the body x axis, a non-finite sample is skipped
	 * @param min_acceleration [m/s^2] required for the whole transition thrust. Example: 0.8 asks for 0.61 if 24%
	 * of the transition thrust was already there at the start.
	 * @return true once, if the fitted acceleration stays below min_acceleration
	 */
	bool update(hrt_abstime timestamp_sample, float thrust_share, float forward_acceleration, float min_acceleration);

private:
	hrt_abstime _last_sample{0};
	float _start_share{0.f};
	float _weighted_speed_gain{0.f}; ///< [m/s] forward acceleration integrated, weighted by the added thrust
	float _weighted_time{0.f}; ///< [s] time integrated, weighted by the added thrust squared
	bool _checking{false};
};
