/****************************************************************************
 *
 *   Copyright (c) 2020 ECL Development Team. All rights reserved.
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
 * Helper class to check the state_reset_status member of the ekf object.
 * Used for checking if state resets are logged correctly.
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */

#pragma once

#include <memory>
#include "EKF/ekf.h"

class ResetLoggingChecker {
public:
	ResetLoggingChecker(std::shared_ptr<Ekf> ekf) : _ekf(ekf) {}

	// call immediately before state reset
	void capturePreResetState();

	// call immediately after state reset
	void capturePostResetState();

	bool isVelocityDeltaLoggedCorrectly(float accuracy);

	bool isHorizontalVelocityResetCounterIncreasedBy(int offset);

	bool isVerticalVelocityResetCounterIncreasedBy(int offset);

	bool isPositionDeltaLoggedCorrectly(float accuracy);

	bool isHorizontalPositionResetCounterIncreasedBy(int offset);

	bool isVerticalPositionResetCounterIncreasedBy(int offset);

private:
	std::shared_ptr<Ekf> _ekf;

	Vector3f velocity_before_reset;
	Vector3f position_before_reset;
	int horz_vel_reset_counter_before_reset;
	int vert_vel_reset_counter_before_reset;
	int horz_pos_reset_counter_before_reset;
	int vert_pos_reset_counter_before_reset;

	Vector3f velocity_after_reset;
	Vector3f position_after_reset;
	int horz_vel_reset_counter_after_reset;
	int vert_vel_reset_counter_after_reset;
	int horz_pos_reset_counter_after_reset;
	int vert_pos_reset_counter_after_reset;

	Vector3f logged_delta_velocity;
	Vector3f logged_delta_position;
};
