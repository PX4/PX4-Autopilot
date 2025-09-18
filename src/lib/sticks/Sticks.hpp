/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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
 * @file Sticks.hpp
 *
 * Library abstracting stick input from manual_control_setpoint
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <matrix/matrix/math.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/failsafe_flags.h>

class Sticks : public ModuleParams
{
public:
	Sticks(ModuleParams *parent);
	~Sticks() = default;

	// Checks for updated manual control input & updates internal values
	bool checkAndUpdateStickInputs();

	bool isAvailable() { return _input_available; };

	// Position : 0 : pitch, 1 : roll, 2 : throttle, 3 : yaw
	const matrix::Vector4f &getPosition() { return _positions; }; // Raw stick position, no deadzone
	const matrix::Vector4f &getPositionExpo() { return _positions_expo; }; // Deadzone and expo applied

	// Helper functions to get stick values more intuitively
	float getRoll() const { return _positions(1); }
	float getRollExpo() const { return _positions_expo(1); }
	float getPitch() const { return _positions(0); }
	float getPitchExpo() const { return _positions_expo(0); }
	float getYaw() const { return _positions(3); }
	float getYawExpo() const { return _positions_expo(3); }
	float getThrottleZeroCentered() const { return -_positions(2); } // Convert Z-axis(down) command to Up-axis frame
	float getThrottleZeroCenteredExpo() const { return -_positions_expo(2); }
	const matrix::Vector2f getPitchRoll() { return _positions.slice<2, 1>(0, 0); }
	const matrix::Vector2f getPitchRollExpo() { return _positions_expo.slice<2, 1>(0, 0); }

	const matrix::Vector<float, 6> &getAux() const { return _aux_positions; }

	/**
	 * Limit the the horizontal input from a square shaped joystick gimbal to a unit circle
	 * @param v Vector containing x, y, z axis of the joystick gimbal. x, y get adjusted
	 */
	static void limitStickUnitLengthXY(matrix::Vector2f &v);

	/**
	 * Rotate horizontal component of joystick input into the vehicle body orientation
	 * @param v Vector containing x, y, z axis of the joystick input.
	 * @param yaw Current vehicle yaw heading
	 * @param yaw_setpoint Current yaw setpoint if it's locked else NAN
	 */
	static void rotateIntoHeadingFrameXY(matrix::Vector2f &v, const float yaw, const float yaw_setpoint);

private:
	bool _input_available{false};
	matrix::Vector4f _positions; ///< unmodified manual stick inputs that usually move vehicle in x, y, z and yaw direction
	matrix::Vector4f _positions_expo; ///< modified manual sticks using expo function

	matrix::Vector<float, 6> _aux_positions;

	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _failsafe_flags_sub{ORB_ID(failsafe_flags)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_HOLD_DZ>) _param_mpc_hold_dz,
		(ParamFloat<px4::params::MPC_XY_MAN_EXPO>) _param_mpc_xy_man_expo,
		(ParamFloat<px4::params::MPC_Z_MAN_EXPO>) _param_mpc_z_man_expo,
		(ParamFloat<px4::params::MPC_YAW_EXPO>) _param_mpc_yaw_expo
	)
};
