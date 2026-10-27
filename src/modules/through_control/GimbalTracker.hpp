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
 * @file GimbalTracker.hpp
 *
 * Gimbal pitch tracking of the THROUGH mode (spec section 7, step 7).
 *
 * The simulated gimbal is not stabilized: its angles are joint angles relative to
 * the vehicle (VERIFICATION.md V5/V6). The camera axis therefore moves in the body
 * x-z plane, and the command is computed in body frame:
 *
 *   pitch_des  = atan2(d_b.z, d_b.x),  d_b = R_nb^T * u        (+ = camera down)
 *   pitch_meas = same for the measured camera x axis
 *   pitch_cmd  = pitch_meas + K * (pitch_des - pitch_meas)
 *
 * Internally "+ = down"; the gimbal manager interface uses "+ = up" (FRD pitch),
 * the sign is flipped when the setpoint is published.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/gimbal_manager_set_attitude.h>
#include <uORB/topics/gimbal_manager_status.h>
#include <uORB/topics/vehicle_command.h>

namespace through
{

class GimbalTracker
{
public:
	static constexpr float PITCH_DOWN_MIN = -0.5236f; ///< -30 deg (camera up), spec 7 step 7
	static constexpr float PITCH_DOWN_MAX = 1.5533f;  ///< +89 deg (the gimbal flips beyond -90 deg, V5)

	/** Elevation of a NED direction below the body x axis, in the body x-z plane [rad] (+ = down). */
	static float bodyPitchDown(const matrix::Quatf &q_nb, const matrix::Vector3f &dir_ned);

	/** Spec step 7 blend, constrained to the gimbal range [rad] (+ = down). */
	static float pitchCommand(float pitch_meas, float pitch_des, float gain);

	void setIds(uint8_t sysid, uint8_t compid) { _sysid = sysid; _compid = compid; }

	/** Request primary control of the gimbal manager (retried every second until granted). */
	void acquire(hrt_abstime now);

	/** Point the gimbal back to neutral and release the gimbal manager control if it was requested. */
	void release();

	/** True while the gimbal manager reports this component as primary control. */
	bool inControl() const { return _in_control; }

	/** Publish a body-frame pitch setpoint (+ = down), roll 0, yaw following the vehicle. */
	void commandPitch(float pitch_down);

	float lastCommand() const { return _last_command; }

private:
	void updateStatus();
	void publishConfigure(float sysid, float compid);

	uORB::Subscription _gimbal_manager_status_sub{ORB_ID(gimbal_manager_status)};
	uORB::Publication<gimbal_manager_set_attitude_s> _gimbal_manager_set_attitude_pub{ORB_ID(gimbal_manager_set_attitude)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	uint8_t _sysid{1};
	uint8_t _compid{1};
	bool _requested{false};
	bool _in_control{false};
	hrt_abstime _last_request_time{0};
	float _last_command{NAN};
};

} // namespace through
