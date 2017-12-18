/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskManual.hpp
 *
 * Flight task for the normal, manual position controlled flight
 * where stick inputs map basically to the velocity setpoint
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include "FlightTask.hpp"
#include <uORB/topics/manual_control_setpoint.h>

class FlightTaskManual : public FlightTask
{
public:
	FlightTaskManual(control::SuperBlock *parent, const char *name);

	virtual ~FlightTaskManual() = default;

	bool initializeSubscriptions(SubscriptionArray &subscription_array) override;

	bool activate() override;

	bool applyCommandParameters(const vehicle_command_s &command) override { return FlightTask::applyCommandParameters(command); };

	bool updateInitialize() override;

	bool update() override;

protected:
	matrix::Vector<float, 4> _sticks;
	bool _evaluateSticks();
	bool _sticks_data_required = true; /**< let sibling task define if it depends on stick data */

	float _get_input_frame_yaw() { return _yaw; }
	virtual void _scaleVelocity(matrix::Vector3f &velocity);

	control::BlockParamFloat _z_vel_max_up; /**< maximal vertical velocity when flying upwards with the stick */
	control::BlockParamFloat _z_vel_max_down; /**< maximal vertical velocity when flying downwards with the stick */

private:
	uORB::Subscription<manual_control_setpoint_s> *_sub_manual_control_setpoint{nullptr};

	control::BlockParamFloat _xy_vel_man_expo; /**< ratio of exponential curve for stick input in xy direction pos mode */
	control::BlockParamFloat _z_vel_man_expo; /**< ratio of exponential curve for stick input in xy direction pos mode */
	control::BlockParamFloat _hold_dz; /**< deadzone around the center for the sticks when flying in position mode */
	control::BlockParamFloat _velocity_hor_manual; /**< target velocity in manual controlled mode at full speed */
	control::BlockParamFloat _hold_max_xy; /**< velocity threshold to switch into horizontal position hold */
	control::BlockParamFloat _hold_max_z; /**< velocity threshold to switch into vertical position hold */
	control::BlockParamFloat _man_yaw_max; /**< maximal rotation speed around yaw axis with full stick input */

	matrix::Vector3f _hold_position; /**< position at which the vehicle stays while the input is zero velocity */
	float _hold_yaw = 0.f; /**< absolute yaw which gets updated by the yawspeed input */

	void _updateYaw();

	bool _evaluate_sticks();

};
