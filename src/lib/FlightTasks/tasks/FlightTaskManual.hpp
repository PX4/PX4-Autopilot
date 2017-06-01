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
 * Flight task for the normal, legacy, manual position controlled flight
 * where stick inputs map basically to the velocity setpoint
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include "FlightTask.hpp"

class FlightTaskManual : public FlightTask
{
public:
	FlightTaskManual(SuperBlock *parent, const char *name) :
		FlightTask(parent, name),
		_xy_vel_man_expo(nullptr, "MPC_XY_MAN_EXPO"),
		_z_vel_man_expo(nullptr, "MPC_Z_MAN_EXPO"),
		_hold_dz(nullptr, "MPC_HOLD_DZ"),
		_velocity_hor_manual(nullptr, "MPC_VEL_MANUAL"),
		_z_vel_max_up(nullptr, "MPC_Z_VEL_MAX_UP"),
		_z_vel_max_down(nullptr, "MPC_Z_VEL_MAX_DN"),
		_sub_control_state(ORB_ID(control_state), 0, 0, &getSubscriptions())
	{};
	virtual ~FlightTaskManual() {};

	/**
	 * Call once on the event where you switch to the task
	 * @return 0 on success, >0 on error otherwise
	 */
	virtual int activate()
	{
		FlightTask::activate();
		return 0;
	};

	/**
	 * Call once on the event of switching away from the task
	 * 	@return 0 on success, >0 on error otherwise
	 */
	virtual int disable()
	{
		FlightTask::disable();
		return 0;
	};

	/**
	 * Call regularly in the control loop cycle to execute the task
	 * @return 0 on success, >0 on error otherwise
	 */
	virtual int update()
	{
		FlightTask::update();

		matrix::Vector3f man_vel_sp;
		man_vel_sp(0) = math::expo_deadzone(_sticks(0), _xy_vel_man_expo.get(), _hold_dz.get());
		man_vel_sp(1) = math::expo_deadzone(_sticks(1), _xy_vel_man_expo.get(), _hold_dz.get());
		man_vel_sp(2) = -math::expo_deadzone(_sticks(2), _z_vel_man_expo.get(), _hold_dz.get());

		const float man_vel_hor_length = matrix::Vector2f(man_vel_sp.data()).length();

		/* saturate such that magnitude is never larger than 1 */
		if (man_vel_hor_length > 1.0f) {
			man_vel_sp(0) /= man_vel_hor_length;
			man_vel_sp(1) /= man_vel_hor_length;
		}

		/* rotate setpoint to be in NED frame */
		man_vel_sp = matrix::Dcmf(matrix::Eulerf(0.0f, 0.0f, get_input_frame_yaw())) * man_vel_sp;

		/* scale smaller than unit length vector to maximal manual speed (m/s) */
		matrix::Vector3f vel_scale(_velocity_hor_manual.get(),
					   _velocity_hor_manual.get(),
					   (man_vel_sp(2) > 0.0f) ? _z_vel_max_down.get() : _z_vel_max_up.get());
		man_vel_sp = man_vel_sp.emult(vel_scale);

		_set_position_setpoint(matrix::Vector3f(NAN, NAN, NAN));
		_set_velocity_setpoint(man_vel_sp);
		return 0;
	};

protected:
	float get_input_frame_yaw()
	{
		matrix::Quatf q(&_sub_control_state.get().q[0]);
		matrix::Eulerf euler_angles(q);
		return euler_angles(2);
	};

private:
	control::BlockParamFloat _xy_vel_man_expo; /**< ratio of exponential curve for stick input in xy direction pos mode */
	control::BlockParamFloat _z_vel_man_expo; /**< ratio of exponential curve for stick input in xy direction pos mode */
	control::BlockParamFloat _hold_dz; /**< deadzone around the center for the sticks when flying in position mode */
	control::BlockParamFloat _velocity_hor_manual; /**< target velocity in manual controlled mode at full speed */
	control::BlockParamFloat _z_vel_max_up; /**< maximal vertical velocity when flying upwards with the stick */
	control::BlockParamFloat _z_vel_max_down; /**< maximal vertical velocity when flying downwards with the stick */

	uORB::Subscription<control_state_s> _sub_control_state;

};
