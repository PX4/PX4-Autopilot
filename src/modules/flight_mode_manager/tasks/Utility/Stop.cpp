/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
#include "Stop.hpp"

Stop::Stop(ModuleParams *parent) :
	ModuleParams(parent)
{
	// update params of the position smoothing
	_position_smoothing.setCruiseSpeed(_param_mpc_xy_vel_max.get());
	_position_smoothing.setHorizontalTrajectoryGain(_param_mpc_xy_traj_p.get());
	_position_smoothing.setMaxAllowedHorizontalError(_param_mpc_xy_err_max.get());
	_position_smoothing.setTargetAcceptanceRadius(_param_nav_mc_alt_rad.get());
	_position_smoothing.setVerticalAcceptanceRadius(_param_nav_mc_alt_rad.get());

	// Update the constraints of the trajectories
	_position_smoothing.setMaxVelocityXY(_param_mpc_xy_vel_max.get());
	_position_smoothing.setMaxVelocityZ(math::max(_param_mpc_z_vel_max_up.get(), _param_mpc_z_vel_max_dn.get()));
	_position_smoothing.setMaxAccelerationXY(_param_mpc_acc_hor.get());
	_position_smoothing.setMaxAccelerationZ(math::max(_param_mpc_acc_down_max.get(), _param_mpc_acc_up_max.get()));
	_position_smoothing.setMaxJerkXY(_param_mpc_jerk_auto.get());
	_position_smoothing.setMaxJerkZ(_param_mpc_jerk_max.get());
}

void Stop::getConstraints(vehicle_constraints_s &constraints)
{
	constraints.speed_down = 1.2f * _param_mpc_z_vel_max_dn.get();
	constraints.speed_up = 1.2f * _param_mpc_z_vel_max_up.get();
	constraints.speed_xy = 1.2f * _param_mpc_xy_vel_max.get();
	_position_smoothing.setMaxJerkXY(_param_mpc_jerk_auto.get());
	_position_smoothing.setMaxJerkZ(_param_mpc_jerk_max.get());

	if (_exceeded_max_vel) {
		constraints.speed_down = math::max(fabsf(_position_smoothing.getCurrentVelocityZ()), constraints.speed_down);
		constraints.speed_up = math::max(fabsf(_position_smoothing.getCurrentVelocityZ()), constraints.speed_up);
		constraints.speed_xy = math::max(_position_smoothing.getCurrentVelocityXY().norm(), constraints.speed_xy);

		_position_smoothing.setMaxAccelerationXY(CONSTANTS_ONE_G);
		_position_smoothing.setMaxAccelerationZ(2 * CONSTANTS_ONE_G);
		_position_smoothing.setMaxJerk(CONSTANTS_ONE_G);
		_position_smoothing.setMaxJerkZ(CONSTANTS_ONE_G);
	}
}

void
Stop::initialize(const Vector3f &acceleration, const Vector3f &velocity, const Vector3f &position,
		 const float &deltatime)
{
	if (velocity.isAllNan() || position.isAllNan() || acceleration.isAllNan()) {
		PX4_ERR("Initialized stop with invalid values");
	}

	_isActive = true;
	_wasActive = false;

	_position_smoothing.reset(acceleration, velocity, position);
	update(acceleration, velocity, position, deltatime);
}

void
Stop::update(const Vector3f &acceleration, const Vector3f &velocity, const Vector3f &position, const float &deltatime)
{
	if (checkMaxVelocityLimit(velocity) && !_exceeded_max_vel) {
		_exceeded_max_vel = true;

	} else if (_position_smoothing.getCurrentVelocityZ() < 0.01f
		   && _position_smoothing.getCurrentVelocityZ() > -0.01f
		   && !_position_smoothing.getCurrentVelocityXY().longerThan(0.01f)) {
		// deactivate when the vehicle has come to a full stop
		_isActive = false;
		_wasActive = true;
		_stop_position = position;
	}

	PositionSmoothing::PositionSmoothingSetpoints out_setpoints;
	// Generate the setpoints
	_position_smoothing.generateSetpoints(
		position,
		_stop_position,
		Vector3f{0.f, 0.f, 0.f},
		deltatime,
		true,
		out_setpoints
	);
	_jerk_setpoint = out_setpoints.jerk;
	_acceleration_setpoint = out_setpoints.acceleration;
	_velocity_setpoint = out_setpoints.velocity;
	_position_setpoint = out_setpoints.position;
	_unsmoothed_velocity = out_setpoints.unsmoothed_velocity;
}

bool
Stop::checkMaxVelocityLimit(const Vector3f &velocity, const float &factor)
{
	const bool exceeded_vel_z = velocity(2) > (factor * _param_mpc_z_vel_max_dn.get())
				    || velocity(2) < -(factor * _param_mpc_z_vel_max_up.get());

	const bool exceeded_vel_xy = velocity.xy().norm() > _param_mpc_xy_vel_max.get();

	if ((exceeded_vel_xy || exceeded_vel_z)) {
		PX4_DEBUG("exceeded_vel_xy: %d, exceeded_vel_z: %d", exceeded_vel_xy, exceeded_vel_z);
	}

	return (exceeded_vel_xy || exceeded_vel_z);
}
