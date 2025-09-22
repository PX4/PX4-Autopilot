/****************************************************************************
 *
 *   Copyright (c) 2018-2023 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskNaor.cpp
 */

 #include "FlightTaskNaor.hpp"
 #include <float.h>
 #include <mathlib/mathlib.h>
 #include <geo/geo.h>
 #include <lib/motion_planning/PositionSmoothing.hpp>


 #define MAZ_Z_VELOCITY 1.2f


 using namespace matrix;

 bool FlightTaskNaor::updateInitialize()
 {

	 _visual_odometry_sub.update(&_visual_odometry);
	 bool ret = FlightTask::updateInitialize();

	 _sticks.checkAndUpdateStickInputs();

	 if (_sticks_data_required) {
		 ret = ret && _sticks.isAvailable();
	 }

	 return ret;
 }

 bool FlightTaskNaor::activate(const trajectory_setpoint_s &last_setpoint)
 {
	 bool ret = FlightTask::activate(last_setpoint);
	 _yaw_setpoint = NAN;
	 _yawspeed_setpoint = 0.f;
	 _acceleration_setpoint = Vector3f(0.f, 0.f, NAN); // altitude is controlled from position/velocity
	 _position_setpoint(2) = NAN;
	 _velocity_setpoint(2) = 0.f;
	 _stick_yaw.reset(_yaw, _unaided_yaw);

	 return ret;
 }



 void FlightTaskNaor::_throttleToVelocity()
 {
	 _velocity_setpoint(2) = map(_sticks.getPositionExpo()(2), -1.0f, 1.0f, -MAZ_Z_VELOCITY, MAZ_Z_VELOCITY);
 }


 void FlightTaskNaor::_updateSetpoints_acc(Vector2f acceleration_setpoint)
 {
	 _stick_yaw.generateYawSetpoint(_yawspeed_setpoint, _yaw_setpoint, _sticks.getYawExpo(), _yaw, _deltatime, _unaided_yaw);
	 _acceleration_setpoint.xy() = acceleration_setpoint;
	 _position_setpoint(0) = NAN;
	 _position_setpoint(1) = NAN;
	 _velocity_setpoint(0) = NAN;
	 _velocity_setpoint(1) = NAN;


 }


 void FlightTaskNaor::_updateTrajectoryBoundaries()
 {
	 // update params of the position smoothing
	 _position_smoothing.setMaxAllowedHorizontalError(_param_mpc_xy_err_max.get());
	 _position_smoothing.setHorizontalTrajectoryGain(_param_mpc_xy_traj_p.get());
	 _position_smoothing.setMaxVelocityXY(_param_mpc_xy_vel_max.get());

 }



 void FlightTaskNaor::_updateSetpoints_velo(Vector2f marker_relative_position){
	 //velocity command from the camera
	 _stick_yaw.generateYawSetpoint(_yawspeed_setpoint, _yaw_setpoint, _sticks.getYawExpo(), _yaw, _deltatime, _unaided_yaw); // gave yaw control

	 _naor_debug.timestamp = hrt_absolute_time();
	 _naor_debug.land_target_x = _visual_odometry.position[0];
	 _naor_debug.land_target_y = _visual_odometry.position[1];
	 _naor_debug.velocity_setpoint_command_right = _velocity_setpoint(0);
	 _naor_debug.velocity_setpoint_command_forward = _velocity_setpoint(1);
	 _naor_debug.current_velocity_right = _velocity(0);
	 _naor_debug.current_velocity_forward = _velocity(1);
	 _naor_debug.openloop_command_right = marker_relative_position(0);
	 _naor_debug.openloop_command_forword = marker_relative_position(1);
	 _naor_debug_pub.publish(_naor_debug);
	 _velocity_setpoint.xy() = marker_relative_position;

 }


 void FlightTaskNaor::_goToOrigin(){

	 _origin_x = _visual_odometry.position[0];
	 _origin_y = _visual_odometry.position[1];

	 _position_setpoint(0) = 0.0f;
	 _position_setpoint(1) = 0.0f;

	 _velocity_setpoint(0) = NAN;
	 _velocity_setpoint(1) = NAN;

 }


 bool FlightTaskNaor::_checkTakeoff()
 {
	 // Only use velocity-based takeoff detection, ignore position/altitude constraints
	 bool velocity_triggered_takeoff = false;

	 if (PX4_ISFINITE(_velocity_setpoint(2))) {
		 velocity_triggered_takeoff = _velocity_setpoint(2) < -0.3f;
	 }

	 return velocity_triggered_takeoff; // No altitude/position constraints
 }


 // void FlightTaskNaor::setpos(Vector2f position_setpoint)

 bool FlightTaskNaor::_check_timer(uint64_t _timestamp)
 {
	 uint64_t current_time = hrt_absolute_time();
	 uint64_t minus = (float)(current_time - _timestamp);
	 return minus < _timer_threshold; /// all the time here is at microseconds
 }

 bool FlightTaskNaor::update()
 {
	 bool ret = FlightTask::update();

	 // Check if MPC is available by checking if MPC parameters are valid
	 bool mpc_available = PX4_ISFINITE(_param_mpc_xy_err_max.get()) &&
			      PX4_ISFINITE(_param_mpc_xy_traj_p.get()) &&
			      PX4_ISFINITE(_param_mpc_xy_vel_max.get());

	 if (mpc_available) {
		 _updateTrajectoryBoundaries();
	 }

	 _throttleToVelocity(); /// caommand the thr for the usr
	 bool _mode = _check_timer(_visual_odometry.timestamp);


	 switch (_mode) {
		 case true: {
			 matrix::Vector2f camera_setpoint;
			 camera_setpoint(0) = 0.0f;
			 camera_setpoint(1) = 0.0f;
			 _updateSetpoints_velo(camera_setpoint);
			 break;
		 }
		 case false:
			 _updateSetpoints_acc(_stick_tilt_xy.generateAccelerationSetpoints(_sticks.getPitchRoll(), _deltatime, _yaw,_yaw_setpoint));
			 break;
	 }
	 _constraints.want_takeoff = _checkTakeoff();
	 // Don't override the speed constraints to NAN - let the base class handle them properly
	 return ret;
 }
