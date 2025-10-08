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
#include <px4_platform_common/log.h>



#define MAZ_Z_VELOCITY 1.2f


using namespace matrix;

bool FlightTaskNaor::updateInitialize()
{

	_visual_odometry_sub.update(&_visual_odometry);
	_pid_consts_sub.update(&_pid_consts);

	_pid_consts_sub.update(&_pid_consts);
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
	_acceleration_setpoint = Vector3f(NAN, NAN, NAN); // altitude is controlled from position/velocity
	// _position_setpoint(2) = NAN;
	// _velocity_setpoint.setNaN();
	_stick_yaw.reset(_yaw, _unaided_yaw);
	_first_time = true;

	if (_first_time) {
		// First time initialization - use defaults if no PID constants received yet
		if (_pid_consts.timestamp == 0) {
			// No PID constants received yet, use defaults
			first_pid_init(_pid_consts);
			// Set a timestamp so the PID controllers get initialized
			_pid_consts.timestamp = hrt_absolute_time();
		}

		// Always apply the PID constants on first activation
		_pid_x.setGains(_pid_consts.x_ned[0], _pid_consts.x_ned[1], _pid_consts.x_ned[2]);
		_pid_y.setGains(_pid_consts.y_ned[0], _pid_consts.y_ned[1], _pid_consts.y_ned[2]);
		_pid_x.setOutputLimit(_pid_consts.x_ned_max_output);
		_pid_y.setOutputLimit(_pid_consts.y_ned_max_output);
		_pid_x.setIntegralLimit(_pid_consts.x_ned_integral_max);
		_pid_y.setIntegralLimit(_pid_consts.y_ned_integral_max);
		_pid_x.resetIntegral();
		_pid_y.resetIntegral();
		_pid_x.resetDerivative();
		_pid_y.resetDerivative();
		_last_pid_update_timestamp = _pid_consts.timestamp;

		PX4_INFO("PID controllers initialized");
		PX4_INFO("X PID: P=%.3f, I=%.3f, D=%.3f, Max=%.3f",
			 (double)_pid_consts.x_ned[0], (double)_pid_consts.x_ned[1],
			 (double)_pid_consts.x_ned[2], (double)_pid_consts.x_ned_max_output);
		PX4_INFO("Y PID: P=%.3f, I=%.3f, D=%.3f, Max=%.3f",
			 (double)_pid_consts.y_ned[0], (double)_pid_consts.y_ned[1],
			 (double)_pid_consts.y_ned[2], (double)_pid_consts.y_ned_max_output);

		_position_to_hold(0) = 0.0f;
		_position_to_hold(1) = 0.0f;
		_position_to_hold(2) = -2.0f;
		_first_time = false;
	}
	return ret;
}



void FlightTaskNaor::_throttleToVelocity()
{
	_velocity_setpoint(2) = map(_sticks.getThrottleZeroCenteredExpo(), -1.0f, 1.0f, -MAZ_Z_VELOCITY, MAZ_Z_VELOCITY);
}


void FlightTaskNaor::_updateSetpoints_acc(Vector2f acceleration_setpoint)
{
	// Scale down yaw input for slower response
	float scaled_yaw_input = _sticks.getYawExpo() * 0.5f; // Reduce to 50% of original input
	_stick_yaw.generateYawSetpoint(_yawspeed_setpoint, _yaw_setpoint, scaled_yaw_input, _yaw, _deltatime, _unaided_yaw);
	_acceleration_setpoint.xy() = acceleration_setpoint;
	_position_setpoint(0) = NAN;
	_position_setpoint(1) = NAN;
	_velocity_setpoint(0) = NAN;
	_velocity_setpoint(1) = NAN;
}


void FlightTaskNaor::first_pid_init(pid_consts_s pid_data){
	// Initialize default PID constants in the struct
	_pid_consts.x_ned[0] = 0.7f;	// P term
	_pid_consts.x_ned[1] = 0.18f;  // I term
	_pid_consts.x_ned[2] = 0.06f;  // D term
	_pid_consts.y_ned[0] = 0.7f;	// P term
	_pid_consts.y_ned[1] = 0.18f;  // I term
	_pid_consts.y_ned[2] = 0.06f;  // D term
	_pid_consts.altitude[0] = 1.0f;	// P term
	_pid_consts.altitude[1] = 0.1f;	// I term
	_pid_consts.altitude[2] = 0.02f;	// D term
	_pid_consts.x_ned_integral_max = 0.1f;	// Integral limit
	_pid_consts.y_ned_integral_max = 0.1f;	// Integral limit
	_pid_consts.x_ned_max_output = 0.35f;  // Max velocity output
	_pid_consts.y_ned_max_output = 0.35f;  // Max velocity output
	_pid_consts.altitude_integral_max = 1.0f;	// Integral limit
	_pid_consts.altitude_output = 0.5f;	// Max velocity output

	_pid_x.setGains(pid_data.x_ned[0], pid_data.x_ned[1], pid_data.x_ned[2]);
	_pid_y.setGains(pid_data.y_ned[0], pid_data.y_ned[1], pid_data.y_ned[2]);
	_pid_x.setOutputLimit(pid_data.x_ned_max_output);
	_pid_y.setOutputLimit(pid_data.y_ned_max_output);
	_pid_x.setIntegralLimit(pid_data.x_ned_integral_max);
	_pid_y.setIntegralLimit(pid_data.y_ned_integral_max);
	_pid_x.resetIntegral();
	_pid_y.resetIntegral();
	_pid_x.resetDerivative();
	_pid_y.resetDerivative();
	_last_pid_update_timestamp = pid_data.timestamp;

	PX4_INFO("X PID: P=%.1f, I=%.1f, D=%.1f, Max=%.1f",
		 (double)_pid_consts.x_ned[0], (double)_pid_consts.x_ned[1],
		 (double)_pid_consts.x_ned[2], (double)_pid_consts.x_ned_max_output);
	PX4_INFO("Y PID: P=%.1f, I=%.1f, D=%.1f, Max=%.1f",
		 (double)_pid_consts.y_ned[0], (double)_pid_consts.y_ned[1],
		 (double)_pid_consts.y_ned[2], (double)_pid_consts.y_ned_max_output);
	PX4_INFO("Altitude PID: P=%.1f, I=%.1f, D=%.1f, Max=%.1f",
		 (double)_pid_consts.altitude[0], (double)_pid_consts.altitude[1],
		 (double)_pid_consts.altitude[2], (double)_pid_consts.altitude_output);
	_updateTrajectoryBoundaries();

}

bool FlightTaskNaor::has_pid_constants_updated(pid_consts_s pid_data){
	// Use timestamp to detect if PID constants have been updated
	// This is much more efficient than comparing all individual values
	return (pid_data.timestamp > _last_pid_update_timestamp);
}

void FlightTaskNaor::update_pid_constants(pid_consts_s pid_data){


	if (!has_pid_constants_updated(pid_data)){
		return;
	}

	_pid_x.setGains(pid_data.x_ned[0], pid_data.x_ned[1], pid_data.x_ned[2]);
	_pid_y.setGains(pid_data.y_ned[0], pid_data.y_ned[1], pid_data.y_ned[2]);
	_pid_x.setOutputLimit(pid_data.x_ned_max_output);
	_pid_y.setOutputLimit(pid_data.y_ned_max_output);
	_pid_x.setIntegralLimit(pid_data.x_ned_integral_max);
	_pid_y.setIntegralLimit(pid_data.y_ned_integral_max);
	_pid_x.resetIntegral();
	_pid_y.resetIntegral();
	_pid_x.resetDerivative();
	_pid_y.resetDerivative();
	_last_pid_update_timestamp = pid_data.timestamp;

	PX4_INFO("X PID: P=%.1f, I=%.1f, D=%.1f, Max=%.1f",
		 (double)_pid_consts.x_ned[0], (double)_pid_consts.x_ned[1],
		 (double)_pid_consts.x_ned[2], (double)_pid_consts.x_ned_max_output);
	PX4_INFO("Y PID: P=%.1f, I=%.1f, D=%.1f, Max=%.1f",
		 (double)_pid_consts.y_ned[0], (double)_pid_consts.y_ned[1],
		 (double)_pid_consts.y_ned[2], (double)_pid_consts.y_ned_max_output);
	PX4_INFO("Altitude PID: P=%.1f, I=%.1f, D=%.1f, Max=%.1f",
		 (double)_pid_consts.altitude[0], (double)_pid_consts.altitude[1],
		 (double)_pid_consts.altitude[2], (double)_pid_consts.altitude_output);
	_updateTrajectoryBoundaries();


}

void FlightTaskNaor::_updateTrajectoryBoundaries()
{
	// update params of the position smoothing
	_velocity_smoothing_x.setMaxVel(_pid_consts.x_ned_max_output);
	_velocity_smoothing_y.setMaxVel(_pid_consts.y_ned_max_output);
	_velocity_smoothing_x.setMaxAccel(0.1);
	_velocity_smoothing_y.setMaxAccel(0.1);
	_velocity_smoothing_x.setMaxJerk(0.2);
	_velocity_smoothing_y.setMaxJerk(0.2);

}

float FlightTaskNaor::zero_velocity_setpoint(float vel){

	if (fabsf(vel) < 0.07f){
		return 0.0f;
	}
	return vel;
}


float FlightTaskNaor::total_Error_calc(){
	matrix::Vector3f Error;
	Error = _position_to_hold - _position;
	return Error.length();
}



void FlightTaskNaor::_execute_trajectory(){
	//velocity command from the camera
	// Scale down yaw input for slower response
	float scaled_yaw_input = _sticks.getYawExpo() * 0.2f; // Reduce to 50% of original input
	_stick_yaw.generateYawSetpoint(_yawspeed_setpoint, _yaw_setpoint, scaled_yaw_input, _yaw, _deltatime, _unaided_yaw);
	_velocity_setpoint = pid_operation(_position_to_hold);
	PX4_INFO("velocity data x: %.3f y: %.3f z: %.3f", (double)_velocity_setpoint(0), (double)_velocity_setpoint(1), (double)_velocity_setpoint(2));
	_position_setpoint(0) = NAN;
	_position_setpoint(1) = NAN;

	// _throttleToVelocity();
}

matrix::Vector3f FlightTaskNaor::pid_operation(matrix::Vector3f target_position){
	matrix::Vector3f velocity_command;
	_pid_x.setSetpoint(target_position(0));
	_pid_y.setSetpoint(target_position(1));
	_pid_altitude.setSetpoint(target_position(2));

	float vel_sp_x = zero_velocity_setpoint(_pid_x.update(_position(0), _deltatime));
	float vel_sp_y = zero_velocity_setpoint(_pid_y.update(_position(1), _deltatime));

	velocity_command(0) = vel_sp_x;
	velocity_command(1) = vel_sp_y;
	velocity_command(2) = NAN;
	return velocity_command;
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



bool FlightTaskNaor::_check_timer(uint64_t _timestamp)
{
	uint64_t current_time = hrt_absolute_time();
	uint64_t minus = (float)(current_time - _timestamp);
	return minus < _timer_threshold; /// all the time here is at microseconds
}

void FlightTaskNaor::_publish_debug_topics()
{

	_super_hold_debug.timestamp = hrt_absolute_time();
	_super_hold_debug.first_x = _position_to_hold(0);
	_super_hold_debug.first_y = _position_to_hold(1);
	_super_hold_debug.first_z = _position_to_hold(2);
	_super_hold_debug.error_x = _position_to_hold(0) - _position(0);
	_super_hold_debug.error_y = _position_to_hold(1) - _position(1);
	_super_hold_debug.error_z = _position_to_hold(2) - _position(2);
	_super_hold_debug.input_x = _position(0);
	_super_hold_debug.input_y = _position(1);
	_super_hold_debug.input_z = _position(2);
	_super_hold_debug.integral_x = _pid_x.getIntegral();
	_super_hold_debug.integral_y = _pid_y.getIntegral();
	_super_hold_debug.integral_z = _pid_altitude.getIntegral();
	_super_hold_debug.out_vel_x = _velocity_setpoint(0);
	_super_hold_debug.out_vel_y = _velocity_setpoint(1);
	_super_hold_debug.out_vel_z = _velocity_setpoint(2);
	_super_hold_debug_pub.publish(_super_hold_debug);
}


bool FlightTaskNaor::update()
{
	bool ret = FlightTask::update();
	update_pid_constants(_pid_consts);


	_execute_trajectory();
	_position_setpoint(2) = _position_to_hold(2);
	_publish_debug_topics();
	// bool _mode = _check_timer(_visual_odometry.timestamp);

	_constraints.want_takeoff = _checkTakeoff();
	// Don't override the speed constraints to NAN - let the base class handle them properly
	return ret;
}
