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
 * @file SmoothingZ.hpp
 *
 * This Class is used for smoothing the velocity setpoints in Z-direction.
 */

#pragma once

#include <systemlib/param/param.h>

/* User intention: brake or acceleration */
enum class ManualIntentionZ {
	brake,
	acceleration,
};

class ManualSmoothingZ
{
public:
	ManualSmoothingZ(const float &vel, const float &stick);
	~ManualSmoothingZ() {};

	/* Smooths velocity setpoint based
	 * on flight direction.
	 * @param vel_sp[2] array: vel_sp[0] = current velocity set-point;
	 * 					 	   vel_sp[1] = previous velocity set-point
	 * 		  vel_sp will contain smoothed current previous set-point.
	 * @param dt: time delta in seconds
	 */
	void smoothVelFromSticks(float &vel_sp, const float dt);


	/* Getter methods */
	float getMaxAcceleration() { return _max_acceleration; }
	ManualIntentionZ getIntention() { return _intention; }

	/* Overwrite methods:
	 * Needed if different parameter values than default required.
	 */
	void overwriteAccelerationUp(float acc_max_up) {_acc_max_up = acc_max_up;};
	void overwriteAccelerationDown(float acc_max_down) {_acc_max_down = acc_max_down;};
	void overwriteJerkMax(float jerk_max) {_jerk_max = jerk_max;};

private:

	/* User intention: brake or acceleration */
	ManualIntentionZ _intention{ManualIntentionZ::acceleration};

	/* Dependency injection: vehicle velocity in z-direction;
	 * stick input in z-direction
	 */
	const float &_vel;
	const float &_stick;

	/* Acceleration that depends on vehicle state
	 * _acc_max_down <= _acc_state_dependent <= _acc_max_up
	 */
	float _acc_state_dependent{0.0f};
	float _vel_sp_prev;
	float _max_acceleration;

	/* Params */
	param_t _acc_max_up_h{PARAM_INVALID};
	param_t _acc_max_down_h{PARAM_INVALID};
	param_t _jerk_max_h{PARAM_INVALID};
	float _acc_max_up{0.0f};
	float _acc_max_down{0.0f};
	float _jerk_max{10000.0f};
	int _parameter_sub{-1};

	/* Helper methods */
	void velocitySlewRate(float &vel_sp, const float dt);
	void setParams();
	void updateParams();
	void updateAcceleration(float &vel_sp, const float dt);
	void setMaxAcceleration();

};
