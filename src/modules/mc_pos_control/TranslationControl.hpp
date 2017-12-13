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
 * @file TranslationControl.hpp
 *
 * @inputs: position-, velocity-, acceleration-, thrust- setpoints
 * @outputs: thrust vector
 *
 */

#include <matrix/matrix/math.hpp>

#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

#pragma once

class TranslationControl
{
public:
	TranslationControl();

	~TranslationControl() {};

	void updateState(struct vehicle_local_position_s state);
	void updateSetpoint(struct vehicle_local_position_setpoint_s setpoint);

private:

	matrix::Vector3f _pos{0.0f, 0.0f, 0.0f};
	matrix::Vector3f _vel{0.0f, 0.0f, 0.0f};
	matrix::Vector3f _acc{0.0f, 0.0f, 0.0f};
	matrix::Vector3f _thr{0.0f, 0.0f, 0.0f};
	float _yaw{0.0f};

	matrix::Vector3f _pos_sp{0.0f, 0.0f, 0.0f};
	matrix::Vector3f _vel_sp{0.0f, 0.0f, 0.0f};
	matrix::Vector3f _acc_sp{0.0f, 0.0f, 0.0f};
	matrix::Vector3f _thr_sp{0.0f, 0.0f, 0.0f};
	float _yaw_sp{0.0f};
	float _yawspeed_sp{0.0f};
};
