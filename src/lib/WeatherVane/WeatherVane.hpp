/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file WeatherVane.hpp
 * @author Ivo Drescher
 * @author Roman Bapst <roman@auterion.com>
 *
 * Weathervane controller.
 *
 */

#pragma once

#include <px4_module_params.h>
#include <matrix/matrix/math.hpp>

#define MASK_MAN_EN 		(1<<0)	// weathervane enabled for manual position control
#define MASK_AUTO_TK_EN 	(1<<1)	// weathervane enabled for takeoff in auto mode
#define MASK_AUTO_LND_EN 	(1<<2)	// weathervane enabled for landing in auto mode
#define MASK_AUTO_LTR_EN 	(1<<3)	// weathervane enabled for loiter in auto mode

class WeatherVane : public ModuleParams
{
public:
	WeatherVane();

	~WeatherVane() = default;

	bool should_run_in_manual();

	bool should_run_in_mission(uint8_t type);

	void update(const matrix::Quatf &q_sp_prev, float yaw);

	float get_weathervane_yawrate();

	void update_parameters() { ModuleParams::updateParams(); }

private:
	matrix::Dcmf _R_sp_prev;	// previous attitude setpoint rotation matrix
	float _yaw = 0.0f;			// current yaw angle

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::WV_CONFIG>) _wv_config,
		(ParamFloat<px4::params::WV_ROLL_MIN>) _wv_min_roll,
		(ParamFloat<px4::params::WV_GAIN>) _wv_gain,
		(ParamFloat<px4::params::WV_YRATE_MAX>) _wv_max_yaw_rate
	)

};
