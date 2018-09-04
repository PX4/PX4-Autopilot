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

#include <matrix/matrix/math.hpp>

class WeatherVane
{
public:
	WeatherVane();

	~WeatherVane() {};

	void activate() {_is_active = true;}

	void deactivate() {_is_active = false;}

	bool is_active() {return _is_active;}

	void update(matrix::Quatf q_sp_prev, float yaw);

	float get_weathervane_yawrate();

	void set_weathervane_gain(float gain) {_wv_gain = gain;}

	void set_min_roll_rad(float min_roll_rad) {_wv_min_roll_rad = min_roll_rad;}

	void set_yawrate_max_rad(float yawrate_max_rad) {_wv_yawrate_max_rad = yawrate_max_rad;}

private:
	matrix::Quatf _q_sp_prev;	// previous attitude setpoint quaternion
	float _yaw = 0.0f;					// current yaw angle

	bool _is_active = true;

	float _wv_gain = 1.0f;		// gain that maps excessive roll angle setpoint to yawrate setoint [1/s]
	float _wv_min_roll_rad = 0.01f;		// minimum roll angle setpoint for the controller to output a non-zero yawrate setpoint
	float _wv_yawrate_max_rad = 1.0f;	// maximum yaw-rate the controller will output

};
