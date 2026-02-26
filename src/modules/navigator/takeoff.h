/***************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file takeoff.h
 *
 * Helper class to take off
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#pragma once

#include "navigator_mode.h"
#include "mission_block.h"
#include <lib/mathlib/mathlib.h>

class Takeoff : public MissionBlock
{
public:
	Takeoff(Navigator *navigator);
	~Takeoff() = default;

	void on_activation() override;
	void on_active() override;

	void setLoiterPosition(matrix::Vector2d loiter_location) { _loiter_position_lat_lon = loiter_location; }
	void setLoiterAltitudeAmsl(const float height_m) { _loiter_altitude_msl = height_m; }

private:

	enum class fw_takeoff_state {
		CLIMBOUT = 0,
		GO_TO_LOITER
	} _fw_takeoff_state;

	void set_takeoff_position();
	matrix::Vector2d _loiter_position_lat_lon{static_cast<double>(NAN), static_cast<double>(NAN)};
	float _loiter_altitude_msl{NAN};
};
