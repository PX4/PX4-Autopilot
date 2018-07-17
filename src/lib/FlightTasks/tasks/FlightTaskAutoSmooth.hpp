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
 * @file FlightTaskAutoSmooth.hpp
 *
 * Flight task for smooth, autonomous, gps driven mode. 
 * 
 */

#pragma once

#include "FlightTaskAuto.hpp"
#include "lib/bezier/BezierQuad.hpp"
#include "Utility/StraightLine.hpp"
#include <iostream>

class FlightTaskAutoSmooth : public FlightTaskAuto
{
public:
	FlightTaskAutoSmooth();
	virtual ~FlightTaskAutoSmooth() = default;
	bool activate() override;
	bool update() override;

protected:

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskAuto,
					(ParamFloat<px4::params::NAV_ACC_RAD>) NAV_ACC_RAD, // acceptance radius at which waypoints are updated
					(ParamFloat<px4::params::MIS_YAW_ERR>) MIS_YAW_ERR // yaw-error threshold
				    )

	void updateParams() override; /**< See ModuleParam class */

private:
	void _reset(); /**< Resets member variables to current vehicle state */
	bezier::BezierQuad_f _b;
	StraightLine _sl;
	matrix::Vector3f _target_prev;
	matrix::Vector3f _internal_curr;
	matrix::Vector3f _internal_prev;
	matrix::Vector3f _internal_next;
	matrix::Vector3f _pt_0;
	matrix::Vector3f _pt_1;

	bool _internal_triplets_update = true;
	bool _pt0_reached_once = false;

	void _update_internal_triplets();
};
