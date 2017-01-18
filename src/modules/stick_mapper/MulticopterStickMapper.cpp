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

#include "MulticopterStickMapper.hpp"

MulticopterStickMapper::MulticopterStickMapper() :

// acro
	_mc_acro_roll_max(this, "MC_ACRO_R_MAX"),
	_mc_acro_pitch_max(this, "MC_ACRO_P_MAX"),
	_mc_acro_yaw_max(this, "MC_ACRO_Y_MAX"),

	_mc_man_roll_max(this, "MPC_MAN_R_MAX"),
	_mc_man_pitch_max(this, "MPC_MAN_P_MAX"),
	_mc_man_yawrate_max(this, "MPC_MAN_Y_MAX")

{
	parameters_update();

}

MulticopterStickMapper::~MulticopterStickMapper()
{
}

void
MulticopterStickMapper::parameters_update()
{
	mc_parameters_update();
}

void
MulticopterStickMapper::mc_parameters_update()
{
	updateParams();

	_man_roll_scale = 0.0f;
	_man_pitch_scale = 0.0f;
	_man_yaw_scale = 0.0f;

	_acro_roll_max = _mc_acro_roll_max.get();
	_acro_pitch_max = _mc_acro_pitch_max.get();
	_acro_yaw_max = _mc_acro_yaw_max.get();

	_man_roll_max = _mc_man_roll_max.get();
	_man_pitch_max = _mc_man_pitch_max.get();

	_roll_offset = 0.0f;
	_pitch_offset = 0.0f;

}
