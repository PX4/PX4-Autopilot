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

#include "FixedwingStickMapper.hpp"

FixedwingStickMapper::FixedwingStickMapper() :

	// manual input scaling
	_fw_man_roll_scale(nullptr, "FW_MAN_R_SC"),
	_fw_man_pitch_scale(nullptr, "FW_MAN_P_SC"),
	_fw_man_yaw_scale(nullptr, "FW_MAN_Y_SC"),

	// acro maximum rates
	_fw_acro_roll_max(nullptr, "FW_ACRO_R_MAX"),
	_fw_acro_pitch_max(nullptr, "FW_ACRO_P_MAX"),
	_fw_acro_yaw_max(nullptr, "FW_ACRO_Y_MAX"),

	// roll and pitch offsets
	_fw_man_roll_max(nullptr, "FW_MAN_R_MAX"),
	_fw_man_pitch_max(nullptr, "FW_MAN_P_MAX"),

	_fw_roll_offset(nullptr, "FW_RSP_OFF"),
	_fw_pitch_offset(nullptr, "FW_PSP_OFF")

{
	parameters_update();

}

FixedwingStickMapper::~FixedwingStickMapper()
{
}

void
FixedwingStickMapper::parameters_update()
{
	fw_parameters_update();
}

void
FixedwingStickMapper::fw_parameters_update()
{
	updateParams();

	_man_roll_scale = _fw_man_roll_scale.get();
	_man_pitch_scale = _fw_man_pitch_scale.get();
	_man_yaw_scale = _fw_man_yaw_scale.get();

	_acro_roll_max = _fw_acro_roll_max.get();
	_acro_pitch_max = _fw_acro_pitch_max.get();
	_acro_yaw_max = _fw_acro_yaw_max.get();

	_man_roll_max = _fw_man_roll_max.get();
	_man_pitch_max = _fw_man_pitch_max.get();

	_roll_offset = _fw_roll_offset.get();
	_pitch_offset = _fw_pitch_offset.get();

}
