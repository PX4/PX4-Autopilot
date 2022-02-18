/****************************************************************************
*
*   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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


#pragma once

#include <stdint.h>
#include <lib/parameters/param.h>

namespace gimbal
{

struct Parameters {
	int32_t mnt_mode_in;
	int32_t mnt_mode_out;
	int32_t mnt_mav_sysid_v1;
	int32_t mnt_mav_compid_v1;
	float mnt_ob_lock_mode;
	float mnt_ob_norm_mode;
	int32_t mnt_man_pitch;
	int32_t mnt_man_roll;
	int32_t mnt_man_yaw;
	int32_t mnt_do_stab;
	float mnt_range_pitch;
	float mnt_range_roll;
	float mnt_range_yaw;
	float mnt_off_pitch;
	float mnt_off_roll;
	float mnt_off_yaw;
	int32_t mav_sysid;
	int32_t mav_compid;
	float mnt_rate_pitch;
	float mnt_rate_yaw;
	int32_t mnt_rc_in_mode;
	float mnt_lnd_p_min;
	float mnt_lnd_p_max;
};

struct ParameterHandles {
	param_t mnt_mode_in;
	param_t mnt_mode_out;
	param_t mnt_mav_sysid_v1;
	param_t mnt_mav_compid_v1;
	param_t mnt_ob_lock_mode;
	param_t mnt_ob_norm_mode;
	param_t mnt_man_pitch;
	param_t mnt_man_roll;
	param_t mnt_man_yaw;
	param_t mnt_do_stab;
	param_t mnt_range_pitch;
	param_t mnt_range_roll;
	param_t mnt_range_yaw;
	param_t mnt_off_pitch;
	param_t mnt_off_roll;
	param_t mnt_off_yaw;
	param_t mav_sysid;
	param_t mav_compid;
	param_t mnt_rate_pitch;
	param_t mnt_rate_yaw;
	param_t mnt_rc_in_mode;
	param_t mnt_lnd_p_min;
	param_t mnt_lnd_p_max;
};

} /* namespace gimbal */
