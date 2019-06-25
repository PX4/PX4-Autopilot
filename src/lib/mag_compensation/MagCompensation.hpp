/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file MagCompensation.hpp
 * @author Roman Bapst <roman@auterion.com>
 *
 *  Library for magnetometer data compensation.
 *
 */

#pragma once

#include <px4_module_params.h>
#include <matrix/matrix/math.hpp>

class MagCompensator : ModuleParams
{
public:
	MagCompensator();

	~MagCompensator() = default;

	void update_throttle_and_armed_flag(float throttle, bool armed);

	void calculate_mag_corrected(matrix::Vector3f &mag);

	void update_parameters() { ModuleParams::updateParams(); }

	uint32_t get_target_mag_id() { return _param_mag_compensation_id.get(); }

private:
	float _throttle{0};
	bool _armed{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CAL_DMAGX_D_THR>) _param_d_magX_d_throttle,
		(ParamFloat<px4::params::CAL_DMAGY_D_THR>) _param_d_magY_d_throttle,
		(ParamFloat<px4::params::CAL_DMAGZ_D_THR>) _param_d_magZ_d_throttle,
		(ParamInt<px4::params::CAL_MAG_COMP_ID>) _param_mag_compensation_id
	)

};
