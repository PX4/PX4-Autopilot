/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
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
 * @file RCMap.hpp
 *
 * Base/interface class to map RC input data to manual_control_setpoint.
 *
 * @author Dennis Mannhart <dennis@yuneecresearch.com>
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include "RCMap.hpp"

#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/input_rc.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <px4_platform_common/module_params.h>

namespace sensors
{

class RCMap : public ModuleParams
{

public:
	RCMap() : ModuleParams(nullptr)
	{}

	virtual ~RCMap() = default;

	virtual int map(manual_control_setpoint_s &man, const input_rc_s &input_rc, int32_t &parameters) = 0;

	enum class Error : int {
		None = 0,
		Version
	};

	enum class AUX : int {
		nothing = 0,
		mission,
		flexi_release
	};

protected:
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RCMAP_AUX>)    _param_rcmap_aux
	);

};

} // namespace sensors
