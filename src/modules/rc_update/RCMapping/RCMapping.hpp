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
 * @file RCMapping.hpp
 *
 * Class deciding and switching between different input mappings from different remotes.
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include "RCMap.hpp"
#include "RCMapST16.hpp"
#include "RCMapST10C.hpp"

namespace sensors
{

class RCMapping
{
public:
	RCMapping() = default;
	virtual ~RCMapping() = default;

	int map(manual_control_setpoint_s &manual_control_setpoint, const input_rc_s &input_rc,
		int32_t &parameters)
	{
		// check for the M4 raw output channel mapping version embedded in channel 9 to decide which remote it is
		_version = input_rc.values[9 - 1] >> 8 & 0xF;

		switch (_version) {
		case RCMapST16::RAW_CHANNEL_MAPPING_VER_ST16:
			return _st16.map(manual_control_setpoint, input_rc, parameters);

		case RCMapST10C::RAW_CHANNEL_MAPPING_VER_ST10C:
			return _st10c.map(manual_control_setpoint, input_rc, parameters);

		default:
			return (int)RCMap::Error::Version;
		}
	}

	int mapSlave(manual_control_setpoint_s &manual_control_setpoint, const input_rc_s &slave_rc, int32_t parameters)
	{
		// the case where the slave != master is NOT yet supported
		switch (_version) {
		case RCMapST16::RAW_CHANNEL_MAPPING_VER_ST16:
			return _st16.mapSlave(manual_control_setpoint, slave_rc);

		default:
			return (int)RCMap::Error::Version;
		}
	}

private:
	RCMapST16 _st16;
	RCMapST10C _st10c;
	int _version;
};

} // namespace sensors
