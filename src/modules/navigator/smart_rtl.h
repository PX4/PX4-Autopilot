/***************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/module_params.h>

#include "navigator_mode.h"
#include "mission_block.h"
#include "tracker.h"

class Navigator;

class SmartRTL : public MissionBlock, public ModuleParams
{
public:
	SmartRTL(Navigator *navigator);
	~SmartRTL() = default;

	void on_inactive() override;
	void on_activation() override;
	void on_active() override;

private:
	// Inits a setpoint
	void init_setpoint(position_setpoint_s &sp);

	// Updates the deadline from the current time and parameters.
	void update_deadline();

	// Returns the current distance/bearing to a setpoint
	float distance_to_setpoint(position_setpoint_s &sp);
	float bearing_to_setpoint(position_setpoint_s &sp);

	// Prints information about the specified setpoint
	void dump_setpoint(const char *name, position_setpoint_s &sp);

	// Advances the setpoint triplet by loading the next position from the return path.
	// Returns true if progress was made.
	bool advance_setpoint_triplet(position_setpoint_triplet_s *pos_sp_triplet);

	Tracker *_tracker;

	Tracker::path_finding_context_t current_return_context;
	Tracker::path_finding_context_t next_return_context;

	hrt_abstime deadline{HRT_ABSTIME_MAX}; // This deadline makes sure that progress is made.
	bool land_after_deadline; // If true and the deadline is reached, land, otherwise, fall back to basic RTL.

	hrt_abstime next_log{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RTL_FALLBCK_DLY>) _param_rtl_fallbck_dly,
		(ParamFloat<px4::params::RTL_LAND_DELAY>) _param_rtl_land_delay
	)
};
