/***************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file DaaCrosstrack.h
 *
 * Helper class to do detect and avoid based on crosstrack distance.
 *
 */

#pragma once

#include "../DaaHelper.h"

#include <cstdint>

#include <px4_platform_common/module_params.h>

static constexpr float kTrafficToUavDistanceExtension{1000.0f};

/**
 * @brief Crosstrack-distance DAA standard.
 *
 * Projects the traffic forward along its heading and declares a single
 * high-priority conflict when the ownship is within the configured crosstrack,
 * vertical separation, and time-to-collision thresholds. Useful for
 * transponders that have a reliable heading but no standardised conflict zones.
 */
class DaaCrosstrack : public ModuleParams
{
public:
	DaaCrosstrack();

	// Crosstrack-based conflict level for one traffic target.
	uint8_t calculate_daa_stats(const aircraft_state_s &uav_state, const aircraft_state_s &traffic_state,
				    daa_stats_s &daa_stats) const;

	// Refresh the crosstrack thresholds from parameters. False on invalid values.
	bool try_setting_params();

private:
	float _crosstrack_separation_m{500.f};
	float _vertical_separation_m{500.f};
	int _collision_time_threshold_s{60};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::NAV_TRAFF_A_VER>) _param_nav_traff_a_ver,
		(ParamFloat<px4::params::NAV_TRAFF_A_HOR>) _param_nav_traff_a_hor,
		(ParamInt<px4::params::NAV_TRAFF_COLL_T>) _param_nav_traff_coll_t
	)
};
