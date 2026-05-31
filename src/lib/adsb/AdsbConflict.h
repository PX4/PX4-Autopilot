/****************************************************************************
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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <lib/geo/geo.h>
#if defined(CONFIG_NAVIGATOR_ADSB_F3442) && CONFIG_NAVIGATOR_ADSB_F3442
#include "f3442_standard/DaaF3442.h"
#else
#include "crosstrack_based_daa/DaaCrosstrack.h"
#endif
#include "DaaHelper.h"
#include <matrix/math.hpp>

#include <drivers/drv_hrt.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/detect_and_avoid.h>

using namespace time_literals;

static constexpr uint64_t kConflictWarningTimeout{60_s};

class AdsbConflict
{
public:
	AdsbConflict() = default;
	~AdsbConflict() = default;

	/**
	 * @brief Validate the ownship + transponder inputs and run them through the built standard.
	 *
	 * Returns false on non-finite inputs or when the built standard needs a heading
	 * that the report does not provide.
	 */
	bool handle_traffic(const matrix::Vector2d &uav_lat_lon, const float uav_alt,
			    const float uav_heading,
			    const matrix::Vector3f &uav_vel_ned, const transponder_report_s &transponder_report, detect_and_avoid_s &daa_output);

	/** @brief Refresh the built standard's parameter cache. Returns false on any invalid value. */
	bool try_updating_params();

private:
#if defined(CONFIG_NAVIGATOR_ADSB_F3442) && CONFIG_NAVIGATOR_ADSB_F3442
	DaaF3442 _daa;
#else
	DaaCrosstrack _daa;
#endif
};
