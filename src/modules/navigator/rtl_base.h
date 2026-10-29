/***************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file rtl_base.h
 *
 * Helper class for Return modes using the mission
 *
 */

#pragma once

#include "mission_base.h"
#include <uORB/topics/rtl_time_estimate.h>
#include <matrix/math.hpp>

class RtlBase : public MissionBase
{
public:
	RtlBase(Navigator *navigator, int32_t dataman_cache_size_signed):
		MissionBase(navigator, dataman_cache_size_signed, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) {};
	virtual ~RtlBase() = default;

	virtual rtl_time_estimate_s calc_rtl_time_estimate() = 0;

	virtual void setReturnAltMin(bool min) { (void)min;};

	virtual void setRtlAlt(float alt) { (void)alt;};

	/** Refresh the cached mission before RTL selects the destination and activates this mode. */
	void refreshMission() { updateMavlinkMission(); }

#if CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
	virtual matrix::Vector2d getRtlPlannerDestination() { return {(double)NAN, (double)NAN}; }
#endif // CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE

protected:
	/** Return never takes off, so it does not start its items while landed, also when they reload. */
	bool canStartMission() const override { return !_land_detected_sub.get().landed; }

	/** Convert a caller-selected position item into a waypoint, retaining loiter-to-altitude. */
	static void makeRtlPositionItem(mission_item_s &item)
	{
		if (item.nav_cmd != NAV_CMD_LOITER_TO_ALT) {
			item.nav_cmd = NAV_CMD_WAYPOINT;
		}

		item.autocontinue = true;
		item.time_inside = 0.f;
	}
};
