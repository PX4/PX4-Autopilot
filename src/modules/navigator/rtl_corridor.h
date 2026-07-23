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
/**
 * @file rtl_corridor.h
 *
 * Fly the flight corridor graph's shortest path to the nest or a rally point.
 *
 * Flies through all corridor waypoints, including the final one (the standoff
 * point directly above the landing pad, at the node's own altitude), then hands
 * off to RtlDirect for the terminal descent and landing in place. Never reaches
 * its own "land" state.
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module_params.h>
#include <dataman/dataman.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/rtl_time_estimate.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>

#include <lib/rtl/rtl_time_estimator.h>
#include "mission_block.h"
#include "navigation.h"

class Navigator;

class RtlCorridor : public MissionBlock, public ModuleParams
{
public:
	RtlCorridor(Navigator *navigator);
	~RtlCorridor() = default;

	void on_activation() override;
	void on_active() override;
	void on_inactivation() override;
	void on_inactive() override;

	/**
	 * @brief Calculate the estimated time needed to reach the final corridor
	 * waypoint and complete the landing procedure there.
	 */
	rtl_time_estimate_s calc_rtl_time_estimate();

	/**
	 * @brief Set the ordered corridor waypoint sequence to fly (start node
	 * first, nest/rally goal node last). Only takes effect before activation,
	 * mirroring RtlDirect::setRtlPosition().
	 */
	void setPath(const mission_corridor_node_s *waypoints, uint8_t num_waypoints);

	/**
	 * @brief True once the vehicle has reached the final corridor waypoint (the
	 * standoff point above the landing pad). RTL hands off to RtlDirect for the
	 * terminal descent and landing at finalWaypoint() once this is true.
	 */
	bool hasReachedFinalWaypoint() const { return _reached_final_waypoint; }

	/**
	 * @brief The final corridor waypoint (the nest, or the chosen rally
	 * point). Only valid if setPath() was called with num_waypoints > 0.
	 */
	const mission_corridor_node_s &finalWaypoint() const
	{
		return _waypoints[_num_waypoints > 0 ? _num_waypoints - 1 : 0];
	}

private:
	void setCurrentWaypointItem();

	mission_corridor_node_s _waypoints[DM_KEY_CORRIDOR_NODES_MAX] {};
	uint8_t _num_waypoints{0};
	uint8_t _current_index{0};
	bool _reached_final_waypoint{false};

	RtlTimeEstimator _rtl_time_estimator;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RTL_DESCEND_ALT>) _param_rtl_descend_alt,
		(ParamFloat<px4::params::RTL_LAND_DELAY>)  _param_rtl_land_delay
	)

	uORB::SubscriptionData<vehicle_global_position_s> _global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};
};
