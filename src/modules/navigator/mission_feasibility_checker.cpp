/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file mission_feasibility_checker.cpp
 * Provides checks if mission is feasible given the navigation capabilities
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 */

#include "mission_feasibility_checker.h"

#include <geo/geo.h>
#include <math.h>
#include <mathlib/mathlib.h>
#include <mavlink/mavlink_log.h>
#include <fw_pos_control_l1/landingslope.h>
#include <systemlib/err.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <uORB/topics/fence.h>
#include <uORB/topics/vehicle_command.h>

MissionFeasibilityChecker::MissionFeasibilityChecker() :
	_mavlink_fd(-1),
	_capabilities_sub(-1),
	_initDone(false),
	_dist_1wp_ok(false)
{
	_nav_caps = {0};
}


bool MissionFeasibilityChecker::checkMissionFeasible(int mavlink_fd, bool isRotarywing,
	dm_item_t dm_current, size_t nMissionItems, Geofence &geofence,
	float home_alt, bool home_valid, double curr_lat, double curr_lon, float max_waypoint_distance, bool &warning_issued,
	float default_acceptance_rad,
	bool condition_landed)
{
	bool failed = false;
	bool warned = false;
	/* Init if not done yet */
	init();

	_mavlink_fd = mavlink_fd;

	// first check if we have a valid position
	if (!home_valid /* can later use global / local pos for finer granularity */) {
		failed = true;
		warned = true;
		mavlink_log_info(_mavlink_fd, "Not yet ready for mission, no position lock.");
	} else {
		failed = failed || !check_dist_1wp(dm_current, nMissionItems, curr_lat, curr_lon, max_waypoint_distance, warning_issued);
	}

	// check if all mission item commands are supported
	failed = failed || !checkMissionItemValidity(dm_current, nMissionItems, condition_landed);
	failed = failed || !checkGeofence(dm_current, nMissionItems, geofence);
	failed = failed || !checkHomePositionAltitude(dm_current, nMissionItems, home_alt, home_valid, warned);
	failed = failed || !checkMissionFeasibleFixedwing(dm_current, nMissionItems, geofence, home_alt, home_valid);


	return failed;
}

bool MissionFeasibilityChecker::checkMissionFeasibleRotarywing(dm_item_t dm_current, size_t nMissionItems,
	Geofence &geofence, float home_alt, bool home_valid, float default_acceptance_rad)
{
	/* Check if all all waypoints are above the home altitude, only return false if bool throw_error = true */
	for (size_t i = 0; i < nMissionItems; i++) {
		struct mission_item_s missionitem;
		const ssize_t len = sizeof(struct mission_item_s);

		if (dm_read(dm_current, i, &missionitem, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			return false;
		}

		// look for a takeoff waypoint
		if (missionitem.nav_cmd == NAV_CMD_TAKEOFF) {
			// make sure that the altitude of the waypoint is at least one meter larger than the acceptance radius
			// this makes sure that the takeoff waypoint is not reached before we are at least one meter in the air
			float takeoff_alt = missionitem.altitude_is_relative
				      ? missionitem.altitude
			              : missionitem.altitude - home_alt;
			// check if we should use default acceptance radius
			float acceptance_radius = default_acceptance_rad;

			if (missionitem.acceptance_radius > NAV_EPSILON_POSITION) {
				acceptance_radius = missionitem.acceptance_radius;
			}

			if (takeoff_alt - 1.0f < acceptance_radius) {
				mavlink_log_critical(_mavlink_fd, "Mission rejected: Takeoff altitude too low!");
				return false;
			}
		}
	}

	// all checks have passed
	return true;
}

bool MissionFeasibilityChecker::checkMissionFeasibleFixedwing(dm_item_t dm_current, size_t nMissionItems, Geofence &geofence, float home_alt, bool home_valid)
{
	/* Update fixed wing navigation capabilites */
	updateNavigationCapabilities();

	/* Perform checks and issue feedback to the user for all checks */
	bool resLanding = checkFixedWingLanding(dm_current, nMissionItems);

	/* Mission is only marked as feasible if all checks return true */
	return resLanding;
}

bool MissionFeasibilityChecker::checkGeofence(dm_item_t dm_current, size_t nMissionItems, Geofence &geofence)
{
	/* Check if all mission items are inside the geofence (if we have a valid geofence) */
	if (geofence.valid()) {
		for (size_t i = 0; i < nMissionItems; i++) {
			struct mission_item_s missionitem;
			const ssize_t len = sizeof(missionitem);

			if (dm_read(dm_current, i, &missionitem, len) != len) {
				/* not supposed to happen unless the datamanager can't access the SD card, etc. */
				return false;
			}

			if (!geofence.inside_polygon(missionitem.lat, missionitem.lon, missionitem.altitude)) {
				mavlink_log_critical(_mavlink_fd, "Geofence violation for waypoint %d", i);
				return false;
			}
		}
	}

	return true;
}

bool MissionFeasibilityChecker::checkHomePositionAltitude(dm_item_t dm_current, size_t nMissionItems,
	float home_alt, bool home_valid, bool &warning_issued, bool throw_error)
{
	/* Check if all all waypoints are above the home altitude, only return false if bool throw_error = true */
	for (size_t i = 0; i < nMissionItems; i++) {
		struct mission_item_s missionitem;
		const ssize_t len = sizeof(struct mission_item_s);

		if (dm_read(dm_current, i, &missionitem, len) != len) {
			warning_issued = true;
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			return false;
		}

		/* always reject relative alt without home set */
		if (missionitem.altitude_is_relative && !home_valid) {
			mavlink_log_critical(_mavlink_fd, "Rejecting Mission: No home pos, WP %d uses rel alt", i);
			warning_issued = true;
			return false;
		}

		/* calculate the global waypoint altitude */
		float wp_alt = (missionitem.altitude_is_relative) ? missionitem.altitude + home_alt : missionitem.altitude;

		if (home_alt > wp_alt) {

			warning_issued = true;

			if (throw_error) {
				mavlink_log_critical(_mavlink_fd, "Rejecting Mission: Waypoint %d below home", i);
				return false;
			} else	{
				mavlink_log_critical(_mavlink_fd, "Warning: Waypoint %d below home", i);
				return true;
			}
		}
	}

	return true;
}

bool MissionFeasibilityChecker::checkMissionItemValidity(dm_item_t dm_current, size_t nMissionItems, bool condition_landed) {
	// do not allow mission if we find unsupported item
	for (size_t i = 0; i < nMissionItems; i++) {
		struct mission_item_s missionitem;
		const ssize_t len = sizeof(struct mission_item_s);

		if (dm_read(dm_current, i, &missionitem, len) != len) {
			// not supposed to happen unless the datamanager can't access the SD card, etc.
			mavlink_log_critical(_mavlink_fd, "Rejecting Mission: Cannot access SD card");
			return true;
		}

		// check if we find unsupported items and reject mission if so
		if (missionitem.nav_cmd != NAV_CMD_IDLE &&
			missionitem.nav_cmd != NAV_CMD_WAYPOINT &&
			missionitem.nav_cmd != NAV_CMD_LOITER_UNLIMITED &&
			missionitem.nav_cmd != NAV_CMD_LOITER_TURN_COUNT &&
			missionitem.nav_cmd != NAV_CMD_LOITER_TIME_LIMIT &&
			missionitem.nav_cmd != NAV_CMD_LAND &&
			missionitem.nav_cmd != NAV_CMD_TAKEOFF &&
			missionitem.nav_cmd != NAV_CMD_PATHPLANNING &&
			missionitem.nav_cmd != NAV_CMD_DO_JUMP &&
			missionitem.nav_cmd != NAV_CMD_DO_SET_SERVO &&
			missionitem.nav_cmd != vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL &&
			missionitem.nav_cmd != vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION) {

			mavlink_log_critical(_mavlink_fd, "Rejecting mission item %i: unsupported action.", (int)(i+1));
			return true;
		}

		// check if the mission starts with a land command while the vehicle is landed
		if (missionitem.nav_cmd == NAV_CMD_LAND &&
			i == 0 &&
			condition_landed) {

			mavlink_log_critical(_mavlink_fd, "Rejecting mission that starts with LAND command while vehicle is landed.");
			return true;
		}


	}
	return true;
}

bool MissionFeasibilityChecker::checkFixedWingLanding(dm_item_t dm_current, size_t nMissionItems)
{
	/* Go through all mission items and search for a landing waypoint
	 * if landing waypoint is found: the previous waypoint is checked to be at a feasible distance and altitude given the landing slope */


	for (size_t i = 0; i < nMissionItems; i++) {
		struct mission_item_s missionitem;
		const ssize_t len = sizeof(missionitem);
		dm_read(dm_current, i, &missionitem, len);

		if (missionitem.nav_cmd == NAV_CMD_LAND) {
			struct mission_item_s missionitem_previous;
			if (i != 0) {
			 dm_read(dm_current, i-1, &missionitem_previous, len);

				float wp_distance = get_distance_to_next_waypoint(missionitem_previous.lat , missionitem_previous.lon, missionitem.lat, missionitem.lon);
				 Landingslope::getLandingSlopeAbsoluteAltitude(wp_distance, missionitem.altitude, _nav_caps.landing_horizontal_slope_displacement, _nav_caps.landing_slope_angle_rad);
				 Landingslope::getLandingSlopeWPDistance(missionitem_previous.altitude, missionitem.altitude, _nav_caps.landing_horizontal_slope_displacement, _nav_caps.landing_slope_angle_rad);

			}
		}
	}

	/* No landing waypoints or no waypoints */
	return false;
}

bool
MissionFeasibilityChecker::check_dist_1wp(dm_item_t dm_current, size_t nMissionItems, double curr_lat, double curr_lon, float dist_first_wp, bool &warning_issued)
{
	if (_dist_1wp_ok) {
		/* always return true after at least one successful check */
		return false;
	}

	/* check if first waypoint is not too far from home */
	if (dist_first_wp > 0.0f) {
		struct mission_item_s mission_item;

		/* find first waypoint (with lat/lon) item in datamanager */
		for (unsigned i = 0; i < nMissionItems; i++) {
			if (dm_read(dm_current, i,
					&mission_item, sizeof(mission_item_s)) == sizeof(mission_item_s)) {
				/* Check non navigation item */
									/* check distance from current position to item */
				get_distance_to_next_waypoint(
							mission_item.lat, mission_item.lon, curr_lat, curr_lon);



			}
		}

		/* no waypoints found in mission, then we will not fly far away */
		_dist_1wp_ok = false;


	}
	return false;

}

void MissionFeasibilityChecker::updateNavigationCapabilities()
{
	(void)orb_copy(ORB_ID(navigation_capabilities), _capabilities_sub, &_nav_caps);
}

void MissionFeasibilityChecker::init()
{
	if (!_initDone) {

		_capabilities_sub = orb_subscribe(ORB_ID(navigation_capabilities));

		_initDone = true;
	}
}
