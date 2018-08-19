/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @author Sander Smeets <sander@droneslab.com>
 */

#include "mission_feasibility_checker.h"

#include "mission_block.h"
#include "navigator.h"

#include <drivers/drv_pwm_output.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/landing_slope/Landingslope.hpp>
#include <systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/position_controller_landing_status.h>

bool
MissionFeasibilityChecker::checkMissionFeasible(const mission_s &mission,
		float max_distance_to_1st_waypoint, float max_distance_between_waypoints,
		bool land_start_req)
{
	bool failed = false;
	bool warned = false;

	// first check if we have a valid position
	const bool home_valid = _navigator->home_position_valid();
	const bool home_alt_valid = _navigator->home_alt_valid();

	if (!home_alt_valid) {
		failed = true;
		warned = true;
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "Not yet ready for mission, no position lock.");

	} else {
		failed = failed || !checkDistanceToFirstWaypoint(mission, max_distance_to_1st_waypoint);
	}

	const float home_alt = _navigator->get_home_position()->alt;

	// check if all mission item commands are supported
	failed = failed || !checkMissionItemValidity(mission);
	failed = failed || !checkDistancesBetweenWaypoints(mission, max_distance_between_waypoints);
	failed = failed || !checkGeofence(mission, home_alt, home_valid);
	failed = failed || !checkHomePositionAltitude(mission, home_alt, home_alt_valid, warned);

	// VTOL always respects rotary wing feasibility
	if (_navigator->get_vstatus()->is_rotary_wing || _navigator->get_vstatus()->is_vtol) {
		failed = failed || !checkRotarywing(mission, home_alt, home_alt_valid);

	} else {
		failed = failed || !checkFixedwing(mission, home_alt, home_alt_valid, land_start_req);
	}

	return !failed;
}

bool
MissionFeasibilityChecker::checkRotarywing(const mission_s &mission, float home_alt, bool home_alt_valid)
{
	for (size_t i = 0; i < mission.count; i++) {
		struct mission_item_s missionitem = {};
		const ssize_t len = sizeof(struct mission_item_s);

		if (dm_read((dm_item_t)mission.dataman_id, i, &missionitem, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			return false;
		}

		// look for a takeoff waypoint
		if (missionitem.nav_cmd == NAV_CMD_TAKEOFF) {
			// make sure that the altitude of the waypoint is at least one meter larger than the altitude acceptance radius
			// this makes sure that the takeoff waypoint is not reached before we are at least one meter in the air
			const float takeoff_alt = missionitem.altitude_is_relative ? missionitem.altitude : missionitem.altitude - home_alt;

			// check if we should use default acceptance radius
			float acceptance_radius = _navigator->get_altitude_acceptance_radius();

			// if a specific acceptance radius has been defined, use that one instead
			if (missionitem.acceptance_radius > NAV_EPSILON_POSITION) {
				acceptance_radius = missionitem.acceptance_radius;
			}

			if (takeoff_alt - 1.0f < acceptance_radius) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: Takeoff altitude too low!");
				return false;
			}
		}
	}

	// all checks have passed
	return true;
}

bool
MissionFeasibilityChecker::checkFixedwing(const mission_s &mission, float home_alt, bool home_alt_valid,
		bool land_start_req)
{
	/* Perform checks and issue feedback to the user for all checks */
	bool resTakeoff = checkFixedWingTakeoff(mission, home_alt, home_alt_valid);
	bool resLanding = checkFixedWingLanding(mission, land_start_req);

	/* Mission is only marked as feasible if all checks return true */
	return (resTakeoff && resLanding);
}

bool
MissionFeasibilityChecker::checkGeofence(const mission_s &mission, float home_alt, bool home_valid)
{
	if (_navigator->get_geofence().isHomeRequired() && !home_valid) {
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence requires valid home position");
		return false;
	}

	/* Check if all mission items are inside the geofence (if we have a valid geofence) */
	if (_navigator->get_geofence().valid()) {
		for (size_t i = 0; i < mission.count; i++) {
			struct mission_item_s missionitem = {};
			const ssize_t len = sizeof(missionitem);

			if (dm_read((dm_item_t)mission.dataman_id, i, &missionitem, len) != len) {
				/* not supposed to happen unless the datamanager can't access the SD card, etc. */
				return false;
			}

			if (missionitem.altitude_is_relative && !home_valid) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence requires valid home position");
				return false;
			}

			// Geofence function checks against home altitude amsl
			missionitem.altitude = missionitem.altitude_is_relative ? missionitem.altitude + home_alt : missionitem.altitude;

			if (MissionBlock::item_contains_position(missionitem) && !_navigator->get_geofence().check(missionitem)) {

				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence violation for waypoint %d", i + 1);
				return false;
			}
		}
	}

	return true;
}

bool
MissionFeasibilityChecker::checkHomePositionAltitude(const mission_s &mission, float home_alt, bool home_alt_valid,
		bool throw_error)
{
	/* Check if all waypoints are above the home altitude */
	for (size_t i = 0; i < mission.count; i++) {
		struct mission_item_s missionitem = {};
		const ssize_t len = sizeof(struct mission_item_s);

		if (dm_read((dm_item_t)mission.dataman_id, i, &missionitem, len) != len) {
			_navigator->get_mission_result()->warning = true;
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			return false;
		}

		/* reject relative alt without home set */
		if (missionitem.altitude_is_relative && !home_alt_valid && MissionBlock::item_contains_position(missionitem)) {

			_navigator->get_mission_result()->warning = true;

			if (throw_error) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: No home pos, WP %d uses rel alt", i + 1);
				return false;

			} else	{
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Warning: No home pos, WP %d uses rel alt", i + 1);
				return true;
			}
		}

		/* calculate the global waypoint altitude */
		float wp_alt = (missionitem.altitude_is_relative) ? missionitem.altitude + home_alt : missionitem.altitude;

		if ((home_alt > wp_alt) && MissionBlock::item_contains_position(missionitem)) {

			_navigator->get_mission_result()->warning = true;

			if (throw_error) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: Waypoint %d below home", i + 1);
				return false;

			} else	{
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Warning: Waypoint %d below home", i + 1);
				return true;
			}
		}
	}

	return true;
}

bool
MissionFeasibilityChecker::checkMissionItemValidity(const mission_s &mission)
{
	// do not allow mission if we find unsupported item
	for (size_t i = 0; i < mission.count; i++) {
		struct mission_item_s missionitem;
		const ssize_t len = sizeof(struct mission_item_s);

		if (dm_read((dm_item_t)mission.dataman_id, i, &missionitem, len) != len) {
			// not supposed to happen unless the datamanager can't access the SD card, etc.
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: Cannot access SD card");
			return false;
		}

		// check if we find unsupported items and reject mission if so
		if (missionitem.nav_cmd != NAV_CMD_IDLE &&
		    missionitem.nav_cmd != NAV_CMD_WAYPOINT &&
		    missionitem.nav_cmd != NAV_CMD_LOITER_UNLIMITED &&
		    missionitem.nav_cmd != NAV_CMD_LOITER_TIME_LIMIT &&
		    missionitem.nav_cmd != NAV_CMD_RETURN_TO_LAUNCH &&
		    missionitem.nav_cmd != NAV_CMD_LAND &&
		    missionitem.nav_cmd != NAV_CMD_TAKEOFF &&
		    missionitem.nav_cmd != NAV_CMD_LOITER_TO_ALT &&
		    missionitem.nav_cmd != NAV_CMD_VTOL_TAKEOFF &&
		    missionitem.nav_cmd != NAV_CMD_VTOL_LAND &&
		    missionitem.nav_cmd != NAV_CMD_DELAY &&
		    missionitem.nav_cmd != NAV_CMD_DO_JUMP &&
		    missionitem.nav_cmd != NAV_CMD_DO_CHANGE_SPEED &&
		    missionitem.nav_cmd != NAV_CMD_DO_SET_HOME &&
		    missionitem.nav_cmd != NAV_CMD_DO_SET_SERVO &&
		    missionitem.nav_cmd != NAV_CMD_DO_LAND_START &&
		    missionitem.nav_cmd != NAV_CMD_DO_TRIGGER_CONTROL &&
		    missionitem.nav_cmd != NAV_CMD_DO_DIGICAM_CONTROL &&
		    missionitem.nav_cmd != NAV_CMD_IMAGE_START_CAPTURE &&
		    missionitem.nav_cmd != NAV_CMD_IMAGE_STOP_CAPTURE &&
		    missionitem.nav_cmd != NAV_CMD_VIDEO_START_CAPTURE &&
		    missionitem.nav_cmd != NAV_CMD_VIDEO_STOP_CAPTURE &&
		    missionitem.nav_cmd != NAV_CMD_DO_MOUNT_CONFIGURE &&
		    missionitem.nav_cmd != NAV_CMD_DO_MOUNT_CONTROL &&
		    missionitem.nav_cmd != NAV_CMD_DO_SET_ROI &&
		    missionitem.nav_cmd != NAV_CMD_DO_SET_ROI_LOCATION &&
		    missionitem.nav_cmd != NAV_CMD_DO_SET_ROI_WPNEXT_OFFSET &&
		    missionitem.nav_cmd != NAV_CMD_DO_SET_ROI_NONE &&
		    missionitem.nav_cmd != NAV_CMD_DO_SET_CAM_TRIGG_DIST &&
		    missionitem.nav_cmd != NAV_CMD_DO_SET_CAM_TRIGG_INTERVAL &&
		    missionitem.nav_cmd != NAV_CMD_SET_CAMERA_MODE &&
		    missionitem.nav_cmd != NAV_CMD_DO_VTOL_TRANSITION) {

			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: item %i: unsupported cmd: %d", (int)(i + 1),
					     (int)missionitem.nav_cmd);
			return false;
		}

		/* Check non navigation item */
		if (missionitem.nav_cmd == NAV_CMD_DO_SET_SERVO) {

			/* check actuator number */
			if (missionitem.params[0] < 0 || missionitem.params[0] > 5) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Actuator number %d is out of bounds 0..5",
						     (int)missionitem.params[0]);
				return false;
			}

			/* check actuator value */
			if (missionitem.params[1] < -PWM_DEFAULT_MAX || missionitem.params[1] > PWM_DEFAULT_MAX) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(),
						     "Actuator value %d is out of bounds -PWM_DEFAULT_MAX..PWM_DEFAULT_MAX", (int)missionitem.params[1]);
				return false;
			}
		}

		// check if the mission starts with a land command while the vehicle is landed
		if ((i == 0) && missionitem.nav_cmd == NAV_CMD_LAND && _navigator->get_land_detected()->landed) {

			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: starts with landing");
			return false;
		}
	}

	return true;
}

bool
MissionFeasibilityChecker::checkFixedWingTakeoff(const mission_s &mission, float home_alt, bool home_alt_valid)
{
	for (size_t i = 0; i < mission.count; i++) {
		struct mission_item_s missionitem = {};
		const ssize_t len = sizeof(struct mission_item_s);

		if (dm_read((dm_item_t)mission.dataman_id, i, &missionitem, len) != len) {
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
			float acceptance_radius = _navigator->get_default_acceptance_radius();

			if (missionitem.acceptance_radius > NAV_EPSILON_POSITION) {
				acceptance_radius = missionitem.acceptance_radius;
			}

			if (takeoff_alt - 1.0f < acceptance_radius) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: Takeoff altitude too low!");
				return false;
			}
		}
	}

	// all checks have passed
	return true;
}

bool
MissionFeasibilityChecker::checkFixedWingLanding(const mission_s &mission, bool land_start_req)
{
	/* Go through all mission items and search for a landing waypoint
	 * if landing waypoint is found: the previous waypoint is checked to be at a feasible distance and altitude given the landing slope */

	bool landing_valid = false;

	bool land_start_found = false;
	size_t do_land_start_index = 0;
	size_t landing_approach_index = 0;

	for (size_t i = 0; i < mission.count; i++) {
		struct mission_item_s missionitem;
		const ssize_t len = sizeof(missionitem);

		if (dm_read((dm_item_t)mission.dataman_id, i, &missionitem, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			return false;
		}

		// if DO_LAND_START found then require valid landing AFTER
		if (missionitem.nav_cmd == NAV_CMD_DO_LAND_START) {
			if (land_start_found) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: more than one land start.");
				return false;

			} else {
				land_start_found = true;
				do_land_start_index = i;
			}
		}

		if (missionitem.nav_cmd == NAV_CMD_LAND) {
			mission_item_s missionitem_previous {};

			if (i > 0) {
				landing_approach_index = i - 1;

				if (dm_read((dm_item_t)mission.dataman_id, landing_approach_index, &missionitem_previous, len) != len) {
					/* not supposed to happen unless the datamanager can't access the SD card, etc. */
					return false;
				}

				if (MissionBlock::item_contains_position(missionitem_previous)) {

					uORB::Subscription<position_controller_landing_status_s> landing_status{ORB_ID(position_controller_landing_status)};
					landing_status.forcedUpdate();

					const bool landing_status_valid = (landing_status.get().timestamp > 0);
					const float wp_distance = get_distance_to_next_waypoint(missionitem_previous.lat, missionitem_previous.lon,
								  missionitem.lat, missionitem.lon);

					if (landing_status_valid && (wp_distance > landing_status.get().flare_length)) {
						/* Last wp is before flare region */

						const float delta_altitude = missionitem.altitude - missionitem_previous.altitude;

						if (delta_altitude < 0) {

							const float horizontal_slope_displacement = landing_status.get().horizontal_slope_displacement;
							const float slope_angle_rad = landing_status.get().slope_angle_rad;
							const float slope_alt_req = Landingslope::getLandingSlopeAbsoluteAltitude(wp_distance, missionitem.altitude,
										    horizontal_slope_displacement, slope_angle_rad);

							if (missionitem_previous.altitude > slope_alt_req + 1.0f) {
								/* Landing waypoint is above altitude of slope at the given waypoint distance (with small tolerance for floating point discrepancies) */
								mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: adjust landing approach.");

								const float wp_distance_req = Landingslope::getLandingSlopeWPDistance(missionitem_previous.altitude,
											      missionitem.altitude, horizontal_slope_displacement, slope_angle_rad);

								mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Move down %d m or move further away by %d m.",
										     (int)ceilf(slope_alt_req - missionitem_previous.altitude),
										     (int)ceilf(wp_distance_req - wp_distance));

								return false;
							}

						} else {
							/* Landing waypoint is above last waypoint */
							mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: landing above last waypoint.");
							return false;
						}

					} else {
						/* Last wp is in flare region */
						mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: waypoint within landing flare.");
						return false;
					}

					landing_valid = true;

				} else {
					// mission item before land doesn't have a position
					mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: need landing approach.");
					return false;
				}

			} else {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: starts with land waypoint.");
				return false;
			}
		}
	}

	if (land_start_req && !land_start_found) {
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: land start required.");
		return false;
	}

	if (land_start_found && (!landing_valid || (do_land_start_index > landing_approach_index))) {
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: invalid land start.");
		return false;
	}

	/* No landing waypoints or no waypoints */
	return true;
}

bool
MissionFeasibilityChecker::checkDistanceToFirstWaypoint(const mission_s &mission, float max_distance)
{
	if (max_distance <= 0.0f) {
		/* param not set, check is ok */
		return true;
	}

	/* find first waypoint (with lat/lon) item in datamanager */
	for (size_t i = 0; i < mission.count; i++) {

		struct mission_item_s mission_item {};

		if (!(dm_read((dm_item_t)mission.dataman_id, i, &mission_item, sizeof(mission_item_s)) == sizeof(mission_item_s))) {
			/* error reading, mission is invalid */
			mavlink_log_info(_navigator->get_mavlink_log_pub(), "Error reading offboard mission.");
			return false;
		}

		/* check only items with valid lat/lon */
		if (!MissionBlock::item_contains_position(mission_item)) {
			continue;
		}

		/* check distance from current position to item */
		float dist_to_1wp = get_distance_to_next_waypoint(
					    mission_item.lat, mission_item.lon,
					    _navigator->get_home_position()->lat, _navigator->get_home_position()->lon);

		if (dist_to_1wp < max_distance) {

			return true;

		} else {
			/* item is too far from home */
			mavlink_log_critical(_navigator->get_mavlink_log_pub(),
					     "First waypoint too far away: %d meters, %d max.",
					     (int)dist_to_1wp, (int)max_distance);

			_navigator->get_mission_result()->warning = true;
			return false;
		}
	}

	/* no waypoints found in mission, then we will not fly far away */
	return true;
}

bool
MissionFeasibilityChecker::checkDistancesBetweenWaypoints(const mission_s &mission, float max_distance)
{
	if (max_distance <= 0.0f) {
		/* param not set, check is ok */
		return true;
	}

	double last_lat = (double)NAN;
	double last_lon = (double)NAN;

	/* Go through all waypoints */
	for (size_t i = 0; i < mission.count; i++) {

		struct mission_item_s mission_item {};

		if (!(dm_read((dm_item_t)mission.dataman_id, i, &mission_item, sizeof(mission_item_s)) == sizeof(mission_item_s))) {
			/* error reading, mission is invalid */
			mavlink_log_info(_navigator->get_mavlink_log_pub(), "Error reading offboard mission.");
			return false;
		}

		/* check only items with valid lat/lon */
		if (!MissionBlock::item_contains_position(mission_item)) {
			continue;
		}

		/* Compare it to last waypoint if already available. */
		if (PX4_ISFINITE(last_lat) && PX4_ISFINITE(last_lon)) {

			/* check distance from current position to item */
			const float dist_between_waypoints = get_distance_to_next_waypoint(
					mission_item.lat, mission_item.lon,
					last_lat, last_lon);

			if (dist_between_waypoints > max_distance) {
				/* item is too far from home */
				mavlink_log_critical(_navigator->get_mavlink_log_pub(),
						     "Distance between waypoints too far: %d meters, %d max.",
						     (int)dist_between_waypoints, (int)max_distance);

				_navigator->get_mission_result()->warning = true;
				return false;
			}
		}

		last_lat = mission_item.lat;
		last_lon = mission_item.lon;
	}

	/* We ran through all waypoints and have not found any distances between waypoints that are too far. */
	return true;
}
