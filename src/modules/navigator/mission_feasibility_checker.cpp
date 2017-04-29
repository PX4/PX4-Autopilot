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
#include <fw_pos_control_l1/landingslope.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/fence.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/fw_pos_ctrl_status.h>
#include <uORB/topics/vehicle_status.h>

bool
MissionFeasibilityChecker::checkMissionFeasible(const mission_s &mission, float max_waypoint_distance,
		bool land_start_req)
{
	dm_item_t dm_current = DM_KEY_WAYPOINTS_OFFBOARD(mission.dataman_id);
	const size_t nMissionItems = mission.count;

	const bool isRotarywing = (_navigator->get_vstatus()->is_rotary_wing || _navigator->get_vstatus()->is_vtol);

	Geofence &geofence = _navigator->get_geofence();
	fw_pos_ctrl_status_s *fw_pos_ctrl_status = _navigator->get_fw_pos_ctrl_status();
	const float home_alt = _navigator->get_home_position()->alt;
	const bool home_valid = _navigator->home_position_valid();



	bool &warning_issued = _navigator->get_mission_result()->warning;
	const float default_acceptance_rad  = _navigator->get_default_acceptance_radius();
	const bool landed = _navigator->get_land_detected()->landed;

	bool failed = false;
	bool warned = false;

	// first check if we have a valid position
	if (!home_valid) {
		failed = true;
		warned = true;
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "Not yet ready for mission, no position lock.");

	} else {
		const double lat = _navigator->get_home_position()->lat;
		const double lon = _navigator->get_home_position()->lon;

		failed = failed
			 || !check_dist_1wp(dm_current, nMissionItems, lat, lon, max_waypoint_distance, warning_issued);
	}

	// check if all mission item commands are supported
	failed = failed || !checkMissionItemValidity(dm_current, nMissionItems, landed);
	failed = failed || !checkGeofence(dm_current, nMissionItems, geofence, home_alt, home_valid);
	failed = failed || !checkHomePositionAltitude(dm_current, nMissionItems, home_alt, home_valid, warned);

	if (isRotarywing) {
		failed = failed
			 || !checkRotarywing(dm_current, nMissionItems, home_alt, home_valid, default_acceptance_rad);

	} else {
		failed = failed
			 || !checkFixedwing(dm_current, nMissionItems, fw_pos_ctrl_status, home_alt, home_valid,
					    default_acceptance_rad, land_start_req);
	}

	return !failed;
}

bool
MissionFeasibilityChecker::checkRotarywing(dm_item_t dm_current, size_t nMissionItems,
		float home_alt, bool home_valid, float default_acceptance_rad)
{
	for (size_t i = 0; i < nMissionItems; i++) {
		struct mission_item_s missionitem = {};
		const ssize_t len = sizeof(struct mission_item_s);

		if (dm_read(dm_current, i, &missionitem, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			return false;
		}

		// look for a takeoff waypoint
		if (missionitem.nav_cmd == NAV_CMD_TAKEOFF) {
			// make sure that the altitude of the waypoint is at least one meter larger than the acceptance radius
			// this makes sure that the takeoff waypoint is not reached before we are at least one meter in the air
			float takeoff_alt = missionitem.altitude_is_relative ? missionitem.altitude : missionitem.altitude - home_alt;

			// check if we should use default acceptance radius
			float acceptance_radius = default_acceptance_rad;

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
MissionFeasibilityChecker::checkFixedwing(dm_item_t dm_current, size_t nMissionItems,
		fw_pos_ctrl_status_s *fw_pos_ctrl_status, float home_alt, bool home_valid,
		float default_acceptance_rad, bool land_start_req)
{
	/* Perform checks and issue feedback to the user for all checks */
	bool resTakeoff = checkFixedWingTakeoff(dm_current, nMissionItems, home_alt, home_valid, land_start_req);
	bool resLanding = checkFixedWingLanding(dm_current, nMissionItems, fw_pos_ctrl_status, land_start_req);

	/* Mission is only marked as feasible if all checks return true */
	return (resTakeoff && resLanding);
}

bool
MissionFeasibilityChecker::checkGeofence(dm_item_t dm_current, size_t nMissionItems, Geofence &geofence, float home_alt,
		bool home_valid)
{

	if (geofence.isHomeRequired() && !home_valid) {
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence requires valid home position");
		return false;
	}

	/* Check if all mission items are inside the geofence (if we have a valid geofence) */
	if (geofence.valid()) {
		for (size_t i = 0; i < nMissionItems; i++) {
			struct mission_item_s missionitem = {};
			const ssize_t len = sizeof(missionitem);

			if (dm_read(dm_current, i, &missionitem, len) != len) {
				/* not supposed to happen unless the datamanager can't access the SD card, etc. */
				return false;
			}

			if (missionitem.altitude_is_relative && !home_valid) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence requires valid home position");
				return false;
			}

			// Geofence function checks against home altitude amsl
			missionitem.altitude = missionitem.altitude_is_relative ? missionitem.altitude + home_alt : missionitem.altitude;

			if (MissionBlock::item_contains_position(&missionitem) && !geofence.inside(missionitem)) {

				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence violation for waypoint %d", i + 1);
				return false;
			}
		}
	}

	return true;
}

bool
MissionFeasibilityChecker::checkHomePositionAltitude(dm_item_t dm_current, size_t nMissionItems,
		float home_alt, bool home_valid, bool &warning_issued, bool throw_error)
{
	/* Check if all waypoints are above the home altitude */
	for (size_t i = 0; i < nMissionItems; i++) {
		struct mission_item_s missionitem = {};
		const ssize_t len = sizeof(struct mission_item_s);

		if (dm_read(dm_current, i, &missionitem, len) != len) {
			warning_issued = true;
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			return false;
		}

		/* reject relative alt without home set */
		if (missionitem.altitude_is_relative && !home_valid && MissionBlock::item_contains_position(&missionitem)) {

			warning_issued = true;

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

		if ((home_alt > wp_alt) && MissionBlock::item_contains_position(&missionitem)) {

			warning_issued = true;

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
MissionFeasibilityChecker::checkMissionItemValidity(dm_item_t dm_current, size_t nMissionItems, bool landed)
{
	// do not allow mission if we find unsupported item
	for (size_t i = 0; i < nMissionItems; i++) {
		struct mission_item_s missionitem;
		const ssize_t len = sizeof(struct mission_item_s);

		if (dm_read(dm_current, i, &missionitem, len) != len) {
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
		    missionitem.nav_cmd != NAV_CMD_DO_SET_SERVO &&
		    missionitem.nav_cmd != NAV_CMD_DO_LAND_START &&
		    missionitem.nav_cmd != NAV_CMD_DO_DIGICAM_CONTROL &&
		    missionitem.nav_cmd != NAV_CMD_IMAGE_START_CAPTURE &&
		    missionitem.nav_cmd != NAV_CMD_IMAGE_STOP_CAPTURE &&
		    missionitem.nav_cmd != NAV_CMD_VIDEO_START_CAPTURE &&
		    missionitem.nav_cmd != NAV_CMD_VIDEO_STOP_CAPTURE &&
		    missionitem.nav_cmd != NAV_CMD_DO_MOUNT_CONFIGURE &&
		    missionitem.nav_cmd != NAV_CMD_DO_MOUNT_CONTROL &&
		    missionitem.nav_cmd != NAV_CMD_DO_SET_ROI &&
		    missionitem.nav_cmd != NAV_CMD_ROI &&
		    missionitem.nav_cmd != NAV_CMD_DO_SET_CAM_TRIGG_DIST &&
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
		if (missionitem.nav_cmd == NAV_CMD_LAND && i == 0 && landed) {

			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: starts with landing");
			return false;
		}
	}

	return true;
}

bool
MissionFeasibilityChecker::checkFixedWingTakeoff(dm_item_t dm_current, size_t nMissionItems,
		float home_alt, bool home_valid, float default_acceptance_rad)
{
	for (size_t i = 0; i < nMissionItems; i++) {
		struct mission_item_s missionitem = {};
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
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: Takeoff altitude too low!");
				return false;
			}
		}
	}

	// all checks have passed
	return true;
}

bool
MissionFeasibilityChecker::checkFixedWingLanding(dm_item_t dm_current, size_t nMissionItems,
		fw_pos_ctrl_status_s *fw_pos_ctrl_status, bool land_start_req)
{
	/* Go through all mission items and search for a landing waypoint
	 * if landing waypoint is found: the previous waypoint is checked to be at a feasible distance and altitude given the landing slope */

	bool landing_valid = false;

	bool land_start_found = false;
	size_t do_land_start_index = 0;
	size_t landing_approach_index = 0;

	for (size_t i = 0; i < nMissionItems; i++) {
		struct mission_item_s missionitem;
		const ssize_t len = sizeof(missionitem);

		if (dm_read(dm_current, i, &missionitem, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			return false;
		}

		// if DO_LAND_START found then require valid landing AFTER
		if (missionitem.nav_cmd == NAV_CMD_DO_LAND_START) {
			if (land_start_found) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: more than one land start");
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

				if (dm_read(dm_current, landing_approach_index, &missionitem_previous, len) != len) {
					/* not supposed to happen unless the datamanager can't access the SD card, etc. */
					return false;
				}

				if (MissionBlock::item_contains_position(&missionitem_previous)) {
					float wp_distance = get_distance_to_next_waypoint(missionitem_previous.lat, missionitem_previous.lon, missionitem.lat,
							    missionitem.lon);

					float slope_alt_req = Landingslope::getLandingSlopeAbsoluteAltitude(wp_distance, missionitem.altitude,
							      fw_pos_ctrl_status->landing_horizontal_slope_displacement, fw_pos_ctrl_status->landing_slope_angle_rad);

					float wp_distance_req = Landingslope::getLandingSlopeWPDistance(missionitem_previous.altitude, missionitem.altitude,
								fw_pos_ctrl_status->landing_horizontal_slope_displacement, fw_pos_ctrl_status->landing_slope_angle_rad);

					if (wp_distance > fw_pos_ctrl_status->landing_flare_length) {
						/* Last wp is before flare region */

						const float delta_altitude = missionitem.altitude - missionitem_previous.altitude;

						if (delta_altitude < 0) {
							if (missionitem_previous.altitude > slope_alt_req) {
								/* Landing waypoint is above altitude of slope at the given waypoint distance */
								mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: adjust landing approach");
								mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Move down %.1fm or move further away by %.1fm",
										     (double)(slope_alt_req - missionitem_previous.altitude),
										     (double)(wp_distance_req - wp_distance));

								return false;
							}

						} else {
							/* Landing waypoint is above last waypoint */
							mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: landing above last waypoint");
							return false;
						}

					} else {
						/* Last wp is in flare region */
						mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: waypoint within landing flare");
						return false;
					}

					landing_valid = true;

				} else {
					// mission item before land doesn't have a position
					mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: need landing approach");
					return false;
				}

			} else {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: starts with land waypoint");
				return false;
			}
		}
	}

	if (land_start_req && !land_start_found) {
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: land start required");
		return false;
	}

	if (land_start_found && (!landing_valid || (do_land_start_index > landing_approach_index))) {
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: invalid land start");
		return false;
	}

	/* No landing waypoints or no waypoints */
	return true;
}

bool
MissionFeasibilityChecker::check_dist_1wp(dm_item_t dm_current, size_t nMissionItems, double curr_lat, double curr_lon,
		float dist_first_wp, bool &warning_issued)
{
	/* check if first waypoint is not too far from home */
	if (dist_first_wp > 0.0f) {
		struct mission_item_s mission_item = {};

		/* find first waypoint (with lat/lon) item in datamanager */
		for (size_t i = 0; i < nMissionItems; i++) {
			if (dm_read(dm_current, i, &mission_item, sizeof(mission_item_s)) == sizeof(mission_item_s)) {
				if (MissionBlock::item_contains_position(&mission_item)) {
					/* check only items with valid lat/lon */

					/* check distance from current position to item */
					float dist_to_1wp = get_distance_to_next_waypoint(mission_item.lat, mission_item.lon, curr_lat, curr_lon);

					if (dist_to_1wp < dist_first_wp) {
						if (dist_to_1wp > ((dist_first_wp * 3) / 2)) {
							/* allow at 2/3 distance, but warn */
							mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Warning: First waypoint very far: %d m", (int)dist_to_1wp);
							warning_issued = true;
						}

						return true;

					} else {
						/* item is too far from home */
						mavlink_log_critical(_navigator->get_mavlink_log_pub(), "First waypoint too far: %d m > %d, refusing mission",
								     (int)dist_to_1wp, (int)dist_first_wp);
						warning_issued = true;
						return false;
					}
				}

			} else {
				/* error reading, mission is invalid */
				mavlink_log_info(_navigator->get_mavlink_log_pub(), "error reading offboard mission");
				return false;
			}
		}

		/* no waypoints found in mission, then we will not fly far away */
		return true;

	} else {
		return true;
	}
}
