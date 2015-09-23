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

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

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
	float home_alt, bool home_valid, double curr_lat, double curr_lon, float max_waypoint_distance, bool &warning_issued)
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
	failed = failed || !checkMissionItemValidity(dm_current, nMissionItems);
	failed = failed || !checkGeofence(dm_current, nMissionItems, geofence);
	failed = failed || !checkHomePositionAltitude(dm_current, nMissionItems, home_alt, home_valid, warned);

	if (isRotarywing) {
		failed = failed || !checkMissionFeasibleRotarywing(dm_current, nMissionItems, geofence, home_alt, home_valid);
	} else {
		failed = failed || !checkMissionFeasibleFixedwing(dm_current, nMissionItems, geofence, home_alt, home_valid);
	}

	if (!failed) {
		mavlink_log_info(_mavlink_fd, "Mission checked and ready.");
	}

	return !failed;
}

bool MissionFeasibilityChecker::checkMissionFeasibleRotarywing(dm_item_t dm_current, size_t nMissionItems, Geofence &geofence, float home_alt, bool home_valid)
{
	/* no custom rotary wing checks yet */
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

bool MissionFeasibilityChecker::checkMissionItemValidity(dm_item_t dm_current, size_t nMissionItems) {
	// do not allow mission if we find unsupported item
	for (size_t i = 0; i < nMissionItems; i++) {
		struct mission_item_s missionitem;
		const ssize_t len = sizeof(struct mission_item_s);

		if (dm_read(dm_current, i, &missionitem, len) != len) {
			// not supposed to happen unless the datamanager can't access the SD card, etc.
			mavlink_log_critical(_mavlink_fd, "Rejecting Mission: Cannot access SD card");
			return false;
		}

		// check if we find unsupported item and reject mission if so
		if (missionitem.nav_cmd != NAV_CMD_IDLE &&
			missionitem.nav_cmd != NAV_CMD_WAYPOINT &&
			missionitem.nav_cmd != NAV_CMD_LOITER_UNLIMITED &&
			missionitem.nav_cmd != NAV_CMD_LOITER_TURN_COUNT &&
			missionitem.nav_cmd != NAV_CMD_LOITER_TIME_LIMIT &&
			missionitem.nav_cmd != NAV_CMD_LAND &&
			missionitem.nav_cmd != NAV_CMD_TAKEOFF &&
			missionitem.nav_cmd != NAV_CMD_ROI &&
			missionitem.nav_cmd != NAV_CMD_PATHPLANNING &&
			missionitem.nav_cmd != NAV_CMD_DO_JUMP &&
			missionitem.nav_cmd != NAV_CMD_DO_SET_SERVO) {

			mavlink_log_critical(_mavlink_fd, "Rejecting mission item %i: unsupported action.", (int)(i+1));
			return false;
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
		if (dm_read(dm_current, i, &missionitem, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			return false;
		}

		if (missionitem.nav_cmd == NAV_CMD_LAND) {
			struct mission_item_s missionitem_previous;
			if (i != 0) {
				if (dm_read(dm_current, i-1, &missionitem_previous, len) != len) {
					/* not supposed to happen unless the datamanager can't access the SD card, etc. */
					return false;
				}

				float wp_distance = get_distance_to_next_waypoint(missionitem_previous.lat , missionitem_previous.lon, missionitem.lat, missionitem.lon);
				float slope_alt_req = Landingslope::getLandingSlopeAbsoluteAltitude(wp_distance, missionitem.altitude, _nav_caps.landing_horizontal_slope_displacement, _nav_caps.landing_slope_angle_rad);
				float wp_distance_req = Landingslope::getLandingSlopeWPDistance(missionitem_previous.altitude, missionitem.altitude, _nav_caps.landing_horizontal_slope_displacement, _nav_caps.landing_slope_angle_rad);
				float delta_altitude = missionitem.altitude - missionitem_previous.altitude;
//				warnx("wp_distance %.2f, delta_altitude %.2f, missionitem_previous.altitude %.2f, missionitem.altitude %.2f, slope_alt_req %.2f, wp_distance_req %.2f",
//						wp_distance, delta_altitude, missionitem_previous.altitude, missionitem.altitude, slope_alt_req, wp_distance_req);
//				warnx("_nav_caps.landing_horizontal_slope_displacement %.4f, _nav_caps.landing_slope_angle_rad %.4f, _nav_caps.landing_flare_length %.4f",
//						_nav_caps.landing_horizontal_slope_displacement, _nav_caps.landing_slope_angle_rad, _nav_caps.landing_flare_length);

				if (wp_distance > _nav_caps.landing_flare_length) {
					/* Last wp is before flare region */

					if (delta_altitude < 0) {
						if (missionitem_previous.altitude <= slope_alt_req) {
							/* Landing waypoint is at or below altitude of slope at the given waypoint distance: this is ok, aircraft will intersect the slope */
							return true;
						} else {
							/* Landing waypoint is above altitude of slope at the given waypoint distance */
							mavlink_log_critical(_mavlink_fd, "Landing: last waypoint too high/too close");
							mavlink_log_critical(_mavlink_fd, "Move down to %.1fm or move further away by %.1fm",
									(double)(slope_alt_req),
									(double)(wp_distance_req - wp_distance));
							return false;
						}
					} else {
						/* Landing waypoint is above last waypoint */
						mavlink_log_critical(_mavlink_fd, "Landing waypoint above last nav waypoint");
						return false;
					}
				} else {
					/* Last wp is in flare region */
					//xxx give recommendations
					mavlink_log_critical(_mavlink_fd, "Warning: Landing: last waypoint in flare region");
					return false;
				}
			} else {
				mavlink_log_critical(_mavlink_fd, "Warning: starting with land waypoint");
				return false;
			}
		}
	}

	/* No landing waypoints or no waypoints */
	return true;
}

bool
MissionFeasibilityChecker::check_dist_1wp(dm_item_t dm_current, size_t nMissionItems, double curr_lat, double curr_lon, float dist_first_wp, bool &warning_issued)
{
	if (_dist_1wp_ok) {
		/* always return true after at least one successful check */
		return true;
	}

	/* check if first waypoint is not too far from home */
	if (dist_first_wp > 0.0f) {
		struct mission_item_s mission_item;

		/* find first waypoint (with lat/lon) item in datamanager */
		for (unsigned i = 0; i < nMissionItems; i++) {
			if (dm_read(dm_current, i,
					&mission_item, sizeof(mission_item_s)) == sizeof(mission_item_s)) {
				/* Check non navigation item */
				if (mission_item.nav_cmd == NAV_CMD_DO_SET_SERVO){

					/* check actuator number */
					if (mission_item.actuator_num < 0 || mission_item.actuator_num > 5) {
						mavlink_log_critical(_mavlink_fd, "Actuator number %d is out of bounds 0..5", (int)mission_item.actuator_num);
						warning_issued = true;
						return false;
					}
					/* check actuator value */
					if (mission_item.actuator_value < -2000 || mission_item.actuator_value > 2000) {
						mavlink_log_critical(_mavlink_fd, "Actuator value %d is out of bounds -2000..2000", (int)mission_item.actuator_value);
						warning_issued = true;
						return false;
					}
				}
				/* check only items with valid lat/lon */
				else if ( mission_item.nav_cmd == NAV_CMD_WAYPOINT ||
						mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
						mission_item.nav_cmd == NAV_CMD_LOITER_TURN_COUNT ||
						mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED ||
						mission_item.nav_cmd == NAV_CMD_TAKEOFF ||
						mission_item.nav_cmd == NAV_CMD_PATHPLANNING) {

					/* check distance from current position to item */
					float dist_to_1wp = get_distance_to_next_waypoint(
							mission_item.lat, mission_item.lon, curr_lat, curr_lon);

					if (dist_to_1wp < dist_first_wp) {
						_dist_1wp_ok = true;
						if (dist_to_1wp > ((dist_first_wp * 3) / 2)) {
							/* allow at 2/3 distance, but warn */
							mavlink_log_critical(_mavlink_fd, "Warning: First waypoint very far: %d m", (int)dist_to_1wp);
							warning_issued = true;
						}
						return true;

					} else {
						/* item is too far from home */
						mavlink_log_critical(_mavlink_fd, "Waypoint too far: %d m,[MIS_DIST_1WP=%d]", (int)dist_to_1wp, (int)dist_first_wp);
						warning_issued = true;
						return false;
					}
				}

			} else {
				/* error reading, mission is invalid */
				mavlink_log_info(_mavlink_fd, "error reading offboard mission");
				return false;
			}
		}

		/* no waypoints found in mission, then we will not fly far away */
		_dist_1wp_ok = true;
		return true;

	} else {
		return true;
	}
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
