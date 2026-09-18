/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include "mission_feasibility_checker.h"

#include "mission_item_utils.h"
#include "navigator.h"

#include <drivers/drv_pwm_output.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <px4_platform_common/events.h>

bool
MissionFeasibilityChecker::checkMissionFeasible(const mission_s &mission)
{
	// Reset warning flag
	_navigator->get_mission_result()->warning = false;

	// first check if we have a valid position
	const bool home_valid = _navigator->home_global_position_valid();
	const bool home_alt_valid = _navigator->home_alt_valid();

	// An empty mission is the normal "no mission loaded" state, not a rejection: the commander
	// surfaces auto_mission_missing via arming checks when mission mode is actually requested.
	if ((int)mission.count <= 0) {
		return false;
	}

	if (!home_alt_valid) {
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "Not yet ready for mission, no position lock.\t");
		events::send(events::ID("navigator_mis_no_pos_lock"), events::Log::Info, "Not yet ready for mission, no position lock");
		return false;
	}

	bool failed = false;

	for (size_t i = 0; i < mission.count; i++) {
		struct mission_item_s missionitem = {};

		bool success = _dataman_client.readSync((dm_item_t)mission.mission_dataman_id, i,
							reinterpret_cast<uint8_t *>(&missionitem),
							sizeof(mission_item_s));

		if (!success) {
			_navigator->get_mission_result()->warning = true;
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			logDatamanReadFailure(i, mission.mission_dataman_id);
			return false;
		}

		if (!_feasibility_checker.processNextItem(missionitem, i, mission.count)) {
			failed = true;
			break;
		}

	}

	failed |= _feasibility_checker.someCheckFailed();

	failed |= !checkMissionAgainstGeofence(mission, _navigator->get_home_position()->alt, home_valid);

	_navigator->get_mission_result()->warning = failed;

	return !failed;
}

bool
MissionFeasibilityChecker::checkMissionAgainstGeofence(const mission_s &mission, float home_alt, bool home_valid)
{
	Geofence &geofence = _navigator->get_geofence();

	if (geofence.isHomeRequired() && !home_valid) {
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence requires valid home position\t");
		events::send(events::ID("navigator_mis_geofence_no_home"), {events::Log::Error, events::LogInternal::Info},
			     "Geofence requires a valid home position");
		return false;
	}

	if (!geofence.isReadyForPathChecks()) {
		logGeofenceUnavailable();
		return false;
	}

	/* Check mission positions and the paths between them. */
	if (geofence.valid()) {
		GeofencePathBatch batch{};
		matrix::Vector2<double> previous_position{};
		bool have_previous_position = false;

		for (size_t i = 0; i < mission.count; i++) {
			struct mission_item_s missionitem = {};

			bool success = _dataman_client.readSync((dm_item_t)mission.mission_dataman_id, i,
								reinterpret_cast<uint8_t *>(&missionitem),
								sizeof(mission_item_s));

			if (!success) {
				if (!checkGeofencePathBatch(batch)) {
					return false;
				}

				/* not supposed to happen unless the datamanager can't access the SD card, etc. */
				logDatamanReadFailure(i, mission.mission_dataman_id);
				return false;
			}

			if (!mission_item_contains_position(missionitem)) {
				continue;
			}

			if (missionitem.altitude_is_relative && !home_valid) {
				if (!checkGeofencePathBatch(batch)) {
					return false;
				}

				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence requires valid home position\t");
				events::send(events::ID("navigator_mis_geofence_no_home2"), {events::Log::Error, events::LogInternal::Info},
					     "Geofence requires a valid home position");
				return false;
			}

			// Geofence function checks against home altitude amsl
			missionitem.altitude = missionitem.altitude_is_relative ? missionitem.altitude + home_alt : missionitem.altitude;

			bool point_valid = PX4_ISFINITE(missionitem.lat) && PX4_ISFINITE(missionitem.lon)
					   && PX4_ISFINITE(missionitem.altitude)
					   && fabs(missionitem.lat) <= 90.0 && fabs(missionitem.lon) <= 180.0;

			if (point_valid) {
				if (!have_previous_position) {
					// Check polygon membership once; the paths check all later positions.
					point_valid = geofence.checkPointAgainstAllGeofences(missionitem.lat, missionitem.lon, missionitem.altitude);

				} else {
					// Home-distance and altitude limits are separate from the horizontal fence shapes.
					point_valid = geofence.isCloserThanMaxDistToHome(missionitem.lat, missionitem.lon, missionitem.altitude)
						      && geofence.isBelowMaxAltitude(missionitem.altitude)
						      && geofence.isWithinAltitudeBand(missionitem.altitude);
				}
			}

			if (!point_valid) {
				// Report an earlier buffered path breach before this waypoint's failure.
				if (!checkGeofencePathBatch(batch)) {
					return false;
				}

				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence violation for waypoint %zu\t", i + 1);
				events::send<int16_t>(events::ID("navigator_mis_geofence_violation"), {events::Log::Error, events::LogInternal::Info},
						      "Geofence violation for waypoint {1}",
						      i + 1);
				return false;
			}

			const matrix::Vector2<double> position{missionitem.lat, missionitem.lon};

			if (!have_previous_position) {
				// A zero-length path also rejects a lone waypoint on a fence boundary.
				previous_position = position;
				have_previous_position = true;
			}

			batch.paths[batch.count] = {previous_position, position};
			batch.mission_indices[batch.count++] = static_cast<uint16_t>(i);
			previous_position = position;

			if (batch.count == kGeofencePathBatchSize && !checkGeofencePathBatch(batch)) {
				return false;
			}
		}

		return checkGeofencePathBatch(batch);
	}

	return true;
}

bool MissionFeasibilityChecker::checkGeofencePathBatch(GeofencePathBatch &batch)
{
	if (batch.count == 0) {
		return true;
	}

	bool clear[kGeofencePathBatchSize] {};

	if (!_navigator->get_geofence().checkPathBatch(batch.paths, batch.count, clear)) {
		logGeofenceUnavailable();
		return false;
	}

	for (size_t i = 0; i < batch.count; ++i) {
		if (!clear[i]) {
			const uint16_t waypoint = batch.mission_indices[i] + 1;
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence breach on path to waypoint %u\t",
					     static_cast<unsigned>(waypoint));
			events::send<uint16_t>(events::ID("navigator_mis_geofence_path_violation"),
			{events::Log::Error, events::LogInternal::Info},
			"Geofence breach on path to waypoint {1}", waypoint);
			return false;
		}
	}

	batch.count = 0;
	return true;
}

void MissionFeasibilityChecker::logGeofenceUnavailable()
{
	mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: geofence path check unavailable\t");
	events::send(events::ID("navigator_mis_geofence_unavailable"), {events::Log::Error, events::LogInternal::Info},
		     "Mission rejected: geofence path check unavailable");
}

void MissionFeasibilityChecker::logDatamanReadFailure(const size_t mission_item, const uint8_t dataman_id)
{
	mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: dataman read failed at item %zu (dm_id=%u)\t", mission_item,
			     static_cast<unsigned>(dataman_id));
	events::send<uint16_t, uint8_t>(events::ID("navigator_mis_dm_read_fail"), {events::Log::Error, events::LogInternal::Info},
					"Mission rejected: dataman read failed at item {1} (dm_id={2})", static_cast<uint16_t>(mission_item), dataman_id);
}
