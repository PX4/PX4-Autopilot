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

#include "mission_block.h"
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

	if (!geofence.valid()) {
		return true;
	}

	enum class Failure { None, DatamanRead, NoHome, Waypoint, Loiter } failure = Failure::None;
	size_t failed_item = 0;
#if defined(CONFIG_NAVIGATOR_GEOFENCE_PATH_CHECKS)
	const vehicle_status_s &status = *_navigator->get_vstatus();
	// A VTOL may transition to fixed-wing after upload.
	const bool circling_vehicle = status.is_vtol || status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	/* Check mission positions and the paths between them. */
	static GeofencePathBatch batch{}; // keep the shared batch off the stack.
	batch.count = 0;
	matrix::Vector2d previous_position{};
	bool have_previous_position = false;

	// A loiter the fixed-wing leaves towards the next position, and the loiter after the last one.
	struct {
		bool active{false};
		bool counter_clockwise{false};
		bool force_heading{false};
		bool exit_xtrack{false};
		float radius{0.f};
	} exit_loiter;

	struct {
		bool pending{false};
		uint16_t index{0};
		float radius{0.f};
		float altitude{0.f};
	} end_loiter;

#endif // CONFIG_NAVIGATOR_GEOFENCE_PATH_CHECKS

	for (size_t i = 0; i < mission.count; i++) {
		struct mission_item_s missionitem = {};

		bool success = _dataman_client.readSync((dm_item_t)mission.mission_dataman_id, i,
							reinterpret_cast<uint8_t *>(&missionitem),
							sizeof(mission_item_s));

		if (!success) {
			failure = Failure::DatamanRead;
			failed_item = i;
			break;
		}

#if defined(CONFIG_NAVIGATOR_GEOFENCE_PATH_CHECKS)

		if (missionitem.nav_cmd == NAV_CMD_DO_JUMP) {
			if (!checkJumpDestinations(mission, missionitem, i, have_previous_position ? &previous_position : nullptr, batch)) {
				return false;
			}

			continue;
		}

#endif // CONFIG_NAVIGATOR_GEOFENCE_PATH_CHECKS

		if (!mission_item_contains_position(missionitem)) {
			continue;
		}

		if (missionitem.altitude_is_relative && !home_valid) {
			failure = Failure::NoHome;
			failed_item = i;
			break;
		}

		// Geofence function checks against home altitude amsl
		missionitem.altitude = missionitem.altitude_is_relative ? missionitem.altitude + home_alt : missionitem.altitude;

		bool point_valid = PX4_ISFINITE(missionitem.lat) && PX4_ISFINITE(missionitem.lon)
				   && PX4_ISFINITE(missionitem.altitude)
				   && fabs(missionitem.lat) <= 90.0 && fabs(missionitem.lon) <= 180.0;

#if defined(CONFIG_NAVIGATOR_GEOFENCE_PATH_CHECKS)

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

#else
		// Without path checks every position is checked against the whole fence.
		point_valid = point_valid && geofence.checkPointAgainstAllGeofences(missionitem.lat, missionitem.lon,
				missionitem.altitude);
#endif // CONFIG_NAVIGATOR_GEOFENCE_PATH_CHECKS

		if (!point_valid) {
			failure = Failure::Waypoint;
			failed_item = i;
			break;
		}

#if defined(CONFIG_NAVIGATOR_GEOFENCE_PATH_CHECKS)
		const matrix::Vector2d position {missionitem.lat, missionitem.lon};

		if (!have_previous_position) {
			// A zero-length path also rejects a lone waypoint on a fence boundary.
			previous_position = position;
			have_previous_position = true;
		}

		if (exit_loiter.active) {
			// With a forced heading the fixed-wing leaves on the tangent of its turn direction. Otherwise it
			// leaves wherever the loiter ends and both tangents bound the straight exits. Without exit
			// xtrack the tracked line still starts at the centre, see is_mission_item_reached_or_completed().
			const matrix::Vector2d exit_point = MissionBlock::loiterExitPoint(previous_position, position, exit_loiter.radius,
							    exit_loiter.counter_clockwise);
			const matrix::Vector2d other_point = MissionBlock::loiterExitPoint(previous_position, position, exit_loiter.radius,
							     !exit_loiter.counter_clockwise);

			if (!addGeofencePath(batch, {exit_point, position}, i)
			    || (!exit_loiter.force_heading && !addGeofencePath(batch, {other_point, position}, i))
			    || (!exit_loiter.exit_xtrack && !addGeofencePath(batch, {previous_position, position}, i))) {
				return false;
			}

		} else if (!addGeofencePath(batch, {previous_position, position}, i)) {
			return false;
		}

		previous_position = position;
		exit_loiter.active = false;
		end_loiter.pending = false;

		if (!circling_vehicle) {
			continue;
		}

		// Same radius as mission_item_to_position_setpoint().
		const float radius = fabsf(missionitem.loiter_radius) > FLT_EPSILON ? fabsf(missionitem.loiter_radius) :
				     _navigator->get_default_loiter_rad();
		const bool loiter_item = missionitem.nav_cmd == NAV_CMD_LOITER_UNLIMITED
					 || missionitem.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT
					 || missionitem.nav_cmd == NAV_CMD_LOITER_TO_ALT;
		const bool landing_item = missionitem.nav_cmd == NAV_CMD_LAND || missionitem.nav_cmd == NAV_CMD_VTOL_LAND;

		if (loiter_item) {
			if (!PX4_ISFINITE(radius) || radius <= 0.f
			    || !geofence.isCloserThanMaxDistToHome(missionitem.lat, missionitem.lon, missionitem.altitude, radius)) {
				failure = Failure::Loiter;
				failed_item = i;
				break;
			}

			// Keep the circle after its incoming leg so the first breach is reported in mission order.
			if (!addGeofencePath(batch, {position, position, radius}, i)) {
				return false;
			}

			exit_loiter.active = missionitem.nav_cmd != NAV_CMD_LOITER_UNLIMITED;
			exit_loiter.counter_clockwise = missionitem.loiter_radius < 0.f;
			exit_loiter.force_heading = missionitem.force_heading;
			exit_loiter.exit_xtrack = missionitem.loiter_exit_xtrack;
			exit_loiter.radius = radius;

		} else if (!landing_item) {
			// If the mission ends here the vehicle loiters at this position, see setEndOfMissionItems().
			end_loiter.pending = true;
			end_loiter.index = static_cast<uint16_t>(i);
			end_loiter.radius = radius;
			end_loiter.altitude = missionitem.altitude;
		}

#endif // CONFIG_NAVIGATOR_GEOFENCE_PATH_CHECKS
	}

#if defined(CONFIG_NAVIGATOR_GEOFENCE_PATH_CHECKS)

	if (failure == Failure::None && end_loiter.pending) {
		if (!PX4_ISFINITE(end_loiter.radius) || end_loiter.radius <= 0.f
		    || !geofence.isCloserThanMaxDistToHome(previous_position(0), previous_position(1), end_loiter.altitude,
				    end_loiter.radius)) {
			failure = Failure::Loiter;
			failed_item = end_loiter.index;

		} else if (!addGeofencePath(batch, {previous_position, previous_position, end_loiter.radius}, end_loiter.index)) {
			return false;
		}
	}

	// Report an earlier buffered path breach before the current item's failure.
	if (!checkGeofencePathBatch(batch)) {
		return false;
	}

#endif // CONFIG_NAVIGATOR_GEOFENCE_PATH_CHECKS

	switch (failure) {
	case Failure::None:
		return true;

	case Failure::DatamanRead:
		logDatamanReadFailure(failed_item, mission.mission_dataman_id);
		break;

	case Failure::NoHome:
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence requires valid home position\t");
		events::send(events::ID("navigator_mis_geofence_no_home2"), {events::Log::Error, events::LogInternal::Info},
			     "Geofence requires a valid home position");
		break;

	case Failure::Waypoint:
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence violation for waypoint %zu\t", failed_item + 1);
		events::send<int16_t>(events::ID("navigator_mis_geofence_violation"), {events::Log::Error, events::LogInternal::Info},
				      "Geofence violation for waypoint {1}", failed_item + 1);
		break;

	case Failure::Loiter:
		logGeofenceLoiterBreach(static_cast<uint16_t>(failed_item + 1));
		break;
	}

	return false;
}

#if defined(CONFIG_NAVIGATOR_GEOFENCE_PATH_CHECKS)
bool MissionFeasibilityChecker::checkJumpDestinations(const mission_s &mission, const mission_item_s &jump_item,
		size_t jump_index, const matrix::Vector2d *previous_position, GeofencePathBatch &batch)
{
	const auto in_range = [&mission](int32_t index) {
		return index >= 0 && index < mission.count;
	};

	// Execution checks the target even when the jump never repeats.
	if (!in_range(jump_item.do_jump_mission_index)) {
		return rejectGeofenceJump(batch, jump_index);
	}

	// do_jump_current_count is execution state that activation and index changes reset, so every
	// jump that can ever be taken is checked as if it will be.
	if (jump_item.do_jump_repeat_count == 0) {
		return true;
	}

	// Bound memory and reads so branching jump chains or loops cannot stall validation.
	static constexpr size_t kMaxJumpBranches = 16;
	struct Branch {
		int32_t index;
		uint16_t jumps_in_a_row;
	};
	Branch pending[kMaxJumpBranches]; // every slot is written before it is read
	size_t pending_count = 1;
	pending[0] = {jump_item.do_jump_mission_index, 1};
	size_t items_read = 0;
	const size_t read_limit = static_cast<size_t>(mission.count) * kMaxJumpBranches;

	while (pending_count > 0) {
		const Branch branch = pending[--pending_count];
		int32_t index = branch.index;
		uint16_t jumps_in_a_row = branch.jumps_in_a_row;

		while (in_range(index)) {
			if (items_read++ >= read_limit) {
				return rejectGeofenceJump(batch, jump_index);
			}

			mission_item_s candidate; // fully written by a successful read

			if (!_dataman_client.readSync(static_cast<dm_item_t>(mission.mission_dataman_id), index,
						      reinterpret_cast<uint8_t *>(&candidate), sizeof(candidate))) {
				if (checkGeofencePathBatch(batch)) {
					logDatamanReadFailure(index, mission.mission_dataman_id);
				}

				return false;
			}

			if (candidate.nav_cmd == NAV_CMD_DO_JUMP) {
				// getNonJumpItem() gives up after this many jump reads without a non-jump item.
				if (++jumps_in_a_row >= NAV_MAX_JUMP_ITERATION) {
					return rejectGeofenceJump(batch, jump_index);
				}

				if (!in_range(candidate.do_jump_mission_index)) {
					return rejectGeofenceJump(batch, index);
				}

				if (candidate.do_jump_repeat_count > 0) {
					// Once its repeats are spent this jump falls through. Check that leg from the same source too.
					if (candidate.do_jump_mission_index != index + 1) {
						if (pending_count >= kMaxJumpBranches) {
							return rejectGeofenceJump(batch, jump_index);
						}

						pending[pending_count++] = {index + 1, jumps_in_a_row};
					}

					index = candidate.do_jump_mission_index;

				} else {
					++index;
				}

				continue;
			}

			// A non-jump item completes one resolution; the next one starts a new count.
			jumps_in_a_row = 0;

			if (mission_item_contains_position(candidate)) {
				if (previous_position && !addGeofencePath(batch, {*previous_position, {candidate.lat, candidate.lon}}, jump_index, true)) {
					return false;
				}

				break;
			}

			++index;
		}
	}

	// Reaching the mission end leaves no further leg to check.
	return true;
}

bool MissionFeasibilityChecker::rejectGeofenceJump(GeofencePathBatch &batch, size_t jump_index)
{
	if (checkGeofencePathBatch(batch)) {
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: DO_JUMP %zu path cannot be resolved\t",
				     jump_index + 1);
		events::send<uint16_t>(events::ID("navigator_mis_do_jump_invalid"),
		{events::Log::Error, events::LogInternal::Info},
		"Mission rejected: DO_JUMP {1} path cannot be resolved", static_cast<uint16_t>(jump_index + 1));
	}

	return false;
}

bool MissionFeasibilityChecker::addGeofencePath(GeofencePathBatch &batch, const Geofence::PathCheck &path,
		size_t mission_index, bool is_jump)
{
	batch.paths[batch.count] = path;
	batch.is_jump[batch.count] = is_jump;
	batch.mission_indices[batch.count++] = static_cast<uint16_t>(mission_index);
	return batch.count < kGeofencePathBatchSize || checkGeofencePathBatch(batch);
}

bool MissionFeasibilityChecker::checkGeofencePathBatch(GeofencePathBatch &batch)
{
	if (batch.count == 0) {
		return true;
	}

	if (!_navigator->get_geofence().checkPathBatch(batch.paths, batch.count, batch.results)) {
		logGeofenceUnavailable();
		return false;
	}

	for (size_t i = 0; i < batch.count; ++i) {
		if (!batch.results[i]) {
			const uint16_t waypoint = batch.mission_indices[i] + 1;

			if (batch.paths[i].end_radius > 0.f) {
				logGeofenceLoiterBreach(waypoint);

			} else if (batch.is_jump[i]) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence breach on DO_JUMP %u path\t",
						     static_cast<unsigned>(waypoint));
				events::send<uint16_t>(events::ID("navigator_mis_geofence_jump_breach"),
				{events::Log::Error, events::LogInternal::Info},
				"Geofence breach on DO_JUMP {1} path", waypoint);

			} else {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence breach on path to waypoint %u\t",
						     static_cast<unsigned>(waypoint));
				events::send<uint16_t>(events::ID("navigator_mis_geofence_path_violation"),
				{events::Log::Error, events::LogInternal::Info},
				"Geofence breach on path to waypoint {1}", waypoint);
			}

			return false;
		}
	}

	batch.count = 0;
	return true;
}
#endif // CONFIG_NAVIGATOR_GEOFENCE_PATH_CHECKS

void MissionFeasibilityChecker::logGeofenceLoiterBreach(uint16_t waypoint)
{
	mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence breach by loiter circle of waypoint %u\t",
			     static_cast<unsigned>(waypoint));
	events::send<uint16_t>(events::ID("navigator_mis_geofence_loiter_breach"),
	{events::Log::Error, events::LogInternal::Info},
	"Geofence breach by loiter circle of waypoint {1}", waypoint);
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
