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
 * @file detect_and_avoid.cpp
 *
 * Helper class to do detect and avoid traffic
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "../navigator.h"

#include <cinttypes>
#include <lib/mathlib/mathlib.h>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>

#include <parameters/param.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>

using namespace time_literals;

// Buffer containing the tracker changes collected over one cycle.
// Defined as an uninitialized static so the storage lives in .bss (AXI_SRAM on FMU targets).
// With 37 change slots, this is roughly 1.8 KB on current targets.
static conflict_cycle_changes_s _cycle_changes;

DetectAndAvoid::DetectAndAvoid(Navigator *navigator) :
	MissionBlock(navigator, 0),
	ModuleParams(navigator)
{
	_detect_and_avoid_pub.advertise();
	_detect_and_avoid_most_urgent_pub.advertise();
}

void DetectAndAvoid::reset()
{
	_is_activated = false;
#if defined(CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC)
	stop_fake_traffic();
#endif // CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC
	clear_conflicts();
}

void DetectAndAvoid::clear_conflicts()
{
	_conflict_tracker.clear();
	_conflict_notifier.reset();
	_most_urgent_conflict_changed = true;
	_previous_action = DaaAction::kDisabled;
	_prev_most_urgent_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_NONE;
	_time_last_buffer_clean = 0;
}

void DetectAndAvoid::on_inactivation()
{
	PX4_DEBUG("DAA inactivation");
	reset();
	publish_most_urgent_conflict_if_changed();
}

void DetectAndAvoid::on_activation()
{
	reset();
	update_activation_status();
	publish_most_urgent_conflict_if_changed();
}

void DetectAndAvoid::update_activation_status()
{
	ModuleParams::updateParams();

	if (!_param_daa_en.get()) {
		if (_is_activated) {
			PX4_DEBUG("DAA: module disabled");
			reset();
		}

		return;
	}

	if (!_adsb_conflict_detector.try_updating_params()) {
		if (_is_activated) {
			reset();
		}

		PX4_ERR("DAA invalid params. Traffic warnings and actions disabled");
		return;
	}

	if (!_is_activated) {
		PX4_DEBUG("DAA: init ok");
		_is_activated = true;
	}
}

bool DetectAndAvoid::has_elapsed(hrt_abstime &last_time, const hrt_abstime interval)
{
	if ((last_time == 0) || (hrt_elapsed_time(&last_time) > interval)) {
		last_time = hrt_absolute_time();
		return true;
	}

	return false;
}

void DetectAndAvoid::on_active()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		update_activation_status();
	}

	if (_is_activated) {
		process_traffic();
	}

	publish_most_urgent_conflict_if_changed();
}

void DetectAndAvoid::process_traffic()
{
	if (!uav_pose_valid_and_updated()) {
		if (!_conflict_tracker.empty()
		    || _conflict_tracker.most_urgent().conflict_level != detect_and_avoid_s::DAA_CONFLICT_LVL_NONE) {
			PX4_DEBUG("DAA: uav pose stale, clearing conflicts");
			clear_conflicts(); // do not call reset() to stay activated once the pose recovers
		}

		return;
	}

#if defined(CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC)
	process_fake_traffic();
#endif // CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC

	_cycle_changes.clear();

	if (has_elapsed(_time_last_buffer_clean, kRemoveStaleConflictsTime)) {
		conflict_tracker_changes_s stale_changes{};

		if (_conflict_tracker.remove_stale_conflicts(hrt_absolute_time(),
				static_cast<hrt_abstime>(_param_daa_traff_tout.get()) * 1_s, stale_changes)) {
			update_most_urgent_conflict();
		}

		collect_tracker_changes(stale_changes);
	}

	if (process_transponder_queue()) {
		update_most_urgent_conflict();
	}

	_conflict_notifier.report_cycle(_cycle_changes, _conflict_tracker, notifier_cycle_context());

	if (_most_urgent_conflict_changed) {
		evaluate_and_publish_action();

		_prev_most_urgent_conflict_level = _conflict_tracker.most_urgent().conflict_level;
	}
}

void DetectAndAvoid::collect_tracker_changes(const conflict_tracker_changes_s &changes)
{
	for (size_t i = 0; i < changes.size(); ++i) {
		if (!_cycle_changes.push_back(changes[i])) {
			// Sized for the worst case; push_back can only fail if that bound is wrong.
			PX4_ERR("DAA: cycle changes overflow");
			break;
		}
	}
}

uint8_t DetectAndAvoid::warning_levels_mask() const
{
	uint8_t mask = 0;

	for (uint8_t level = detect_and_avoid_s::DAA_CONFLICT_LVL_NONE;
	     level <= detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL; ++level) {
		if (get_action_from_conflict_level(level) >= DaaAction::kWarnOnly) {
			mask |= static_cast<uint8_t>(1u << level);
		}
	}

	return mask;
}

ConflictNotifier::cycle_context_s DetectAndAvoid::notifier_cycle_context() const
{
	ConflictNotifier::cycle_context_s context{};
	context.prev_most_urgent_level = _prev_most_urgent_conflict_level;
	context.warning_levels_mask = warning_levels_mask();
	context.status_notif_interval = static_cast<hrt_abstime>(math::max<int32_t>(0, _param_daa_notif_state.get())) * 1_s;
	return context;
}

bool DetectAndAvoid::process_transponder_queue()
{
	bool buffer_updated = false;
	transponder_report_s transponder_report{};

	for (uint8_t processed_reports = 0; processed_reports < transponder_report_s::ORB_QUEUE_LENGTH; ++processed_reports) {

		if (!_traffic_sub.update(&transponder_report)) {
			break;
		}

#if defined(DEBUG_BUILD)
		debug_print_transponder_report(transponder_report);
#endif

		if (!transponder_data_valid(transponder_report)) {
			PX4_DEBUG("DAA: transponder data not valid.");
			continue;
		}

		const bool report_processed = process_transponder_report(transponder_report);
		buffer_updated = buffer_updated || report_processed;
	}

	return buffer_updated;
}

bool DetectAndAvoid::process_transponder_report(const transponder_report_s &transponder_report)
{
	DaaEncodedId encoded_id{};

	if (!identify_traffic_report(transponder_report, encoded_id)) {
		return false;
	}

	const daa_input_s daa_input = prepare_daa_input(transponder_report);

	detect_and_avoid_s daa_output{};

	if (!_adsb_conflict_detector.calculate_daa_output(daa_input, daa_output)) {
		PX4_DEBUG("DAA: Failed to calculate DAA output, skipping report.");
		return false;
	}

	if (daa_output.conflict_level == detect_and_avoid_s::DAA_CONFLICT_LVL_NONE
	    && !_conflict_tracker.contains(encoded_id)) {
		return false;
	}

	conflict_info_s current_conflict{};
	current_conflict.encoded_id = encoded_id;
	current_conflict.latest_update_timestamp = transponder_report.timestamp;
	current_conflict.conflict_level = daa_output.conflict_level;
	current_conflict.aircraft_dist = hypotf(daa_output.aircraft_dist_hor, daa_output.aircraft_dist_vert);

	daa_output.unique_id = encoded_id.id;
	daa_output.unique_id_encoding = encoded_id.encoding;
	daa_output.timestamp = transponder_report.timestamp;
	_detect_and_avoid_pub.publish(daa_output);

#if defined(DEBUG_BUILD)
	PX4_DEBUG("DAA: transponder data processed, conflict detected.");
	debug_print_conflict_info(current_conflict);
#endif

	conflict_tracker_changes_s tracker_changes{};
	const bool buffer_updated = _conflict_tracker.apply_conflict(current_conflict, tracker_changes);

	collect_tracker_changes(tracker_changes);

	return buffer_updated;
}

bool DetectAndAvoid::identify_traffic_report(const transponder_report_s &transponder_report,
		DaaEncodedId &encoded_id) const
{
	encoded_id = DaaEncodedId::from_report(transponder_report);

	if (encoded_id.id == 0) {
		PX4_DEBUG("DAA: No valid unique ID, skipping report");
		return false;
	}

	if (is_self_detection(encoded_id)) {
		PX4_DEBUG("DAA: Self detection, skipping report.");
		return false;
	}

#if defined(DEBUG_BUILD)
	char encoded_id_str[kUtmGuidMsgLength];
	encoded_id.to_string(encoded_id_str, sizeof(encoded_id_str));
	PX4_DEBUG("DAA: unique ID: %s (int:%lu)", encoded_id_str, encoded_id.id);
#endif

	return true;
}

#if !defined(CONSTRAINED_FLASH) && !defined(__PX4_NUTTX)
void DetectAndAvoid::print_status() const
{
	PX4_INFO("DAA: buf %u/%u, lvl %u, dist %.1fm",
		 static_cast<unsigned>(_conflict_tracker.size()),
		 static_cast<unsigned>(kDaaMaxTraffic),
		 static_cast<unsigned>(_conflict_tracker.most_urgent().conflict_level),
		 static_cast<double>(_conflict_tracker.most_urgent().aircraft_dist));
}
#endif // !CONSTRAINED_FLASH && !__PX4_NUTTX

void DetectAndAvoid::publish_most_urgent_conflict_if_changed()
{
	if (!_most_urgent_conflict_changed) {
		return;
	}

	_most_urgent_conflict_changed = false;

	const conflict_info_s &most_urgent_conflict = _conflict_tracker.most_urgent();

	detect_and_avoid_most_urgent_s daa_status{};

	daa_status.timestamp = hrt_absolute_time();
	daa_status.unique_id = most_urgent_conflict.encoded_id.id;
	daa_status.unique_id_encoding = most_urgent_conflict.encoded_id.encoding;
	daa_status.has_action = get_action_from_conflict_level(most_urgent_conflict.conflict_level) > DaaAction::kWarnOnly;
	daa_status.conflict_level = most_urgent_conflict.conflict_level;
	daa_status.aircraft_dist = most_urgent_conflict.aircraft_dist;

	_detect_and_avoid_most_urgent_pub.publish(daa_status);
}

void DetectAndAvoid::update_most_urgent_conflict()
{
	// Every call follows a buffer mutation, so the published status must be
	// refreshed even if the most urgent conflict itself is unchanged.
	_most_urgent_conflict_changed = true;

#if defined(DEBUG_BUILD)
	debug_print_buffer_status();
#endif

	_conflict_tracker.refresh_most_urgent();
}

void DetectAndAvoid::evaluate_and_publish_action()
{
	const conflict_info_s &most_urgent_conflict = _conflict_tracker.most_urgent();

	const daa_action_decision_s decision = DaaActionPolicy::decide(
			most_urgent_conflict.conflict_level, _prev_most_urgent_conflict_level,
			_navigator->get_vstatus()->nav_state,
			_navigator->get_land_detected()->landed,
			_navigator->get_vstatus()->arming_state == vehicle_status_s::ARMING_STATE_ARMED,
			_previous_action, action_params());

	if (decision.warn_on_ground) {
		_conflict_notifier.maybe_notify_action_on_ground(decision.ground_warning_cause,
				most_urgent_conflict.conflict_level, notifier_cycle_context());
	}

	if (decision.action_command != DaaAction::kDisabled) {
		PX4_DEBUG("DAA: publish action %d", (int)decision.action_command);
		publish_action_command(decision.action_command);

		if (decision.announce_action) {
			_conflict_notifier.notify_new_action(most_urgent_conflict, decision.action_command);
		}

		_previous_action = decision.action_command;
	}
}

bool DetectAndAvoid::is_self_detection(const DaaEncodedId &encoded_id) const
{
	switch (encoded_id.encoding) {
	case detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO: {
			const int32_t own_icao = _vehicle_adsb_icao.get();

			if (own_icao >= 0 && static_cast<uint32_t>(encoded_id.id) == static_cast<uint32_t>(own_icao)) {
				PX4_DEBUG("DAA: Received own main ICAO.");
				return true;
			}

			const int32_t own_icao_2 = _vehicle_adsb_icao_2.get();

			if (own_icao_2 >= 0 && static_cast<uint32_t>(encoded_id.id) == static_cast<uint32_t>(own_icao_2)) {
				PX4_DEBUG("DAA: Received own secondary ICAO.");
				return true;
			}

			break;
		}

	case detect_and_avoid_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN: {

			// Extract the lower and upper 32 bits (first and last 4 characters respectively)
			const uint32_t lower = static_cast<uint32_t>(encoded_id.id & 0xFFFFFFFF);
			const uint32_t upper = static_cast<uint32_t>((encoded_id.id >> 32) & 0xFFFFFFFF);

			if (lower == static_cast<uint32_t>(_vehicle_adsb_callsign_1.get()) &&
			    upper == static_cast<uint32_t>(_vehicle_adsb_callsign_2.get())) {
				PX4_DEBUG("DAA: Received own Callsign.");
				return true;
			}

			break;
		}

	case detect_and_avoid_s::UNIQUE_ID_ENCODING_UAS_ID: {
#ifndef BOARD_HAS_NO_UUID
			px4_guid_t px4_guid {};

			if (board_get_px4_guid(px4_guid) != PX4_GUID_BYTE_LENGTH) {
				PX4_DEBUG("DAA: Failed to get own UAS ID.");
				return false;
			}

			if (encoded_id.id == DaaEncodedId::last_uas_id_bytes_to_uint64(px4_guid)) {
				PX4_DEBUG("DAA: Received own UAS ID.");
				return true;
			}

#endif

			break;
		}

	default:
		break;
	}

	return false;
}

daa_input_s DetectAndAvoid::prepare_daa_input(const transponder_report_s &transponder_report)
{
	daa_input_s daa_input{};

	// Process uav pose
	const vehicle_global_position_s &global_position = *_navigator->get_global_position();
	daa_input.uav_lat_lon = matrix::Vector2d(global_position.lat, global_position.lon);
	daa_input.uav_alt = global_position.alt;
	const vehicle_local_position_s &local_position = *_navigator->get_local_position();
	daa_input.uav_vel_ned = matrix::Vector3f(local_position.vx, local_position.vy, local_position.vz);

	// Set infinite velocities to zero to at least detect the fixed-size boundaries in the F3442
	for (int i = 0; i < 3; ++i) {
		if (!PX4_ISFINITE(daa_input.uav_vel_ned(i))) {
			daa_input.uav_vel_ned(i) = 0.f;
		}
	}

	static constexpr float kMinGroundSpeedForCourseHeading{0.5f};
	const float uav_horizontal_speed = daa_input.uav_vel_ned.xy().norm();
	daa_input.uav_heading = (uav_horizontal_speed < kMinGroundSpeedForCourseHeading) ?
				local_position.heading : atan2f(daa_input.uav_vel_ned(1), daa_input.uav_vel_ned(0));

	daa_input.transponder_report = transponder_report;

	// Default to zero velocity if vel not valid PX4_ADSB_FLAGS_VALID_VELOCITY
	if (!(daa_input.transponder_report.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY)) {
		daa_input.transponder_report.ver_velocity = 0.f;
		daa_input.transponder_report.hor_velocity = 0.f;
	}

	// Over-write transponder vertical velocity if requested
#if defined(CONFIG_NAVIGATOR_ADSB_F3442) && CONFIG_NAVIGATOR_ADSB_F3442

	if (_param_daa_en_dflt_vel.get()) {
		// If velocity is zero, assume positive velocity i.e. aircraft going up
		const float velocity_sign = (daa_input.transponder_report.ver_velocity >= 0) ? 1.f : -1.f;
		daa_input.transponder_report.ver_velocity = velocity_sign * _param_daa_dflt_vel.get();
	}

#endif // CONFIG_NAVIGATOR_ADSB_F3442

	return daa_input;
}

bool DetectAndAvoid::uav_pose_valid_and_updated() const
{
	const vehicle_global_position_s &global_position = *_navigator->get_global_position();

	if (!PX4_ISFINITE(global_position.lat) || !PX4_ISFINITE(global_position.lon) || !PX4_ISFINITE(global_position.alt)) {
		PX4_DEBUG("DAA: invalid global pose");
		return false;
	}

	if (global_position.timestamp == 0 || hrt_elapsed_time(&global_position.timestamp) > kRemoveStaleConflictsTime) {
		PX4_DEBUG("DAA: stale global pose");
		return false;
	}

	const hrt_abstime local_position_timestamp = _navigator->get_local_position()->timestamp;

	if (local_position_timestamp == 0 || hrt_elapsed_time(&local_position_timestamp) > kRemoveStaleConflictsTime) {
		PX4_DEBUG("DAA: stale local position");
		return false;
	}

	return true;
}

bool DetectAndAvoid::transponder_data_valid(const transponder_report_s &transponder_report) const
{
	if (!PX4_ISFINITE(transponder_report.lat) || !PX4_ISFINITE(transponder_report.lon)) {
		PX4_DEBUG("DAA: transponder data rejected, invalid lat/lon.");
		return false;
	}

	if (!PX4_ISFINITE(transponder_report.altitude)) {
		PX4_DEBUG("DAA: transponder data rejected, invalid altitude.");
		return false;
	}

	uint16_t required_flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
				  transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;

#if !defined(CONFIG_NAVIGATOR_ADSB_F3442) || !CONFIG_NAVIGATOR_ADSB_F3442
	required_flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
			  transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;
#endif // !CONFIG_NAVIGATOR_ADSB_F3442

	if ((transponder_report.flags & required_flags) != required_flags) {
		PX4_DEBUG("DAA: transponder data rejected, missing flags.");
		return false;
	}

#if !defined(CONFIG_NAVIGATOR_ADSB_F3442) || !CONFIG_NAVIGATOR_ADSB_F3442

	if (!PX4_ISFINITE(transponder_report.heading)) {
		PX4_DEBUG("DAA: transponder data rejected, invalid heading.");
		return false;
	}

#endif // !CONFIG_NAVIGATOR_ADSB_F3442

	const hrt_abstime timeout_us = static_cast<hrt_abstime>(_param_daa_traff_tout.get()) * 1_s;

	if (transponder_report.timestamp == 0 || hrt_elapsed_time(&transponder_report.timestamp) > timeout_us) {
		PX4_DEBUG("DAA: transponder data rejected, too old.");
		return false;
	}

	return true;
}

DaaAction DetectAndAvoid::get_action_from_conflict_level(const uint8_t conflict_level) const
{
	return DaaActionPolicy::action_from_conflict_level(conflict_level, action_params());
}

daa_action_params_s DetectAndAvoid::action_params() const
{
	daa_action_params_s params{};

#if defined(CONFIG_NAVIGATOR_ADSB_F3442) && CONFIG_NAVIGATOR_ADSB_F3442
	params.lvl_low_act = _param_daa_lvl_low_act.get();
	params.lvl_med_act = _param_daa_lvl_med_act.get();
	params.lvl_high_act = _param_daa_lvl_high_act.get();
	params.lvl_crit_act = _param_daa_lvl_crit_act.get();
#else
	params.traff_avoid = _param_nav_traff_avoid.get();
#endif // CONFIG_NAVIGATOR_ADSB_F3442

	return params;
}

void DetectAndAvoid::publish_action_command(const DaaAction requested_action)
{
	vehicle_command_s vcmd = {};

	switch (requested_action) {
	case DaaAction::kDisabled:
	case DaaAction::kWarnOnly: {
			// No action early return
			return;
		}

	case DaaAction::kPositionHoldMode: {
			vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
			vcmd.param1 = 1;
			vcmd.param2 = PX4_CUSTOM_MAIN_MODE_AUTO;
			vcmd.param3 = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
			break;
		}

	case DaaAction::kReturnMode: {
			_navigator->get_rtl()->set_return_alt_min(true);
			vcmd.command = vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;
			break;
		}

	case DaaAction::kLandMode: {
			vcmd.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
			break;
		}

	case DaaAction::kTerminate: {
			vcmd.param1 = 1;
			vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION;
			break;
		}

	default:
		break;
	}

	_navigator->publish_vehicle_command(vcmd);
}


#if defined(DEBUG_BUILD)
void DetectAndAvoid::debug_print_buffer_status()
{
	const conflict_info_s &most_urgent_conflict = _conflict_tracker.most_urgent();

	char encoded_id_str[kUtmGuidMsgLength];
	most_urgent_conflict.encoded_id.to_string(encoded_id_str, sizeof(encoded_id_str));

	const int time_since_last_comm = static_cast<int>((hrt_absolute_time() -
					 most_urgent_conflict.latest_update_timestamp) / 1_s);
	const uint16_t aircraft_dist = static_cast<uint16_t>(fabsf(most_urgent_conflict.aircraft_dist));

	const int buff_size = static_cast<int>(_conflict_tracker.size());

	PX4_DEBUG("Buffer status: %d conflict(s), max lvl %d, prev max lvl %d,",
		  buff_size,
		  most_urgent_conflict.conflict_level,
		  _prev_most_urgent_conflict_level);

	PX4_DEBUG("Max conflict: Unique ID %lu, ID str %s, lvl %d, distance %d, last comm %d sec \n",
		  most_urgent_conflict.encoded_id.id,
		  encoded_id_str,
		  most_urgent_conflict.conflict_level,
		  aircraft_dist,
		  time_since_last_comm);
}

void DetectAndAvoid::debug_print_conflict_info(const conflict_info_s &conflict)
{
	char encoded_id_str[kUtmGuidMsgLength];
	conflict.encoded_id.to_string(encoded_id_str, sizeof(encoded_id_str));

	const int time_since_last_comm = static_cast<int>((hrt_absolute_time() - conflict.latest_update_timestamp) / 1_s);
	const uint16_t aircraft_dist = static_cast<uint16_t>(fabsf(conflict.aircraft_dist));

	PX4_DEBUG("ID: uint %lu, ID str %s, lvl %d, distance %d, last comm %d sec \n",
		  conflict.encoded_id.id,
		  encoded_id_str,
		  conflict.conflict_level,
		  aircraft_dist,
		  time_since_last_comm);
}

void DetectAndAvoid::debug_print_transponder_report(const transponder_report_s &transponder_report)
{
	const int traffic_direction = math::degrees(transponder_report.heading) + 180;

	// Unique ID conversions
	uint64_t uas_id_int = DaaEncodedId::last_uas_id_bytes_to_uint64(transponder_report.uas_id);
	char uas_id_char[kUtmGuidMsgLength];
	DaaEncodedId::convert_uas_id_uint64_to_str(uas_id_int, uas_id_char);

	uint64_t callsign_int = DaaEncodedId::callsign_to_uint64(transponder_report.callsign);
	char callsign[kCallsignLength];
	DaaEncodedId::convert_uint64_callsign_to_str(callsign_int, callsign);

	uint64_t icao_address = static_cast<uint64_t>(transponder_report.icao_address);
	char icao_str[kIcaoLength];
	DaaEncodedId::convert_icao_uint32_to_hex_str(icao_address, icao_str, sizeof(icao_str));

	PX4_DEBUG("ADSB_IN: ICAO uint %lu, ICAO str %s",
		  icao_address,
		  icao_str);
	PX4_DEBUG("ADSB_IN: Callsign uint %lu, Callsign str %s",
		  callsign_int,
		  callsign);
	PX4_DEBUG("ADSB_IN: UAS_ID uint %lu, UAS_ID str %s",
		  uas_id_int,
		  uas_id_char);

	// Log which flags are enabled in one line using printf
	PX4_DEBUG("ADSB_IN: Flags missing: %s%s%s%s%s%s%s",
		  (transponder_report.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS) ? "" : "coord ",
		  (transponder_report.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE) ? "" : "alt ",
		  (transponder_report.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING) ? "" : "hdg ",
		  (transponder_report.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY) ? "" : "vel ",
		  (transponder_report.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN) ? "" : "callsign ",
		  (transponder_report.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_SQUAWK) ? "" : "squawk ",
		  (transponder_report.flags & transponder_report_s::PX4_ADSB_FLAGS_RETRANSLATE) ? "" : "Retranslate ");

	PX4_DEBUG("ADSB_IN: lat %.2f, lon %.2f, alt %.2f, hdg %.d, vel hor %.1f, vel vert %.1f \n",
		  (double)transponder_report.lat,
		  (double)transponder_report.lon,
		  (double)transponder_report.altitude,
		  traffic_direction,
		  (double)transponder_report.hor_velocity,
		  (double)transponder_report.ver_velocity);
}
#endif // DEBUG_BUILD
