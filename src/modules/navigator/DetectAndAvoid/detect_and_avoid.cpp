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
#include <commander/px4_custom_mode.h>
#include <lib/adsb/DaaEncodedId.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/board_common.h>
#include <px4_platform_common/log.h>
#include <uORB/topics/vehicle_command.h>

using namespace time_literals;

static_assert(kUasIdByteLength == PX4_GUID_BYTE_LENGTH, "UAS ID size must match the platform GUID size");

static_assert(detect_and_avoid_s::ORB_QUEUE_LENGTH >= transponder_report_s::ORB_QUEUE_LENGTH,
	      "DAA output queue must hold one full traffic-input burst");

// Static so the buffer lives in .bss instead of Navigator's allocation.
conflict_tracker_changes_s DetectAndAvoid::_cycle_changes;

DetectAndAvoid::DetectAndAvoid(Navigator *navigator) :
	MissionBlock(navigator, 0),
	ModuleParams(navigator)
{
	_detect_and_avoid_pub.advertise();
	_detect_and_avoid_most_urgent_pub.advertise();
#if defined(CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC)
	_fake_traffic_pub.advertise();
#endif // CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC
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
	refresh_ownship_ids();

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
	daa_input_s daa_input{};

	if (!gather_ownship_input(daa_input)) {
		if (!_conflict_tracker.empty()
		    || _conflict_tracker.most_urgent().conflict_level != detect_and_avoid_s::DAA_CONFLICT_LVL_NONE) {
			PX4_DEBUG("DAA: uav pose stale, clearing conflicts");
			clear_conflicts();
		}

		return;
	}

#if defined(CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC)
	process_fake_traffic();
#endif // CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC

	_cycle_changes.clear();

	const hrt_abstime now = hrt_absolute_time();

	if ((_time_last_buffer_clean == 0) || ((now - _time_last_buffer_clean) > kOwnshipPositionTimeout)) {
		_time_last_buffer_clean = now;

		if (_conflict_tracker.remove_stale_conflicts(now,
				static_cast<hrt_abstime>(_param_daa_traff_tout.get()) * 1_s, _cycle_changes)) {
			update_most_urgent_conflict();
		}
	}

	if (process_transponder_queue(daa_input)) {
		update_most_urgent_conflict();
	}

	_conflict_notifier.report_cycle(_cycle_changes, _conflict_tracker, notifier_cycle_context());

	if (_most_urgent_conflict_changed) {
		evaluate_and_publish_action();
		_prev_most_urgent_conflict_level = _conflict_tracker.most_urgent().conflict_level;

		if (_prev_most_urgent_conflict_level == detect_and_avoid_s::DAA_CONFLICT_LVL_NONE) {
			_previous_action = DaaAction::kDisabled;
		}
	}
}

ConflictNotifier::cycle_context_s DetectAndAvoid::notifier_cycle_context() const
{
	ConflictNotifier::cycle_context_s context{};
	context.prev_most_urgent_level = _prev_most_urgent_conflict_level;

	for (uint8_t level = detect_and_avoid_s::DAA_CONFLICT_LVL_LOW;
	     level <= detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL; ++level) {
		if (get_action_from_conflict_level(level) >= DaaAction::kWarnOnly) {
			context.warning_levels_mask |= static_cast<uint8_t>(1u << level);
		}
	}

	context.status_notif_interval = static_cast<hrt_abstime>(math::max<int32_t>(0, _param_daa_notif_state.get())) * 1_s;
	return context;
}

bool DetectAndAvoid::transponder_data_valid(const transponder_report_s &report, const hrt_abstime now,
		const hrt_abstime timeout_us)
{
	if (!AdsbConflict::valid_wgs84_coordinates(report.lat, report.lon)) {
		PX4_DEBUG("DAA: transponder data rejected, invalid lat/lon.");
		return false;
	}

	if (!PX4_ISFINITE(report.altitude)) {
		PX4_DEBUG("DAA: transponder data rejected, invalid altitude.");
		return false;
	}

	uint16_t required_flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
				  transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;

#if !defined(CONFIG_NAVIGATOR_ADSB_F3442) || !CONFIG_NAVIGATOR_ADSB_F3442
	required_flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
			  transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;
#endif // !CONFIG_NAVIGATOR_ADSB_F3442

	if ((report.flags & required_flags) != required_flags) {
		PX4_DEBUG("DAA: transponder data rejected, missing flags.");
		return false;
	}

#if !defined(CONFIG_NAVIGATOR_ADSB_F3442) || !CONFIG_NAVIGATOR_ADSB_F3442

	if (!PX4_ISFINITE(report.heading)) {
		PX4_DEBUG("DAA: transponder data rejected, invalid heading.");
		return false;
	}

#endif // !CONFIG_NAVIGATOR_ADSB_F3442

	if (report.timestamp == 0 || report.timestamp > now) {
		PX4_DEBUG("DAA: transponder data rejected, invalid timestamp.");
		return false;
	}

	const hrt_abstime local_age = now - report.timestamp;
	const hrt_abstime source_age = static_cast<hrt_abstime>(report.tslc) * 1_s;

	if (local_age > timeout_us || source_age > (timeout_us - local_age)) {
		PX4_DEBUG("DAA: transponder data rejected, too old.");
		return false;
	}

	return true;
}

bool DetectAndAvoid::process_transponder_queue(daa_input_s &daa_input)
{
	bool buffer_updated = false;
	transponder_report_s transponder_report{};

	const hrt_abstime traffic_timeout_us = static_cast<hrt_abstime>(_param_daa_traff_tout.get()) * 1_s;

	for (uint8_t processed_reports = 0; processed_reports < transponder_report_s::ORB_QUEUE_LENGTH; ++processed_reports) {

		if (!_traffic_sub.update(&transponder_report)) {
			break;
		}

#if defined(DEBUG_BUILD)
		debug_print_transponder_report(transponder_report);
#endif

		if (!transponder_data_valid(transponder_report, hrt_absolute_time(), traffic_timeout_us)) {
			PX4_DEBUG("DAA: transponder data not valid.");
			continue;
		}

		const bool report_processed = process_transponder_report(daa_input, transponder_report);
		buffer_updated = buffer_updated || report_processed;
	}

	return buffer_updated;
}

bool DetectAndAvoid::process_transponder_report(daa_input_s &daa_input,
		const transponder_report_s &transponder_report)
{
	const DaaEncodedId encoded_id = DaaEncodedId::identify_traffic_report(transponder_report, _ownship_ids);

	if (encoded_id.id == 0) {
		return false;
	}

	daa_input.transponder_report = transponder_report;

	if (!(daa_input.transponder_report.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY)) {
		daa_input.transponder_report.ver_velocity = 0.f;
		daa_input.transponder_report.hor_velocity = 0.f;
	}

#if defined(CONFIG_NAVIGATOR_ADSB_F3442) && CONFIG_NAVIGATOR_ADSB_F3442

	if (_param_daa_en_dflt_vel.get()) {
		const float velocity_sign = (daa_input.transponder_report.ver_velocity >= 0.f) ? 1.f : -1.f;
		daa_input.transponder_report.ver_velocity = velocity_sign * _param_daa_dflt_vel.get();
	}

#endif // CONFIG_NAVIGATOR_ADSB_F3442

	detect_and_avoid_s daa_output{};

	if (!_adsb_conflict_detector.calculate_daa_output(daa_input, daa_output)) {
		PX4_DEBUG("DAA: Failed to calculate DAA output, skipping report.");
		return false;
	}

	daa_output.unique_id = encoded_id.id;
	daa_output.unique_id_encoding = encoded_id.encoding;
	daa_output.timestamp = transponder_report.timestamp;
	_detect_and_avoid_pub.publish(daa_output);

	if (daa_output.conflict_level == detect_and_avoid_s::DAA_CONFLICT_LVL_NONE
	    && !_conflict_tracker.contains(encoded_id)) {
		return false;
	}

	conflict_info_s current_conflict{};
	current_conflict.encoded_id = encoded_id;
	current_conflict.latest_update_timestamp = transponder_report.timestamp;
	current_conflict.conflict_level = daa_output.conflict_level;
	current_conflict.aircraft_dist = daa_output.aircraft_dist;

#if defined(DEBUG_BUILD)
	PX4_DEBUG("DAA: transponder data processed, conflict detected.");
	debug_print_conflict_info(current_conflict);
#endif

	return _conflict_tracker.apply_conflict(current_conflict, _cycle_changes);
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

void DetectAndAvoid::refresh_ownship_ids()
{
	_ownship_ids = {};
	_ownship_ids.icao = _vehicle_adsb_icao.get();
	_ownship_ids.icao_2 = _vehicle_adsb_icao_2.get();

	_ownship_ids.callsign = DaaEncodedId::callsign_params_to_uint64(_vehicle_adsb_callsign_1.get(),
				_vehicle_adsb_callsign_2.get());

#ifndef BOARD_HAS_NO_UUID
	px4_guid_t px4_guid {};

	if (board_get_px4_guid(px4_guid) == PX4_GUID_BYTE_LENGTH) {
		_ownship_ids.uas_id = DaaEncodedId::last_uas_id_bytes_to_uint64(px4_guid);
		_ownship_ids.uas_id_valid = true;

	} else {
		PX4_DEBUG("DAA: Failed to get own UAS ID.");
	}

#endif // BOARD_HAS_NO_UUID
}

bool DetectAndAvoid::gather_ownship_input(daa_input_s &daa_input) const
{
	const vehicle_global_position_s &global_position = *_navigator->get_global_position();

	if (!global_position.lat_lon_valid || !global_position.alt_valid
	    || !AdsbConflict::valid_wgs84_coordinates(global_position.lat, global_position.lon)
	    || !PX4_ISFINITE(global_position.alt)) {
		PX4_DEBUG("DAA: invalid global pose");
		return false;
	}

	if (global_position.timestamp == 0 || hrt_elapsed_time(&global_position.timestamp) > kOwnshipPositionTimeout) {
		PX4_DEBUG("DAA: stale global pose");
		return false;
	}

	const vehicle_local_position_s &local_position = *_navigator->get_local_position();

	if (local_position.timestamp == 0 || hrt_elapsed_time(&local_position.timestamp) > kOwnshipPositionTimeout) {
		PX4_DEBUG("DAA: stale local position");
		return false;
	}

	daa_input.uav_lat_lon = matrix::Vector2d(global_position.lat, global_position.lon);
	daa_input.uav_alt = global_position.alt;

#if defined(CONFIG_NAVIGATOR_ADSB_F3442) && CONFIG_NAVIGATOR_ADSB_F3442
	// F3442's static alert volumes remain usable when a velocity group is unavailable.
	daa_input.uav_vel_ned(0) = local_position.v_xy_valid && PX4_ISFINITE(local_position.vx) ? local_position.vx : 0.f;
	daa_input.uav_vel_ned(1) = local_position.v_xy_valid && PX4_ISFINITE(local_position.vy) ? local_position.vy : 0.f;
	daa_input.uav_vel_ned(2) = local_position.v_z_valid && PX4_ISFINITE(local_position.vz) ? local_position.vz : 0.f;
#else

	if (!local_position.v_xy_valid || !local_position.v_z_valid
	    || !PX4_ISFINITE(local_position.vx) || !PX4_ISFINITE(local_position.vy)
	    || !PX4_ISFINITE(local_position.vz)) {
		PX4_DEBUG("DAA: invalid local velocity");
		return false;
	}

	daa_input.uav_vel_ned = matrix::Vector3f(local_position.vx, local_position.vy, local_position.vz);
#endif // CONFIG_NAVIGATOR_ADSB_F3442

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
			_navigator->set_rtl_return_alt_min(true);
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

	PX4_DEBUG("Max conflict: Unique ID %" PRIu64 ", ID str %s, lvl %d, distance %d, last comm %d sec \n",
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

	PX4_DEBUG("ID: uint %" PRIu64 ", ID str %s, lvl %d, distance %d, last comm %d sec \n",
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

	PX4_DEBUG("ADSB_IN: ICAO uint %" PRIu64 ", ICAO str %s",
		  icao_address,
		  icao_str);
	PX4_DEBUG("ADSB_IN: Callsign uint %" PRIu64 ", Callsign str %s",
		  callsign_int,
		  callsign);
	PX4_DEBUG("ADSB_IN: UAS_ID uint %" PRIu64 ", UAS_ID str %s",
		  uas_id_int,
		  uas_id_char);

	// Log which flags are enabled in one line using printf
	PX4_DEBUG("ADSB_IN: Flags missing: %s%s%s%s%s%s",
		  (transponder_report.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS) ? "" : "coord ",
		  (transponder_report.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE) ? "" : "alt ",
		  (transponder_report.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING) ? "" : "hdg ",
		  (transponder_report.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY) ? "" : "vel ",
		  (transponder_report.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN) ? "" : "callsign ",
		  (transponder_report.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_SQUAWK) ? "" : "squawk ");

	PX4_DEBUG("ADSB_IN: lat %.2f, lon %.2f, alt %.2f, hdg %.d, vel hor %.1f, vel vert %.1f \n",
		  (double)transponder_report.lat,
		  (double)transponder_report.lon,
		  (double)transponder_report.altitude,
		  traffic_direction,
		  (double)transponder_report.hor_velocity,
		  (double)transponder_report.ver_velocity);
}
#endif // DEBUG_BUILD
