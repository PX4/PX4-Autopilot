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
 * @file detect_and_avoid_notify.cpp
 *
 * Notify traffic status
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "detect_and_avoid.h"

#include <lib/mathlib/mathlib.h>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>

#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <uORB/uORB.h>

using namespace time_literals;

#if defined(DEBUG_BUILD)
void DetectAndAvoid::debug_print_buffer_status()
{
	char encoded_id_str[kUtmGuidMsgLength];
	_most_urgent_conflict.encoded_id.to_string(encoded_id_str, sizeof(encoded_id_str));

	const int time_since_last_comm = static_cast<int>((hrt_absolute_time() -
					 _most_urgent_conflict.latest_update_timestamp) / 1_s);
	const uint16_t aircraft_dist = static_cast<uint16_t>(fabsf(_most_urgent_conflict.aircraft_dist));

	const int buff_size = static_cast<int>(_traffic_buffer.size());

	PX4_DEBUG("Buffer status: %d conflict(s), max lvl %d, prev max lvl %d,",
		  buff_size,
		  _most_urgent_conflict.conflict_level,
		  _prev_most_urgent_conflict_level);

	PX4_DEBUG("Max conflict: Unique ID %lu, ID str %s, lvl %d, distance %d, last comm %d sec \n",
		  _most_urgent_conflict.encoded_id.id,
		  encoded_id_str,
		  _most_urgent_conflict.conflict_level,
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
#endif

bool DetectAndAvoid::must_notify(const uint8_t current_conflict_level, const hrt_abstime time_last_notified,
				 const hrt_abstime interval, const uint8_t previous_conflict_level) const
{
	const bool current_lvl_requires_warning = conflict_lvl_requires_warning(current_conflict_level);
	const bool time_to_notify = (time_last_notified == 0) || ((hrt_absolute_time() - time_last_notified) > interval);

	// Force notification if conflict level changed and at least one of the levels requires notifications.
	// Callers that only need periodic throttling should pass previous_conflict_level = current_conflict_level.
	const bool prev_action_requires_warning = previous_conflict_level != 0
			&& conflict_lvl_requires_warning(previous_conflict_level);
	const bool force_notification = (previous_conflict_level != current_conflict_level) &&
					(prev_action_requires_warning || current_lvl_requires_warning);

	return force_notification || (current_lvl_requires_warning && time_to_notify);
}

bool DetectAndAvoid::conflict_lvl_requires_warning(const uint8_t conflict_level) const
{
	return get_action_from_conflict_level(conflict_level) >= DaaAction::kWarnOnly;
}

bool DetectAndAvoid::conflict_lvl_requires_action(const uint8_t conflict_level) const
{
	return get_action_from_conflict_level(conflict_level) > DaaAction::kWarnOnly;
}

void DetectAndAvoid::notify_traffic_ignored(const conflict_info_s &conflict_info, const IgnoreTrafficCause cause)
{
	char encoded_id_str[kUtmGuidMsgLength];
	conflict_info.encoded_id.to_string(encoded_id_str, sizeof(encoded_id_str));

	_time_last_traffic_ignored = hrt_absolute_time();

	mavlink_log_info(&_mavlink_log_pub, "DAA %s ignored (%d) lvl %d.\t",
			 encoded_id_str,
			 static_cast<uint8_t>(cause),
			 conflict_info.conflict_level);
	/* EVENT
	* @description
	* - ID: {1}
	* - cause: {2} (0:buffer full, 1:update failed, 2:removal failed, 3:insert failed, 4:level lookup failed, 5:invalid index)
	* - conflict level: {3}
	*/
	events::send<uint64_t, uint8_t, uint8_t>(events::ID("navigator_traffic_ignore"), events::Log::Warning,
			"DAA: ignored",
			conflict_info.encoded_id.id, static_cast<uint8_t>(cause), conflict_info.conflict_level);
}

void DetectAndAvoid::notify_traffic_removed(const conflict_info_s &conflict_info, const RemoveBufferCause cause)
{
	const int time_since_last_comm = static_cast<int>((hrt_absolute_time() - conflict_info.latest_update_timestamp) / 1_s);

	char encoded_id_str[kUtmGuidMsgLength];
	conflict_info.encoded_id.to_string(encoded_id_str, sizeof(encoded_id_str));

	mavlink_log_warning(&_mavlink_log_pub, "DAA %s out (%d) lvl %d (%ds).\t",
			    encoded_id_str,
			    static_cast<uint8_t>(cause),
			    conflict_info.conflict_level,
			    time_since_last_comm);
	/* EVENT
	* @description
	* - ID: {1}
	* - cause: {2} (0:stale, 1:buffer full)
	* - conflict level: {3}
	* - last seen: {4} s ago
	*/
	events::send<uint64_t, uint8_t, uint8_t, float>(events::ID("navigator_traffic_remove"), events::Log::Warning,
			"DAA: removed",
			conflict_info.encoded_id.id, static_cast<uint8_t>(cause), conflict_info.conflict_level, time_since_last_comm);
}

bool DetectAndAvoid::mavlink_log_conflict_by_level(const uint8_t conflict_level,
		const char message_buffer[kMaxLogMsgSize], events::Log &log_level)
{
	switch (conflict_level) {
	case detect_and_avoid_s::DAA_CONFLICT_LVL_NONE:
		return false;

	case detect_and_avoid_s::DAA_CONFLICT_LVL_LOW:
	case detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM:
		log_level = events::Log::Warning;
		break;

	case detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH:
		log_level = events::Log::Critical;
		break;

	case detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL:
		log_level = events::Log::Emergency;
		break;

	default:
		return false;
	}

	mavlink_log_warning(&_mavlink_log_pub, "%s.\t", message_buffer);
	return true;
}

void DetectAndAvoid::notify_conflict_level(const conflict_info_s &conflict_info,
		const uint8_t previous_conflict_level, const ConflictNotifyKind kind)
{
	const uint8_t conflict_level = conflict_info.conflict_level;

	// Secondary conflicts are only announced on a level change; the most urgent conflict also
	// emits a periodic status while the level is unchanged.
	if (kind == ConflictNotifyKind::kSecondary && conflict_level == previous_conflict_level) {
		return;
	}

	const uint16_t aircraft_dist = static_cast<uint16_t>(fabsf(conflict_info.aircraft_dist));

	char encoded_id_str[kUtmGuidMsgLength];
	conflict_info.encoded_id.to_string(encoded_id_str, sizeof(encoded_id_str));

	const char *const prefix = (kind == ConflictNotifyKind::kSecondary) ? "DAA SEC:"
				   : (kind == ConflictNotifyKind::kMostUrgentNew) ? "DAA New and Main:" : "DAA Main:";

	events::Log log_level = events::Log::Warning;

	if (conflict_level >= previous_conflict_level) {
		// Escalation, or (most urgent only) a periodic update at the unchanged level.
		const bool escalation = conflict_level > previous_conflict_level;

		char message_buffer[kMaxLogMsgSize];
		snprintf(message_buffer, kMaxLogMsgSize, escalation ? "%s %s lvl UP %d. %d m" : "%s %s lvl %d. %d m",
			 prefix, encoded_id_str, conflict_level, aircraft_dist);

		// Escalations and the first "new + most urgent" report carry level-based severity;
		// a routine most-urgent update stays informational.
		if (escalation || kind == ConflictNotifyKind::kMostUrgentNew) {
			if (!mavlink_log_conflict_by_level(conflict_level, message_buffer, log_level)) {
				return;
			}

		} else {
			mavlink_log_info(&_mavlink_log_pub, "%s.\t", message_buffer);
		}

	} else if (conflict_level == detect_and_avoid_s::DAA_CONFLICT_LVL_NONE) {
		// De-escalation to no conflict. For the most-urgent path this is the last tracked conflict.
		if (kind == ConflictNotifyKind::kSecondary) {
			mavlink_log_info(&_mavlink_log_pub, "%s %s solved. %d m.\t", prefix, encoded_id_str, aircraft_dist);

		} else {
			mavlink_log_info(&_mavlink_log_pub, "DAA all conflicts solved.\t");
		}

	} else {
		// De-escalation to a lower, still-active level.
		mavlink_log_info(&_mavlink_log_pub, "%s %s lvl DOWN %d. %d m.\t", prefix, encoded_id_str, conflict_level,
				 aircraft_dist);
	}

	// A single structured event covers every transition: the current and previous levels make the
	// direction (escalation, reduction, solved) explicit, and the kind distinguishes the most-urgent
	// conflict from secondary traffic without a separate event ID per case.
	/* EVENT
	 * @description
	 * - ID: {1}
	 * - conflict level: {2}
	 * - previous level: {3}
	 * - distance: {4m}
	 * - kind: {5} (0:main, 1:new+main, 2:secondary)
	 */
	events::send<uint64_t, uint8_t, uint8_t, uint32_t, uint8_t>(events::ID("navigator_traffic_conflict_update"),
			log_level, "DAA conflict update",
			conflict_info.encoded_id.id, conflict_level, previous_conflict_level, aircraft_dist,
			static_cast<uint8_t>(kind));
}

void DetectAndAvoid::notify_new_conflict(const conflict_info_s &conflict_info)
{
	const uint16_t aircraft_dist = static_cast<uint16_t>(fabsf(conflict_info.aircraft_dist));
	const uint8_t conflict_level = conflict_info.conflict_level;

	char encoded_id_str[kUtmGuidMsgLength];
	conflict_info.encoded_id.to_string(encoded_id_str, sizeof(encoded_id_str));

	char message_buffer[kMaxLogMsgSize];
	snprintf(message_buffer, kMaxLogMsgSize,
		 "DAA New %s lvl %d. %d m",
		 encoded_id_str, conflict_level, aircraft_dist);

	events::Log log_level = events::Log::Warning;

	if (mavlink_log_conflict_by_level(conflict_level, message_buffer, log_level)) {

		/* EVENT
		* @description
		* - ID: {1}
		* - conflict level: {2}
		* - distance: {3m}
		*/
		events::send<uint64_t, uint8_t, uint32_t>(events::ID("navigator_new_traffic"),
				log_level,
				"New traffic",
				conflict_info.encoded_id.id, conflict_info.conflict_level, aircraft_dist);
	}
}

void DetectAndAvoid::notify_action_on_ground(const NotifyLandedActCause cause)
{
	// The blocked action (takeoff/arm) is the only difference between the two cases,
	// so a single format string and event ID are reused to save flash.
	const char *blocked_action = nullptr;
	events::Log log_level = events::Log::Critical;

	switch (cause) {
	case NotifyLandedActCause::kConflictAndArmed:
		blocked_action = "takeoff";
		log_level = events::Log::Critical;
		break;

	case NotifyLandedActCause::kConflictAndDisarmed:
		blocked_action = "arm";
		log_level = events::Log::Warning;
		break;

	default:
		PX4_DEBUG("DAA: invalid landed cause");
		return;
	}

	mavlink_log_critical(&_mavlink_log_pub, "DAA do not %s until air conflict solved!\t", blocked_action);
	events::send(events::ID("navigator_traffic_ground_conflict"), log_level,
		     "DAA: resolve air-traffic conflict before flight");
}

void DetectAndAvoid::notify_new_action(const conflict_info_s &conflict_info, const DaaAction action)
{
	const uint16_t aircraft_dist = static_cast<uint16_t>(fabsf(conflict_info.aircraft_dist));
	const uint8_t conflict_level = conflict_info.conflict_level;

	// The action is carried as an event argument, so a single event ID covers
	// every automated response instead of one ID per action.
	const char *action_name = nullptr;
	events::Log log_level = events::Log::Warning;

	switch (action) {
	case DaaAction::kWarnOnly:
		action_name = "Warning";
		break;

	case DaaAction::kPositionHoldMode:
		action_name = "Hold";
		break;

	case DaaAction::kReturnMode:
		action_name = "Return";
		log_level = events::Log::Critical;
		break;

	case DaaAction::kLandMode:
		action_name = "Land";
		log_level = events::Log::Critical;
		break;

	case DaaAction::kTerminate:
		action_name = "Terminate";
		log_level = events::Log::Emergency;
		break;

	case DaaAction::kDisabled:
	default:
		// No automated response to announce.
		return;
	}

	char encoded_id_str[kUtmGuidMsgLength];
	conflict_info.encoded_id.to_string(encoded_id_str, sizeof(encoded_id_str));

	if (action == DaaAction::kTerminate) {
		mavlink_log_emergency(&_mavlink_log_pub, "DAA %s act: %s! lvl %d dist %dm\t",
				      encoded_id_str, action_name, conflict_level, aircraft_dist);

	} else {
		mavlink_log_warning(&_mavlink_log_pub, "DAA %s: %s! lvl %d. %d m.\t",
				    encoded_id_str, action_name, conflict_level, aircraft_dist);
	}

	/* EVENT
	 * @description
	 * - ID: {1}
	 * - action: {2} (1:warn, 2:return, 3:land, 4:hold, 5:terminate)
	 * - conflict level: {3}
	 * - distance: {4m}
	 */
	events::send<uint64_t, uint8_t, uint8_t, uint32_t>(events::ID("navigator_traffic_action"), log_level,
			"DAA automated action",
			conflict_info.encoded_id.id, daa_action_to_action_param(action), conflict_level, aircraft_dist);
}
