/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file mavlink_mission.cpp
 * MAVLink mission manager implementation.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

#include "mavlink_mission.h"
#include "mavlink_main.h"

#include <lib/geo/geo.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/events.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <navigator/navigation.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <crc32.h>

using matrix::wrap_2pi;

dm_item_t MavlinkMissionManager::_mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
dm_item_t MavlinkMissionManager::_safepoint_dataman_id = DM_KEY_SAFE_POINTS_0;
dm_item_t MavlinkMissionManager::_fence_dataman_id = DM_KEY_FENCE_POINTS_0;
bool MavlinkMissionManager::_dataman_init = false;
uint16_t MavlinkMissionManager::_count[3] = { 0, 0, 0 };
uint32_t MavlinkMissionManager::_crc32[3] = { 0, 0, 0 };
int32_t MavlinkMissionManager::_current_seq = 0;
bool MavlinkMissionManager::_transfer_in_progress = false;
constexpr uint16_t MavlinkMissionManager::MAX_COUNT[];

#define CHECK_SYSID_COMPID_MISSION(_msg)		(_msg.target_system == mavlink_system.sysid && \
		((_msg.target_component == mavlink_system.compid) || \
		 (_msg.target_component == MAV_COMP_ID_MISSIONPLANNER) || \
		 (_msg.target_component == MAV_COMP_ID_ALL)))

MavlinkMissionManager::MavlinkMissionManager(Mavlink *mavlink) :
	_mavlink(mavlink)
{
	if (!_dataman_init) {
		_dataman_init = true;

		mission_s mission_state;
		bool success = _dataman_client.readSync(DM_KEY_MISSION_STATE, 0, reinterpret_cast<uint8_t *>(&mission_state),
							sizeof(mission_s));

		if (success) {
			init_offboard_mission(mission_state);
			load_geofence_stats();
			load_safepoint_stats();

		} else {
			PX4_WARN("offboard mission init failed");
		}

		update_active_mission(_mission_dataman_id, _count[MAV_MISSION_TYPE_MISSION], _current_seq,
				      _crc32[MAV_MISSION_TYPE_MISSION], false);
	}

	_my_mission_dataman_id = _mission_dataman_id;
	_my_fence_dataman_id = _fence_dataman_id;
	_my_safepoint_dataman_id = _safepoint_dataman_id;
}

void
MavlinkMissionManager::init_offboard_mission(const mission_s &mission_state)
{
	_mission_dataman_id = (dm_item_t)mission_state.mission_dataman_id;
	_my_mission_dataman_id = _mission_dataman_id;
	_count[MAV_MISSION_TYPE_MISSION] = mission_state.count;
	_crc32[MAV_MISSION_TYPE_MISSION] = mission_state.mission_id;
	_current_seq = mission_state.current_seq;
	_land_start_marker = mission_state.land_start_index;
	_land_marker = mission_state.land_index;
}

bool
MavlinkMissionManager::load_geofence_stats()
{
	mission_stats_entry_s stats;
	// initialize fence points count
	bool success = _dataman_client.readSync(DM_KEY_FENCE_POINTS_STATE, 0, reinterpret_cast<uint8_t *>(&stats),
						sizeof(mission_stats_entry_s));

	if (success) {
		_count[MAV_MISSION_TYPE_FENCE] = stats.num_items;
		_crc32[MAV_MISSION_TYPE_FENCE] = stats.opaque_id;
		_fence_dataman_id = static_cast<dm_item_t>(stats.dataman_id);
		_my_fence_dataman_id = _fence_dataman_id;
	}

	return success;
}

bool
MavlinkMissionManager::load_safepoint_stats()
{
	mission_stats_entry_s stats;
	// initialize safe points count
	bool success = _dataman_client.readSync(DM_KEY_SAFE_POINTS_STATE, 0, reinterpret_cast<uint8_t *>(&stats),
						sizeof(mission_stats_entry_s));

	if (success) {
		_count[MAV_MISSION_TYPE_RALLY] = stats.num_items;
		_crc32[MAV_MISSION_TYPE_RALLY] = stats.opaque_id;
		_safepoint_dataman_id = static_cast<dm_item_t>(stats.dataman_id);
		_my_safepoint_dataman_id = _safepoint_dataman_id;
	}

	return success;
}

/**
 * Publish mission topic to notify navigator about changes.
 */
void
MavlinkMissionManager::update_active_mission(dm_item_t mission_dataman_id, uint16_t count, int32_t seq, uint32_t crc32,
		bool write_to_dataman)
{
	/* update active mission state */
	_mission_dataman_id = mission_dataman_id;
	_my_mission_dataman_id = _mission_dataman_id;
	_count[MAV_MISSION_TYPE_MISSION] = count;
	_crc32[MAV_MISSION_TYPE_MISSION] = crc32;
	_current_seq = seq;

	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.mission_dataman_id = mission_dataman_id;
	mission.fence_dataman_id = _fence_dataman_id;
	mission.safepoint_dataman_id = _safepoint_dataman_id;
	mission.count = count;
	mission.current_seq = seq;
	mission.mission_id = _crc32[MAV_MISSION_TYPE_MISSION];
	mission.geofence_id = _crc32[MAV_MISSION_TYPE_FENCE];
	mission.safe_points_id = _crc32[MAV_MISSION_TYPE_RALLY];
	mission.land_start_index = _land_start_marker;
	mission.land_index = _land_marker;

	if (write_to_dataman) {
		bool success = _dataman_client.writeSync(DM_KEY_MISSION_STATE, 0, reinterpret_cast<uint8_t *>(&mission),
				sizeof(mission_s));

		if (!success) {
			PX4_ERR("Can't update mission state in Dataman");
		}
	}

	_offboard_mission_pub.publish(mission);
}

int
MavlinkMissionManager::update_geofence_count(dm_item_t fence_dataman_id, unsigned count, uint32_t crc32)
{
	_fence_dataman_id = fence_dataman_id;
	_my_fence_dataman_id = fence_dataman_id;

	mission_stats_entry_s stats;
	stats.num_items = count;
	stats.opaque_id = crc32;
	stats.dataman_id = fence_dataman_id;

	/* update stats in dataman */
	bool success = _dataman_client.writeSync(DM_KEY_FENCE_POINTS_STATE, 0, reinterpret_cast<uint8_t *>(&stats),
			sizeof(mission_stats_entry_s));

	if (success) {
		_count[MAV_MISSION_TYPE_FENCE] = count;
		_crc32[MAV_MISSION_TYPE_FENCE] = crc32;

	} else {

		if (_filesystem_errcount++ < FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT) {
			_mavlink->send_statustext_critical("Mission storage: Unable to write to microSD\t");
			events::send(events::ID("mavlink_mission_storage_write_failure2"), events::Log::Critical,
				     "Mission: Unable to write to storage");
		}

		return PX4_ERROR;
	}

	update_active_mission(_mission_dataman_id, _count[MAV_MISSION_TYPE_MISSION], _current_seq,
			      _crc32[MAV_MISSION_TYPE_MISSION],
			      false);
	return PX4_OK;
}

int
MavlinkMissionManager::update_safepoint_count(dm_item_t safepoint_dataman_id, unsigned count, uint32_t crc32)
{
	_safepoint_dataman_id = safepoint_dataman_id;
	_my_safepoint_dataman_id = safepoint_dataman_id;

	mission_stats_entry_s stats;
	stats.num_items = count;
	stats.opaque_id = crc32;
	stats.dataman_id = safepoint_dataman_id;

	/* update stats in dataman */
	bool success = _dataman_client.writeSync(DM_KEY_SAFE_POINTS_STATE, 0, reinterpret_cast<uint8_t *>(&stats),
			sizeof(mission_stats_entry_s));

	if (success) {
		_count[MAV_MISSION_TYPE_RALLY] = count;
		_crc32[MAV_MISSION_TYPE_RALLY] = crc32;

	} else {

		if (_filesystem_errcount++ < FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT) {
			_mavlink->send_statustext_critical("Mission storage: Unable to write to microSD\t");
			events::send(events::ID("mavlink_mission_storage_write_failure3"), events::Log::Critical,
				     "Mission: Unable to write to storage");
		}

		return PX4_ERROR;
	}

	update_active_mission(_mission_dataman_id, _count[MAV_MISSION_TYPE_MISSION], _current_seq,
			      _crc32[MAV_MISSION_TYPE_MISSION],
			      false);
	return PX4_OK;
}

void
MavlinkMissionManager::send_mission_ack(uint8_t sysid, uint8_t compid, uint8_t type, uint32_t opaque_id)
{
	mavlink_mission_ack_t wpa{};

	wpa.target_system = sysid;
	wpa.target_component = compid;
	wpa.type = type;
	wpa.mission_type = _mission_type;
	wpa.opaque_id = opaque_id;

	mavlink_msg_mission_ack_send_struct(_mavlink->get_channel(), &wpa);

	PX4_DEBUG("WPM: Send MISSION_ACK type %u to ID %u", wpa.type, wpa.target_system);
}

void
MavlinkMissionManager::send_mission_current(uint16_t seq)
{
	mavlink_mission_current_t wpc{};
	wpc.seq = seq;
	wpc.total = _count[MAV_MISSION_TYPE_MISSION] > 0 ? _count[MAV_MISSION_TYPE_MISSION] : UINT16_MAX;
	wpc.mission_id = _crc32[MAV_MISSION_TYPE_MISSION];
	wpc.fence_id = _crc32[MAV_MISSION_TYPE_FENCE];
	wpc.rally_points_id = _crc32[MAV_MISSION_TYPE_RALLY];
	mavlink_msg_mission_current_send_struct(_mavlink->get_channel(), &wpc);

	PX4_DEBUG("WPM: Send MISSION_CURRENT seq %d", seq);
}

void
MavlinkMissionManager::send_mission_count(uint8_t sysid, uint8_t compid, uint16_t count, MAV_MISSION_TYPE mission_type,
		uint32_t opaque_id)
{
	_time_last_sent = hrt_absolute_time();

	mavlink_mission_count_t wpc{};

	wpc.target_system = sysid;
	wpc.target_component = compid;
	wpc.count = count;
	wpc.mission_type = mission_type;
	wpc.opaque_id = opaque_id;

	mavlink_msg_mission_count_send_struct(_mavlink->get_channel(), &wpc);

	PX4_DEBUG("WPM: Send MISSION_COUNT %u to ID %u, mission type=%i", wpc.count, wpc.target_system, mission_type);
}

void
MavlinkMissionManager::send_mission_item(uint8_t sysid, uint8_t compid, uint16_t seq)
{
	mission_item_s mission_item{};
	bool read_success = false;

	switch (_mission_type) {

	case MAV_MISSION_TYPE_MISSION: {
			read_success = _dataman_client.readSync(_mission_dataman_id, seq, reinterpret_cast<uint8_t *>(&mission_item),
								sizeof(mission_item_s));
		}
		break;

	case MAV_MISSION_TYPE_FENCE: { // Read a geofence point
			mission_fence_point_s mission_fence_point;
			read_success = _dataman_client.readSync(_fence_dataman_id, seq,
								reinterpret_cast<uint8_t *>(&mission_fence_point), sizeof(mission_fence_point_s));

			mission_item.nav_cmd = mission_fence_point.nav_cmd;
			mission_item.frame = mission_fence_point.frame;
			mission_item.lat = mission_fence_point.lat;
			mission_item.lon = mission_fence_point.lon;
			mission_item.altitude = mission_fence_point.alt;

			if (mission_fence_point.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION ||
			    mission_fence_point.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION) {
				mission_item.vertex_count = mission_fence_point.vertex_count;

			} else {
				mission_item.circle_radius = mission_fence_point.circle_radius;
			}
		}
		break;

	case MAV_MISSION_TYPE_RALLY: { // Read a safe point / rally point
			read_success = _dataman_client.readSync(_safepoint_dataman_id, seq, reinterpret_cast<uint8_t *>(&mission_item),
								sizeof(mission_item_s));
		}
		break;

	default:
		_mavlink->send_statustext_critical("Received unknown mission type, abort.\t");
		events::send(events::ID("mavlink_mission_recv_unknown_mis_type"), events::Log::Error,
			     "Received unknown mission type, abort");
		break;
	}

	if (read_success) {
		_time_last_sent = hrt_absolute_time();

		if (_int_mode) {
			mavlink_mission_item_int_t wp{};
			format_mavlink_mission_item(&mission_item, reinterpret_cast<mavlink_mission_item_t *>(&wp));

			wp.target_system = sysid;
			wp.target_component = compid;
			wp.seq = seq;
			wp.current = (_current_seq == seq) ? 1 : 0;

			mavlink_msg_mission_item_int_send_struct(_mavlink->get_channel(), &wp);

			PX4_DEBUG("WPM: Send MISSION_ITEM_INT seq %u to ID %u", wp.seq, wp.target_system);

		} else {
			mavlink_mission_item_t wp{};
			format_mavlink_mission_item(&mission_item, &wp);

			wp.target_system = sysid;
			wp.target_component = compid;
			wp.seq = seq;
			wp.current = (_current_seq == seq) ? 1 : 0;

			mavlink_msg_mission_item_send_struct(_mavlink->get_channel(), &wp);

			PX4_DEBUG("WPM: Send MISSION_ITEM seq %u to ID %u", wp.seq, wp.target_system);
		}

	} else {
		send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);

		if (_filesystem_errcount++ < FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT) {
			mavlink_log_critical(_mavlink->get_mavlink_log_pub(),
					     "Mission storage: Unable to read from storage, type: %" PRId8 "\t", (uint8_t)_mission_type);
			/* EVENT
			 * @description Mission type: {1}
			 */
			events::send<uint8_t>(events::ID("mavlink_mission_storage_read_failure"), events::Log::Error,
					      "Mission: Unable to read from storage", _mission_type);
		}

		PX4_DEBUG("WPM: Send MISSION_ITEM ERROR: could not read seq %u from dataman ID %i", seq, _mission_dataman_id);
	}
}

uint16_t
MavlinkMissionManager::current_max_item_count()
{
	if (_mission_type >= sizeof(MAX_COUNT) / sizeof(MAX_COUNT[0])) {
		PX4_ERR("WPM: MAX_COUNT out of bounds (%u)", _mission_type);
		return 0;
	}

	return MAX_COUNT[_mission_type];
}

uint16_t
MavlinkMissionManager::current_item_count()
{
	if (_mission_type >= sizeof(_count) / sizeof(_count[0])) {
		PX4_ERR("WPM: _count out of bounds (%u)", _mission_type);
		return 0;
	}

	return _count[_mission_type];
}

uint32_t
MavlinkMissionManager::get_current_mission_type_crc()
{
	if (_mission_type >= sizeof(_crc32) / sizeof(_crc32[0])) {
		PX4_ERR("WPM: _crc32 out of bounds (%u)", _mission_type);
		return 0;
	}

	return _crc32[_mission_type];
}

void
MavlinkMissionManager::send_mission_request(uint8_t sysid, uint8_t compid, uint16_t seq)
{
	if (seq < current_max_item_count()) {
		_time_last_sent = hrt_absolute_time();

		if (_int_mode) {
			mavlink_mission_request_int_t wpr{};
			wpr.target_system = sysid;
			wpr.target_component = compid;
			wpr.seq = seq;
			wpr.mission_type = _mission_type;
			mavlink_msg_mission_request_int_send_struct(_mavlink->get_channel(), &wpr);

			PX4_DEBUG("WPM: Send MISSION_REQUEST_INT seq %u to ID %u", wpr.seq, wpr.target_system);

		} else {

			mavlink_mission_request_t wpr{};
			wpr.target_system = sysid;
			wpr.target_component = compid;
			wpr.seq = seq;
			wpr.mission_type = _mission_type;

			mavlink_msg_mission_request_send_struct(_mavlink->get_channel(), &wpr);

			PX4_DEBUG("WPM: Send MISSION_REQUEST seq %u to ID %u", wpr.seq, wpr.target_system);
		}

	} else {
		_mavlink->send_statustext_critical("ERROR: Waypoint index exceeds list capacity\t");
		events::send<uint16_t>(events::ID("mavlink_mission_wp_index_exceeds_list"), events::Log::Error,
				       "Waypoint index eceeds list capacity (maximum: {1})", current_max_item_count());

		PX4_DEBUG("WPM: Send MISSION_REQUEST ERROR: seq %u exceeds list capacity", seq);
	}
}

void
MavlinkMissionManager::send_mission_item_reached(uint16_t seq)
{
	mavlink_mission_item_reached_t wp_reached{};

	wp_reached.seq = seq;

	mavlink_msg_mission_item_reached_send_struct(_mavlink->get_channel(), &wp_reached);

	PX4_DEBUG("WPM: Send MISSION_ITEM_REACHED reached_seq %u", wp_reached.seq);
}

void
MavlinkMissionManager::send()
{
	// do not send anything over high latency communication
	if (_mavlink->get_mode() == Mavlink::MAVLINK_MODE_IRIDIUM) {
		return;
	}

	mission_result_s mission_result{};

	if (_mission_result_sub.update(&mission_result)) {

		if (_current_seq != mission_result.seq_current) {

			_current_seq = mission_result.seq_current;

			PX4_DEBUG("WPM: got mission result, new current_seq: %ld", _current_seq);

			if (mission_result.seq_total > 0) {
				if (mission_result.seq_current < mission_result.seq_total) {
					send_mission_current(_current_seq);

				} else {
					_mavlink->send_statustext_critical("ERROR: wp index out of bounds\t");
					events::send<uint16_t, uint16_t>(events::ID("mavlink_mission_wp_index_out_of_bounds"), events::Log::Error,
									 "Waypoint index out of bounds (current {1} \\>= total {2})", mission_result.seq_current, mission_result.seq_total);
				}
			}
		}

		if (_last_reached != mission_result.seq_reached) {

			_last_reached = mission_result.seq_reached;
			_reached_sent_count = 0;

			PX4_DEBUG("WPM: got mission result, new seq_reached: %ld", _last_reached);

			if ((mission_result.seq_total > 0) && (_last_reached >= 0)) {
				send_mission_item_reached((uint16_t)_last_reached);
			}
		}

		if (mission_result.item_do_jump_changed
		    && (mission_result.seq_total > 0)
		    && (mission_result.item_changed_index < mission_result.seq_total)) {

			/* Send a mission item again if the remaining DO_JUMPs has changed, but don't interfere
			* if there are ongoing transfers happening already. */
			if (_state == MAVLINK_WPM_STATE_IDLE) {
				_mission_type = MAV_MISSION_TYPE_MISSION;
				send_mission_item(_transfer_partner_sysid, _transfer_partner_compid, mission_result.item_changed_index);
			}
		}

	} else if (_slow_rate_limiter.check(hrt_absolute_time())) {
		send_mission_current(_current_seq);

		if ((_count[MAV_MISSION_TYPE_MISSION] > 0) && (_current_seq >= 0)) {
			// send the reached message another 10 times
			if (_last_reached >= 0 && (_reached_sent_count < 10)) {
				send_mission_item_reached((uint16_t)_last_reached);
				_reached_sent_count++;
			}
		}
	}

	/* check for timed-out operations */
	if (_state == MAVLINK_WPM_STATE_GETLIST && (_time_last_sent > 0)
	    && hrt_elapsed_time(&_time_last_sent) > MAVLINK_MISSION_RETRY_TIMEOUT_DEFAULT) {

		// try to request item again after timeout
		send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);

	} else if (_state != MAVLINK_WPM_STATE_IDLE && (_time_last_recv > 0)
		   && hrt_elapsed_time(&_time_last_recv) > MAVLINK_MISSION_PROTOCOL_TIMEOUT_DEFAULT) {

		_mavlink->send_statustext_critical("Operation timeout\t");
		events::send(events::ID("mavlink_mission_op_timeout"), events::Log::Error,
			     "Operation timeout, aborting transfer");

		PX4_DEBUG("WPM: Last operation (state=%d) timed out, changing state to MAVLINK_WPM_STATE_IDLE", _state);

		switch_to_idle_state();

		// since we are giving up, reset this state also, so another request can be started.
		_transfer_in_progress = false;

	} else if (_state == MAVLINK_WPM_STATE_IDLE) {
		// reset flags
		_time_last_sent = 0;
		_time_last_recv = 0;
	}
}

void
MavlinkMissionManager::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_MISSION_ACK:
		handle_mission_ack(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
		handle_mission_set_current(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
		handle_mission_request_list(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_REQUEST:
		handle_mission_request(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
		handle_mission_request_int(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_COUNT:
		handle_mission_count(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_ITEM:
		handle_mission_item(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_ITEM_INT:
		handle_mission_item_int(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
		handle_mission_clear_all(msg);
		break;

	default:
		break;
	}
}

void
MavlinkMissionManager::handle_mission_ack(const mavlink_message_t *msg)
{
	mavlink_mission_ack_t wpa{};
	mavlink_msg_mission_ack_decode(msg, &wpa);

	if (CHECK_SYSID_COMPID_MISSION(wpa)) {
		if ((msg->sysid == _transfer_partner_sysid && msg->compid == _transfer_partner_compid)) {
			if (_state == MAVLINK_WPM_STATE_SENDLIST && _mission_type == wpa.mission_type) {

				_time_last_recv = hrt_absolute_time();

				if (wpa.type == MAV_MISSION_ACCEPTED && _transfer_seq == current_item_count()) {
					PX4_DEBUG("WPM: MISSION_ACK OK all items sent, switch to state IDLE");

				} else if (wpa.type == MAV_MISSION_OPERATION_CANCELLED) {
					PX4_DEBUG("WPM: MISSION_ACK CANCELLED, switch to state IDLE");

				} else {
					PX4_DEBUG("WPM: MISSION_ACK ERROR: not all items sent, switch to state IDLE anyway");
				}

				switch_to_idle_state();

			} else if (_state == MAVLINK_WPM_STATE_GETLIST) {

				// INT or float mode is not supported
				if (wpa.type == MAV_MISSION_UNSUPPORTED) {

					if (_int_mode) {
						_int_mode = false;
						send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);

					} else {
						_int_mode = true;
						send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
					}

				} else if (wpa.type == MAV_MISSION_OPERATION_CANCELLED) {
					PX4_DEBUG("WPM: MISSION_ACK CANCELLED, switch to state IDLE");
					switch_to_idle_state();
					_transfer_in_progress = false;

				} else if (wpa.type != MAV_MISSION_ACCEPTED) {
					PX4_WARN("Mission ack result was %d", wpa.type);
				}
			}

		} else {
			_mavlink->send_statustext_critical("REJ. WP CMD: partner id mismatch\t");
			events::send(events::ID("mavlink_mission_partner_id_mismatch"), events::Log::Error,
				     "Rejecting waypoint command, component or system ID mismatch");

			PX4_DEBUG("WPM: MISSION_ACK ERR: ID mismatch");
		}
	}
}

void
MavlinkMissionManager::handle_mission_set_current(const mavlink_message_t *msg)
{
	mavlink_mission_set_current_t wpc;
	mavlink_msg_mission_set_current_decode(msg, &wpc);

	if (CHECK_SYSID_COMPID_MISSION(wpc)) {
		if (_state == MAVLINK_WPM_STATE_IDLE) {
			_time_last_recv = hrt_absolute_time();

			if (wpc.seq < _count[MAV_MISSION_TYPE_MISSION]) {
				update_active_mission(_mission_dataman_id, _count[MAV_MISSION_TYPE_MISSION], wpc.seq, _crc32[MAV_MISSION_TYPE_MISSION]);

			} else {
				PX4_ERR("WPM: MISSION_SET_CURRENT seq=%d ERROR: not in list", wpc.seq);

				_mavlink->send_statustext_critical("WPM: WP CURR CMD: Not in list\t");
				events::send(events::ID("mavlink_mission_seq_out_of_bounds"), events::Log::Error,
					     "New mission waypoint sequence out of bounds");
			}

		} else {
			PX4_DEBUG("WPM: MISSION_SET_CURRENT ERROR: busy");

			_mavlink->send_statustext_critical("WPM: IGN WP CURR CMD: Busy\t");
			events::send(events::ID("mavlink_mission_state_busy"), events::Log::Error,
				     "Mission manager currently busy, ignoring new waypoint index");
		}
	}
}


void
MavlinkMissionManager::handle_mission_request_list(const mavlink_message_t *msg)
{
	mavlink_mission_request_list_t wprl;
	mavlink_msg_mission_request_list_decode(msg, &wprl);

	if (CHECK_SYSID_COMPID_MISSION(wprl)) {
		const bool maybe_completed = (_transfer_seq == current_item_count());

		// If all mission items have been sent and a new mission request list comes in, we can proceed even if  MISSION_ACK was
		// never received. This could happen on a quick reconnect that doesn't trigger MAVLINK_MISSION_PROTOCOL_TIMEOUT_DEFAULT
		if (maybe_completed) {
			switch_to_idle_state();
		}

		if (_state == MAVLINK_WPM_STATE_IDLE || (_state == MAVLINK_WPM_STATE_SENDLIST
				&& (uint8_t)_mission_type == wprl.mission_type)) {
			_time_last_recv = hrt_absolute_time();

			_state = MAVLINK_WPM_STATE_SENDLIST;
			_mission_type = (MAV_MISSION_TYPE)wprl.mission_type;

			// make sure our item counts are up-to-date
			switch (_mission_type) {
			case MAV_MISSION_TYPE_FENCE:
				load_geofence_stats();
				break;

			case MAV_MISSION_TYPE_RALLY:
				load_safepoint_stats();
				break;

			default:
				break;
			}

			_transfer_seq = 0;
			_transfer_count = current_item_count();
			_transfer_partner_sysid = msg->sysid;
			_transfer_partner_compid = msg->compid;

			if (_transfer_count > 0) {
				PX4_DEBUG("WPM: MISSION_REQUEST_LIST OK, %u mission items to send, mission type=%i", _transfer_count, _mission_type);

			} else {
				PX4_DEBUG("WPM: MISSION_REQUEST_LIST OK nothing to send, mission is empty, mission type=%i", _mission_type);
			}

			send_mission_count(msg->sysid, msg->compid, _transfer_count, _mission_type, get_current_mission_type_crc());

		} else {
			PX4_DEBUG("WPM: MISSION_REQUEST_LIST ERROR: busy");

			_mavlink->send_statustext_info("Mission download request ignored, already active\t");
			events::send(events::ID("mavlink_mission_req_ignored"), events::Log::Warning,
				     "Mission download request ignored, already active");
		}
	}
}


void
MavlinkMissionManager::handle_mission_request(const mavlink_message_t *msg)
{
	// The request comes in the old float mode, so we switch to it.
	if (_int_mode) {
		_int_mode = false;
	}

	handle_mission_request_both(msg);
}

void
MavlinkMissionManager::handle_mission_request_int(const mavlink_message_t *msg)
{
	// The request comes in the new int mode, so we switch to it.
	if (!_int_mode) {
		_int_mode = true;
	}

	handle_mission_request_both(msg);
}

void
MavlinkMissionManager::handle_mission_request_both(const mavlink_message_t *msg)
{
	/* The mavlink_message_t could also be a mavlink_mission_request_int_t, however the structs
	 * are basically the same, so we can ignore it. */
	mavlink_mission_request_t wpr;
	mavlink_msg_mission_request_decode(msg, &wpr);

	if (CHECK_SYSID_COMPID_MISSION(wpr)) {
		if (msg->sysid == _transfer_partner_sysid && msg->compid == _transfer_partner_compid) {
			if (_state == MAVLINK_WPM_STATE_SENDLIST) {

				if (_mission_type != wpr.mission_type) {
					PX4_WARN("WPM: Unexpected mission type (%u %u)", wpr.mission_type, _mission_type);
					return;
				}

				_time_last_recv = hrt_absolute_time();

				/* _transfer_seq contains sequence of expected request */
				if (wpr.seq == _transfer_seq && _transfer_seq < _transfer_count) {
					PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) seq %u from ID %u", wpr.seq, msg->sysid);

					_transfer_seq++;

				} else if (wpr.seq == _transfer_seq - 1) {
					PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) seq %u from ID %u (again)", wpr.seq, msg->sysid);

				} else {
					if (_transfer_seq > 0 && _transfer_seq < _transfer_count) {
						PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: seq %u from ID %u unexpected, must be %i or %i", wpr.seq, msg->sysid,
							  _transfer_seq - 1, _transfer_seq);

					} else if (_transfer_seq <= 0) {
						PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: seq %u from ID %u unexpected, must be %i", wpr.seq, msg->sysid,
							  _transfer_seq);

					} else {
						PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: seq %u from ID %u unexpected, must be %i", wpr.seq, msg->sysid,
							  _transfer_seq - 1);
					}

					switch_to_idle_state();

					send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
					_mavlink->send_statustext_critical("WPM: REJ. CMD: Req. WP was unexpected\t");
					events::send(events::ID("mavlink_mission_wp_unexpected"), events::Log::Error,
						     "Unexpected waypoint index, aborting transfer");
					return;
				}

				/* double check bounds in case of items count changed */
				if (wpr.seq < current_item_count()) {
					send_mission_item(_transfer_partner_sysid, _transfer_partner_compid, wpr.seq);

				} else {
					PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: seq %u out of bound [%u, %u]", wpr.seq, wpr.seq,
						  current_item_count() - 1);

					switch_to_idle_state();

					send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
					_mavlink->send_statustext_critical("WPM: REJ. CMD: Req. WP was unexpected\t");
					events::send(events::ID("mavlink_mission_wp_unexpected2"), events::Log::Error,
						     "Unexpected waypoint index, aborting mission transfer");
				}

			} else if (_state == MAVLINK_WPM_STATE_IDLE) {
				PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: no transfer");

				// Silently ignore this as some OSDs have buggy mission protocol implementations
				//_mavlink->send_statustext_critical("IGN MISSION_ITEM_REQUEST(_INT): No active transfer");

			} else {
				PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: busy (state %d).", _state);

				_mavlink->send_statustext_critical("WPM: REJ. CMD: Busy\t");
				events::send(events::ID("mavlink_mission_mis_req_ignored_busy"), events::Log::Error,
					     "Ignoring mission request, currently busy");
			}

		} else {
			_mavlink->send_statustext_critical("WPM: REJ. CMD: partner id mismatch\t");
			events::send(events::ID("mavlink_mission_partner_id_mismatch2"), events::Log::Error,
				     "Rejecting mission request command, component or system ID mismatch");

			PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: rejected, partner ID mismatch");
		}
	}
}


void
MavlinkMissionManager::handle_mission_count(const mavlink_message_t *msg)
{
	mavlink_mission_count_t wpc;
	mavlink_msg_mission_count_decode(msg, &wpc);

	if (CHECK_SYSID_COMPID_MISSION(wpc)) {
		if (_state == MAVLINK_WPM_STATE_IDLE) {
			_time_last_recv = hrt_absolute_time();

			if (_transfer_in_progress) {
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
				return;
			}

			_transfer_in_progress = true;
			_mission_type = (MAV_MISSION_TYPE)wpc.mission_type;
			_transfer_current_crc32 = 0;

			if (wpc.count > current_max_item_count()) {
				PX4_DEBUG("WPM: MISSION_COUNT ERROR: too many waypoints (%d), supported: %d", wpc.count, current_max_item_count());

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_NO_SPACE);
				_transfer_in_progress = false;
				return;
			}

			if (wpc.count == 0) {
				PX4_DEBUG("WPM: MISSION_COUNT 0, clearing waypoints list and staying in state MAVLINK_WPM_STATE_IDLE");

				switch (_mission_type) {
				case MAV_MISSION_TYPE_MISSION:

					_land_start_marker = -1;
					_land_marker = -1;

					/* alternate dataman ID anyway to let navigator know about changes */

					if (_mission_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0) {
						update_active_mission(DM_KEY_WAYPOINTS_OFFBOARD_1, 0, 0, 0);

					} else {
						update_active_mission(DM_KEY_WAYPOINTS_OFFBOARD_0, 0, 0, 0);
					}

					break;

				case MAV_MISSION_TYPE_FENCE:
					update_geofence_count(_fence_dataman_id == DM_KEY_FENCE_POINTS_0 ? DM_KEY_FENCE_POINTS_1 : DM_KEY_FENCE_POINTS_0, 0, 0);
					break;

				case MAV_MISSION_TYPE_RALLY:
					update_safepoint_count(_safepoint_dataman_id == DM_KEY_SAFE_POINTS_0 ? DM_KEY_SAFE_POINTS_1 : DM_KEY_SAFE_POINTS_0, 0,
							       0);
					break;

				default:
					PX4_ERR("mission type %u not handled", _mission_type);
					break;
				}

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED);
				_transfer_in_progress = false;
				return;
			}

			PX4_DEBUG("WPM: MISSION_COUNT %u from ID %u, changing state to MAVLINK_WPM_STATE_GETLIST", wpc.count, msg->sysid);

			switch (_mission_type) {
			case MAV_MISSION_TYPE_MISSION:
				_transfer_dataman_id = (_mission_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 ? DM_KEY_WAYPOINTS_OFFBOARD_1 :
							DM_KEY_WAYPOINTS_OFFBOARD_0);	// use inactive storage for transmission
				break;

			case MAV_MISSION_TYPE_FENCE:
				_transfer_dataman_id = (_fence_dataman_id == DM_KEY_FENCE_POINTS_0 ? DM_KEY_FENCE_POINTS_1 :
							DM_KEY_FENCE_POINTS_0);	// use inactive storage for transmission
				break;

			case MAV_MISSION_TYPE_RALLY:
				_transfer_dataman_id = (_safepoint_dataman_id == DM_KEY_SAFE_POINTS_0 ? DM_KEY_SAFE_POINTS_1 :
							DM_KEY_SAFE_POINTS_0);	// use inactive storage for transmission
				break;

			default:
				PX4_ERR("mission type %u not handled", _mission_type);
				_transfer_in_progress = false;
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_INVALID);
				return;
			}

			_state = MAVLINK_WPM_STATE_GETLIST;
			_transfer_seq = 0;
			_transfer_partner_sysid = msg->sysid;
			_transfer_partner_compid = msg->compid;
			_transfer_count = wpc.count;
			_transfer_current_seq = -1;
			_transfer_land_start_marker = -1;
			_transfer_land_marker = -1;

		} else if (_state == MAVLINK_WPM_STATE_GETLIST) {
			_time_last_recv = hrt_absolute_time();

			if (_transfer_seq == 0) {
				/* looks like our MISSION_REQUEST was lost, try again */
				PX4_DEBUG("WPM: MISSION_COUNT %u from ID %u (again)", wpc.count, msg->sysid);

			} else {
				PX4_DEBUG("WPM: MISSION_COUNT ERROR: busy, already receiving seq %u", _transfer_seq);

				_mavlink->send_statustext_critical("WPM: REJ. CMD: Busy\t");
				events::send(events::ID("mavlink_mission_getlist_busy"), events::Log::Error,
					     "Mission upload busy, already receiving waypoint");

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
				return;
			}

		} else {
			PX4_DEBUG("WPM: MISSION_COUNT ERROR: busy, state %i", _state);

			_mavlink->send_statustext_critical("WPM: IGN MISSION_COUNT: Busy\t");
			events::send(events::ID("mavlink_mission_ignore_mis_count"), events::Log::Error,
				     "Mission upload busy, ignoring MISSION_COUNT");
			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			return;
		}

		send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
	}
}

void
MavlinkMissionManager::switch_to_idle_state()
{
	_state = MAVLINK_WPM_STATE_IDLE;
}


void
MavlinkMissionManager::handle_mission_item(const mavlink_message_t *msg)
{
	if (_int_mode) {
		// It seems that we should be using the float mode, let's switch out of int mode.
		_int_mode = false;
	}

	handle_mission_item_both(msg);
}

void
MavlinkMissionManager::handle_mission_item_int(const mavlink_message_t *msg)
{
	if (!_int_mode) {
		// It seems that we should be using the int mode, let's switch to it.
		_int_mode = true;
	}

	handle_mission_item_both(msg);
}

void
MavlinkMissionManager::handle_mission_item_both(const mavlink_message_t *msg)
{

	// The mavlink_message could also contain a mavlink_mission_item_int_t. We ignore that here
	// and take care of it later in parse_mavlink_mission_item depending on _int_mode.

	mavlink_mission_item_t wp;
	mavlink_msg_mission_item_decode(msg, &wp);

	if (CHECK_SYSID_COMPID_MISSION(wp)) {

		if (wp.mission_type != _mission_type) {
			PX4_WARN("WPM: Unexpected mission type (%d %d)", (int)wp.mission_type, (int)_mission_type);
			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			return;
		}

		if (_state == MAVLINK_WPM_STATE_GETLIST) {
			_time_last_recv = hrt_absolute_time();

			if (wp.seq != _transfer_seq) {
				PX4_DEBUG("WPM: MISSION_ITEM ERROR: seq %u was not the expected %u", wp.seq, _transfer_seq);

				/* Item sequence not expected, ignore item */
				return;
			}

		} else if (_state == MAVLINK_WPM_STATE_IDLE) {
			if (_transfer_seq == wp.seq + 1) {
				// Assume this is a duplicate, where we already successfully got all mission items,
				// but the GCS did not receive the last ack and sent the same item again
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED, _transfer_current_crc32);

			} else {
				PX4_DEBUG("WPM: MISSION_ITEM ERROR: no transfer");

				_mavlink->send_statustext_critical("IGN MISSION_ITEM: No transfer\t");
				events::send(events::ID("mavlink_mission_no_transfer"), events::Log::Error,
					     "Ignoring mission item, no transfer in progress");
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			}

			return;

		} else {
			PX4_DEBUG("WPM: MISSION_ITEM ERROR: busy, state %i", _state);

			_mavlink->send_statustext_critical("IGN MISSION_ITEM: Busy\t");
			events::send(events::ID("mavlink_mission_mis_item_busy"), events::Log::Error,
				     "Ignoring mission item, busy");
			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			return;
		}

		struct mission_item_s mission_item = {};

		int ret = parse_mavlink_mission_item(&wp, &mission_item);

		if (ret != PX4_OK) {
			PX4_DEBUG("WPM: MISSION_ITEM ERROR: seq %u invalid item", wp.seq);

			_mavlink->send_statustext_critical("IGN MISSION_ITEM: Invalid item\t");
			events::send(events::ID("mavlink_mission_mis_item_invalid"), events::Log::Error,
				     "Ignoring mission item, invalid item");

			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, ret);
			switch_to_idle_state();
			_transfer_in_progress = false;
			return;
		}

		_transfer_current_crc32 = crc32_for_mission_item(wp, _transfer_current_crc32);

		bool write_failed = false;
		bool check_failed = false;

		switch (_mission_type) {

		case MAV_MISSION_TYPE_MISSION: {
				// check that we don't get a wrong item (hardening against wrong client implementations, the list here
				// does not need to be complete)
				if (mission_item.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION ||
				    mission_item.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION ||
				    mission_item.nav_cmd == MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION ||
				    mission_item.nav_cmd == MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION ||
				    mission_item.nav_cmd == MAV_CMD_NAV_RALLY_POINT) {
					check_failed = true;

				} else {

					write_failed = !_dataman_client.writeSync(_transfer_dataman_id, wp.seq,
							reinterpret_cast<uint8_t *>(&mission_item),
							sizeof(struct mission_item_s));

					// Check for land start marker
					if ((mission_item.nav_cmd == MAV_CMD_DO_LAND_START) && (_transfer_land_start_marker == -1)) {
						_transfer_land_start_marker = wp.seq;
					}

					// Check for land index
					if (((mission_item.nav_cmd == MAV_CMD_NAV_VTOL_LAND) || (mission_item.nav_cmd == MAV_CMD_NAV_LAND))
					    && (_transfer_land_marker == -1)) {
						_transfer_land_marker = wp.seq;

						if (_transfer_land_start_marker == -1) {
							_transfer_land_start_marker = _transfer_land_marker;
						}
					}

					if (!write_failed) {
						/* waypoint marked as current */
						if (wp.current) {
							_transfer_current_seq = wp.seq;
						}
					}
				}
			}
			break;

		case MAV_MISSION_TYPE_FENCE: { // Write a geofence point
				mission_fence_point_s mission_fence_point;
				mission_fence_point.nav_cmd = mission_item.nav_cmd;
				mission_fence_point.lat = mission_item.lat;
				mission_fence_point.lon = mission_item.lon;
				mission_fence_point.alt = mission_item.altitude;

				if (mission_item.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION ||
				    mission_item.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION) {
					mission_fence_point.vertex_count = mission_item.vertex_count;

					if (mission_item.vertex_count < 3) { // feasibility check
						PX4_ERR("Fence: too few vertices");
						check_failed = true;
					}

				} else {
					mission_fence_point.circle_radius = mission_item.circle_radius;
				}

				mission_fence_point.frame = mission_item.frame;

				if (!check_failed) {
					write_failed = !_dataman_client.writeSync(_transfer_dataman_id, wp.seq,
							reinterpret_cast<uint8_t *>(&mission_fence_point), sizeof(mission_fence_point_s));
				}

			}
			break;

		case MAV_MISSION_TYPE_RALLY: { // Write a safe point / rally point
				write_failed = !_dataman_client.writeSync(_transfer_dataman_id, wp.seq,
						reinterpret_cast<uint8_t *>(&mission_item), sizeof(mission_item_s), 2_s);
			}
			break;

		default:
			_mavlink->send_statustext_critical("Received unknown mission type, abort.\t");
			events::send(events::ID("mavlink_mission_unknown_mis_type"), events::Log::Error,
				     "Received unknown mission type, abort");
			break;
		}

		if (write_failed || check_failed) {
			PX4_DEBUG("WPM: MISSION_ITEM ERROR: error writing seq %u to dataman ID %i", wp.seq, _transfer_dataman_id);

			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);

			if (write_failed) {
				_mavlink->send_statustext_critical("Unable to write on micro SD\t");
				events::send(events::ID("mavlink_mission_storage_failure"), events::Log::Error,
					     "Mission: unable to write to storage");
			}

			switch_to_idle_state();
			_transfer_in_progress = false;
			return;
		}

		/* waypoint marked as current */
		if (wp.current) {
			_transfer_current_seq = wp.seq;
		}

		PX4_DEBUG("WPM: MISSION_ITEM seq %u received", wp.seq);

		_transfer_seq = wp.seq + 1;

		if (_transfer_seq == _transfer_count) {
			/* got all new mission items successfully */
			PX4_DEBUG("WPM: MISSION_ITEM got all %u items, current_seq=%ld, changing state to MAVLINK_WPM_STATE_IDLE",
				  _transfer_count, _transfer_current_seq);

			ret = 0;

			switch (_mission_type) {
			case MAV_MISSION_TYPE_MISSION:
				_land_start_marker = _transfer_land_start_marker;
				_land_marker = _transfer_land_marker;

				// Only need to update if the mission actually changed
				if (_transfer_current_crc32 != _crc32[MAV_MISSION_TYPE_MISSION]) {
					update_active_mission(_transfer_dataman_id, _transfer_count, _transfer_current_seq, _transfer_current_crc32);
				}

				break;

			case MAV_MISSION_TYPE_FENCE:

				// Only need to update if the mission actually changed
				if (_transfer_current_crc32 != _crc32[MAV_MISSION_TYPE_FENCE]) {
					ret = update_geofence_count(_transfer_dataman_id, _transfer_count, _transfer_current_crc32);
				}

				break;

			case MAV_MISSION_TYPE_RALLY:

				// Only need to update if the mission actually changed
				if (_transfer_current_crc32 != _crc32[MAV_MISSION_TYPE_RALLY]) {
					ret = update_safepoint_count(_transfer_dataman_id, _transfer_count, _transfer_current_crc32);
				}

				break;

			default:
				PX4_ERR("mission type %u not handled", _mission_type);
				break;
			}

			// Note: the switch to idle needs to happen after update_geofence_count is called, for proper unlocking order
			switch_to_idle_state();


			if (ret == PX4_OK) {
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED, _transfer_current_crc32);

			} else {
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			}

			_transfer_in_progress = false;

		} else {
			/* request next item */
			send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
		}
	}
}


void
MavlinkMissionManager::handle_mission_clear_all(const mavlink_message_t *msg)
{
	mavlink_mission_clear_all_t wpca;
	mavlink_msg_mission_clear_all_decode(msg, &wpca);

	if (CHECK_SYSID_COMPID_MISSION(wpca)) {

		if (_state == MAVLINK_WPM_STATE_IDLE) {
			/* don't touch mission items storage itself, but only items count in mission state */
			_time_last_recv = hrt_absolute_time();

			_mission_type = (MAV_MISSION_TYPE)wpca.mission_type; // this is needed for the returned ack
			int ret = 0;

			switch (wpca.mission_type) {
			case MAV_MISSION_TYPE_MISSION:
				_land_start_marker = -1;
				_land_marker = -1;
				update_active_mission(_mission_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 ? DM_KEY_WAYPOINTS_OFFBOARD_1 :
						      DM_KEY_WAYPOINTS_OFFBOARD_0, 0, 0, 0);
				break;

			case MAV_MISSION_TYPE_FENCE:
				ret = update_geofence_count(_fence_dataman_id == DM_KEY_FENCE_POINTS_0 ? DM_KEY_FENCE_POINTS_1 : DM_KEY_FENCE_POINTS_0,
							    0, 0);
				break;

			case MAV_MISSION_TYPE_RALLY:
				ret = update_safepoint_count(_safepoint_dataman_id == DM_KEY_SAFE_POINTS_0 ? DM_KEY_SAFE_POINTS_1 :
							     DM_KEY_SAFE_POINTS_0, 0, 0);
				break;

			case MAV_MISSION_TYPE_ALL:
				_land_start_marker = -1;
				_land_marker = -1;
				update_active_mission(_mission_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 ? DM_KEY_WAYPOINTS_OFFBOARD_1 :
						      DM_KEY_WAYPOINTS_OFFBOARD_0, 0, 0, 0);
				ret = update_geofence_count(_fence_dataman_id == DM_KEY_FENCE_POINTS_0 ? DM_KEY_FENCE_POINTS_1 : DM_KEY_FENCE_POINTS_0,
							    0, 0);
				ret = update_safepoint_count(_safepoint_dataman_id == DM_KEY_SAFE_POINTS_0 ? DM_KEY_SAFE_POINTS_1 :
							     DM_KEY_SAFE_POINTS_0, 0, 0) || ret;
				break;

			default:
				PX4_ERR("mission type %u not handled", _mission_type);
				break;
			}

			if (ret == PX4_OK) {
				PX4_DEBUG("WPM: CLEAR_ALL OK (mission_type=%i)", _mission_type);

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED);

			} else {
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			}

		} else {
			_mavlink->send_statustext_critical("WPM: IGN CLEAR CMD: Busy\t");
			events::send(events::ID("mavlink_mission_ignore_clear"), events::Log::Error,
				     "Ignoring mission clear command, busy");

			PX4_DEBUG("WPM: CLEAR_ALL IGNORED: busy");
		}
	}
}

int
MavlinkMissionManager::parse_mavlink_mission_item(const mavlink_mission_item_t *mavlink_mission_item,
		struct mission_item_s *mission_item)
{
	if (mavlink_mission_item->frame == MAV_FRAME_GLOBAL ||
	    mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT ||
	    (_int_mode && (mavlink_mission_item->frame == MAV_FRAME_GLOBAL_INT ||
			   mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT))) {
		// This is a mission item with a global coordinate

		// Switch to int mode if that is what we are receiving
		if ((mavlink_mission_item->frame == MAV_FRAME_GLOBAL_INT ||
		     mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)) {
			_int_mode = true;
		}

		if (_int_mode) {
			/* The argument is actually a mavlink_mission_item_int_t in int_mode.
			 * mavlink_mission_item_t and mavlink_mission_item_int_t have the same
			 * alignment, so we can just swap float for int32_t. */
			const mavlink_mission_item_int_t *item_int
				= reinterpret_cast<const mavlink_mission_item_int_t *>(mavlink_mission_item);
			mission_item->lat = ((double)item_int->x) * 1e-7;
			mission_item->lon = ((double)item_int->y) * 1e-7;

		} else {
			mission_item->lat = (double)mavlink_mission_item->x;
			mission_item->lon = (double)mavlink_mission_item->y;
		}

		mission_item->altitude = mavlink_mission_item->z;

		if (mavlink_mission_item->frame == MAV_FRAME_GLOBAL ||
		    mavlink_mission_item->frame == MAV_FRAME_GLOBAL_INT) {
			mission_item->altitude_is_relative = false;

		} else if (mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT ||
			   mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
			mission_item->altitude_is_relative = true;
		}

		// Depending on the received MAV_CMD_* (MAVLink Commands), assign the corresponding
		// NAV_CMD value to the mission item's nav_cmd.
		switch (mavlink_mission_item->command) {
		case MAV_CMD_NAV_WAYPOINT:
			mission_item->nav_cmd = NAV_CMD_WAYPOINT;
			mission_item->time_inside = mavlink_mission_item->param1;
			mission_item->acceptance_radius = mavlink_mission_item->param2;
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_NAV_LOITER_UNLIM:
			mission_item->nav_cmd = NAV_CMD_LOITER_UNLIMITED;
			mission_item->loiter_radius = mavlink_mission_item->param3;
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_NAV_LOITER_TIME:
			mission_item->nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
			mission_item->time_inside = mavlink_mission_item->param1;
			mission_item->force_heading = (mavlink_mission_item->param2 > 0);
			mission_item->loiter_radius = mavlink_mission_item->param3;
			mission_item->loiter_exit_xtrack = (mavlink_mission_item->param4 > 0);
			// Yaw is only valid for multicopter but we set it always because
			// it's just ignored for fixedwing.
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_NAV_LAND:
			mission_item->nav_cmd = NAV_CMD_LAND;
			// TODO: abort alt param1
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			mission_item->land_precision = mavlink_mission_item->param2;
			break;

		case MAV_CMD_NAV_TAKEOFF:
			mission_item->nav_cmd = NAV_CMD_TAKEOFF;
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));

			break;

		case MAV_CMD_NAV_LOITER_TO_ALT:
			mission_item->nav_cmd = NAV_CMD_LOITER_TO_ALT;
			mission_item->force_heading = (mavlink_mission_item->param1 > 0);
			mission_item->loiter_radius = mavlink_mission_item->param2;
			mission_item->loiter_exit_xtrack = (mavlink_mission_item->param4 > 0);
			break;

		case MAV_CMD_NAV_ROI:
		case MAV_CMD_DO_SET_ROI:
			if ((int)mavlink_mission_item->param1 == MAV_ROI_LOCATION) {
				mission_item->nav_cmd = NAV_CMD_DO_SET_ROI;
				mission_item->params[0] = MAV_ROI_LOCATION;

				mission_item->params[6] = mavlink_mission_item->z;

			} else if ((int)mavlink_mission_item->param1 == MAV_ROI_NONE) {
				mission_item->nav_cmd = NAV_CMD_DO_SET_ROI;
				mission_item->params[0] = MAV_ROI_NONE;

			} else {
				return MAV_MISSION_INVALID_PARAM1;
			}

			break;

		case MAV_CMD_DO_SET_ROI_LOCATION:
			mission_item->nav_cmd = NAV_CMD_DO_SET_ROI_LOCATION;
			mission_item->params[6] = mavlink_mission_item->z;
			break;

		case MAV_CMD_NAV_VTOL_TAKEOFF:
		case MAV_CMD_NAV_VTOL_LAND:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_CONDITION_GATE:
			mission_item->nav_cmd = NAV_CMD_CONDITION_GATE;
			break;

		case MAV_CMD_NAV_FENCE_RETURN_POINT:
			mission_item->nav_cmd = NAV_CMD_FENCE_RETURN_POINT;
			break;

		case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
		case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			mission_item->vertex_count = (uint16_t)(mavlink_mission_item->param1 + 0.5f);
			break;

		case MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
		case MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			mission_item->circle_radius = mavlink_mission_item->param1;
			break;

		case MAV_CMD_NAV_RALLY_POINT:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			break;

		default:
			mission_item->nav_cmd = NAV_CMD_INVALID;
			PX4_DEBUG("Unsupported command %d", mavlink_mission_item->command);

			return MAV_MISSION_UNSUPPORTED;
		}

		mission_item->frame = mavlink_mission_item->frame;

	} else if (mavlink_mission_item->frame == MAV_FRAME_MISSION) {

		// This is a mission item with no coordinates

		mission_item->params[0] = mavlink_mission_item->param1;
		mission_item->params[1] = mavlink_mission_item->param2;
		mission_item->params[2] = mavlink_mission_item->param3;
		mission_item->params[3] = mavlink_mission_item->param4;

		if (_int_mode) {
			/* The argument is actually a mavlink_mission_item_int_t in int_mode.
			 * mavlink_mission_item_t and mavlink_mission_item_int_t have the same
			 * alignment, so we can just swap float for int32_t. */
			const mavlink_mission_item_int_t *item_int
				= reinterpret_cast<const mavlink_mission_item_int_t *>(mavlink_mission_item);
			mission_item->params[4] = ((double)item_int->x);
			mission_item->params[5] = ((double)item_int->y);

		} else {
			mission_item->params[4] = (double)mavlink_mission_item->x;
			mission_item->params[5] = (double)mavlink_mission_item->y;
		}

		mission_item->params[6] = mavlink_mission_item->z;

		switch (mavlink_mission_item->command) {
		case MAV_CMD_DO_JUMP:
			mission_item->nav_cmd = NAV_CMD_DO_JUMP;
			mission_item->do_jump_mission_index = mavlink_mission_item->param1;
			mission_item->do_jump_current_count = 0;
			mission_item->do_jump_repeat_count = mavlink_mission_item->param2;
			break;

		case MAV_CMD_NAV_ROI:
		case MAV_CMD_DO_SET_ROI: {
				const int roi_mode = mavlink_mission_item->param1;

				if (roi_mode == MAV_ROI_NONE || roi_mode == MAV_ROI_WPNEXT || roi_mode == MAV_ROI_WPINDEX) {
					mission_item->nav_cmd = NAV_CMD_DO_SET_ROI;

				} else {
					return MAV_MISSION_INVALID_PARAM1;
				}
			}
			break;

		case MAV_CMD_DO_WINCH:
		case MAV_CMD_DO_GRIPPER:
			mission_item->nav_cmd = mavlink_mission_item->command;
			MavlinkMissionManager::copy_params_from_mavlink_to_mission_item(mission_item, mavlink_mission_item, 1, 2);
			break;

		case MAV_CMD_DO_CHANGE_SPEED:
		case MAV_CMD_DO_SET_HOME:
		case MAV_CMD_DO_LAND_START:
		case MAV_CMD_DO_TRIGGER_CONTROL:
		case MAV_CMD_DO_DIGICAM_CONTROL:
		case MAV_CMD_DO_MOUNT_CONFIGURE:
		case MAV_CMD_DO_MOUNT_CONTROL:
		case NAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
		case NAV_CMD_SET_CAMERA_FOCUS:
		case NAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE:
		case MAV_CMD_IMAGE_START_CAPTURE:
		case MAV_CMD_IMAGE_STOP_CAPTURE:
		case MAV_CMD_VIDEO_START_CAPTURE:
		case MAV_CMD_VIDEO_STOP_CAPTURE:
		case MAV_CMD_DO_CONTROL_VIDEO:
		case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
		case MAV_CMD_OBLIQUE_SURVEY:
		case MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
		case MAV_CMD_SET_CAMERA_MODE:
		case MAV_CMD_DO_VTOL_TRANSITION:
		case MAV_CMD_NAV_DELAY:
		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
		case MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET:
		case MAV_CMD_DO_SET_ROI_NONE:
		case MAV_CMD_CONDITION_DELAY:
		case MAV_CMD_CONDITION_DISTANCE:
		case MAV_CMD_DO_SET_ACTUATOR:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			break;

		default:
			mission_item->nav_cmd = NAV_CMD_INVALID;

			PX4_DEBUG("Unsupported command %d", mavlink_mission_item->command);

			return MAV_MISSION_UNSUPPORTED;
		}

		mission_item->frame = MAV_FRAME_MISSION;

	} else {
		PX4_DEBUG("Unsupported frame %d", mavlink_mission_item->frame);

		return MAV_MISSION_UNSUPPORTED_FRAME;
	}

	mission_item->autocontinue = mavlink_mission_item->autocontinue;
	// mission_item->index = mavlink_mission_item->seq;

	mission_item->origin = ORIGIN_MAVLINK;

	return MAV_MISSION_ACCEPTED;
}


int
MavlinkMissionManager::format_mavlink_mission_item(const struct mission_item_s *mission_item,
		mavlink_mission_item_t *mavlink_mission_item)
{
	mavlink_mission_item->frame = mission_item->frame;
	mavlink_mission_item->command = mission_item->nav_cmd;
	mavlink_mission_item->autocontinue = mission_item->autocontinue;
	mavlink_mission_item->mission_type = _mission_type;

	/* default mappings for generic commands */
	if (mission_item->frame == MAV_FRAME_MISSION) {
		mavlink_mission_item->param1 = mission_item->params[0];
		mavlink_mission_item->param2 = mission_item->params[1];
		mavlink_mission_item->param3 = mission_item->params[2];
		mavlink_mission_item->param4 = mission_item->params[3];

		mavlink_mission_item->x = mission_item->params[4];
		mavlink_mission_item->y = mission_item->params[5];

		if (_int_mode) {
			// This function actually receives a mavlink_mission_item_int_t in _int_mode
			// which has the same alignment as mavlink_mission_item_t and the only
			// difference is int32_t vs. float for x and y.
			mavlink_mission_item_int_t *item_int =
				reinterpret_cast<mavlink_mission_item_int_t *>(mavlink_mission_item);

			item_int->x = round(mission_item->params[4]);
			item_int->y = round(mission_item->params[5]);

		} else {
			mavlink_mission_item->x = (float)mission_item->params[4];
			mavlink_mission_item->y = (float)mission_item->params[5];
		}

		mavlink_mission_item->z = mission_item->params[6];

		switch (mavlink_mission_item->command) {
		case NAV_CMD_DO_JUMP:
			mavlink_mission_item->param1 = mission_item->do_jump_mission_index;
			mavlink_mission_item->param2 = mission_item->do_jump_repeat_count;
			break;

		case NAV_CMD_DO_CHANGE_SPEED:
		case NAV_CMD_DO_SET_HOME:
		case NAV_CMD_DO_LAND_START:
		case NAV_CMD_DO_TRIGGER_CONTROL:
		case NAV_CMD_DO_DIGICAM_CONTROL:
		case NAV_CMD_IMAGE_START_CAPTURE:
		case NAV_CMD_IMAGE_STOP_CAPTURE:
		case NAV_CMD_VIDEO_START_CAPTURE:
		case NAV_CMD_VIDEO_STOP_CAPTURE:
		case NAV_CMD_DO_CONTROL_VIDEO:
		case NAV_CMD_DO_MOUNT_CONFIGURE:
		case NAV_CMD_DO_MOUNT_CONTROL:
		case NAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
		case NAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE:
		case NAV_CMD_DO_SET_ROI:
		case NAV_CMD_DO_SET_CAM_TRIGG_DIST:
		case NAV_CMD_OBLIQUE_SURVEY:
		case NAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
		case NAV_CMD_SET_CAMERA_MODE:
		case NAV_CMD_SET_CAMERA_ZOOM:
		case NAV_CMD_SET_CAMERA_FOCUS:
		case NAV_CMD_DO_VTOL_TRANSITION:
			break;

		default:
			return PX4_ERROR;
		}

	} else {
		mavlink_mission_item->param1 = 0.0f;
		mavlink_mission_item->param2 = 0.0f;
		mavlink_mission_item->param3 = 0.0f;
		mavlink_mission_item->param4 = 0.0f;

		if (_int_mode) {
			// This function actually receives a mavlink_mission_item_int_t in _int_mode
			// which has the same alignment as mavlink_mission_item_t and the only
			// difference is int32_t vs. float for x and y.
			mavlink_mission_item_int_t *item_int =
				reinterpret_cast<mavlink_mission_item_int_t *>(mavlink_mission_item);

			item_int->x = round(mission_item->lat * 1e7);
			item_int->y = round(mission_item->lon * 1e7);

		} else {
			mavlink_mission_item->x = (float)mission_item->lat;
			mavlink_mission_item->y = (float)mission_item->lon;
		}

		mavlink_mission_item->z = mission_item->altitude;

		if (mission_item->altitude_is_relative) {
			if (_int_mode) {
				mavlink_mission_item->frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;

			} else {
				mavlink_mission_item->frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
			}

		} else {
			if (_int_mode) {
				mavlink_mission_item->frame = MAV_FRAME_GLOBAL_INT;

			} else {
				mavlink_mission_item->frame = MAV_FRAME_GLOBAL;
			}
		}

		switch (mission_item->nav_cmd) {
		case NAV_CMD_WAYPOINT:
			mavlink_mission_item->param1 = mission_item->time_inside;
			mavlink_mission_item->param2 = mission_item->acceptance_radius;
			mavlink_mission_item->param4 = math::degrees(mission_item->yaw);
			break;

		case NAV_CMD_LOITER_UNLIMITED:
			mavlink_mission_item->param3 = mission_item->loiter_radius;
			mavlink_mission_item->param4 = math::degrees(mission_item->yaw);
			break;

		case NAV_CMD_LOITER_TIME_LIMIT:
			mavlink_mission_item->param1 = mission_item->time_inside;
			mavlink_mission_item->param2 = mission_item->force_heading;
			mavlink_mission_item->param3 = mission_item->loiter_radius;
			mavlink_mission_item->param4 = mission_item->loiter_exit_xtrack;
			break;

		case NAV_CMD_LAND:
			// TODO: param1 abort alt
			mavlink_mission_item->param2 = mission_item->land_precision;
			mavlink_mission_item->param4 = math::degrees(mission_item->yaw);
			break;

		case NAV_CMD_TAKEOFF:
			mavlink_mission_item->param4 = math::degrees(mission_item->yaw);
			break;

		case NAV_CMD_LOITER_TO_ALT:
			mavlink_mission_item->param1 = mission_item->force_heading;
			mavlink_mission_item->param2 = mission_item->loiter_radius;
			mavlink_mission_item->param4 = mission_item->loiter_exit_xtrack;
			break;

		case MAV_CMD_NAV_VTOL_TAKEOFF:
		case MAV_CMD_NAV_VTOL_LAND:
			mavlink_mission_item->param4 = math::degrees(mission_item->yaw);
			break;

		case MAV_CMD_NAV_FENCE_RETURN_POINT:
			break;

		case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
		case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
			mavlink_mission_item->param1 = (float)mission_item->vertex_count;
			break;

		case MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
		case MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
			mavlink_mission_item->param1 = mission_item->circle_radius;
			break;

		case MAV_CMD_NAV_RALLY_POINT:
			break;


		default:
			return PX4_ERROR;
		}
	}

	return PX4_OK;
}

void MavlinkMissionManager::copy_params_from_mavlink_to_mission_item(struct mission_item_s *mission_item,
		const mavlink_mission_item_t *mavlink_mission_item, int8_t start_idx, int8_t end_idx)
{
	// Copy each param1 ~ 7 if they are within the range specified
	if (start_idx <= 1 && 1 <= end_idx) {
		mission_item->params[0] = mavlink_mission_item->param1;
	}

	if (start_idx <= 2 && 2 <= end_idx) {
		mission_item->params[1] = mavlink_mission_item->param2;
	}

	if (start_idx <= 3 && 3 <= end_idx) {
		mission_item->params[2] = mavlink_mission_item->param3;
	}

	if (start_idx <= 4 && 4 <= end_idx) {
		mission_item->params[3] = mavlink_mission_item->param4;
	}

	/* Param5, 6 and 7 are named x, y and z since it is used as position coordinates as well */
	if (start_idx <= 5 && 5 <= end_idx) {
		mission_item->params[4] = mavlink_mission_item->x;
	}

	if (start_idx <= 6 && 6 <= end_idx) {
		mission_item->params[5] = mavlink_mission_item->y;
	}

	if (start_idx <= 7 && 7 <= end_idx) {
		mission_item->params[6] = mavlink_mission_item->z;
	}
}

void MavlinkMissionManager::check_active_mission()
{
	// do not send anything over high latency communication
	if (_mavlink->get_mode() == Mavlink::MAVLINK_MODE_IRIDIUM) {
		return;
	}

	if (_mission_sub.updated()) {
		_mission_sub.update();

		if ((_mission_sub.get().geofence_id != _crc32[MAV_MISSION_TYPE_FENCE])
		    || (_my_fence_dataman_id != (dm_item_t) _mission_sub.get().fence_dataman_id)) {
			load_geofence_stats();
		}

		if ((_mission_sub.get().safe_points_id != _crc32[MAV_MISSION_TYPE_RALLY])
		    || (_my_safepoint_dataman_id != (dm_item_t) _mission_sub.get().safepoint_dataman_id)) {
			load_safepoint_stats();
		}

		if ((_mission_sub.get().mission_id != _crc32[MAV_MISSION_TYPE_MISSION])
		    || (_my_mission_dataman_id != (dm_item_t)_mission_sub.get().mission_dataman_id)) {
			PX4_DEBUG("WPM: New mission detected (possibly over different Mavlink instance) Updating");
			init_offboard_mission(_mission_sub.get());
			send_mission_count(_transfer_partner_sysid, _transfer_partner_compid, _count[MAV_MISSION_TYPE_MISSION],
					   MAV_MISSION_TYPE_MISSION, _crc32[MAV_MISSION_TYPE_MISSION]);
		}
	}
}

uint32_t MavlinkMissionManager::crc32_for_mission_item(const mavlink_mission_item_t &mission_item, uint32_t prev_crc32)
{
	union {
		CrcMissionItem_t item;
		uint8_t raw[sizeof(CrcMissionItem_t)];
	} u;

	u.item.frame = mission_item.frame;
	u.item.command = mission_item.command;
	u.item.autocontinue = mission_item.autocontinue;
	u.item.params[0] = mission_item.param1;
	u.item.params[1] = mission_item.param2;
	u.item.params[2] = mission_item.param3;
	u.item.params[3] = mission_item.param4;
	u.item.params[4] = mission_item.x;
	u.item.params[5] = mission_item.y;
	u.item.params[6] = mission_item.z;

	return crc32part(u.raw, sizeof(u), prev_crc32);
}
