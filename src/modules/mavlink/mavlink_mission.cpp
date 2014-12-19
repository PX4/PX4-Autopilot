/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "mavlink_mission.h"
#include "mavlink_main.h"

#include <math.h>
#include <lib/geo/geo.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include <dataman/dataman.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#define CHECK_SYSID_COMPID_MISSION(_msg)		(_msg.target_system == mavlink_system.sysid && \
						((_msg.target_component == mavlink_system.compid) || \
						(_msg.target_component == MAV_COMP_ID_MISSIONPLANNER) || \
						(_msg.target_component == MAV_COMP_ID_ALL)))

MavlinkMissionManager::MavlinkMissionManager(Mavlink *mavlink) : MavlinkStream(mavlink),
	_state(MAVLINK_WPM_STATE_IDLE),
	_time_last_recv(0),
	_time_last_sent(0),
	_action_timeout(MAVLINK_MISSION_PROTOCOL_TIMEOUT_DEFAULT),
	_retry_timeout(MAVLINK_MISSION_RETRY_TIMEOUT_DEFAULT),
	_max_count(DM_KEY_WAYPOINTS_OFFBOARD_0_MAX),
	_dataman_id(0),
	_count(0),
	_current_seq(0),
	_transfer_dataman_id(0),
	_transfer_count(0),
	_transfer_seq(0),
	_transfer_current_seq(0),
	_transfer_partner_sysid(0),
	_transfer_partner_compid(0),
	_offboard_mission_sub(-1),
	_mission_result_sub(-1),
	_offboard_mission_pub(-1),
	_slow_rate_limiter(_interval / 10.0f),
	_verbose(false)
{
	_offboard_mission_sub = orb_subscribe(ORB_ID(offboard_mission));
	_mission_result_sub = orb_subscribe(ORB_ID(mission_result));

	init_offboard_mission();
}

MavlinkMissionManager::~MavlinkMissionManager()
{
	close(_offboard_mission_pub);
	close(_mission_result_sub);
}

unsigned
MavlinkMissionManager::get_size()
{
	if (_state == MAVLINK_WPM_STATE_SENDLIST) {
		return MAVLINK_MSG_ID_MISSION_ITEM_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;

	} else if (_state == MAVLINK_WPM_STATE_GETLIST) {
		return MAVLINK_MSG_ID_MISSION_REQUEST + MAVLINK_NUM_NON_PAYLOAD_BYTES;

	} else {
		return 0;
	}
}

void
MavlinkMissionManager::init_offboard_mission()
{
	mission_s mission_state;
	if (dm_read(DM_KEY_MISSION_STATE, 0, &mission_state, sizeof(mission_s)) == sizeof(mission_s)) {
		_dataman_id = mission_state.dataman_id;
		_count = mission_state.count;
		_current_seq = mission_state.current_seq;

		warnx("offboard mission init: dataman_id=%d, count=%u, current_seq=%d", _dataman_id, _count, _current_seq);

	} else {
		_dataman_id = 0;
		_count = 0;
		_current_seq = 0;
		warnx("offboard mission init: ERROR, reading mission state failed");
	}
}

/**
 * Write new mission state to dataman and publish offboard_mission topic to notify navigator about changes.
 */
int
MavlinkMissionManager::update_active_mission(int dataman_id, unsigned count, int seq)
{
	struct mission_s mission;

	mission.dataman_id = dataman_id;
	mission.count = count;
	mission.current_seq = seq;

	/* update mission state in dataman */
	int res = dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission, sizeof(mission_s));

	if (res == sizeof(mission_s)) {
		/* update active mission state */
		_dataman_id = dataman_id;
		_count = count;
		_current_seq = seq;

		/* mission state saved successfully, publish offboard_mission topic */
		if (_offboard_mission_pub < 0) {
			_offboard_mission_pub = orb_advertise(ORB_ID(offboard_mission), &mission);

		} else {
			orb_publish(ORB_ID(offboard_mission), _offboard_mission_pub, &mission);
		}

		return OK;

	} else {
		warnx("ERROR: can't save mission state");
		_mavlink->send_statustext(MAV_SEVERITY_CRITICAL, "ERROR: can't save mission state");

		return ERROR;
	}
}

void
MavlinkMissionManager::send_mission_ack(uint8_t sysid, uint8_t compid, uint8_t type)
{
	mavlink_mission_ack_t wpa;

	wpa.target_system = sysid;
	wpa.target_component = compid;
	wpa.type = type;

	_mavlink->send_message(MAVLINK_MSG_ID_MISSION_ACK, &wpa);

	if (_verbose) { warnx("WPM: Send MISSION_ACK type %u to ID %u", wpa.type, wpa.target_system); }
}


void
MavlinkMissionManager::send_mission_current(uint16_t seq)
{
	if (seq < _count) {
		mavlink_mission_current_t wpc;

		wpc.seq = seq;

		_mavlink->send_message(MAVLINK_MSG_ID_MISSION_CURRENT, &wpc);

	} else if (seq == 0 && _count == 0) {
		/* don't broadcast if no WPs */

	} else {
		if (_verbose) { warnx("WPM: Send MISSION_CURRENT ERROR: seq %u out of bounds", seq); }

		_mavlink->send_statustext_critical("ERROR: wp index out of bounds");
	}
}


void
MavlinkMissionManager::send_mission_count(uint8_t sysid, uint8_t compid, uint16_t count)
{
	_time_last_sent = hrt_absolute_time();

	mavlink_mission_count_t wpc;

	wpc.target_system = sysid;
	wpc.target_component = compid;
	wpc.count = _count;

	_mavlink->send_message(MAVLINK_MSG_ID_MISSION_COUNT, &wpc);

	if (_verbose) { warnx("WPM: Send MISSION_COUNT %u to ID %u", wpc.count, wpc.target_system); }
}


void
MavlinkMissionManager::send_mission_item(uint8_t sysid, uint8_t compid, uint16_t seq)
{
	dm_item_t dm_item = DM_KEY_WAYPOINTS_OFFBOARD(_dataman_id);
	struct mission_item_s mission_item;

	if (dm_read(dm_item, seq, &mission_item, sizeof(struct mission_item_s)) == sizeof(struct mission_item_s)) {
		_time_last_sent = hrt_absolute_time();

		/* create mission_item_s from mavlink_mission_item_t */
		mavlink_mission_item_t wp;
		format_mavlink_mission_item(&mission_item, &wp);

		wp.target_system = sysid;
		wp.target_component = compid;
		wp.seq = seq;
		wp.current = (_current_seq == seq) ? 1 : 0;

		_mavlink->send_message(MAVLINK_MSG_ID_MISSION_ITEM, &wp);

		if (_verbose) { warnx("WPM: Send MISSION_ITEM seq %u to ID %u", wp.seq, wp.target_system); }

	} else {
		send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
		_mavlink->send_statustext_critical("Unable to read from micro SD");

		if (_verbose) { warnx("WPM: Send MISSION_ITEM ERROR: could not read seq %u from dataman ID %i", seq, _dataman_id); }
	}
}


void
MavlinkMissionManager::send_mission_request(uint8_t sysid, uint8_t compid, uint16_t seq)
{
	if (seq < _max_count) {
		_time_last_sent = hrt_absolute_time();

		mavlink_mission_request_t wpr;
		wpr.target_system = sysid;
		wpr.target_component = compid;
		wpr.seq = seq;

		_mavlink->send_message(MAVLINK_MSG_ID_MISSION_REQUEST, &wpr);

		if (_verbose) { warnx("WPM: Send MISSION_REQUEST seq %u to ID %u", wpr.seq, wpr.target_system); }

	} else {
		_mavlink->send_statustext_critical("ERROR: Waypoint index exceeds list capacity");

		if (_verbose) { warnx("WPM: Send MISSION_REQUEST ERROR: seq %u exceeds list capacity", seq); }
	}
}


void
MavlinkMissionManager::send_mission_item_reached(uint16_t seq)
{
	mavlink_mission_item_reached_t wp_reached;

	wp_reached.seq = seq;

	_mavlink->send_message(MAVLINK_MSG_ID_MISSION_ITEM_REACHED, &wp_reached);

	if (_verbose) { warnx("WPM: Send MISSION_ITEM_REACHED reached_seq %u", wp_reached.seq); }
}


void
MavlinkMissionManager::send(const hrt_abstime now)
{
	bool updated = false;
	orb_check(_mission_result_sub, &updated);

	if (updated) {
		mission_result_s mission_result;
		orb_copy(ORB_ID(mission_result), _mission_result_sub, &mission_result);

		_current_seq = mission_result.seq_current;

		if (_verbose) { warnx("WPM: got mission result, new current_seq: %d", _current_seq); }

		if (mission_result.reached) {
			send_mission_item_reached((uint16_t)mission_result.seq_reached);
		}

		send_mission_current(_current_seq);

		if (mission_result.item_do_jump_changed) {
			/* send a mission item again if the remaining DO_JUMPs has changed */
			send_mission_item(_transfer_partner_sysid, _transfer_partner_compid,
					  (uint16_t)mission_result.item_changed_index);
		}

	} else {
		if (_slow_rate_limiter.check(now)) {
			send_mission_current(_current_seq);
		}
	}

	/* check for timed-out operations */
	if (_state != MAVLINK_WPM_STATE_IDLE && hrt_elapsed_time(&_time_last_recv) > _action_timeout) {
		_mavlink->send_statustext_critical("Operation timeout");

		if (_verbose) { warnx("WPM: Last operation (state=%u) timed out, changing state to MAVLINK_WPM_STATE_IDLE", _state); }

		_state = MAVLINK_WPM_STATE_IDLE;

	} else if (_state == MAVLINK_WPM_STATE_GETLIST && hrt_elapsed_time(&_time_last_sent) > _retry_timeout) {
		/* try to request item again after timeout */
		send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);

	} else if (_state == MAVLINK_WPM_STATE_SENDLIST && hrt_elapsed_time(&_time_last_sent) > _retry_timeout) {
		if (_transfer_seq == 0) {
			/* try to send items count again after timeout */
			send_mission_count(_transfer_partner_sysid, _transfer_partner_compid, _transfer_count);

		} else {
			/* try to send item again after timeout */
			send_mission_item(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq - 1);
		}
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

	case MAVLINK_MSG_ID_MISSION_COUNT:
		handle_mission_count(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_ITEM:
		handle_mission_item(msg);
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
	mavlink_mission_ack_t wpa;
	mavlink_msg_mission_ack_decode(msg, &wpa);

	if (CHECK_SYSID_COMPID_MISSION(wpa)) {
		if ((msg->sysid == _transfer_partner_sysid && msg->compid == _transfer_partner_compid)) {
			if (_state == MAVLINK_WPM_STATE_SENDLIST) {
				_time_last_recv = hrt_absolute_time();

				if (_transfer_seq == _count) {
					if (_verbose) { warnx("WPM: MISSION_ACK OK all items sent, switch to state IDLE"); }

				} else {
					_mavlink->send_statustext_critical("WPM: ERR: not all items sent -> IDLE");

					if (_verbose) { warnx("WPM: MISSION_ACK ERROR: not all items sent, switch to state IDLE anyway"); }
				}

				_state = MAVLINK_WPM_STATE_IDLE;
			}

		} else {
			_mavlink->send_statustext_critical("REJ. WP CMD: partner id mismatch");

			if (_verbose) { warnx("WPM: MISSION_ACK ERROR: rejected, partner ID mismatch"); }
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

			if (wpc.seq < _count) {
				if (update_active_mission(_dataman_id, _count, wpc.seq) == OK) {
					if (_verbose) { warnx("WPM: MISSION_SET_CURRENT seq=%d OK", wpc.seq); }

				} else {
					if (_verbose) { warnx("WPM: MISSION_SET_CURRENT seq=%d ERROR", wpc.seq); }

					_mavlink->send_statustext_critical("WPM: WP CURR CMD: Error setting ID");
				}

			} else {
				if (_verbose) { warnx("WPM: MISSION_SET_CURRENT seq=%d ERROR: not in list", wpc.seq); }

				_mavlink->send_statustext_critical("WPM: WP CURR CMD: Not in list");
			}

		} else {
			if (_verbose) { warnx("WPM: MISSION_SET_CURRENT ERROR: busy"); }

			_mavlink->send_statustext_critical("WPM: IGN WP CURR CMD: Busy");
		}
	}
}


void
MavlinkMissionManager::handle_mission_request_list(const mavlink_message_t *msg)
{
	mavlink_mission_request_list_t wprl;
	mavlink_msg_mission_request_list_decode(msg, &wprl);

	if (CHECK_SYSID_COMPID_MISSION(wprl)) {
		if (_state == MAVLINK_WPM_STATE_IDLE || _state == MAVLINK_WPM_STATE_SENDLIST) {
			_time_last_recv = hrt_absolute_time();

			if (_count > 0) {
				_state = MAVLINK_WPM_STATE_SENDLIST;
				_transfer_seq = 0;
				_transfer_count = _count;
				_transfer_partner_sysid = msg->sysid;
				_transfer_partner_compid = msg->compid;

				if (_verbose) { warnx("WPM: MISSION_REQUEST_LIST OK, %u mission items to send", _transfer_count); }

			} else {
				if (_verbose) { warnx("WPM: MISSION_REQUEST_LIST OK nothing to send, mission is empty"); }

				_mavlink->send_statustext_info("WPM: mission is empty");
			}

			send_mission_count(msg->sysid, msg->compid, _count);

		} else {
			if (_verbose) { warnx("WPM: MISSION_REQUEST_LIST ERROR: busy"); }

			_mavlink->send_statustext_critical("IGN REQUEST LIST: Busy");
		}
	}
}


void
MavlinkMissionManager::handle_mission_request(const mavlink_message_t *msg)
{
	mavlink_mission_request_t wpr;
	mavlink_msg_mission_request_decode(msg, &wpr);

	if (CHECK_SYSID_COMPID_MISSION(wpr)) {
		if (msg->sysid == _transfer_partner_sysid && msg->compid == _transfer_partner_compid) {
			if (_state == MAVLINK_WPM_STATE_SENDLIST) {
				_time_last_recv = hrt_absolute_time();

				/* _transfer_seq contains sequence of expected request */
				if (wpr.seq == _transfer_seq && _transfer_seq < _transfer_count) {
					if (_verbose) { warnx("WPM: MISSION_ITEM_REQUEST seq %u from ID %u", wpr.seq, msg->sysid); }

					_transfer_seq++;

				} else if (wpr.seq == _transfer_seq - 1) {
					if (_verbose) { warnx("WPM: MISSION_ITEM_REQUEST seq %u from ID %u (again)", wpr.seq, msg->sysid); }

				} else {
					if (_transfer_seq > 0 && _transfer_seq < _transfer_count) {
						if (_verbose) { warnx("WPM: MISSION_ITEM_REQUEST ERROR: seq %u from ID %u unexpected, must be %i or %i", wpr.seq, msg->sysid, _transfer_seq - 1, _transfer_seq); }

					} else if (_transfer_seq <= 0) {
						if (_verbose) { warnx("WPM: MISSION_ITEM_REQUEST ERROR: seq %u from ID %u unexpected, must be %i", wpr.seq, msg->sysid, _transfer_seq); }

					} else {
						if (_verbose) { warnx("WPM: MISSION_ITEM_REQUEST ERROR: seq %u from ID %u unexpected, must be %i", wpr.seq, msg->sysid, _transfer_seq - 1); }
					}

					_state = MAVLINK_WPM_STATE_IDLE;

					send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
					_mavlink->send_statustext_critical("WPM: REJ. CMD: Req. WP was unexpected");
					return;
				}

				/* double check bounds in case of items count changed */
				if (wpr.seq < _count) {
					send_mission_item(_transfer_partner_sysid, _transfer_partner_compid, wpr.seq);

				} else {
					if (_verbose) { warnx("WPM: MISSION_ITEM_REQUEST ERROR: seq %u out of bound [%u, %u]", (unsigned)wpr.seq, (unsigned)wpr.seq, (unsigned)_count - 1); }

					_state = MAVLINK_WPM_STATE_IDLE;

					send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
					_mavlink->send_statustext_critical("WPM: REJ. CMD: Req. WP was unexpected");
				}

			} else if (_state == MAVLINK_WPM_STATE_IDLE) {
				if (_verbose) { warnx("WPM: MISSION_ITEM_REQUEST ERROR: no transfer"); }

				_mavlink->send_statustext_critical("IGN MISSION_ITEM_REQUEST: No active transfer");

			} else {
				if (_verbose) { warnx("WPM: MISSION_ITEM_REQUEST ERROR: busy (state %d).", _state); }

				_mavlink->send_statustext_critical("WPM: REJ. CMD: Busy");
			}

		} else {
			_mavlink->send_statustext_critical("WPM: REJ. CMD: partner id mismatch");

			if (_verbose) { warnx("WPM: MISSION_ITEM_REQUEST ERROR: rejected, partner ID mismatch"); }
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

			if (wpc.count > _max_count) {
				if (_verbose) { warnx("WPM: MISSION_COUNT ERROR: too many waypoints (%d), supported: %d", wpc.count, _max_count); }

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_NO_SPACE);
				return;
			}

			if (wpc.count == 0) {
				if (_verbose) { warnx("WPM: MISSION_COUNT 0, clearing waypoints list and staying in state MAVLINK_WPM_STATE_IDLE"); }

				/* alternate dataman ID anyway to let navigator know about changes */
				update_active_mission(_dataman_id == 0 ? 1 : 0, 0, 0);
				_mavlink->send_statustext_info("WPM: COUNT 0: CLEAR MISSION");

				// TODO send ACK?
				return;
			}

			if (_verbose) { warnx("WPM: MISSION_COUNT %u from ID %u, changing state to MAVLINK_WPM_STATE_GETLIST", wpc.count, msg->sysid); }

			_state = MAVLINK_WPM_STATE_GETLIST;
			_transfer_seq = 0;
			_transfer_partner_sysid = msg->sysid;
			_transfer_partner_compid = msg->compid;
			_transfer_count = wpc.count;
			_transfer_dataman_id = _dataman_id == 0 ? 1 : 0;	// use inactive storage for transmission
			_transfer_current_seq = -1;

		} else if (_state == MAVLINK_WPM_STATE_GETLIST) {
			_time_last_recv = hrt_absolute_time();

			if (_transfer_seq == 0) {
				/* looks like our MISSION_REQUEST was lost, try again */
				if (_verbose) { warnx("WPM: MISSION_COUNT %u from ID %u (again)", wpc.count, msg->sysid); }

				_mavlink->send_statustext_info("WP CMD OK TRY AGAIN");

			} else {
				if (_verbose) { warnx("WPM: MISSION_COUNT ERROR: busy, already receiving seq %u", _transfer_seq); }

				_mavlink->send_statustext_critical("WPM: REJ. CMD: Busy");
				return;
			}

		} else {
			if (_verbose) { warnx("WPM: MISSION_COUNT ERROR: busy, state %i", _state); }

			_mavlink->send_statustext_critical("WPM: IGN MISSION_COUNT: Busy");
			return;
		}

		send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
	}
}


void
MavlinkMissionManager::handle_mission_item(const mavlink_message_t *msg)
{
	mavlink_mission_item_t wp;
	mavlink_msg_mission_item_decode(msg, &wp);

	if (CHECK_SYSID_COMPID_MISSION(wp)) {
		if (_state == MAVLINK_WPM_STATE_GETLIST) {
			_time_last_recv = hrt_absolute_time();

			if (wp.seq != _transfer_seq) {
				if (_verbose) { warnx("WPM: MISSION_ITEM ERROR: seq %u was not the expected %u", wp.seq, _transfer_seq); }

				/* don't send request here, it will be performed in eventloop after timeout */
				return;
			}

		} else if (_state == MAVLINK_WPM_STATE_IDLE) {
			if (_verbose) { warnx("WPM: MISSION_ITEM ERROR: no transfer"); }

			_mavlink->send_statustext_critical("IGN MISSION_ITEM: No transfer");
			return;

		} else {
			if (_verbose) { warnx("WPM: MISSION_ITEM ERROR: busy, state %i", _state); }

			_mavlink->send_statustext_critical("IGN MISSION_ITEM: Busy");
			return;
		}

		struct mission_item_s mission_item;
		int ret = parse_mavlink_mission_item(&wp, &mission_item);

		if (ret != OK) {
			if (_verbose) { warnx("WPM: MISSION_ITEM ERROR: seq %u invalid item", wp.seq); }

			_mavlink->send_statustext_critical("IGN MISSION_ITEM: Busy");

			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, ret);
			_state = MAVLINK_WPM_STATE_IDLE;
			return;
		}

		dm_item_t dm_item = DM_KEY_WAYPOINTS_OFFBOARD(_transfer_dataman_id);

		if (dm_write(dm_item, wp.seq, DM_PERSIST_POWER_ON_RESET, &mission_item, sizeof(struct mission_item_s)) != sizeof(struct mission_item_s)) {
			if (_verbose) { warnx("WPM: MISSION_ITEM ERROR: error writing seq %u to dataman ID %i", wp.seq, _transfer_dataman_id); }

			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			_mavlink->send_statustext_critical("Unable to write on micro SD");
			_state = MAVLINK_WPM_STATE_IDLE;
			return;
		}

		/* waypoint marked as current */
		if (wp.current) {
			_transfer_current_seq = wp.seq;
		}

		if (_verbose) { warnx("WPM: MISSION_ITEM seq %u received", wp.seq); }

		_transfer_seq = wp.seq + 1;

		if (_transfer_seq == _transfer_count) {
			/* got all new mission items successfully */
			if (_verbose) { warnx("WPM: MISSION_ITEM got all %u items, current_seq=%u, changing state to MAVLINK_WPM_STATE_IDLE", _transfer_count, _transfer_current_seq); }

			_mavlink->send_statustext_info("WPM: Transfer complete.");

			_state = MAVLINK_WPM_STATE_IDLE;

			if (update_active_mission(_transfer_dataman_id, _transfer_count, _transfer_current_seq) == OK) {
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED);

			} else {
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			}

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

			if (update_active_mission(_dataman_id == 0 ? 1 : 0, 0, 0) == OK) {
				if (_verbose) { warnx("WPM: CLEAR_ALL OK"); }
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED);

			} else {
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			}

		} else {
			_mavlink->send_statustext_critical("WPM: IGN CLEAR CMD: Busy");

			if (_verbose) { warnx("WPM: CLEAR_ALL IGNORED: busy"); }
		}
	}
}

int
MavlinkMissionManager::parse_mavlink_mission_item(const mavlink_mission_item_t *mavlink_mission_item, struct mission_item_s *mission_item)
{
	/* only support global waypoints for now */
	switch (mavlink_mission_item->frame) {
	case MAV_FRAME_GLOBAL:
		mission_item->lat = (double)mavlink_mission_item->x;
		mission_item->lon = (double)mavlink_mission_item->y;
		mission_item->altitude = mavlink_mission_item->z;
		mission_item->altitude_is_relative = false;
		break;

	case MAV_FRAME_GLOBAL_RELATIVE_ALT:
		mission_item->lat = (double)mavlink_mission_item->x;
		mission_item->lon = (double)mavlink_mission_item->y;
		mission_item->altitude = mavlink_mission_item->z;
		mission_item->altitude_is_relative = true;
		break;

	case MAV_FRAME_LOCAL_NED:
	case MAV_FRAME_LOCAL_ENU:
		return MAV_MISSION_UNSUPPORTED_FRAME;

	case MAV_FRAME_MISSION:
	default:
		return MAV_MISSION_ERROR;
	}

	switch (mavlink_mission_item->command) {
	case MAV_CMD_NAV_TAKEOFF:
		mission_item->pitch_min = mavlink_mission_item->param1;
		break;
	case MAV_CMD_DO_JUMP:
		mission_item->do_jump_mission_index = mavlink_mission_item->param1;
		mission_item->do_jump_current_count = 0;
		mission_item->do_jump_repeat_count = mavlink_mission_item->param2;
		break;
	default:
		mission_item->acceptance_radius = mavlink_mission_item->param2;
		mission_item->time_inside = mavlink_mission_item->param1;
		break;
	}

	mission_item->yaw = _wrap_pi(mavlink_mission_item->param4 * M_DEG_TO_RAD_F);
	mission_item->loiter_radius = fabsf(mavlink_mission_item->param3);
	mission_item->loiter_direction = (mavlink_mission_item->param3 > 0) ? 1 : -1; /* 1 if positive CW, -1 if negative CCW */
	mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;

	mission_item->autocontinue = mavlink_mission_item->autocontinue;
	// mission_item->index = mavlink_mission_item->seq;
	mission_item->origin = ORIGIN_MAVLINK;

	/* reset DO_JUMP count */
	mission_item->do_jump_current_count = 0;

	return MAV_MISSION_ACCEPTED;
}


int
MavlinkMissionManager::format_mavlink_mission_item(const struct mission_item_s *mission_item, mavlink_mission_item_t *mavlink_mission_item)
{
	if (mission_item->altitude_is_relative) {
		mavlink_mission_item->frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;

	} else {
		mavlink_mission_item->frame = MAV_FRAME_GLOBAL;
	}

	switch (mission_item->nav_cmd) {
	case NAV_CMD_TAKEOFF:
		mavlink_mission_item->param1 = mission_item->pitch_min;
		break;

	case NAV_CMD_DO_JUMP:
		mavlink_mission_item->param1 = mission_item->do_jump_mission_index;
		mavlink_mission_item->param2 = mission_item->do_jump_repeat_count - mission_item->do_jump_current_count;
		break;

	default:
		mavlink_mission_item->param2 = mission_item->acceptance_radius;
		mavlink_mission_item->param1 = mission_item->time_inside;
		break;
	}

	mavlink_mission_item->x = (float)mission_item->lat;
	mavlink_mission_item->y = (float)mission_item->lon;
	mavlink_mission_item->z = mission_item->altitude;

	mavlink_mission_item->param4 = mission_item->yaw * M_RAD_TO_DEG_F;
	mavlink_mission_item->param3 = mission_item->loiter_radius * (float)mission_item->loiter_direction;
	mavlink_mission_item->command = mission_item->nav_cmd;
	mavlink_mission_item->autocontinue = mission_item->autocontinue;
	// mavlink_mission_item->seq = mission_item->index;

	return OK;
}
