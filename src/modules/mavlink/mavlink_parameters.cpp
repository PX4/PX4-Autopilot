/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file mavlink_parameters.cpp
 * Mavlink parameters manager implementation.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <stdio.h>

#include <uORB/topics/uavcan_parameter_request.h>
#include <uORB/topics/uavcan_parameter_value.h>

#include "mavlink_parameters.h"
#include "mavlink_main.h"

ORB_DEFINE(uavcan_parameter_request, struct uavcan_parameter_request_s);
ORB_DEFINE(uavcan_parameter_value, struct uavcan_parameter_value_s);
#define HASH_PARAM "_HASH_CHECK"

MavlinkParametersManager::MavlinkParametersManager(Mavlink *mavlink) : MavlinkStream(mavlink),
	_send_all_index(-1),
	_rc_param_map_pub(nullptr),
	_rc_param_map(),
	_uavcan_parameter_request_pub(nullptr),
	_uavcan_parameter_value_sub(-1)
{
}

unsigned
MavlinkParametersManager::get_size()
{
	return MAVLINK_MSG_ID_PARAM_VALUE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
}

void
MavlinkParametersManager::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
			/* request all parameters */
			mavlink_param_request_list_t req_list;
			mavlink_msg_param_request_list_decode(msg, &req_list);

			if (req_list.target_system == mavlink_system.sysid &&
			    (req_list.target_component == mavlink_system.compid || req_list.target_component == MAV_COMP_ID_ALL)) {
				if (_send_all_index < 0) {
					_send_all_index = PARAM_HASH;
				} else {
					/* a restart should skip the hash check on the ground */
					_send_all_index = 0;
				}
			}

			if (req_list.target_system == mavlink_system.sysid && req_list.target_component < 127 &&
			    (req_list.target_component != mavlink_system.compid || req_list.target_component == MAV_COMP_ID_ALL)) {
				// publish list request to UAVCAN driver via uORB.
				uavcan_parameter_request_s req;
				req.message_type = msg->msgid;
				req.node_id = req_list.target_component;
				req.param_index = 0;

				if (_uavcan_parameter_request_pub == nullptr) {
					_uavcan_parameter_request_pub = orb_advertise(ORB_ID(uavcan_parameter_request), &req);
				} else {
					orb_publish(ORB_ID(uavcan_parameter_request), _uavcan_parameter_request_pub, &req);
				}
			}
			break;
		}

	case MAVLINK_MSG_ID_PARAM_SET: {
			/* set parameter */
			mavlink_param_set_t set;
			mavlink_msg_param_set_decode(msg, &set);

			if (set.target_system == mavlink_system.sysid &&
			    (set.target_component == mavlink_system.compid || set.target_component == MAV_COMP_ID_ALL)) {

				/* local name buffer to enforce null-terminated string */
				char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
				strncpy(name, set.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
				/* enforce null termination */
				name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

				/* Whatever the value is, we're being told to stop sending */
				if (strncmp(name, "_HASH_CHECK", sizeof(name)) == 0) {
					_send_all_index = -1;
					/* No other action taken, return */
					return;
				}

				/* attempt to find parameter, set and send it */
				param_t param = param_find_no_notification(name);

				if (param == PARAM_INVALID) {
					char buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
					sprintf(buf, "[pm] unknown param: %s", name);
					_mavlink->send_statustext_info(buf);

				} else {
					/* set and send parameter */
					param_set(param, &(set.param_value));
					send_param(param);
				}
			}

			if (set.target_system == mavlink_system.sysid && set.target_component < 127 &&
			    (set.target_component != mavlink_system.compid || set.target_component == MAV_COMP_ID_ALL)) {
				// publish set request to UAVCAN driver via uORB.
				uavcan_parameter_request_s req;
				req.message_type = msg->msgid;
				req.node_id = set.target_component;
				req.param_index = -1;
				strncpy(req.param_id, set.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
				req.param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';
				if (set.param_type == MAV_PARAM_TYPE_REAL32) {
					req.param_type = MAV_PARAM_TYPE_REAL32;
					req.real_value = set.param_value;
				} else {
					int32_t val;
					memcpy(&val, &set.param_value, sizeof(int32_t));
					req.param_type = MAV_PARAM_TYPE_INT64;
					req.int_value = val;
				}

				if (_uavcan_parameter_request_pub == nullptr) {
					_uavcan_parameter_request_pub = orb_advertise(ORB_ID(uavcan_parameter_request), &req);
				} else {
					orb_publish(ORB_ID(uavcan_parameter_request), _uavcan_parameter_request_pub, &req);
				}
			}
			break;
		}

	case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
			/* request one parameter */
			mavlink_param_request_read_t req_read;
			mavlink_msg_param_request_read_decode(msg, &req_read);

			if (req_read.target_system == mavlink_system.sysid &&
			    (req_read.target_component == mavlink_system.compid || req_read.target_component == MAV_COMP_ID_ALL)) {

				/* when no index is given, loop through string ids and compare them */
				if (req_read.param_index < 0) {
					/* XXX: I left this in so older versions of QGC wouldn't break */
					if (strncmp(req_read.param_id, HASH_PARAM, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN) == 0) {
						/* return hash check for cached params */
						uint32_t hash = param_hash_check();

						/* build the one-off response message */
						mavlink_param_value_t msg;
						msg.param_count = param_count_used();
						msg.param_index = -1;
						strncpy(msg.param_id, HASH_PARAM, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
						msg.param_type = MAV_PARAM_TYPE_UINT32;
						memcpy(&msg.param_value, &hash, sizeof(hash));
						_mavlink->send_message(MAVLINK_MSG_ID_PARAM_VALUE, &msg);
					} else {
						/* local name buffer to enforce null-terminated string */
						char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
						strncpy(name, req_read.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
						/* enforce null termination */
						name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';
						/* attempt to find parameter and send it */
						send_param(param_find_no_notification(name));
					}
				} else {
					/* when index is >= 0, send this parameter again */
					int ret = send_param(param_for_used_index(req_read.param_index));

					if (ret == 1) {
						char buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
						sprintf(buf, "[pm] unknown param ID: %u", req_read.param_index);
						_mavlink->send_statustext_info(buf);
					} else if (ret == 2) {
						char buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
						sprintf(buf, "[pm] failed loading param from storage ID: %u", req_read.param_index);
						_mavlink->send_statustext_info(buf);
					}
				}
			}

			if (req_read.target_system == mavlink_system.sysid && req_read.target_component < 127 &&
				    (req_read.target_component != mavlink_system.compid || req_read.target_component == MAV_COMP_ID_ALL)) {
				// publish set request to UAVCAN driver via uORB.
				uavcan_parameter_request_s req;
				req.message_type = msg->msgid;
				req.node_id = req_read.target_component;
				req.param_index = req_read.param_index;
				strncpy(req.param_id, req_read.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
				req.param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

				if (_uavcan_parameter_request_pub == nullptr) {
					_uavcan_parameter_request_pub = orb_advertise(ORB_ID(uavcan_parameter_request), &req);
				} else {
					orb_publish(ORB_ID(uavcan_parameter_request), _uavcan_parameter_request_pub, &req);
				}
			}
			break;
		}

	case MAVLINK_MSG_ID_PARAM_MAP_RC: {
			/* map a rc channel to a parameter */
			mavlink_param_map_rc_t map_rc;
			mavlink_msg_param_map_rc_decode(msg, &map_rc);

			if (map_rc.target_system == mavlink_system.sysid &&
			    (map_rc.target_component == mavlink_system.compid ||
			     map_rc.target_component == MAV_COMP_ID_ALL)) {

				/* Copy values from msg to uorb using the parameter_rc_channel_index as index */
				size_t i = map_rc.parameter_rc_channel_index;
				_rc_param_map.param_index[i] = map_rc.param_index;
				strncpy(&(_rc_param_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1)]), map_rc.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
				/* enforce null termination */
				_rc_param_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1) + rc_parameter_map_s::PARAM_ID_LEN] = '\0';
				_rc_param_map.scale[i] = map_rc.scale;
				_rc_param_map.value0[i] = map_rc.param_value0;
				_rc_param_map.value_min[i] = map_rc.param_value_min;
				_rc_param_map.value_max[i] = map_rc.param_value_max;
				if (map_rc.param_index == -2) { // -2 means unset map
					_rc_param_map.valid[i] = false;
				} else {
					_rc_param_map.valid[i] = true;
				}
				_rc_param_map.timestamp = hrt_absolute_time();

				if (_rc_param_map_pub == nullptr) {
					_rc_param_map_pub = orb_advertise(ORB_ID(rc_parameter_map), &_rc_param_map);

				} else {
					orb_publish(ORB_ID(rc_parameter_map), _rc_param_map_pub, &_rc_param_map);
				}

			}
			break;
		}

	default:
		break;
	}
}

void
MavlinkParametersManager::send(const hrt_abstime t)
{
	bool space_available = _mavlink->get_free_tx_buf() >= get_size();

	/* Send parameter values received from the UAVCAN topic */
	if (_uavcan_parameter_value_sub < 0) {
		_uavcan_parameter_value_sub = orb_subscribe(ORB_ID(uavcan_parameter_value));
	}

	bool param_value_ready;
	orb_check(_uavcan_parameter_value_sub, &param_value_ready);
	if (space_available && param_value_ready) {
		struct uavcan_parameter_value_s value;
		orb_copy(ORB_ID(uavcan_parameter_value), _uavcan_parameter_value_sub, &value);

		mavlink_param_value_t msg;
		msg.param_count = value.param_count;
		msg.param_index = value.param_index;
		strncpy(msg.param_id, value.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
		if (value.param_type == MAV_PARAM_TYPE_REAL32) {
			msg.param_type = MAVLINK_TYPE_FLOAT;
			msg.param_value = value.real_value;
		} else {
			int32_t val;
			val = (int32_t)value.int_value;
			memcpy(&msg.param_value, &val, sizeof(int32_t));
			msg.param_type = MAVLINK_TYPE_INT32_T;
		}
		_mavlink->send_message(MAVLINK_MSG_ID_PARAM_VALUE, &msg, value.node_id);
	} else if (_send_all_index >= 0 && _mavlink->boot_complete()) {
		/* send all parameters if requested, but only after the system has booted */

		/* skip if no space is available */
		if (!space_available) {
			return;
		}

		/* The first thing we send is a hash of all values for the ground
		 * station to try and quickly load a cached copy of our params
		 */
		if (_send_all_index == PARAM_HASH) {
			/* return hash check for cached params */
			uint32_t hash = param_hash_check();

			/* build the one-off response message */
			mavlink_param_value_t msg;
			msg.param_count = param_count_used();
			msg.param_index = -1;
			strncpy(msg.param_id, HASH_PARAM, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
			msg.param_type = MAV_PARAM_TYPE_UINT32;
			memcpy(&msg.param_value, &hash, sizeof(hash));
			_mavlink->send_message(MAVLINK_MSG_ID_PARAM_VALUE, &msg);

			/* after this we should start sending all params */
			_send_all_index = 0;

			/* No further action, return now */
			return;
		}

		/* look for the first parameter which is used */
		param_t p;
		do {
			/* walk through all parameters, including unused ones */
			p = param_for_index(_send_all_index);
			_send_all_index++;
		} while (p != PARAM_INVALID && !param_used(p));

		if (p != PARAM_INVALID) {
			send_param(p);
		}

		if ((p == PARAM_INVALID) || (_send_all_index >= (int) param_count())) {
			_send_all_index = -1;
		}
	} else if (_send_all_index == PARAM_HASH && hrt_absolute_time() > 20 * 1000 * 1000) {
		/* the boot did not seem to ever complete, warn user and set boot complete */
		_mavlink->send_statustext_critical("WARNING: SYSTEM BOOT INCOMPLETE. CHECK CONFIG.");
		_mavlink->set_boot_complete();
	}
}

int
MavlinkParametersManager::send_param(param_t param)
{
	if (param == PARAM_INVALID) {
		return 1;
	}

	mavlink_param_value_t msg;

	/*
	 * get param value, since MAVLink encodes float and int params in the same
	 * space during transmission, copy param onto float val_buf
	 */
	if (param_get(param, &msg.param_value) != OK) {
		return 2;
	}

	msg.param_count = param_count_used();
	msg.param_index = param_get_used_index(param);

	/* copy parameter name */
	strncpy(msg.param_id, param_name(param), MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);

	/* query parameter type */
	param_type_t type = param_type(param);

	/*
	 * Map onboard parameter type to MAVLink type,
	 * endianess matches (both little endian)
	 */
	if (type == PARAM_TYPE_INT32) {
		msg.param_type = MAVLINK_TYPE_INT32_T;

	} else if (type == PARAM_TYPE_FLOAT) {
		msg.param_type = MAVLINK_TYPE_FLOAT;

	} else {
		msg.param_type = MAVLINK_TYPE_FLOAT;
	}

	_mavlink->send_message(MAVLINK_MSG_ID_PARAM_VALUE, &msg);

	return 0;
}
