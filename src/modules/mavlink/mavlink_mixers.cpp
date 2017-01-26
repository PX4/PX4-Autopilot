/****************************************************************************
 *
 *   Copyright (c) 2015-2017 PX4 Development Team. All rights reserved.
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
 * @file mavlink_mixers.cpp
 * Mixer parameters manager implementation.
 *
 * @author Matthew Coleman <uavflightdirector@gmail.com>
 */

#include <stdio.h>

#include <uORB/topics/mixer_data_request.h>
#include <uORB/topics/mixer_data.h>
#include <uORB/topics/mixer_parameter_set.h>

#include "mavlink_mixers.h"
#include "mavlink_main.h"

MavlinkMixersManager::MavlinkMixersManager(Mavlink *mavlink) : MavlinkStream(mavlink),
	_mixer_data_request_pub(nullptr),
	_mixer_parameter_set_pub(nullptr),
	_mixer_data_sub(-1)
{
}
MavlinkMixersManager::~MavlinkMixersManager()
{
	if (_mixer_data_sub >= 0) {
		orb_unsubscribe(_mixer_data_sub);
	}

	if (_mixer_data_request_pub) {
		orb_unadvertise(_mixer_data_request_pub);
	}

	if (_mixer_parameter_set_pub) {
		orb_unadvertise(_mixer_parameter_set_pub);
	}
}

unsigned
MavlinkMixersManager::get_size()
{
	return MAVLINK_MSG_ID_MIXER_DATA_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
}

unsigned
MavlinkMixersManager::get_size_avg()
{
	return 0;
}

void
MavlinkMixersManager::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_MIXER_DATA_REQUEST: {
			/* set mixer parameter */
			mavlink_mixer_data_request_t req;
			mavlink_msg_mixer_data_request_decode(msg, &req);

			if (req.target_system == mavlink_system.sysid &&
			    (req.target_component == mavlink_system.compid || req.target_component == MAV_COMP_ID_ALL)) {

				// publish mixer data request to uORB
				mixer_data_request_s data_request;

				data_request.mixer_group = req.mixer_group;
				data_request.mixer_index = req.mixer_index;
				data_request.mixer_sub_index = req.mixer_sub_index;
				data_request.parameter_index = req.parameter_index;
				data_request.mixer_data_type = req.data_type;

				//PX4_ERR("data request group:%u mix_index:%u sub_index:%u param_index:%u type:%u",
				req.mixer_group,
				req.mixer_index,
				req.mixer_sub_index,
				req.parameter_index,
				req.data_type);

				if (_mixer_data_request_pub == nullptr) {
				_mixer_data_request_pub = orb_advertise(ORB_ID(mixer_data_request), &data_request);

				} else {
					orb_publish(ORB_ID(mixer_data_request), _mixer_data_request_pub, &data_request);
				}
			}

			break;
		}

	case MAVLINK_MSG_ID_MIXER_PARAMETER_SET: {
			/* set mixer parameter */
			mavlink_mixer_parameter_set_t set;
			mavlink_msg_mixer_parameter_set_decode(msg, &set);

			if (set.target_system == mavlink_system.sysid &&
			    (set.target_component == mavlink_system.compid || set.target_component == MAV_COMP_ID_ALL)) {

				// publish set mixer parameter request to uORB
				mixer_parameter_set_s param_set;

				if (set.param_type == MAV_PARAM_TYPE_REAL32) {
					param_set.param_type = MAV_PARAM_TYPE_REAL32;
					param_set.real_value = set.param_value;

				} else {
					int32_t val;
					memcpy(&val, &param_set.int_value, sizeof(int32_t));
					param_set.param_type = MAV_PARAM_TYPE_INT32;
					param_set.int_value = val;
				}

				param_set.mixer_group = set.mixer_group;
				param_set.mixer_index = set.mixer_index;
				param_set.mixer_sub_index = set.mixer_sub_index;
				param_set.parameter_index = set.parameter_index;

				if (_mixer_parameter_set_pub == nullptr) {
					_mixer_parameter_set_pub = orb_advertise(ORB_ID(mixer_parameter_set), &param_set);

				} else {
					orb_publish(ORB_ID(mixer_parameter_set), _mixer_parameter_set_pub, &param_set);
				}
			}

			break;
		}

	default:
		break;
	}
}

void
MavlinkMixersManager::send(const hrt_abstime t)
{
	bool space_available = _mavlink->get_free_tx_buf() >= get_size();

	/* Send parameter values received from the UAVCAN topic */
	if (_mixer_data_sub < 0) {
		_mixer_data_sub = orb_subscribe(ORB_ID(mixer_data));
	}

	bool mixer_data_ready;
	orb_check(_mixer_data_sub, &mixer_data_ready);

	if (space_available && mixer_data_ready) {
		struct mixer_data_s mixer_data;
		orb_copy(ORB_ID(mixer_data), _mixer_data_sub, &mixer_data);

		//PX4_ERR("_mixer_data_sub has data ready and with data type:%u", mixer_data.mixer_data_type);

		mavlink_mixer_data_t msg;
		msg.target_system = mavlink_system.sysid;
		msg.target_component = 0;
		msg.mixer_group = mixer_data.mixer_group;
		msg.mixer_index = mixer_data.mixer_index;
		msg.mixer_sub_index = mixer_data.mixer_sub_index;
		msg.parameter_index = mixer_data.parameter_index;
		msg.data_type = mixer_data.mixer_data_type;
		msg.param_type = mixer_data.param_type;

		if (msg.data_type == MIXER_DATA_TYPE_PARAMETER) {
			if (mixer_data.param_type == MAV_PARAM_TYPE_REAL32) {
				msg.param_type = MAVLINK_TYPE_FLOAT;
				msg.param_value = mixer_data.real_value;

			} else {
				int32_t val;
				val = (int32_t)mixer_data.int_value;
				memcpy(&msg.param_value, &val, sizeof(int32_t));
				msg.param_type = MAVLINK_TYPE_INT32_T;
			}
		}

		/* Send with default component ID */
		mavlink_msg_mixer_data_send_struct(_mavlink->get_channel(), &msg);
	}
}
