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

#if defined(MIXER_CONFIGURATION)

#include <stdio.h>

#include <uORB/topics/mixer_data_request.h>
#include <uORB/topics/mixer_data.h>
#include <uORB/topics/mixer_parameter_set.h>

#include <systemlib/mixer/mixer_parameters.h>
#include "mavlink_main.h"

static const unsigned mixer_parameter_count[MIXER_PARAMETERS_MIXER_TYPE_COUNT] = MIXER_PARAMETER_COUNTS;

MavlinkMixersManager::MavlinkMixersManager(Mavlink *mavlink) : MavlinkStream(mavlink),
	_request_pending(false),
	_send_all(false),
	_mixer_data_req(),
	_mixer_group(0),
	_mixer_count(0),
	_mixer_sub_count(0),
	_mixer_type(0),
	_mixer_param_count(0),
	_mavlink_mixer_state(MAVLINK_MIXER_STATE_WAITING),
	_mixer_data_request_pub(nullptr),
	_mixer_parameter_set_pub(nullptr),
	_mixer_data_sub(-1)
{
	_interval = 100000;
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

				_request_pending = true;

				// publish mixer data request to uORB
				_mixer_data_req.mixer_group = req.mixer_group;

				if (req.data_type == 100) {     //MIXER_ACTION_SEND_ALL
					_mixer_data_req.mixer_index = 0;
					_mixer_data_req.mixer_sub_index = 0;
					_mixer_data_req.parameter_index = 0;
					_mixer_data_req.mixer_data_type = MIXER_DATA_TYPE_MIXER_COUNT;
					_send_all = true;

				} else {
					_mixer_data_req.mixer_index = req.mixer_index;
					_mixer_data_req.mixer_sub_index = req.mixer_sub_index;
					_mixer_data_req.parameter_index = req.parameter_index;
					_mixer_data_req.mixer_data_type = req.data_type;
					_send_all = false;
				}


				PX4_INFO("data request group:%u mix_index:%u sub_index:%u param_index:%u type:%u",
					 req.mixer_group, req.mixer_index, req.mixer_sub_index, req.parameter_index, req.data_type);

				if (_mixer_data_request_pub == nullptr) {
					_mixer_data_request_pub = orb_advertise(ORB_ID(mixer_data_request), &_mixer_data_req);

				} else {
					orb_publish(ORB_ID(mixer_data_request), _mixer_data_request_pub, &_mixer_data_req);
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

				_request_pending = true;

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

	if (space_available && mixer_data_ready && _request_pending) {
		struct mixer_data_s mixer_data;
		orb_copy(ORB_ID(mixer_data), _mixer_data_sub, &mixer_data);

		mavlink_mixer_data_t msg;
		msg.target_system = mavlink_system.sysid;
		msg.target_component = 0;
		msg.mixer_group = mixer_data.mixer_group;
		msg.mixer_index = mixer_data.mixer_index;
		msg.mixer_sub_index = mixer_data.mixer_sub_index;
		msg.parameter_index = mixer_data.parameter_index;
		msg.data_type = mixer_data.mixer_data_type;
		msg.param_type = mixer_data.param_type;
		msg.data_value = mixer_data.int_value;

		if (_send_all) {
			switch (mixer_data.mixer_data_type) {
			case MIXER_DATA_TYPE_MIXER_COUNT:
				_mixer_count = mixer_data.int_value;
				break;

			case MIXER_DATA_TYPE_SUBMIXER_COUNT:
				_mixer_sub_count = mixer_data.int_value;
				break;

			case MIXER_DATA_TYPE_MIXTYPE:
				_mixer_type = mixer_data.int_value;
				_mixer_param_count = mixer_parameter_count[_mixer_type];
				break;
			}
		}

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

		PX4_INFO("mavlink mixer_data send with group:%u index:%u sub:%u param:%u type:%u data:%u",
			 msg.mixer_group,
			 msg.mixer_index,
			 msg.mixer_sub_index,
			 msg.parameter_index,
			 mixer_data.mixer_data_type,
			 msg.data_value);

		_request_pending = false;

		/* Send with default component ID */
		mavlink_msg_mixer_data_send_struct(_mavlink->get_channel(), &msg);

	} else if (!space_available && mixer_data_ready && _request_pending) {
		PX4_WARN("Space not avialable in buffer to stream mixer data");
	}

	if (_send_all && !_request_pending) {
		switch (_mixer_data_req.mixer_data_type) {
		case MIXER_DATA_TYPE_MIXER_COUNT:
			_mixer_data_req.mixer_index = 0;
			_mixer_data_req.mixer_sub_index = 0;
			_mixer_data_req.parameter_index = 0;
			_mixer_data_req.mixer_data_type = MIXER_DATA_TYPE_SUBMIXER_COUNT;

			if (_mixer_data_request_pub != nullptr) {
				orb_publish(ORB_ID(mixer_data_request), _mixer_data_request_pub, &_mixer_data_req);
			};

			break;

		case MIXER_DATA_TYPE_SUBMIXER_COUNT:
//            PX4_INFO("MIXER send all - got submixer count");
			_mixer_data_req.mixer_data_type = MIXER_DATA_TYPE_MIXTYPE;

//            PX4_INFO("MIXER send all - request mixer type");
			if (_mixer_data_request_pub != nullptr) {
				orb_publish(ORB_ID(mixer_data_request), _mixer_data_request_pub, &_mixer_data_req);
			}

			break;

		case MIXER_DATA_TYPE_MIXTYPE:
			_mixer_data_req.parameter_index = 0;
			_mixer_data_req.mixer_data_type = MIXER_DATA_TYPE_PARAMETER;

			if (_mixer_data_request_pub != nullptr) {
				orb_publish(ORB_ID(mixer_data_request), _mixer_data_request_pub, &_mixer_data_req);
			}

			break;

		case MIXER_DATA_TYPE_PARAMETER:
			_mixer_data_req.parameter_index++;

			if (_mixer_data_req.parameter_index >= _mixer_param_count) {    /**Check if parameter index has exceeded count*/
				_mixer_data_req.parameter_index = 0;
				_mixer_data_req.mixer_sub_index++;

				if (_mixer_data_req.mixer_sub_index > _mixer_sub_count) {   /**Check if submixer index has exceeded count*/
					_mixer_data_req.mixer_sub_index = 0;
					_mixer_data_req.parameter_index = 0;
					_mixer_data_req.mixer_data_type = MIXER_DATA_TYPE_SUBMIXER_COUNT;
					_mixer_data_req.mixer_index++;
				}

				if (_mixer_data_req.mixer_index < _mixer_count) {           /**Check if mixer index has exceeded count*/
					_mixer_data_req.mixer_data_type = MIXER_DATA_TYPE_MIXTYPE;

					if (_mixer_data_request_pub != nullptr) {
						orb_publish(ORB_ID(mixer_data_request), _mixer_data_request_pub, &_mixer_data_req);
					}

				} else {
//                   PX4_INFO("MIXER send all - finished");                  /**Stop here*/
					_send_all = false;
				}

			} else {  /**Send next parameter request */
				if (_mixer_data_request_pub != nullptr) {
					orb_publish(ORB_ID(mixer_data_request), _mixer_data_request_pub, &_mixer_data_req);
				}
			}


			break;

		default:
			_mavlink_mixer_state = MAVLINK_MIXER_STATE_WAITING;
			break;
		}

		_request_pending = true;
	}

}
#endif //MIXER_CONFIGURATION
