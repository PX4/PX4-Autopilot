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


#include "mavlink_main.h"

#if defined(MIXER_TUNING)

#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>

#include <uORB/topics/mixer_data_request.h>
#include <uORB/topics/mixer_data.h>
#include <uORB/topics/mixer_parameter_set.h>

#include <systemlib/mixer/mixer_parameters.h>
#include <systemlib/mixer/mixer_io.h>
#include <drivers/drv_mixer.h>

#define MOUNTPOINT PX4_ROOTFSDIR "/fs/microsd"

//static const char *kMixerLocalData    = MOUNTPOINT "/mixer.local.mix";
//static const char *kMixerFailsafeData = MOUNTPOINT "/mixer.failsafe.mix";
//static const char *kMixerDefaultData = MOUNTPOINT "/mixer.default.mix";

//static const unsigned mixer_parameter_count[MIXER_PARAMETERS_MIXER_TYPE_COUNT] = MIXER_PARAMETER_COUNTS;
//static const unsigned mixer_input_count[MIXER_IO_MIXER_TYPE_COUNT] = MIXER_INPUT_COUNTS;
//static const unsigned mixer_output_count[MIXER_IO_MIXER_TYPE_COUNT] = MIXER_OUTPUT_COUNTS;

MavlinkMixersManager::MavlinkMixersManager(Mavlink *mavlink) : MavlinkStream(mavlink),
	_request_pending(false),
	_send_all_state(MIXERS_SEND_ALL_NONE),
	_send_data_immediate(false),
	_mixer_data_req(),
	_mixer_group(0),
	_mixer_count(0),
	_mixer_sub_count(0),
	_mixer_type(0),
	_mixer_param_count(0),
	_mixer_input_count(0),
	_mixer_output_count(0),
	_mavlink_mixer_state(MAVLINK_MIXER_STATE_WAITING),
	_p_mixer_save_buffer(nullptr),
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
	if (msg->msgid != MAVLINK_MSG_ID_COMMAND_LONG) { return; }

	mavlink_command_long_t cmd;
	mavlink_msg_command_long_decode(msg, &cmd);

	switch (cmd.command) {
	case MAV_CMD_REQUEST_MIXER_DATA:
		_mixer_data_req.mixer_group = cmd.param1;
		_mixer_data_req.mixer_index = cmd.param2;
		_mixer_data_req.mixer_sub_index = cmd.param3;
		_mixer_data_req.parameter_index = cmd.param4;
		_mixer_data_req.connection_type = 0;
		_mixer_data_req.mixer_data_type = cmd.param5;
		_mixer_data_req.connection_type = 0;
		_send_all_state = MIXERS_SEND_ALL_NONE;

		// data types not yet supported
		if ((_mixer_data_req.mixer_data_type == MIXER_DATA_TYPE_PARAMETER_COUNT) ||
		    (_mixer_data_req.mixer_data_type == MIXER_DATA_TYPE_CONNECTION_COUNT)) {
			return;
		}

		_request_pending = true;

//		PX4_INFO("data request group:%u mix_index:%u sub_index:%u param_index:%u type:%u",
//			 _mixer_data_req.mixer_group,  _mixer_data_req.mixer_index, _mixer_data_req.mixer_sub_index,
//			 _mixer_data_req.parameter_index, _mixer_data_req.mixer_data_type);

		if (_mixer_data_request_pub == nullptr) {
			_mixer_data_request_pub = orb_advertise(ORB_ID(mixer_data_request), &_mixer_data_req);

		} else {
			orb_publish(ORB_ID(mixer_data_request), _mixer_data_request_pub, &_mixer_data_req);
		}

		break;

	case MAV_CMD_SET_MIXER_PARAMETER:
		// publish set mixer parameter request to uORB
		mixer_parameter_set_s param_set;

		param_set.mixer_group = cmd.param1;
		param_set.mixer_index = cmd.param2;
		param_set.mixer_sub_index = cmd.param3;
		param_set.parameter_index = cmd.param4;
		_send_all_state = MIXERS_SEND_ALL_NONE;
		_request_pending = true;

		//No non real parameter support for now.
//        if (int(cmd.param6) == MAV_PARAM_TYPE_REAL32) {
//			param_set.param_type = MAV_PARAM_TYPE_REAL32;
//            param_set.real_value = cmd.param_value;

//		} else {
//			int32_t val;
//			memcpy(&val, &param_set.int_value, sizeof(int32_t));
//			param_set.param_type = MAV_PARAM_TYPE_INT32;
//			param_set.int_value = val;
//		}

		param_set.param_type = MAV_PARAM_TYPE_REAL32;
		param_set.real_value = cmd.param5;
		param_set.int_value = -1;

		if (_mixer_parameter_set_pub == nullptr) {
			_mixer_parameter_set_pub = orb_advertise(ORB_ID(mixer_parameter_set), &param_set);

		} else {
			orb_publish(ORB_ID(mixer_parameter_set), _mixer_parameter_set_pub, &param_set);
		}

		break;

	case MAV_CMD_REQUEST_MIXER_SEND_ALL:
		param_set.mixer_group = cmd.param1;
		_send_all_state = MIXERS_SEND_ALL_START;
		break;

	case MAV_CMD_REQUEST_MIXER_CONN: {
			_mixer_data_req.mixer_group = cmd.param1;
			_mixer_data_req.mixer_index = cmd.param2;
			_mixer_data_req.mixer_sub_index = cmd.param3;
			_mixer_data_req.parameter_index = cmd.param5;
			_mixer_data_req.connection_type = cmd.param4;
			_mixer_data_req.mixer_data_type = MIXER_DATA_TYPE_CONNECTION;
			_send_all_state = MIXERS_SEND_ALL_NONE;
			_request_pending = true;

			if (_mixer_data_request_pub == nullptr) {
				_mixer_data_request_pub = orb_advertise(ORB_ID(mixer_data_request), &_mixer_data_req);

			} else {
				orb_publish(ORB_ID(mixer_data_request), _mixer_data_request_pub, &_mixer_data_req);
			}

			break;
		}

	default:
		return;
	}

}


void
MavlinkMixersManager::send(const hrt_abstime t)
{
//    bool space_available = _mavlink->get_free_tx_buf() >= get_size();

//	if (!space_available) { return; }

//	if (_send_data_immediate) {
//		/* Send with default component ID */
//		mavlink_msg_mixer_data_send_struct(_mavlink->get_channel(), &_msg_mixer_data_immediate);
//		_send_data_immediate = false;
//		return;
//	}

//	if (_send_all_state == MIXERS_SEND_ALL_START) {
//		//Reset all values to the start and get the mixer count
//		_mixer_data_req.mixer_index = 0;
//		_mixer_data_req.mixer_sub_index = 0;
//		_mixer_data_req.parameter_index = 0;
//		_mixer_data_req.connection_type = 0;
//		_mixer_data_req.mixer_data_type = MIXER_DATA_TYPE_MIXER_COUNT;

//		if (_mixer_data_request_pub == nullptr) {
//			_mixer_data_request_pub = orb_advertise(ORB_ID(mixer_data_request), &_mixer_data_req);

//		} else {
//			orb_publish(ORB_ID(mixer_data_request), _mixer_data_request_pub, &_mixer_data_req);
//		}

//		_request_pending = true;
//		_send_all_state = MIXERS_SEND_ALL_SUBMIXER_COUNT;
//	}

//	/* Send parameter values received from mixer data topic */
//	if (_mixer_data_sub < 0) {
//		_mixer_data_sub = orb_subscribe(ORB_ID(mixer_data));
//	}

//	bool mixer_data_ready;
//	orb_check(_mixer_data_sub, &mixer_data_ready);

//	if (space_available && mixer_data_ready && _request_pending) {
//		struct mixer_data_s mixer_data;
//		orb_copy(ORB_ID(mixer_data), _mixer_data_sub, &mixer_data);

//		mavlink_mixer_data_t msg;
//		msg.mixer_group = mixer_data.mixer_group;
//		msg.mixer_index = mixer_data.mixer_index;
//		msg.mixer_sub_index = mixer_data.mixer_sub_index;
//		msg.parameter_index = mixer_data.parameter_index;
//		msg.connection_type = mixer_data.connection_type;
//		msg.connection_group = mixer_data.connection_group;
//		msg.data_type = mixer_data.mixer_data_type;
//		msg.param_type = mixer_data.param_type;
//		msg.data_value = mixer_data.int_value;

//		//If sending all as a stream, remember the latest values for mixer shapes
//		if (_send_all_state != MIXERS_SEND_ALL_NONE) {
//			switch (mixer_data.mixer_data_type) {
//			case MIXER_DATA_TYPE_MIXER_COUNT:
//				_mixer_count = mixer_data.int_value;
//				break;

//			case MIXER_DATA_TYPE_SUBMIXER_COUNT:
//				_mixer_sub_count = mixer_data.int_value;
//				break;

//			case MIXER_DATA_TYPE_MIXTYPE:
//				_mixer_type = mixer_data.int_value;
//				_mixer_param_count = mixer_parameter_count[_mixer_type];
//				_mixer_input_count = mixer_input_count[_mixer_type];
//				_mixer_output_count = mixer_output_count[_mixer_type];
//				break;
//			}
//		}

//		if (msg.data_type == MIXER_DATA_TYPE_PARAMETER) {
//			if (mixer_data.param_type == MAV_PARAM_TYPE_REAL32) {
//				msg.param_type = MAVLINK_TYPE_FLOAT;
//				msg.param_value = mixer_data.real_value;

//			} else {
//				int32_t val;
//				val = (int32_t)mixer_data.int_value;
//				memcpy(&msg.param_value, &val, sizeof(int32_t));
//				msg.param_type = MAVLINK_TYPE_INT32_T;
//			}
//		}

//		if (msg.data_type == 112) {
//			if ((_p_mixer_save_buffer != nullptr) || (mixer_data.int_value != -1)) {
//				const char *fname = nullptr;

//				switch (mixer_data.mixer_group) {
//				case 0:
//					fname = kMixerLocalData;
//					break;

//				case 1:
//					fname = kMixerFailsafeData;
//					break;

//				default:
//					fname = kMixerDefaultData;
//					break;
//				}

//				/* Create the mixer definition file */
//				FILE *f = ::fopen(fname, "w");

//				if (f != nullptr) {
//					/* Write the buffer to the file*/
//					signed err = fputs(_p_mixer_save_buffer, f);

//					if (err <= 0) {
//						PX4_ERR("Mixer file write error: %s ", fname);
//						msg.data_value = -1;

//					} else {
//						PX4_INFO("Wrote mixer group:%u to file %s\n", mixer_data.mixer_group, fname);
//					}

//#ifdef __PX4_NUTTX
//					fsync(fileno(f));
//#endif
//					fclose(f);

//				} else {
//					PX4_ERR("not able to create mixer file %s", fname);
//					msg.data_value = -1;
//				}

//			}

//			if (_p_mixer_save_buffer != nullptr) {
//				free(_p_mixer_save_buffer);
//				_p_mixer_save_buffer = nullptr;
//				PX4_INFO("Saving mixer freeing buffer");
//			}
//		}


////        PX4_INFO("mavlink mixer_data send with group:%u index:%u conn_type:%u sub:%u param:%u type:%u data:%u",
////			 msg.mixer_group,
////			 msg.mixer_index,
////			 msg.mixer_sub_index,
////			 msg.parameter_index,
////             msg.connection_type,
////			 mixer_data.mixer_data_type,
////			 msg.data_value);

//		_request_pending = false;

//		/* Send with default component ID */
//		mavlink_msg_mixer_data_send_struct(_mavlink->get_channel(), &msg);

//	} else if (!space_available && mixer_data_ready && _request_pending) {
//		PX4_WARN("Space not avialable in buffer to stream mixer data");
//	}

//	// Request the next piece of data for send all
//	// Case statement handles the last state of the last data sent, not the next one.

//	if (!_request_pending) {
//		switch (uint(_send_all_state)) {

//		case MIXERS_SEND_ALL_SUBMIXER_COUNT: {
//				_mixer_data_req.parameter_index = 0;
//				_mixer_data_req.connection_type = 0;
//				_mixer_data_req.mixer_data_type = MIXER_DATA_TYPE_SUBMIXER_COUNT;

//				_request_pending = true;

//				if (_mixer_data_request_pub != nullptr) {
//					orb_publish(ORB_ID(mixer_data_request), _mixer_data_request_pub, &_mixer_data_req);
//				};

//				_send_all_state = MIXERS_SEND_ALL_MIXER_TYPE;

//				break;
//			}

//		case MIXERS_SEND_ALL_MIXER_TYPE: {
//				// Send the mixer type
//				_mixer_data_req.mixer_data_type = MIXER_DATA_TYPE_MIXTYPE;

//				_request_pending = true;

//				if (_mixer_data_request_pub != nullptr) {
//					orb_publish(ORB_ID(mixer_data_request), _mixer_data_request_pub, &_mixer_data_req);
//				}

//				_send_all_state = MIXERS_SEND_ALL_PARAMETER_COUNT;
//				break;
//			}

//		case MIXERS_SEND_ALL_PARAMETER_COUNT: {
//				//Send the parameter count for the current mixer/submixer
//				_msg_mixer_data_immediate.mixer_group = _mixer_data_req.mixer_group;
//				_msg_mixer_data_immediate.mixer_index = _mixer_data_req.mixer_index;
//				_msg_mixer_data_immediate.mixer_sub_index = _mixer_data_req.mixer_sub_index;
//				_msg_mixer_data_immediate.parameter_index = 0;
//				_msg_mixer_data_immediate.connection_type = 0;
//				_msg_mixer_data_immediate.connection_group = 0;
//				_msg_mixer_data_immediate.data_type = MIXER_DATA_TYPE_PARAMETER_COUNT;
//				_msg_mixer_data_immediate.param_type = 0;
//				_msg_mixer_data_immediate.param_value = 0.0;
//				_msg_mixer_data_immediate.data_value = _mixer_param_count;

//				_send_data_immediate = true;
//				_request_pending = false;

//				_send_all_state = MIXERS_SEND_ALL_INPUT_CONNECTIONS_COUNT;
//				break;
//			}

//		case MIXERS_SEND_ALL_INPUT_CONNECTIONS_COUNT: {
//				//Send the input connection count for the current mixer/submixer
//				_msg_mixer_data_immediate.mixer_group = _mixer_data_req.mixer_group;
//				_msg_mixer_data_immediate.mixer_index = _mixer_data_req.mixer_index;
//				_msg_mixer_data_immediate.mixer_sub_index = _mixer_data_req.mixer_sub_index;
//				_msg_mixer_data_immediate.parameter_index = 0;
//				_msg_mixer_data_immediate.connection_type = 1;
//				_msg_mixer_data_immediate.connection_group = 0;
//				_msg_mixer_data_immediate.data_type = MIXER_DATA_TYPE_CONNECTION_COUNT;
//				_msg_mixer_data_immediate.param_type = 0;
//				_msg_mixer_data_immediate.param_value = 0.0;
//				_msg_mixer_data_immediate.data_value = _mixer_param_count;

//				_send_data_immediate = true;
//				_request_pending = false;

//				_mixer_data_req.parameter_index = 0;
//				_send_all_state = MIXERS_SEND_ALL_INPUT_CONNECTIONS;
//			}

//		case MIXERS_SEND_ALL_INPUT_CONNECTIONS: {
//				if (_mixer_data_req.parameter_index < _mixer_input_count) {
//					/**Send next input connection request */
//					_mixer_data_req.mixer_data_type = MIXER_DATA_TYPE_CONNECTION;
//					_mixer_data_req.connection_type = 1;

//					if (_mixer_data_request_pub != nullptr) {
//						orb_publish(ORB_ID(mixer_data_request), _mixer_data_request_pub, &_mixer_data_req);
//					}

//					_request_pending = true;
//					_mixer_data_req.parameter_index++;

//				} else {
//					_send_all_state = MIXERS_SEND_ALL_OUTPUT_CONNECTIONS_COUNT;
//				}

//				break;
//			}

//		case MIXERS_SEND_ALL_OUTPUT_CONNECTIONS_COUNT: {
//				//Send the output connection count for the current mixer/submixer
//				_msg_mixer_data_immediate.mixer_group = _mixer_data_req.mixer_group;
//				_msg_mixer_data_immediate.mixer_index = _mixer_data_req.mixer_index;
//				_msg_mixer_data_immediate.mixer_sub_index = _mixer_data_req.mixer_sub_index;
//				_msg_mixer_data_immediate.parameter_index = 0;
//				_msg_mixer_data_immediate.connection_type = 0;
//				_msg_mixer_data_immediate.connection_group = 0;
//				_msg_mixer_data_immediate.data_type = MIXER_DATA_TYPE_CONNECTION_COUNT;
//				_msg_mixer_data_immediate.param_type = 0;
//				_msg_mixer_data_immediate.param_value = 0.0;
//				_msg_mixer_data_immediate.data_value = _mixer_param_count;

//				_send_data_immediate = true;
//				_request_pending = false;

//				_send_all_state = MIXERS_SEND_ALL_OUTPUT_CONNECTIONS;
//				break;
//			}

//		case MIXERS_SEND_ALL_OUTPUT_CONNECTIONS: {
//				if (_mixer_data_req.parameter_index < _mixer_output_count) {
//					/**Send next input connection request */
//					_mixer_data_req.mixer_data_type = MIXER_DATA_TYPE_CONNECTION;
//					_mixer_data_req.connection_type = 0;

//					if (_mixer_data_request_pub != nullptr) {
//						orb_publish(ORB_ID(mixer_data_request), _mixer_data_request_pub, &_mixer_data_req);
//					}

//					_request_pending = true;
//					_mixer_data_req.parameter_index++;

//				} else {
//					_send_all_state = MIXERS_SEND_ALL_PARAMETERS_START;
//				}

//				break;
//			}

//		case MIXERS_SEND_ALL_PARAMETERS_START: {
//				_mixer_data_req.parameter_index = 0;
//				_send_all_state = MIXERS_SEND_ALL_PARAMETERS;
//				break;
//			}

//		case MIXERS_SEND_ALL_PARAMETERS: {
//				if (_mixer_data_req.parameter_index < _mixer_param_count) {
//					_mixer_data_req.mixer_data_type = MIXER_DATA_TYPE_PARAMETER;

//					/**Send next parameter request */
//					if (_mixer_data_request_pub != nullptr) {
//						orb_publish(ORB_ID(mixer_data_request), _mixer_data_request_pub, &_mixer_data_req);
//					}

//					_request_pending = true;

//					_mixer_data_req.parameter_index++;

//				} else {
//					_mixer_data_req.parameter_index = 0;
//					_send_all_state = MIXERS_SEND_ALL_PARAMETERS_END;
//				}

//				break;
//			}

//		case MIXERS_SEND_ALL_PARAMETERS_END: {
//				_mixer_data_req.mixer_sub_index++;

//				if (_mixer_data_req.mixer_sub_index > _mixer_sub_count) {   /**Check if submixer index has exceeded count*/
//					_mixer_data_req.mixer_sub_index = 0;
//					_mixer_data_req.parameter_index = 0;
//					_mixer_data_req.mixer_index++;

//				} else {
//					_send_all_state = MIXERS_SEND_ALL_MIXER_TYPE;
//				}

//				if (_mixer_data_req.mixer_index < _mixer_count) {           /**Check if mixer index has exceeded count*/
//					_send_all_state = MIXERS_SEND_ALL_SUBMIXER_COUNT;

//				} else {
//					_send_all_state = MIXERS_SEND_ALL_NONE;
//					PX4_INFO("MIXER send all - finished");                  /**Stop here*/
//				}

//				break;
//			}

//		default:
//			_send_all_state = MIXERS_SEND_ALL_NONE;
//			_mavlink_mixer_state = MAVLINK_MIXER_STATE_WAITING;
//			break;
//		}

//	}

}
#endif //MIXER_TUNING
