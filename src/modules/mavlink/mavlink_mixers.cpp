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
#include <sys/stat.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>

#include <uORB/topics/mixer_parameter.h>
#include <uORB/topics/mixer_parameter_set.h>

#include <systemlib/mixer/mixer_parameters.h>
#include <systemlib/mixer/mixer_io.h>
#include <drivers/drv_mixer.h>

#define MOUNTPOINT PX4_ROOTFSDIR "/fs/microsd"

static const char *kMixerLocalData    = MOUNTPOINT "/mixer.local.mix";
static const char *kMixerFailsafeData = MOUNTPOINT "/mixer.failsafe.mix";
static const char *kMixerDefaultData = MOUNTPOINT "/mixer.default.mix";

static const unsigned mixer_parameter_count[MIXER_PARAMETERS_MIXER_TYPE_COUNT] = MIXER_PARAMETER_COUNTS;
static const unsigned mixer_input_count[MIXER_IO_MIXER_TYPE_COUNT] = MIXER_INPUT_COUNTS;
static const unsigned mixer_output_count[MIXER_IO_MIXER_TYPE_COUNT] = MIXER_OUTPUT_COUNTS;

MavlinkMixersManager::MavlinkMixersManager(Mavlink *mavlink) : MavlinkStream(mavlink),
	_request_pending(false),
	_send_all_state(MIXERS_SEND_ALL_NONE),
	_send_data_immediate(false),
	_mixer_count(0),
	_mixer_sub_count(0),
	_mixer_type(0),
	_mixer_param_count(0),
	_mixer_input_count(0),
	_mixer_output_count(0),
	_p_mixer_save_buffer(nullptr),
	_mixer_parameter_set_pub(nullptr),
	_mixer_parameter_sub(-1),
	_has_checked_px4io(false)
{
	_interval = 100000;
}
MavlinkMixersManager::~MavlinkMixersManager()
{
	if (_mixer_parameter_sub >= 0) {
		orb_unsubscribe(_mixer_parameter_sub);
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

int16_t
MavlinkMixersManager::open_group_as_device(uint16_t group)
{
	int ret = -1;

	if (!_has_checked_px4io) {
		struct stat st;
		_has_px4io = (stat("/dev/px4io", &st) == 0);

		if (_has_px4io) {
			PX4_INFO("Found px4io at /dev/px4io");
		}

		_has_checked_px4io = true;
	}

	if (_has_px4io) {
		switch (group) {
		case 0:
			ret = px4_open("/dev/pwm_output1", 0);

			if (ret == -1) {
				PX4_WARN("Mavlink mixers could not open device /dev/pwm_output1 for group 0");
			}

			break;

		case 1:
			ret = px4_open("/dev/px4io", 0);

			if (ret == -1) {
				PX4_WARN("Mavlink mixers could not open device /dev/px4io for group 1");
			}

			break;
		}

	} else {
		if (group == 0) {
			ret = px4_open("/dev/pwm_output0", 0);

			if (ret == -1) {
				PX4_WARN("Mavlink mixers could not open device /dev/pwm_output0 for group 0");
			}
		}
	}

	return ret;
}

void
MavlinkMixersManager::handle_message(const mavlink_message_t *msg)
{
	if (msg->msgid != MAVLINK_MSG_ID_COMMAND_LONG) { return; }

	mavlink_command_long_t cmd;
	mavlink_msg_command_long_decode(msg, &cmd);

	switch (cmd.command) {
	case MAV_CMD_REQUEST_MIXER_DATA: {
			PX4_INFO("Received mixer data request");
			_msg_mixer_data_immediate.mixer_group = cmd.param1;
			_msg_mixer_data_immediate.mixer_index = cmd.param2;
			_msg_mixer_data_immediate.mixer_sub_index = cmd.param3;
			_msg_mixer_data_immediate.parameter_index = cmd.param4;
			_msg_mixer_data_immediate.data_type = cmd.param5;
			_msg_mixer_data_immediate.connection_type = cmd.param6;
			_msg_mixer_data_immediate.connection_group = 0;

			_msg_mixer_data_immediate.param_value = 0.0;
			_msg_mixer_data_immediate.param_type = 0;
			_msg_mixer_data_immediate.data_value = 0;
			_send_all_state = MIXERS_SEND_ALL_NONE;

			int dev;
			dev = open_group_as_device(_msg_mixer_data_immediate.mixer_group);

			if (dev < 0) {
				_msg_mixer_data_immediate.data_value = -1;
				_msg_mixer_data_immediate.param_value = 0.0;
				_send_data_immediate = true;
				return;
			}

			switch (_msg_mixer_data_immediate.data_type) {
			case MIXER_DATA_TYPE_MIXER_COUNT: {
					PX4_INFO("Received mixer count request");
					int mix_count;
					int ret = px4_ioctl(dev, MIXERIOCGETMIXERCOUNT, (unsigned long)&mix_count);
					px4_close(dev);
					_msg_mixer_data_immediate.param_value = 0.0;

					if (ret < 0) {
						_msg_mixer_data_immediate.data_value = ret;

					} else {
						_msg_mixer_data_immediate.data_value = mix_count;
					}

					_send_data_immediate = true;
					break;
				}

			case MIXER_DATA_TYPE_SUBMIXER_COUNT: {
					int mix_count = _msg_mixer_data_immediate.mixer_index;
					int ret = px4_ioctl(dev, MIXERIOCGETSUBMIXERCOUNT, (unsigned long)&mix_count);
					px4_close(dev);
					_msg_mixer_data_immediate.param_value = 0.0;

					if (ret < 0) {
						_msg_mixer_data_immediate.data_value = ret;

					} else {
						_msg_mixer_data_immediate.data_value = mix_count;
					}

//                    PX4_INFO("MAVlink reqeust for submixer count. group:%u mixer:%u submixer_count:%u",
//                             _msg_mixer_data_immediate.mixer_group,
//                             _msg_mixer_data_immediate.mixer_index,
//                             _msg_mixer_data_immediate.data_value);

					_send_data_immediate = true;
					break;
				}

			case MIXER_DATA_TYPE_MIXTYPE: {
					mixer_type_s type;
					type.mix_index = _msg_mixer_data_immediate.mixer_index;
					type.mix_sub_index = _msg_mixer_data_immediate.mixer_sub_index;
					int ret = px4_ioctl(dev, MIXERIOCGETTYPE, (unsigned long)&type);
					px4_close(dev);
					_msg_mixer_data_immediate.param_value = 0.0;

					if (ret < 0) {
						_msg_mixer_data_immediate.data_value = ret;

					} else {
						_msg_mixer_data_immediate.data_value = type.mix_type;
					}

					_send_data_immediate = true;
					break;
				}

			case MIXER_DATA_TYPE_PARAMETER: {
					mixer_param_s param;
					param.mix_index = _msg_mixer_data_immediate.mixer_index;
					param.mix_sub_index = _msg_mixer_data_immediate.mixer_sub_index;
					param.param_index = _msg_mixer_data_immediate.parameter_index;
					int ret = px4_ioctl(dev, MIXERIOCGETPARAM, (unsigned long)&param);
					px4_close(dev);
					_msg_mixer_data_immediate.param_value = 0.0;
					_msg_mixer_data_immediate.data_value = 0;

					if (ret < 0) {
						_msg_mixer_data_immediate.data_value = ret;

					} else {
						_msg_mixer_data_immediate.param_type = MAV_PARAM_TYPE_REAL32;
						_msg_mixer_data_immediate.param_value = param.value;
					}

					_send_data_immediate = true;
					break;
				}

			case MIXER_DATA_TYPE_PARAMETER_COUNT: {
					mixer_type_s type;
					type.mix_index = _msg_mixer_data_immediate.mixer_index;
					type.mix_sub_index = _msg_mixer_data_immediate.mixer_sub_index;
					int ret = px4_ioctl(dev, MIXERIOCGETTYPE, (unsigned long)&type);
					px4_close(dev);
					_msg_mixer_data_immediate.param_value = 0.0;

					if (ret < 0) {
						_msg_mixer_data_immediate.data_value = ret;

					} else {
						_msg_mixer_data_immediate.data_value = mixer_parameter_count[type.mix_type];
					}

					_send_data_immediate = true;
					break;
				}

			case MIXER_DATA_TYPE_CONNECTION_COUNT: {
					mixer_type_s type;
					type.mix_index = _msg_mixer_data_immediate.mixer_index;
					type.mix_sub_index = _msg_mixer_data_immediate.mixer_sub_index;
					int ret = px4_ioctl(dev, MIXERIOCGETTYPE, (unsigned long)&type);
					px4_close(dev);
					_msg_mixer_data_immediate.param_value = 0.0;

					if (ret < 0) {
						_msg_mixer_data_immediate.data_value = ret;

					} else {
						if (_msg_mixer_data_immediate.connection_type == 0) {
							_msg_mixer_data_immediate.data_value = mixer_output_count[type.mix_type];

						} else {
							_msg_mixer_data_immediate.data_value = mixer_input_count[type.mix_type];
						}
					}

					_send_data_immediate = true;
					break;
				}

			// Not supported or not yet supported
			case MIXER_DATA_TYPE_CONNECTION:

			default:
				return;
			}

			break;
		}

	case MAV_CMD_REQUEST_MIXER_STORE: {
			_msg_mixer_data_immediate.mixer_group = cmd.param1;
			_msg_mixer_data_immediate.mixer_index = 0;
			_msg_mixer_data_immediate.mixer_sub_index = 0;
			_msg_mixer_data_immediate.parameter_index = 0;
			_msg_mixer_data_immediate.data_type = 122;      //Code for store!
			_msg_mixer_data_immediate.connection_type = 0;
			_msg_mixer_data_immediate.connection_group = 0;

			int dev;
			dev = open_group_as_device(_msg_mixer_data_immediate.mixer_group);

			if (dev < 0) {
				_msg_mixer_data_immediate.data_value = -1;
				_send_data_immediate = true;
				return;
			}

			if (_p_mixer_save_buffer == nullptr) {
				_p_mixer_save_buffer = (char *) malloc(1024);
			}

			mixer_config_s config = {_p_mixer_save_buffer, 1024};

			int ret = px4_ioctl(dev, MIXERIOCGETCONFIG, (unsigned long)&config);
			px4_close(dev);

			if (ret < 0) {
				_msg_mixer_data_immediate.data_value = ret;
				_send_data_immediate = true;
				free(_p_mixer_save_buffer);
				_p_mixer_save_buffer = nullptr;
				return;
			}

			const char *fname;

			switch (_msg_mixer_data_immediate.mixer_group) {
			case 0:
				fname = kMixerLocalData;
				break;

			case 1:
				fname = kMixerFailsafeData;
				break;

			default:
				fname = kMixerDefaultData;
				break;
			}

			/* Create the mixer definition file */
			FILE *f = ::fopen(fname, "w");

			if (f == nullptr) {
				PX4_ERR("not able to create mixer file %s", fname);
				_msg_mixer_data_immediate.data_value = ret;
				_send_data_immediate = true;
				free(_p_mixer_save_buffer);
				_p_mixer_save_buffer = nullptr;
				return;
			}

			/* Write the buffer to the file*/
			signed err = fputs(_p_mixer_save_buffer, f);
			fclose(f);

			free(_p_mixer_save_buffer);
			_p_mixer_save_buffer = nullptr;

			if (err < 0) {
				PX4_ERR("Mixer file write error: %s ", fname);
				_msg_mixer_data_immediate.data_value = ret;
				_send_data_immediate = true;
				return;
			}

			PX4_INFO("Wrote mixer group:%u to file %s\n", _msg_mixer_data_immediate.mixer_group, fname);

#ifdef __PX4_NUTTX
			fsync(fileno(f));
#endif
			_msg_mixer_data_immediate.data_value = config.size;
			_send_data_immediate = true;
			break;
		}


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
		_msg_mixer_data_immediate.mixer_group = cmd.param1;
		PX4_INFO("Mavlink request for mixer send all for group:%u", _msg_mixer_data_immediate.mixer_group);
		_send_all_state = MIXERS_SEND_ALL_START;
		break;

	case MAV_CMD_REQUEST_MIXER_CONN: {
			_msg_mixer_data_immediate.mixer_group = cmd.param1;
			_msg_mixer_data_immediate.mixer_index = cmd.param2;
			_msg_mixer_data_immediate.mixer_sub_index = cmd.param3;
			_msg_mixer_data_immediate.parameter_index = cmd.param5;
			_msg_mixer_data_immediate.connection_type = cmd.param4;
			_msg_mixer_data_immediate.data_type = MIXER_DATA_TYPE_CONNECTION;
			_send_all_state = MIXERS_SEND_ALL_NONE;

			int dev = open_group_as_device(_msg_mixer_data_immediate.mixer_group);

			if (dev < 0) {
				_msg_mixer_data_immediate.data_value = -1;
				_msg_mixer_data_immediate.param_value = 0.0;
				_send_data_immediate = true;
				return;
			}

			mixer_connection_s conn;
			conn.mix_index = _msg_mixer_data_immediate.mixer_index;
			conn.mix_sub_index = _msg_mixer_data_immediate.mixer_sub_index;
			conn.connection_index = _msg_mixer_data_immediate.parameter_index;
			conn.connection_type = _msg_mixer_data_immediate.connection_type;

			int ret = px4_ioctl(dev, MIXERIOCGETIOCONNECTION, (unsigned long)&conn);
			px4_close(dev);
			_msg_mixer_data_immediate.data_value = 0;

			if (ret < 0) {
				_msg_mixer_data_immediate.data_value = ret;

			} else {
				_msg_mixer_data_immediate.data_value = conn.connection;
				_msg_mixer_data_immediate.connection_group = conn.connection_group;
			}

			_send_data_immediate = true;

			break;
		}

	default:
		return;
	}

}


void
MavlinkMixersManager::send(const hrt_abstime t)
{
	bool space_available = _mavlink->get_free_tx_buf() >= get_size();

	if (!space_available) { return; }

	if (_send_data_immediate) {
		/* Send with default component ID */
		mavlink_msg_mixer_data_send_struct(_mavlink->get_channel(), &_msg_mixer_data_immediate);
		_send_data_immediate = false;
		PX4_INFO("Sent immediate mixer data");
		return;
	}

	// If not waiting for anything and send all is active, prepare the next item
	if (!_send_data_immediate && !_request_pending && _send_all_state != MIXERS_SEND_ALL_NONE) {
		int dev = open_group_as_device(_msg_mixer_data_immediate.mixer_group);

		if (dev < 0) {
			_send_all_state = MIXERS_SEND_ALL_NONE;
			_send_data_immediate = false;
			return;
		}

		switch (int(_send_all_state)) {
		case MIXERS_SEND_ALL_START: {
				//Reset all values to the start and get the mixer count
				_msg_mixer_data_immediate.mixer_index = 0;
				_msg_mixer_data_immediate.mixer_sub_index = 0;
				_msg_mixer_data_immediate.parameter_index = 0;
				_msg_mixer_data_immediate.connection_type = 0;
				_msg_mixer_data_immediate.data_type = MIXER_DATA_TYPE_MIXER_COUNT;

				int mix_count;
				int ret = px4_ioctl(dev, MIXERIOCGETMIXERCOUNT, (unsigned long)&mix_count);
				px4_close(dev);
				dev = -1;

				if (ret < 0) {
					_msg_mixer_data_immediate.data_value = ret;
					_send_all_state = MIXERS_SEND_ALL_NONE;
					_mixer_count = 0;

				} else {
					_msg_mixer_data_immediate.data_value = mix_count;
					_mixer_count = mix_count;
					_send_all_state = MIXERS_SEND_ALL_SUBMIXER_COUNT;
				}

				mavlink_msg_mixer_data_send_struct(_mavlink->get_channel(), &_msg_mixer_data_immediate);
				break;
			}

		case MIXERS_SEND_ALL_SUBMIXER_COUNT: {
				_msg_mixer_data_immediate.parameter_index = 0;
				_msg_mixer_data_immediate.connection_type = 0;
				_msg_mixer_data_immediate.param_value = 0.0;
				_msg_mixer_data_immediate.data_type = MIXER_DATA_TYPE_SUBMIXER_COUNT;

				int mix_count = _msg_mixer_data_immediate.mixer_index;
				int ret = px4_ioctl(dev, MIXERIOCGETSUBMIXERCOUNT, (unsigned long)&mix_count);
				px4_close(dev);
				dev = -1;

				_msg_mixer_data_immediate.param_value = 0.0;

				if (ret < 0) {
					_msg_mixer_data_immediate.data_value = ret;
					_mixer_sub_count = 0;
					_send_all_state = MIXERS_SEND_ALL_NONE;

				} else {
					_msg_mixer_data_immediate.data_value = mix_count;
					_mixer_sub_count = mix_count;
					_send_all_state = MIXERS_SEND_ALL_MIXER_TYPE;
				}

				mavlink_msg_mixer_data_send_struct(_mavlink->get_channel(), &_msg_mixer_data_immediate);
				break;
			}

		case MIXERS_SEND_ALL_MIXER_TYPE: {
				_msg_mixer_data_immediate.data_type = MIXER_DATA_TYPE_MIXTYPE;

				mixer_type_s type;
				type.mix_index = _msg_mixer_data_immediate.mixer_index;
				type.mix_sub_index = _msg_mixer_data_immediate.mixer_sub_index;

				int ret = px4_ioctl(dev, MIXERIOCGETTYPE, (unsigned long)&type);
				px4_close(dev);
				dev = -1;

				_msg_mixer_data_immediate.param_value = 0.0;

				if (ret < 0) {
					_msg_mixer_data_immediate.data_value = ret;
					_send_all_state = MIXERS_SEND_ALL_NONE;
					_mixer_type = -1;

				} else {
					_msg_mixer_data_immediate.data_value = type.mix_type;

					//Collect mixer metadata for later
					_mixer_type = type.mix_type;
					_mixer_param_count = mixer_parameter_count[_mixer_type];
					_mixer_input_count = mixer_input_count[_mixer_type];
					_mixer_output_count = mixer_output_count[_mixer_type];

					_send_all_state = MIXERS_SEND_ALL_PARAMETER_COUNT;
				}

				mavlink_msg_mixer_data_send_struct(_mavlink->get_channel(), &_msg_mixer_data_immediate);
				break;
			}


		case MIXERS_SEND_ALL_PARAMETER_COUNT: {
				//Send the parameter count for the current mixer/submixer
				_msg_mixer_data_immediate.parameter_index = 0;
				_msg_mixer_data_immediate.connection_type = 0;
				_msg_mixer_data_immediate.connection_group = 0;
				_msg_mixer_data_immediate.data_type = MIXER_DATA_TYPE_PARAMETER_COUNT;
				_msg_mixer_data_immediate.param_type = 0;
				_msg_mixer_data_immediate.param_value = 0.0;
				_msg_mixer_data_immediate.data_value = _mixer_param_count;

				mavlink_msg_mixer_data_send_struct(_mavlink->get_channel(), &_msg_mixer_data_immediate);
				_send_all_state = MIXERS_SEND_ALL_INPUT_CONNECTIONS_COUNT;
				break;
			}

		case MIXERS_SEND_ALL_INPUT_CONNECTIONS_COUNT: {
				//Send the input connection count for the current mixer/submixer
				_msg_mixer_data_immediate.parameter_index = 0;
				_msg_mixer_data_immediate.connection_type = 1;
				_msg_mixer_data_immediate.connection_group = 0;
				_msg_mixer_data_immediate.data_type = MIXER_DATA_TYPE_CONNECTION_COUNT;
				_msg_mixer_data_immediate.param_type = 0;
				_msg_mixer_data_immediate.param_value = 0.0;
				_msg_mixer_data_immediate.data_value = _mixer_input_count;

				mavlink_msg_mixer_data_send_struct(_mavlink->get_channel(), &_msg_mixer_data_immediate);
				_msg_mixer_data_immediate.parameter_index = 0;
				_send_all_state = MIXERS_SEND_ALL_INPUT_CONNECTIONS;
				break;
			}

		case MIXERS_SEND_ALL_INPUT_CONNECTIONS: {
				if (_msg_mixer_data_immediate.parameter_index < _mixer_input_count) {
					/**Send next input connection request */
					_msg_mixer_data_immediate.data_type = MIXER_DATA_TYPE_CONNECTION;
					_msg_mixer_data_immediate.connection_type = 1;

					mixer_connection_s conn;
					conn.mix_index = _msg_mixer_data_immediate.mixer_index;
					conn.mix_sub_index = _msg_mixer_data_immediate.mixer_sub_index;
					conn.connection_index = _msg_mixer_data_immediate.parameter_index;
					conn.connection_type = 1;

					int ret = px4_ioctl(dev, MIXERIOCGETIOCONNECTION, (unsigned long)&conn);
					px4_close(dev);
					dev = -1;

					if (ret < 0) {
						_msg_mixer_data_immediate.data_value = ret;
						_send_all_state = MIXERS_SEND_ALL_NONE;

					} else {
						_msg_mixer_data_immediate.data_value = conn.connection;
						_msg_mixer_data_immediate.connection_group = conn.connection_group;
					}

					mavlink_msg_mixer_data_send_struct(_mavlink->get_channel(), &_msg_mixer_data_immediate);
					_msg_mixer_data_immediate.parameter_index++;
					//_send_all_state = MIXERS_SEND_ALL_INPUT_CONNECTIONS;      //Stay in same state

				} else {
					_send_all_state = MIXERS_SEND_ALL_OUTPUT_CONNECTIONS_COUNT;
				}

				break;
			}

		case MIXERS_SEND_ALL_OUTPUT_CONNECTIONS_COUNT: {
				//Send the input connection count for the current mixer/submixer
				_msg_mixer_data_immediate.parameter_index = 0;
				_msg_mixer_data_immediate.connection_type = 0;
				_msg_mixer_data_immediate.connection_group = 0;
				_msg_mixer_data_immediate.data_type = MIXER_DATA_TYPE_CONNECTION_COUNT;
				_msg_mixer_data_immediate.param_type = 0;
				_msg_mixer_data_immediate.param_value = 0.0;
				_msg_mixer_data_immediate.data_value = _mixer_output_count;

				mavlink_msg_mixer_data_send_struct(_mavlink->get_channel(), &_msg_mixer_data_immediate);

				_msg_mixer_data_immediate.parameter_index = 0;
				_send_all_state = MIXERS_SEND_ALL_OUTPUT_CONNECTIONS;
				break;
			}


		case MIXERS_SEND_ALL_OUTPUT_CONNECTIONS: {
				if (_msg_mixer_data_immediate.parameter_index < _mixer_output_count) {
					/**Send next input connection request */
					_msg_mixer_data_immediate.data_type = MIXER_DATA_TYPE_CONNECTION;
					_msg_mixer_data_immediate.connection_type = 0;

					mixer_connection_s conn;
					conn.mix_index = _msg_mixer_data_immediate.mixer_index;
					conn.mix_sub_index = _msg_mixer_data_immediate.mixer_sub_index;
					conn.connection_index = _msg_mixer_data_immediate.parameter_index;
					conn.connection_type = 0;

					int ret = px4_ioctl(dev, MIXERIOCGETIOCONNECTION, (unsigned long)&conn);
					px4_close(dev);
					dev = -1;

					if (ret < 0) {
						_msg_mixer_data_immediate.data_value = ret;
						_send_all_state = MIXERS_SEND_ALL_NONE;

					} else {
						_msg_mixer_data_immediate.data_value = conn.connection;
						_msg_mixer_data_immediate.connection_group = conn.connection_group;
					}

					mavlink_msg_mixer_data_send_struct(_mavlink->get_channel(), &_msg_mixer_data_immediate);
					_msg_mixer_data_immediate.parameter_index++;

				} else {
					_send_all_state = MIXERS_SEND_ALL_PARAMETERS_START;
				}

				break;
			}

		case MIXERS_SEND_ALL_PARAMETERS_START: {
				_msg_mixer_data_immediate.parameter_index = 0;
				_send_all_state = MIXERS_SEND_ALL_PARAMETERS;
				break;
			}

		case MIXERS_SEND_ALL_PARAMETERS: {
				if (_msg_mixer_data_immediate.parameter_index < _mixer_param_count) {
					_msg_mixer_data_immediate.data_type = MIXER_DATA_TYPE_PARAMETER;

					mixer_param_s param;
					param.mix_index = _msg_mixer_data_immediate.mixer_index;
					param.mix_sub_index = _msg_mixer_data_immediate.mixer_sub_index;
					param.param_index = _msg_mixer_data_immediate.parameter_index;

					int ret = px4_ioctl(dev, MIXERIOCGETPARAM, (unsigned long)&param);
					px4_close(dev);
					dev = -1;

					if (ret < 0) {
						_msg_mixer_data_immediate.param_type = 0;
						_msg_mixer_data_immediate.param_value = 0.0;
						_msg_mixer_data_immediate.data_value = ret;
						_send_all_state = MIXERS_SEND_ALL_NONE;

					} else {
						_msg_mixer_data_immediate.param_type = MAV_PARAM_TYPE_REAL32;
						_msg_mixer_data_immediate.param_value = param.value;
						_msg_mixer_data_immediate.data_value = 0;
						//_send_all_state = MIXERS_SEND_ALL_PARAMETERS;     //Stay in the same state
					}

					mavlink_msg_mixer_data_send_struct(_mavlink->get_channel(), &_msg_mixer_data_immediate);
					_msg_mixer_data_immediate.parameter_index++;

				} else {
					_send_all_state = MIXERS_SEND_ALL_NEXT_MIXER;
				}

				break;
			}


		case MIXERS_SEND_ALL_NEXT_MIXER: {
				_msg_mixer_data_immediate.mixer_sub_index++;

				if (_msg_mixer_data_immediate.mixer_sub_index > _mixer_sub_count) {   /**Check if submixer index has exceeded count*/
					_msg_mixer_data_immediate.mixer_sub_index = 0;
					_msg_mixer_data_immediate.parameter_index = 0;
					_msg_mixer_data_immediate.mixer_index++;

				} else {
					_send_all_state = MIXERS_SEND_ALL_MIXER_TYPE;
				}

				if (_msg_mixer_data_immediate.mixer_index < _mixer_count) {           /**Check if mixer index has exceeded count*/
					_send_all_state = MIXERS_SEND_ALL_SUBMIXER_COUNT;

				} else {
					_send_all_state = MIXERS_SEND_ALL_NONE;
					PX4_INFO("MIXER send all - finished");                           /**Stop here*/
				}

				break;
			}

		default:
			_send_all_state = MIXERS_SEND_ALL_NONE;
			break;
		}

		if (dev != -1) {
			px4_close(dev);
		}

	} //if sending all



	/* Send parameter values received from mixer parameter topic */
	if (_mixer_parameter_sub < 0) {
		_mixer_parameter_sub = orb_subscribe(ORB_ID(mixer_parameter));
	}

	bool mixer_parameter_ready;
	orb_check(_mixer_parameter_sub, &mixer_parameter_ready);

	if (mixer_parameter_ready && _request_pending) {
		struct mixer_parameter_s parameter;
		orb_copy(ORB_ID(mixer_parameter), _mixer_parameter_sub, &parameter);

		_request_pending = false;

		mavlink_mixer_data_t msg;
		msg.mixer_group = parameter.mixer_group;
		msg.mixer_index = parameter.mixer_index;
		msg.mixer_sub_index = parameter.mixer_sub_index;
		msg.parameter_index = parameter.parameter_index;
		msg.connection_type = 0;
		msg.connection_group = 0;
		msg.data_type = MIXER_DATA_TYPE_PARAMETER;
		msg.data_value = 0;

		if (parameter.param_type == MAV_PARAM_TYPE_REAL32) {
			msg.param_type = MAVLINK_TYPE_FLOAT;
			msg.param_value = parameter.real_value;

		} else {
			int32_t val;
			val = (int32_t)parameter.int_value;
			memcpy(&msg.param_value, &val, sizeof(int32_t));
			msg.param_type = MAVLINK_TYPE_INT32_T;
		}

		/* Send with default component ID */
		mavlink_msg_mixer_data_send_struct(_mavlink->get_channel(), &msg);
	}

}
#endif //MIXER_TUNING
