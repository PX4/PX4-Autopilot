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

#include <systemlib/mixer/mixer_types.h>
#include <drivers/drv_mixer.h>

#define MOUNTPOINT PX4_ROOTFSDIR "/fs/microsd"
static const char *kMixerLocalData    = MOUNTPOINT "/mixer.local.mix";
static const char *kMixerFailsafeData = MOUNTPOINT "/mixer.failsafe.mix";
static const char *kMixerDefaultData = MOUNTPOINT "/mixer.default.mix";

MavlinkMixersManager::MavlinkMixersManager(Mavlink *mavlink)
	: MavlinkStream(mavlink)
	, _send_state(MIXERS_SEND_STATE_NONE)
	, _p_mixer_save_buffer(nullptr)
	, _has_checked_px4io(false)
	, _param_count(-1)
{
	_interval = 100000;
}

MavlinkMixersManager::~MavlinkMixersManager()
{
}

unsigned
MavlinkMixersManager::get_size()
{
	return MAVLINK_MSG_ID_MIXER_PARAM_VALUE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
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
		case MIXER_GROUP_LOCAL:
			ret = px4_open("/dev/pwm_output1", 0);

			if (ret == -1) {
				PX4_WARN("Mavlink mixers could not open device /dev/pwm_output1 for group 0");
			}

			break;

		case MIXER_GROUP_FAILSAFE:
			ret = px4_open("/dev/px4io", 0);

			if (ret == -1) {
				PX4_WARN("Mavlink mixers could not open device /dev/px4io for group 1");
			}

			break;
		}

	} else {
		if (group == MIXER_GROUP_LOCAL) {
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
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_COMMAND_LONG: {
			mavlink_command_long_t cmd;
			mavlink_msg_command_long_decode(msg, &cmd);

			switch (cmd.command) {

			case MAV_CMD_REQUEST_MIXER_PARAM_LIST: {
					PX4_INFO("MAV CMD MIXER_PARAM_LIST");
					_msg_mixer_parameter.mixer_group = cmd.param1;
					_msg_mixer_parameter.index = 0;
					PX4_INFO("Mavlink request for mixer send all for group:%u", _msg_mixer_parameter.mixer_group);
					_send_state = MIXERS_SEND_STATE_ALL_PARAMETERS_START;
					break;
				}


			case MAV_CMD_SAVE_MIXER_PARAMS: {
					int dev;
					dev = open_group_as_device(_msg_mixer_parameter.mixer_group);

					if (dev < 0) {
						return;
					}

					if (_p_mixer_save_buffer == nullptr) {
						_p_mixer_save_buffer = (char *) malloc(1024);
					}

					mixer_config_s config = {_p_mixer_save_buffer, 1024};

					int ret = px4_ioctl(dev, MIXERIOCGETCONFIG, (unsigned long)&config);
					px4_close(dev);

					if (ret < 0) {
						free(_p_mixer_save_buffer);
						_p_mixer_save_buffer = nullptr;
						return;
					}

					const char *fname;

					switch (_msg_mixer_parameter.mixer_group) {
					case 1:
						fname = kMixerLocalData;
						break;

					case 2:
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
						free(_p_mixer_save_buffer);
						_p_mixer_save_buffer = nullptr;
						return;
					}

					/* Write the buffer to the file*/
					signed err = fputs(_p_mixer_save_buffer, f);
					fclose(f);

#ifdef __PX4_NUTTX
					fsync(fileno(f));
#endif
					free(_p_mixer_save_buffer);
					_p_mixer_save_buffer = nullptr;

					if (err < 0) {
						PX4_ERR("Mixer file write error: %s ", fname);
						return;
					}

					PX4_INFO("Wrote mixer group:%u to file %s\n", _msg_mixer_parameter.mixer_group, fname);

					break;
				}


			default:
				return;
			}

			break;
		}

	case MAVLINK_MSG_ID_MIXER_PARAM_REQUEST_READ: {
			mavlink_mixer_param_request_read_t request;
			mavlink_msg_mixer_param_request_read_decode(msg, &request);

			_msg_mixer_parameter.mixer_group    = request.mixer_group;
			_msg_mixer_parameter.index          = request.index;

			int dev;
			dev = open_group_as_device(_msg_mixer_parameter.mixer_group);

			if (dev < 0) {
				_msg_mixer_parameter.count = -1;
				return;
			}

			if (_param_count == -1) {
				int ret = px4_ioctl(dev, MIXERIOCGETPARAMCOUNT, (unsigned long)&_param_count);
				px4_close(dev);

				if (ret < 0) {
					_param_count = -1;
					_send_state = MIXERS_SEND_STATE_NONE;
					return;
				}
			}

			mixer_param_s param;
			param.index =  _msg_mixer_parameter.index;

			int ret = px4_ioctl(dev, MIXERIOCGETPARAM, (unsigned long)&param);
			px4_close(dev);

			if (ret < 0) {
				_msg_mixer_parameter.count = -1;

			} else {
				_msg_mixer_parameter.index = param.index;
				_msg_mixer_parameter.mixer_index = param.mix_index;
				_msg_mixer_parameter.mixer_sub_index = param.mix_sub_index;
				_msg_mixer_parameter.parameter_index = param.param_index;
				_msg_mixer_parameter.param_array_size = param.array_size;
				_msg_mixer_parameter.mixer_type = param.mix_type;
				_msg_mixer_parameter.count = _param_count;
				_msg_mixer_parameter.param_type = MAV_PARAM_TYPE_REAL32;
				_msg_mixer_parameter.flags = param.flags;
				strncpy(_msg_mixer_parameter.param_id, param.name, 16);
				memcpy(_msg_mixer_parameter.param_values, param.values, sizeof(_msg_mixer_parameter.param_values));
			}

			_send_state = MIXERS_SEND_STATE_PARAMETER;
			break;
		}

	case MAVLINK_MSG_ID_MIXER_PARAM_SET: {
			mavlink_mixer_param_set_t set;
			mavlink_msg_mixer_param_set_decode(msg, &set);

//			PX4_INFO("Mixer set parameter group:%u index:%u",
//				 set.mixer_group,
//				 set.index,
//				 (double) set.param_values[0]);

			mixer_param_s param;
			int dev;

			// Set main index to value but set all other refs to -1
			// This indicates that those refs should not be used.
			param.index = set.index;
			param.mix_index = -1;
			param.mix_sub_index = -1;
			param.param_index = -1;
			param.param_type = set.param_type;
			param.mix_type = MIXER_TYPES_NONE;
			strcpy(param.name, "");

			//Clear value to zero and only use first value.  px4 mixers specifc
			memset(param.values, 0, sizeof(param.values));
			memcpy(param.values, set.param_values, sizeof(param.values));

			_msg_mixer_parameter.count = _param_count;

			dev = open_group_as_device(set.mixer_group);

			if (dev < 0) {
				_msg_mixer_parameter.count = -1;
				return;
			}

			int ret = px4_ioctl(dev, MIXERIOCSETPARAM, (unsigned long)&param);

			if (ret < 0) {
				PX4_WARN("fail to set mixer parameter");
			}

			//Read back the parameter and request to send it
			ret = px4_ioctl(dev, MIXERIOCGETPARAM, (unsigned long)&param);
			px4_close(dev);

			if (ret < 0) {
				PX4_WARN("fail to read mixer parameter after setting");
				_msg_mixer_parameter.count = -1;
				return;
			}

			_msg_mixer_parameter.index = param.index;
			_msg_mixer_parameter.mixer_index = param.mix_index;
			_msg_mixer_parameter.mixer_sub_index = param.mix_sub_index;
			_msg_mixer_parameter.parameter_index = param.param_index;
			_msg_mixer_parameter.param_array_size = param.array_size;
			_msg_mixer_parameter.mixer_type = param.mix_type;
			_msg_mixer_parameter.param_type = MAV_PARAM_TYPE_REAL32;
			_msg_mixer_parameter.flags = param.flags;
			strncpy(_msg_mixer_parameter.param_id, param.name, 16);
			memcpy(_msg_mixer_parameter.param_values, param.values, sizeof(_msg_mixer_parameter.param_values));

			_send_state = MIXERS_SEND_STATE_PARAMETER;

//			PX4_INFO("mixer param index:%u value:%.4f set success\n", param.index, param.mix_sub_index,
//				 param.param_index,
//				 (double) param.values[0]);
			return;
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

	if (!space_available) { return; }

	switch (_send_state) {
	case MIXERS_SEND_STATE_PARAMETER: {
			mavlink_msg_mixer_param_value_send_struct(_mavlink->get_channel(), &_msg_mixer_parameter);
			_send_state = MIXERS_SEND_STATE_NONE;
			break;
		}

	case MIXERS_SEND_STATE_ALL_PARAMETERS_START: {
			int dev = open_group_as_device(_msg_mixer_parameter.mixer_group);

			if (dev < 0) {
				_send_state = MIXERS_SEND_STATE_NONE;
				return;
			}

			int ret = px4_ioctl(dev, MIXERIOCGETPARAMCOUNT, (unsigned long)&_param_count);
			px4_close(dev);

			if (ret < 0) {
				_param_count = -1;
				_send_state = MIXERS_SEND_STATE_NONE;
				return;
			}

			_msg_mixer_parameter.index = 0;
			_send_state = MIXERS_SEND_STATE_ALL_PARAMETERS;

			break;
		}

	case MIXERS_SEND_STATE_ALL_PARAMETERS: {
			int dev = open_group_as_device(_msg_mixer_parameter.mixer_group);

			if (dev < 0) {
				_send_state = MIXERS_SEND_STATE_NONE;
				return;
			}

			_msg_mixer_parameter.count = _param_count;

			mixer_param_s param;
			param.index =  _msg_mixer_parameter.index;
			memset(param.values, 0, sizeof(param.values));

			int ret = px4_ioctl(dev, MIXERIOCGETPARAM, (unsigned long)&param);
			px4_close(dev);

			if (ret < 0) {
				_msg_mixer_parameter.count = -1;
				_send_state = MIXERS_SEND_STATE_NONE;
				return;
			}

			_msg_mixer_parameter.index = param.index;
			_msg_mixer_parameter.mixer_index = param.mix_index;
			_msg_mixer_parameter.mixer_sub_index = param.mix_sub_index;
			_msg_mixer_parameter.parameter_index = param.param_index;
			_msg_mixer_parameter.param_array_size = param.array_size;
			_msg_mixer_parameter.mixer_type = param.mix_type;
			_msg_mixer_parameter.count = _param_count;
			_msg_mixer_parameter.param_type = MAV_PARAM_TYPE_REAL32;
			_msg_mixer_parameter.flags = param.flags;
			strncpy(_msg_mixer_parameter.param_id, param.name, 16);
			memcpy(_msg_mixer_parameter.param_values, param.values, sizeof(_msg_mixer_parameter.param_values));
			mavlink_msg_mixer_param_value_send_struct(_mavlink->get_channel(), &_msg_mixer_parameter);

			_msg_mixer_parameter.index++;

			if (_msg_mixer_parameter.index >= _param_count) {
				PX4_INFO("Send all parameters done for group:%u", _msg_mixer_parameter.mixer_group);
				_send_state = MIXERS_SEND_STATE_NONE;
				return;
			}


			if (dev != -1) {
				px4_close(dev);
			}

			break;
		}

	default:
		break;
	}

}
#endif //MIXER_TUNING
