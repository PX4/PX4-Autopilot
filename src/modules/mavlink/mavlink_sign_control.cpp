/****************************************************************************
 *
 *   Copyright (c) 2012-2023 PX4 Development Team. All rights reserved.
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
 * @file mavlink_sign_control.cpp
 * Mavlink messages signing control helpers implementation.
 *
 * @author Yulian oifa <yulian.oifa@mobius-software.com>
 */

#include "mavlink_sign_control.h"
#include <sys/stat.h>

static mavlink_signing_streams_t global_mavlink_signing_streams = {};

static const uint32_t unsigned_messages[] = {
	MAVLINK_MSG_ID_RADIO_STATUS,
	MAVLINK_MSG_ID_ADSB_VEHICLE,
	MAVLINK_MSG_ID_COLLISION
};

MavlinkSignControl::MavlinkSignControl()
{
}

MavlinkSignControl::~MavlinkSignControl()
{
}

void MavlinkSignControl::start(int _instance_id, mavlink_status_t *_mavlink_status,
			       mavlink_accept_unsigned_t accept_unsigned_callback)
{
	_mavlink_signing.link_id = _instance_id;
	_mavlink_signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
	_mavlink_signing.accept_unsigned_callback = accept_unsigned_callback;
	is_signing_initialized = false;

	int mkdir_ret = mkdir(MAVLINK_FOLDER_PATH, S_IRWXU);

	if (mkdir_ret != 0 && errno != EEXIST) {
		PX4_ERR("failed creating module storage dir: %s (%i)", MAVLINK_FOLDER_PATH, errno);

	} else {
		int _fd = ::open(MAVLINK_SECRET_FILE, O_CREAT | O_RDONLY, PX4_O_MODE_600);

		if (_fd == -1) {
			if (errno != ENOENT) {
				PX4_ERR("failed creating mavlink secret key file: %s (%i)", MAVLINK_SECRET_FILE, errno);
			}

		} else {
			//if we dont have enough bytes we simply ignore it , because it may be not set yet
			ssize_t bytes_read = ::read(_fd, _mavlink_signing.secret_key, 32);

			if (bytes_read == 32) {
				bytes_read = ::read(_fd, &_mavlink_signing.timestamp, 8);
				if(bytes_read == 8) {
					if(_mavlink_signing.timestamp !=0 || !is_array_all_zeros(_mavlink_signing.secret_key, 32))
						is_signing_initialized = true;
				}
			}

			close(_fd);
		}
	}

	//lets reset it to nulls if it was not read properly
	if(!is_signing_initialized) {
		for (size_t i = 0; i < 32; ++i) {
			_mavlink_signing.secret_key[i] = 0;
		}

		_mavlink_signing.timestamp = 0;
	}

	// copy pointer of the signing to status struct
	_mavlink_status -> signing = &_mavlink_signing;
	_mavlink_status -> signing_streams = &global_mavlink_signing_streams;
}

bool MavlinkSignControl::check_for_signing(const mavlink_message_t *msg)
{
	if (msg->msgid == MAVLINK_MSG_ID_SETUP_SIGNING) {
		mavlink_setup_signing_t setup_signing;
		mavlink_msg_setup_signing_decode(msg, &setup_signing);
		int _fd = ::open(MAVLINK_SECRET_FILE, O_CREAT | O_WRONLY | O_TRUNC, PX4_O_MODE_600);

		if (_fd == -1) {
			if (errno != ENOENT) {
				PX4_ERR("failed opening mavlink secret key file for writing: %s (%i)", MAVLINK_SECRET_FILE, errno);
			}

		} else {
			ssize_t bytes_write = ::write(_fd, setup_signing.secret_key, 32);

			if (bytes_write == 32) {
				bytes_write = ::write(_fd, &setup_signing.initial_timestamp, 8);
				if(bytes_write == 8) {
					//setup signing provides new key , lets update it
					//we update it only in case everything was stored properly
					memcpy(_mavlink_signing.secret_key, setup_signing.secret_key, 32);
					_mavlink_signing.timestamp = setup_signing.initial_timestamp;

					if(setup_signing.initial_timestamp !=0 || !is_array_all_zeros(setup_signing.secret_key, 32))
						is_signing_initialized = true;
					else
						is_signing_initialized = false;
				}
			}

			close(_fd);
		}

		return true;
	}

	return false;
}

void MavlinkSignControl::write_key_and_timestamp()
{
	int _fd = ::open(MAVLINK_SECRET_FILE, O_CREAT | O_WRONLY | O_TRUNC, PX4_O_MODE_600);

	if (_fd == -1) {
		if (errno != ENOENT) {
			PX4_ERR("failed opening mavlink secret key file for writing: %s (%i)", MAVLINK_SECRET_FILE, errno);
		}
	} else {
		ssize_t bytes_write = ::write(_fd, _mavlink_signing.secret_key, 32);

		if (bytes_write == 32) {
			bytes_write = ::write(_fd, &_mavlink_signing.timestamp, 8);
		}

		close(_fd);
	}
}

bool MavlinkSignControl::accept_unsigned(int32_t sign_mode, bool is_usb_uart, uint32_t message_id)
{
	// if signing is not initilized properly or has all zeroes we will allow any message
	if(!is_signing_initialized)
		return true;

	// Always accept a few select messages even if unsigned
	for (unsigned i = 0; i < sizeof(unsigned_messages) / sizeof(unsigned_messages[0]); i++) {
		if (unsigned_messages[i] == message_id) {
			return true;
		}
	}

	switch (sign_mode) {
	// If signing is not required always return true
	case MavlinkSignControl::PROTO_SIGN_OPTIONAL:
		return true;

	// Accept USB links if enabled
	case MavlinkSignControl::PROTO_SIGN_NON_USB:
		return is_usb_uart;

	case MavlinkSignControl::PROTO_SIGN_ALWAYS:

	// fallthrough
	default:
		return false;
	}
}

bool MavlinkSignControl::is_array_all_zeros(uint8_t arr[], size_t size) {
    for (size_t i = 0; i < size; ++i) {
        if (arr[i] != 0) {
            return false;
        }
    }
    return true;
}
