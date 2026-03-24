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
	_is_signing_initialized = false;

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
			ssize_t bytes_read = ::read(_fd, _mavlink_signing.secret_key, MAVLINK_SECRET_KEY_LENGTH);

			if (bytes_read == MAVLINK_SECRET_KEY_LENGTH) {
				bytes_read = ::read(_fd, &_mavlink_signing.timestamp, MAVLINK_SECRET_KEY_TIMESTAMP_LENGTH);

				if (bytes_read == MAVLINK_SECRET_KEY_TIMESTAMP_LENGTH) {
					if (_mavlink_signing.timestamp != 0 || !is_array_all_zeros(_mavlink_signing.secret_key, MAVLINK_SECRET_KEY_LENGTH)) {
						_is_signing_initialized = true;
					}
				}
			}

			close(_fd);
		}
	}

	if (_is_signing_initialized) {
		PX4_INFO("MAVLink signing key loaded successfully");

	} else {
		PX4_WARN("MAVLink signing key not available (missing or invalid)");

		for (size_t i = 0; i < MAVLINK_SECRET_KEY_LENGTH; ++i) {
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
	if (msg->msgid != MAVLINK_MSG_ID_SETUP_SIGNING) {
		return false;
	}

	mavlink_setup_signing_t setup_signing;
	mavlink_msg_setup_signing_decode(msg, &setup_signing);

	memcpy(_mavlink_signing.secret_key, setup_signing.secret_key, MAVLINK_SECRET_KEY_LENGTH);
	_mavlink_signing.timestamp = setup_signing.initial_timestamp;

	if (setup_signing.initial_timestamp != 0 || !is_array_all_zeros(setup_signing.secret_key, MAVLINK_SECRET_KEY_LENGTH)) {
		_is_signing_initialized = true;
		_no_key_warned = false;
		PX4_INFO("MAVLink signing key updated via SETUP_SIGNING");

	} else {
		_is_signing_initialized = false;
		PX4_INFO("MAVLink signing key cleared via SETUP_SIGNING");
	}

	write_key_and_timestamp();

	return true;
}

void MavlinkSignControl::write_key_and_timestamp()
{
	int _fd = ::open(MAVLINK_SECRET_FILE, O_CREAT | O_WRONLY | O_TRUNC, PX4_O_MODE_600);

	if (_fd == -1) {
		if (errno != ENOENT) {
			PX4_ERR("failed opening mavlink secret key file for writing: %s (%i)", MAVLINK_SECRET_FILE, errno);
		}

	} else {
		ssize_t bytes_write = ::write(_fd, _mavlink_signing.secret_key, MAVLINK_SECRET_KEY_LENGTH);

		if (bytes_write == MAVLINK_SECRET_KEY_LENGTH) {
			bytes_write = ::write(_fd, &_mavlink_signing.timestamp, MAVLINK_SECRET_KEY_TIMESTAMP_LENGTH);

			if (bytes_write != MAVLINK_SECRET_KEY_TIMESTAMP_LENGTH) {
				PX4_ERR("failed to write signing key timestamp");
			}

		} else {
			PX4_ERR("failed to write signing key (%zd/%d bytes)", bytes_write, MAVLINK_SECRET_KEY_LENGTH);
		}

		close(_fd);
	}
}

bool MavlinkSignControl::accept_unsigned(int32_t sign_mode, bool is_usb_uart, uint32_t message_id)
{
	// If signing is not required always accept
	if (sign_mode == MavlinkSignControl::PROTO_SIGN_OPTIONAL) {
		return true;
	}

	// Always accept a few select messages even if unsigned
	for (unsigned i = 0; i < sizeof(unsigned_messages) / sizeof(unsigned_messages[0]); i++) {
		if (unsigned_messages[i] == message_id) {
			return true;
		}
	}

	// If signing is configured but key is not available, only allow USB
	// so that SETUP_SIGNING can still be received to provision the key.
	// Note: under PROTO_SIGN_ALWAYS with a valid key, even USB is rejected (switch default).
	// But without a key we intentionally allow USB — otherwise there is no way to provision
	// the key via SETUP_SIGNING and the device is bricked.
	if (!_is_signing_initialized) {
		if (!is_usb_uart && !_no_key_warned) {
			PX4_WARN("MAVLink signing required but key not set — rejecting unsigned messages on non-USB links");
			_no_key_warned = true;
		}

		return is_usb_uart;
	}

	switch (sign_mode) {
	// Accept USB links if enabled
	case MavlinkSignControl::PROTO_SIGN_NON_USB:
		return is_usb_uart;

	case MavlinkSignControl::PROTO_SIGN_ALWAYS:

	// fallthrough
	default:
		return false;
	}
}

bool MavlinkSignControl::is_array_all_zeros(uint8_t arr[], size_t size)
{
	for (size_t i = 0; i < size; ++i) {
		if (arr[i] != 0) {
			return false;
		}
	}

	return true;
}
