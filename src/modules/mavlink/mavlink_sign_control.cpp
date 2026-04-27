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

// Messages accepted without signing per MAVLink spec recommendation.
// HEARTBEAT is required for link discovery/interop but allows spoofed phantom vehicles on GCS.
static const uint32_t unsigned_messages[] = {
	MAVLINK_MSG_ID_HEARTBEAT,
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

void MavlinkSignControl::start(int instance_id, mavlink_status_t *mavlink_status,
			       mavlink_accept_unsigned_t accept_unsigned_callback)
{
	_mavlink_status = mavlink_status;
	_mavlink_signing.link_id = instance_id;
	_mavlink_signing.accept_unsigned_callback = accept_unsigned_callback;
	_is_signing_initialized = false;

	int mkdir_ret = mkdir(MAVLINK_FOLDER_PATH, S_IRWXU);

	if (mkdir_ret != 0 && errno != EEXIST) {
		PX4_ERR("failed creating module storage dir: %s (%i)", MAVLINK_FOLDER_PATH, errno);

	} else {
		int fd = ::open(MAVLINK_SECRET_FILE, O_RDONLY);

		if (fd == -1) {
			if (errno != ENOENT) {
				PX4_ERR("failed opening mavlink secret key file: %s (%i)", MAVLINK_SECRET_FILE, errno);
			}

		} else {
			ssize_t bytes_read = ::read(fd, _mavlink_signing.secret_key, MAVLINK_SECRET_KEY_LENGTH);

			if (bytes_read == MAVLINK_SECRET_KEY_LENGTH) {
				bytes_read = ::read(fd, &_mavlink_signing.timestamp, MAVLINK_SECRET_KEY_TIMESTAMP_LENGTH);

				if (bytes_read == MAVLINK_SECRET_KEY_TIMESTAMP_LENGTH) {
					if (_mavlink_signing.timestamp != 0 || !is_array_all_zeros(_mavlink_signing.secret_key, MAVLINK_SECRET_KEY_LENGTH)) {
						_is_signing_initialized = true;
					}
				}
			}

			close(fd);
		}
	}

	if (!_is_signing_initialized) {
		memset(_mavlink_signing.secret_key, 0, MAVLINK_SECRET_KEY_LENGTH);
		_mavlink_signing.timestamp = 0;
	}

	_update_signing_state();
}

MavlinkSignControl::SetupSigningResult MavlinkSignControl::check_for_signing(const mavlink_message_t *msg)
{
	if (msg->msgid != MAVLINK_MSG_ID_SETUP_SIGNING) {
		return NOT_SETUP_SIGNING;
	}

	mavlink_setup_signing_t setup_signing;
	mavlink_msg_setup_signing_decode(msg, &setup_signing);

	bool new_key_blank = (setup_signing.initial_timestamp == 0
			      && is_array_all_zeros(setup_signing.secret_key, MAVLINK_SECRET_KEY_LENGTH));

	if (new_key_blank) {
		// Disable signing: only allowed if signing is active and the message is signed
		if (!_is_signing_initialized) {
			// Already disabled, nothing to do
			return SIGNING_DISABLED;
		}

		bool msg_is_signed = (msg->incompat_flags & MAVLINK_IFLAG_SIGNED);

		if (!msg_is_signed) {
			PX4_WARN("SETUP_SIGNING blank key rejected: message must be signed");
			return BLANK_KEY_REJECTED;
		}

		memset(_mavlink_signing.secret_key, 0, MAVLINK_SECRET_KEY_LENGTH);
		_mavlink_signing.timestamp = 0;
		_is_signing_initialized = false;

		_update_signing_state();
		write_key_and_timestamp();

		return SIGNING_DISABLED;
	}

	memcpy(_mavlink_signing.secret_key, setup_signing.secret_key, MAVLINK_SECRET_KEY_LENGTH);
	_mavlink_signing.timestamp = setup_signing.initial_timestamp;
	_is_signing_initialized = true;

	_update_signing_state();
	write_key_and_timestamp();

	return KEY_ACCEPTED;
}

void MavlinkSignControl::reload_key()
{
	if (_mavlink_status == nullptr) {
		return;
	}

	_is_signing_initialized = false;

	int fd = ::open(MAVLINK_SECRET_FILE, O_RDONLY);

	if (fd != -1) {
		ssize_t bytes_read = ::read(fd, _mavlink_signing.secret_key, MAVLINK_SECRET_KEY_LENGTH);

		if (bytes_read == MAVLINK_SECRET_KEY_LENGTH) {
			bytes_read = ::read(fd, &_mavlink_signing.timestamp, MAVLINK_SECRET_KEY_TIMESTAMP_LENGTH);

			if (bytes_read == MAVLINK_SECRET_KEY_TIMESTAMP_LENGTH) {
				if (_mavlink_signing.timestamp != 0 || !is_array_all_zeros(_mavlink_signing.secret_key, MAVLINK_SECRET_KEY_LENGTH)) {
					_is_signing_initialized = true;
				}
			}
		}

		close(fd);
	}

	if (!_is_signing_initialized) {
		memset(_mavlink_signing.secret_key, 0, MAVLINK_SECRET_KEY_LENGTH);
		_mavlink_signing.timestamp = 0;
	}

	_update_signing_state();
}

void MavlinkSignControl::_update_signing_state()
{
	if (_is_signing_initialized) {
		_mavlink_signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
		_mavlink_status->signing = &_mavlink_signing;
		_mavlink_status->signing_streams = &global_mavlink_signing_streams;

	} else {
		_mavlink_signing.flags = 0;
		_mavlink_status->signing = nullptr;
		_mavlink_status->signing_streams = nullptr;
	}
}

void MavlinkSignControl::write_key_and_timestamp()
{
	int fd = ::open(MAVLINK_SECRET_FILE, O_CREAT | O_WRONLY | O_TRUNC, PX4_O_MODE_600);

	if (fd == -1) {
		if (errno != ENOENT) {
			PX4_ERR("failed opening mavlink secret key file for writing: %s (%i)", MAVLINK_SECRET_FILE, errno);
		}

	} else {
		ssize_t bytes_write = ::write(fd, _mavlink_signing.secret_key, MAVLINK_SECRET_KEY_LENGTH);

		if (bytes_write == MAVLINK_SECRET_KEY_LENGTH) {
			bytes_write = ::write(fd, &_mavlink_signing.timestamp, MAVLINK_SECRET_KEY_TIMESTAMP_LENGTH);
		}

		close(fd);
	}
}

bool MavlinkSignControl::accept_unsigned(uint32_t message_id)
{
	if (!_is_signing_initialized) {
		return true;
	}

	for (unsigned i = 0; i < sizeof(unsigned_messages) / sizeof(unsigned_messages[0]); i++) {
		if (unsigned_messages[i] == message_id) {
			return true;
		}
	}

	return false;
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
