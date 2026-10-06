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
#include <px4_platform_common/time.h>
#include <sys/stat.h>
#include <time.h>

static mavlink_signing_streams_t global_mavlink_signing_streams = {};

// Messages accepted without signing per MAVLink spec recommendation.
// HEARTBEAT is required for link discovery/interop but allows spoofed phantom vehicles on GCS.
static const uint32_t unsigned_messages[] = {
	MAVLINK_MSG_ID_HEARTBEAT,
	MAVLINK_MSG_ID_RADIO_STATUS,
	MAVLINK_MSG_ID_ADSB_VEHICLE,
	MAVLINK_MSG_ID_COLLISION
};

MavlinkSignControl::MavlinkSignControl(const char *storage_path,
				       TimestampProvider timestamp_provider,
				       const MavlinkSigningStorage::FileOperations *file_operations) :
	_storage(storage_path, file_operations), _timestamp_provider(timestamp_provider)
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
		MavlinkSigningStorage::State stored{};
		const MavlinkSigningStorage::Result result = _storage.load(stored);

		if (result == MavlinkSigningStorage::Result::Loaded) {
			const MavlinkSigningStorage::State state = MavlinkSigningStorage::reconcile(
						{}, false, stored, _current_timestamp());
			_apply_state(state);

		} else if (result != MavlinkSigningStorage::Result::NotFound) {
			PX4_ERR("failed reading mavlink secret key file: %s (%i)", MAVLINK_SECRET_FILE, errno);
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

		MavlinkSigningStorage::State state{};

		if (_storage.replace(state) != MavlinkSigningStorage::Result::Updated) {
			PX4_ERR("failed disabling mavlink signing in storage: %s (%i)", MAVLINK_SECRET_FILE, errno);
			return STORAGE_ERROR;
		}

		_apply_state(state);

		return SIGNING_DISABLED;
	}

	MavlinkSigningStorage::State requested{};
	memcpy(requested.secret_key, setup_signing.secret_key, MAVLINK_SECRET_KEY_LENGTH);
	requested.timestamp = setup_signing.initial_timestamp;

	MavlinkSigningStorage::State current{};
	memcpy(current.secret_key, _mavlink_signing.secret_key, MAVLINK_SECRET_KEY_LENGTH);
	current.timestamp = _mavlink_signing.timestamp;

	MavlinkSigningStorage::State persisted{};

	if (_storage.reconcile_and_replace(requested, current, _is_signing_initialized,
					   _current_timestamp(), persisted) != MavlinkSigningStorage::Result::Updated) {
		PX4_ERR("failed storing mavlink signing key: %s (%i)", MAVLINK_SECRET_FILE, errno);
		return STORAGE_ERROR;
	}

	_apply_state(persisted);

	return KEY_ACCEPTED;
}

void MavlinkSignControl::reload_key()
{
	if (_mavlink_status == nullptr) {
		return;
	}

	MavlinkSigningStorage::State current{};
	memcpy(current.secret_key, _mavlink_signing.secret_key, MAVLINK_SECRET_KEY_LENGTH);
	current.timestamp = _mavlink_signing.timestamp;

	MavlinkSigningStorage::State stored{};
	const MavlinkSigningStorage::Result result = _storage.load(stored);

	if (result == MavlinkSigningStorage::Result::Loaded) {
		const MavlinkSigningStorage::State state = MavlinkSigningStorage::reconcile(
					current, _is_signing_initialized, stored, _current_timestamp());
		_apply_state(state);

	} else {
		MavlinkSigningStorage::State disabled{};
		_apply_state(disabled);

		if (result != MavlinkSigningStorage::Result::NotFound) {
			PX4_ERR("failed reloading mavlink signing key: %s (%i)", MAVLINK_SECRET_FILE, errno);
		}
	}
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

bool MavlinkSignControl::prepare_checkpoint(MavlinkSigningStorage::State &state)
{
	if (!_is_signing_initialized) {
		return false;
	}

	const uint64_t current_timestamp = _current_timestamp();

	if (current_timestamp > _mavlink_signing.timestamp) {
		_mavlink_signing.timestamp = current_timestamp;
	}
	memcpy(state.secret_key, _mavlink_signing.secret_key, MAVLINK_SECRET_KEY_LENGTH);
	state.timestamp = _mavlink_signing.timestamp;
	return true;
}

MavlinkSigningStorage::Result MavlinkSignControl::checkpoint(const MavlinkSigningStorage::State &state)
{
	return _storage.checkpoint(state);
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

void MavlinkSignControl::_apply_state(const MavlinkSigningStorage::State &state)
{
	memcpy(_mavlink_signing.secret_key, state.secret_key, MAVLINK_SECRET_KEY_LENGTH);
	_mavlink_signing.timestamp = state.timestamp;
	_is_signing_initialized = MavlinkSigningStorage::is_enabled(state);
	_update_signing_state();
}

uint64_t MavlinkSignControl::_current_timestamp() const
{
	if (_timestamp_provider != nullptr) {
		return _timestamp_provider();
	}

	struct timespec ts {};

	if (px4_clock_gettime(CLOCK_REALTIME, &ts) != 0) {
		return 0;
	}

	return MavlinkSigningStorage::timestamp_from_unix_time(ts.tv_sec, ts.tv_nsec);
}
