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

#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#ifdef __PX4_NUTTX
#include <nuttx/crc32.h>
#else
#include <crc32.h>
#endif

namespace
{

static constexpr uint32_t MAVLINK_SIGNING_RECORD_MAGIC = 0x4e47534d; // "MSGN"
static constexpr uint8_t MAVLINK_SIGNING_RECORD_VERSION = 1;
static constexpr char MAVLINK_SECRET_FILE_TEMP[] = MAVLINK_SECRET_FILE ".tmp";

struct __attribute__((packed)) MavlinkSigningRecord {
	uint8_t secret_key[MAVLINK_SECRET_KEY_LENGTH];
	uint64_t timestamp;
	uint32_t magic;
	uint8_t version;
	uint8_t reserved[3];
	uint32_t crc;
};

static_assert(offsetof(MavlinkSigningRecord, secret_key) == 0);
static_assert(offsetof(MavlinkSigningRecord, timestamp) == MAVLINK_SECRET_KEY_LENGTH);
static_assert(sizeof(((MavlinkSigningRecord *)nullptr)->timestamp) == MAVLINK_SECRET_KEY_TIMESTAMP_LENGTH);
static_assert(offsetof(MavlinkSigningRecord, magic) == MAVLINK_SECRET_KEY_LENGTH + MAVLINK_SECRET_KEY_TIMESTAMP_LENGTH);

uint32_t signing_record_crc(const MavlinkSigningRecord &record)
{
	return crc32(reinterpret_cast<const uint8_t *>(&record), offsetof(MavlinkSigningRecord, crc));
}

bool write_all(int fd, const void *buffer, size_t size)
{
	const uint8_t *bytes = static_cast<const uint8_t *>(buffer);
	size_t offset = 0;

	while (offset < size) {
		const ssize_t written = ::write(fd, bytes + offset, size - offset);

		if (written > 0) {
			offset += written;

		} else if (written < 0 && errno == EINTR) {
			continue;

		} else {
			if (written == 0) {
				errno = EIO;
			}

			return false;
		}
	}

	return true;
}

bool sync_fd(int fd)
{
#if defined(__PX4_QURT)
	// The QURT compatibility layer exposes a void fsync().
	::fsync(fd);
	return true;
#else
	return ::fsync(fd) == 0;
#endif
}

} // namespace

// Shared by all instances so that a signed message cannot be replayed on another link.
// Guarded by _streams_mutex.
static mavlink_signing_streams_t global_mavlink_signing_streams = {};

pthread_mutex_t MavlinkSignControl::_streams_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t MavlinkSignControl::_key_file_mutex = PTHREAD_MUTEX_INITIALIZER;

// Messages accepted without signing on every link, per MAVLink spec recommendation.
// HEARTBEAT is required for link discovery/interop but allows spoofed phantom vehicles on GCS.
static const uint32_t unsigned_messages[] = {
	MAVLINK_MSG_ID_HEARTBEAT,
	MAVLINK_MSG_ID_RADIO_STATUS
};

// Traffic messages, accepted without signing only on a link configured for a traffic
// receiver. Receivers generally cannot sign, but anyone on another link could use these
// to inject phantom aircraft into traffic avoidance.
static const uint32_t unsigned_traffic_messages[] = {
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
	_storage_error = false;

	uint8_t secret_key[MAVLINK_SECRET_KEY_LENGTH] {};
	uint64_t timestamp = 0;
	KeyLoadResult load_result = KeyLoadResult::Error;

	int mkdir_ret = mkdir(MAVLINK_FOLDER_PATH, S_IRWXU);

	if (mkdir_ret != 0 && errno != EEXIST) {
		PX4_ERR("failed creating module storage dir: %s (%i)", MAVLINK_FOLDER_PATH, errno);

	} else {
		load_result = _read_key_and_timestamp(secret_key, timestamp);
	}

	if (load_result == KeyLoadResult::Active) {
		memcpy(_mavlink_signing.secret_key, secret_key, MAVLINK_SECRET_KEY_LENGTH);
		_mavlink_signing.timestamp = timestamp;
		_is_signing_initialized = true;

	} else {
		memset(_mavlink_signing.secret_key, 0, MAVLINK_SECRET_KEY_LENGTH);
		_mavlink_signing.timestamp = 0;
		_storage_error = load_result == KeyLoadResult::Error;

		if (_storage_error) {
			PX4_ERR("MAVLink signing key storage is invalid: %s", MAVLINK_SECRET_FILE);
		}
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
		const uint8_t blank_key[MAVLINK_SECRET_KEY_LENGTH] {};

		// Changing an active link to disabled requires a signed message.
		if (!_is_signing_initialized) {
			// Persist an explicit disabled record so peer reloads can distinguish
			// intentional disablement from a missing-file storage fault.
			if (!_write_key_and_timestamp(blank_key, 0)) {
				return STORAGE_ERROR;
			}

			_storage_error = false;
			return SIGNING_DISABLED;
		}

		bool msg_is_signed = (msg->incompat_flags & MAVLINK_IFLAG_SIGNED);

		if (!msg_is_signed) {
			PX4_WARN("SETUP_SIGNING blank key rejected: message must be signed");
			return BLANK_KEY_REJECTED;
		}

		if (!_write_key_and_timestamp(blank_key, 0)) {
			return STORAGE_ERROR;
		}

		memset(_mavlink_signing.secret_key, 0, MAVLINK_SECRET_KEY_LENGTH);
		_mavlink_signing.timestamp = 0;
		_is_signing_initialized = false;
		_storage_error = false;

		_update_signing_state();

		return SIGNING_DISABLED;
	}

	if (!_write_key_and_timestamp(setup_signing.secret_key, setup_signing.initial_timestamp)) {
		return STORAGE_ERROR;
	}

	memcpy(_mavlink_signing.secret_key, setup_signing.secret_key, MAVLINK_SECRET_KEY_LENGTH);
	_mavlink_signing.timestamp = setup_signing.initial_timestamp;
	_is_signing_initialized = true;
	_storage_error = false;

	_update_signing_state();

	return KEY_ACCEPTED;
}

void MavlinkSignControl::reload_key()
{
	if (_mavlink_status == nullptr) {
		return;
	}

	uint8_t secret_key[MAVLINK_SECRET_KEY_LENGTH] {};
	uint64_t timestamp = 0;
	const KeyLoadResult load_result = _read_key_and_timestamp(secret_key, timestamp);

	if (load_result == KeyLoadResult::Active) {
		memcpy(_mavlink_signing.secret_key, secret_key, MAVLINK_SECRET_KEY_LENGTH);
		_mavlink_signing.timestamp = timestamp;
		_is_signing_initialized = true;
		_storage_error = false;

	} else if (load_result == KeyLoadResult::Disabled) {
		memset(_mavlink_signing.secret_key, 0, MAVLINK_SECRET_KEY_LENGTH);
		_mavlink_signing.timestamp = 0;
		_is_signing_initialized = false;
		_storage_error = false;

	} else {
		// Keep an active key wired into the parser. If no key was active, the
		// receiver uses _storage_error to reject everything outside the allowlist.
		_storage_error = true;
		PX4_ERR("failed reloading MAVLink signing key, retaining current state: %s", MAVLINK_SECRET_FILE);
		return;
	}

	_update_signing_state();
}

MavlinkSignControl::KeyLoadResult MavlinkSignControl::_read_key_and_timestamp(
	uint8_t secret_key[MAVLINK_SECRET_KEY_LENGTH], uint64_t &timestamp)
{
	uint8_t stored_bytes[sizeof(MavlinkSigningRecord) + 1] {};
	size_t stored_size = 0;
	bool read_error = false;

	pthread_mutex_lock(&_key_file_mutex);
	const int fd = ::open(MAVLINK_SECRET_FILE, O_RDONLY);

	if (fd == -1) {
		const KeyLoadResult result = errno == ENOENT ? KeyLoadResult::Missing : KeyLoadResult::Error;
		pthread_mutex_unlock(&_key_file_mutex);
		return result;
	}

	while (stored_size < sizeof(stored_bytes)) {
		const ssize_t bytes_read = ::read(fd, stored_bytes + stored_size, sizeof(stored_bytes) - stored_size);

		if (bytes_read > 0) {
			stored_size += bytes_read;

		} else if (bytes_read == 0) {
			break;

		} else if (errno != EINTR) {
			read_error = true;
			break;
		}
	}

	if (::close(fd) != 0) {
		read_error = true;
	}

	pthread_mutex_unlock(&_key_file_mutex);

	if (read_error) {
		return KeyLoadResult::Error;
	}

	if (stored_size == sizeof(MavlinkSigningRecord)) {
		MavlinkSigningRecord record{};
		memcpy(&record, stored_bytes, sizeof(record));

		if (record.magic != MAVLINK_SIGNING_RECORD_MAGIC
		    || record.version != MAVLINK_SIGNING_RECORD_VERSION
		    || !is_array_all_zeros(record.reserved, sizeof(record.reserved))
		    || record.crc != signing_record_crc(record)) {
			return KeyLoadResult::Error;
		}

		memcpy(secret_key, record.secret_key, MAVLINK_SECRET_KEY_LENGTH);
		timestamp = record.timestamp;

	} else if (stored_size == MAVLINK_SECRET_KEY_LENGTH + MAVLINK_SECRET_KEY_TIMESTAMP_LENGTH) {
		// Preserve compatibility with the original unversioned key record.
		memcpy(secret_key, stored_bytes, MAVLINK_SECRET_KEY_LENGTH);
		memcpy(&timestamp, stored_bytes + MAVLINK_SECRET_KEY_LENGTH, MAVLINK_SECRET_KEY_TIMESTAMP_LENGTH);

	} else {
		return KeyLoadResult::Error;
	}

	return timestamp == 0 && is_array_all_zeros(secret_key, MAVLINK_SECRET_KEY_LENGTH)
	       ? KeyLoadResult::Disabled
	       : KeyLoadResult::Active;
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

bool MavlinkSignControl::_write_key_and_timestamp(const uint8_t secret_key[MAVLINK_SECRET_KEY_LENGTH],
		uint64_t timestamp)
{
	MavlinkSigningRecord record{};
	memcpy(record.secret_key, secret_key, MAVLINK_SECRET_KEY_LENGTH);
	record.timestamp = timestamp;
	record.magic = MAVLINK_SIGNING_RECORD_MAGIC;
	record.version = MAVLINK_SIGNING_RECORD_VERSION;
	record.crc = signing_record_crc(record);

	bool stored = false;
	bool renamed = false;
	int storage_errno = 0;

	pthread_mutex_lock(&_key_file_mutex);

	if (::unlink(MAVLINK_SECRET_FILE_TEMP) != 0 && errno != ENOENT) {
		storage_errno = errno;
	}

	int fd = -1;

	if (storage_errno == 0) {
		fd = ::open(MAVLINK_SECRET_FILE_TEMP, O_CREAT | O_EXCL | O_WRONLY, PX4_O_MODE_600);

		if (fd == -1) {
			storage_errno = errno;
		}
	}

	if (fd != -1) {
		if (!write_all(fd, &record, sizeof(record)) || !sync_fd(fd)) {
			storage_errno = errno != 0 ? errno : EIO;
		}

		if (::close(fd) != 0 && storage_errno == 0) {
			storage_errno = errno;
		}
	}

	if (storage_errno == 0) {
		if (::rename(MAVLINK_SECRET_FILE_TEMP, MAVLINK_SECRET_FILE) == 0) {
			renamed = true;

		} else {
			storage_errno = errno;
		}
	}

	if (renamed) {
		const int directory_fd = ::open(MAVLINK_FOLDER_PATH, O_RDONLY);

		if (directory_fd == -1) {
			storage_errno = errno;

		} else {
			if (!sync_fd(directory_fd)) {
				storage_errno = errno != 0 ? errno : EIO;
			}

			if (::close(directory_fd) != 0 && storage_errno == 0) {
				storage_errno = errno;
			}
		}

		stored = storage_errno == 0;
	}

	if (!renamed) {
		::unlink(MAVLINK_SECRET_FILE_TEMP);
	}

	pthread_mutex_unlock(&_key_file_mutex);

	if (!stored) {
		PX4_ERR("failed storing MAVLink signing key: %s (%i)", MAVLINK_SECRET_FILE, storage_errno);
	}

	return stored;
}

bool MavlinkSignControl::accept_unsigned(uint32_t message_id, bool traffic_link)
{
	if (!_is_signing_initialized && !_storage_error) {
		return true;
	}

	for (unsigned i = 0; i < sizeof(unsigned_messages) / sizeof(unsigned_messages[0]); i++) {
		if (unsigned_messages[i] == message_id) {
			return true;
		}
	}

	if (traffic_link) {
		for (unsigned i = 0; i < sizeof(unsigned_traffic_messages) / sizeof(unsigned_traffic_messages[0]); i++) {
			if (unsigned_traffic_messages[i] == message_id) {
				return true;
			}
		}
	}

	return false;
}

bool MavlinkSignControl::is_array_all_zeros(const uint8_t arr[], size_t size)
{
	for (size_t i = 0; i < size; ++i) {
		if (arr[i] != 0) {
			return false;
		}
	}

	return true;
}
