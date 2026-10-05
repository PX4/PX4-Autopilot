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

#include "mavlink_signing_storage.h"

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>

namespace
{

pthread_mutex_t storage_mutex = PTHREAD_MUTEX_INITIALIZER;

int file_open(const char *path, int flags, mode_t mode)
{
	return ::open(path, flags, mode);
}

const MavlinkSigningStorage::FileOperations default_file_operations {
	.open = file_open,
	.read = ::read,
	.write = ::write,
	.fsync = ::fsync,
	.close = ::close,
	.rename = ::rename,
	.unlink = ::unlink,
};

bool read_all(const MavlinkSigningStorage::FileOperations &operations, int fd, void *buffer, size_t length)
{
	size_t offset = 0;

	while (offset < length) {
		const ssize_t result = operations.read(fd, static_cast<uint8_t *>(buffer) + offset, length - offset);

		if (result > 0) {
			offset += result;

		} else if (result < 0 && errno == EINTR) {
			continue;

		} else {
			return false;
		}
	}

	return true;
}

bool write_all(const MavlinkSigningStorage::FileOperations &operations, int fd, const void *buffer, size_t length)
{
	size_t offset = 0;

	while (offset < length) {
		const ssize_t result = operations.write(fd, static_cast<const uint8_t *>(buffer) + offset, length - offset);

		if (result > 0) {
			offset += result;

		} else if (result < 0 && errno == EINTR) {
			continue;

		} else {
			return false;
		}
	}

	return true;
}

} // namespace

MavlinkSigningStorage::MavlinkSigningStorage(const char *path, const FileOperations *file_operations) :
	_path(path),
	_file_operations(file_operations != nullptr ? file_operations : & default_file_operations)
{
}

MavlinkSigningStorage::Result MavlinkSigningStorage::load(State &state) const
{
	pthread_mutex_lock(&storage_mutex);
	const Result result = load_unlocked(state);
	pthread_mutex_unlock(&storage_mutex);
	return result;
}

MavlinkSigningStorage::Result MavlinkSigningStorage::replace(const State &state) const
{
	pthread_mutex_lock(&storage_mutex);
	const Result result = write_atomic_unlocked(state);
	pthread_mutex_unlock(&storage_mutex);
	return result;
}

MavlinkSigningStorage::Result MavlinkSigningStorage::checkpoint(const State &state) const
{
	pthread_mutex_lock(&storage_mutex);

	State stored{};
	Result result = load_unlocked(stored);

	if (result == Result::Loaded) {
		if (!keys_equal(state, stored)) {
			result = Result::KeyMismatch;

		} else if (state.timestamp <= stored.timestamp) {
			result = Result::Unchanged;

		} else {
			result = write_atomic_unlocked(state);
		}
	}

	pthread_mutex_unlock(&storage_mutex);
	return result;
}

MavlinkSigningStorage::Result MavlinkSigningStorage::reconcile_and_replace(const State &requested,
		const State &current, bool current_enabled, uint64_t system_timestamp, State &persisted) const
{
	pthread_mutex_lock(&storage_mutex);

	State stored{};
	const Result load_result = load_unlocked(stored);

	if (load_result != Result::Loaded && load_result != Result::NotFound) {
		pthread_mutex_unlock(&storage_mutex);
		return load_result;
	}

	State next = requested;

	if (current_enabled && keys_equal(requested, current)) {
		next.timestamp = std::max(next.timestamp, current.timestamp);
	}

	if (load_result == Result::Loaded && keys_equal(requested, stored)) {
		next.timestamp = std::max(next.timestamp, stored.timestamp);
	}

	next.timestamp = std::max(next.timestamp, system_timestamp);
	const Result result = write_atomic_unlocked(next);

	if (result == Result::Updated) {
		persisted = next;
	}

	pthread_mutex_unlock(&storage_mutex);
	return result;
}

bool MavlinkSigningStorage::is_enabled(const State &state)
{
	if (state.timestamp != 0) {
		return true;
	}

	for (size_t i = 0; i < SecretKeyLength; ++i) {
		if (state.secret_key[i] != 0) {
			return true;
		}
	}

	return false;
}

bool MavlinkSigningStorage::keys_equal(const State &lhs, const State &rhs)
{
	return memcmp(lhs.secret_key, rhs.secret_key, SecretKeyLength) == 0;
}

MavlinkSigningStorage::State MavlinkSigningStorage::reconcile(const State &current, bool current_enabled,
		const State &stored, uint64_t system_timestamp)
{
	State result = stored;

	if (!is_enabled(stored)) {
		return result;
	}

	if (current_enabled && keys_equal(current, stored)) {
		result.timestamp = std::max(result.timestamp, current.timestamp);
	}

	result.timestamp = std::max(result.timestamp, system_timestamp);
	return result;
}

uint64_t MavlinkSigningStorage::timestamp_from_unix_time(int64_t seconds, int32_t nanoseconds)
{
	if (seconds < static_cast<int64_t>(UnixEpochOffsetSeconds) || nanoseconds < 0 || nanoseconds >= 1000000000) {
		return 0;
	}

	return (static_cast<uint64_t>(seconds) - UnixEpochOffsetSeconds) * TimestampTicksPerSecond
	       + static_cast<uint64_t>(nanoseconds) / 10000ULL;
}

MavlinkSigningStorage::Result MavlinkSigningStorage::load_unlocked(State &state) const
{
	const int fd = _file_operations->open(_path, O_RDONLY, 0);

	if (fd < 0) {
		return errno == ENOENT ? Result::NotFound : Result::IoError;
	}

	State loaded{};
	const bool read_succeeded = read_all(*_file_operations, fd, loaded.secret_key, sizeof(loaded.secret_key))
				    && read_all(*_file_operations, fd, &loaded.timestamp, sizeof(loaded.timestamp));
	const bool close_succeeded = _file_operations->close(fd) == 0;

	if (!read_succeeded || !close_succeeded) {
		return Result::IoError;
	}

	state = loaded;
	return Result::Loaded;
}

MavlinkSigningStorage::Result MavlinkSigningStorage::write_atomic_unlocked(const State &state) const
{
	char temporary_path[PATH_MAX];
	const int path_length = snprintf(temporary_path, sizeof(temporary_path), "%s.tmp", _path);

	if (path_length < 0 || static_cast<size_t>(path_length) >= sizeof(temporary_path)) {
		errno = ENAMETOOLONG;
		return Result::IoError;
	}

	const int fd = _file_operations->open(temporary_path, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR);

	if (fd < 0) {
		return Result::IoError;
	}

	const bool write_succeeded = write_all(*_file_operations, fd, state.secret_key, sizeof(state.secret_key))
				     && write_all(*_file_operations, fd, &state.timestamp, sizeof(state.timestamp));
	const bool sync_succeeded = write_succeeded && _file_operations->fsync(fd) == 0;
	const bool close_succeeded = _file_operations->close(fd) == 0;

	if (!write_succeeded || !sync_succeeded || !close_succeeded) {
		_file_operations->unlink(temporary_path);
		return Result::IoError;
	}

	if (_file_operations->rename(temporary_path, _path) != 0) {
		_file_operations->unlink(temporary_path);
		return Result::IoError;
	}

	return Result::Updated;
}
