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

#pragma once

#include <cstddef>
#include <cstdint>
#include <sys/stat.h>
#include <sys/types.h>

class MavlinkSigningStorage
{
public:
	static constexpr size_t SecretKeyLength = 32;
	static constexpr uint64_t UnixEpochOffsetSeconds = 1420070400ULL;
	static constexpr uint64_t TimestampTicksPerSecond = 100000ULL;

	struct State {
		uint8_t secret_key[SecretKeyLength] {};
		uint64_t timestamp{0};
	};

	enum class Result {
		Loaded,
		Updated,
		Unchanged,
		NotFound,
		KeyMismatch,
		IoError
	};

	struct FileOperations {
		int (*open)(const char *path, int flags, mode_t mode);
		ssize_t (*read)(int fd, void *buffer, size_t length);
		ssize_t (*write)(int fd, const void *buffer, size_t length);
		int (*fsync)(int fd);
		int (*close)(int fd);
		int (*rename)(const char *old_path, const char *new_path);
		int (*unlink)(const char *path);
	};

	explicit MavlinkSigningStorage(const char *path, const FileOperations *file_operations = nullptr);

	Result load(State &state) const;
	Result replace(const State &state) const;
	Result checkpoint(const State &state) const;

	static bool is_enabled(const State &state);
	static bool keys_equal(const State &lhs, const State &rhs);
	static State reconcile(const State &current, bool current_enabled, const State &stored,
			       uint64_t system_timestamp);
	static uint64_t timestamp_from_unix_time(int64_t seconds, int32_t nanoseconds);

private:
	Result load_unlocked(State &state) const;
	Result write_atomic_unlocked(const State &state) const;

	const char *_path;
	const FileOperations *_file_operations;
};
