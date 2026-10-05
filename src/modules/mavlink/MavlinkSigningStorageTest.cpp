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

#include <gtest/gtest.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

namespace
{

enum class FailureMode {
	None,
	PartialWrite,
	PartialThenFail,
	Fsync,
	Close,
	Rename
};

FailureMode failure_mode{FailureMode::None};
unsigned write_call_count{0};

int test_open(const char *path, int flags, mode_t mode)
{
	return ::open(path, flags, mode);
}

ssize_t test_read(int fd, void *buffer, size_t length)
{
	return ::read(fd, buffer, length);
}

ssize_t test_write(int fd, const void *buffer, size_t length)
{
	++write_call_count;

	if ((failure_mode == FailureMode::PartialWrite || failure_mode == FailureMode::PartialThenFail)
	    && write_call_count == 1 && length > 1) {
		return ::write(fd, buffer, length - 1);
	}

	if (failure_mode == FailureMode::PartialThenFail && write_call_count == 2) {
		errno = EIO;
		return -1;
	}

	return ::write(fd, buffer, length);
}

int test_fsync(int fd)
{
	if (failure_mode == FailureMode::Fsync) {
		errno = EIO;
		return -1;
	}

	return ::fsync(fd);
}

int test_close(int fd)
{
	const int result = ::close(fd);

	if (failure_mode == FailureMode::Close) {
		errno = EIO;
		return -1;
	}

	return result;
}

int test_rename(const char *old_path, const char *new_path)
{
	if (failure_mode == FailureMode::Rename) {
		errno = EIO;
		return -1;
	}

	return ::rename(old_path, new_path);
}

int test_unlink(const char *path)
{
	return ::unlink(path);
}

const MavlinkSigningStorage::FileOperations test_file_operations {
	.open = test_open,
	.read = test_read,
	.write = test_write,
	.fsync = test_fsync,
	.close = test_close,
	.rename = test_rename,
	.unlink = test_unlink,
};

MavlinkSigningStorage::State state_with(uint8_t key_byte, uint64_t timestamp)
{
	MavlinkSigningStorage::State state{};
	memset(state.secret_key, key_byte, sizeof(state.secret_key));
	state.timestamp = timestamp;
	return state;
}

class MavlinkSigningStorageTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		ASSERT_NE(mkdtemp(_directory), nullptr);
		ASSERT_GT(snprintf(_path, sizeof(_path), "%s/state.bin", _directory), 0);
		ASSERT_GT(snprintf(_temporary_path, sizeof(_temporary_path), "%s.tmp", _path), 0);
		failure_mode = FailureMode::None;
		write_call_count = 0;
	}

	void TearDown() override
	{
		::unlink(_temporary_path);
		::unlink(_path);
		::rmdir(_directory);
	}

	char _directory[64] {"/tmp/mavlink_signing_storage_XXXXXX"};
	char _path[96] {};
	char _temporary_path[100] {};
};

TEST_F(MavlinkSigningStorageTest, LoadAndCheckpointAdvance)
{
	MavlinkSigningStorage storage(_path);
	const auto initial = state_with(0x11, 100);
	const auto advanced = state_with(0x11, 200);

	EXPECT_EQ(storage.replace(initial), MavlinkSigningStorage::Result::Updated);

	MavlinkSigningStorage::State restored{};
	EXPECT_EQ(storage.load(restored), MavlinkSigningStorage::Result::Loaded);
	EXPECT_TRUE(MavlinkSigningStorage::keys_equal(initial, restored));
	EXPECT_EQ(restored.timestamp, initial.timestamp);

	EXPECT_EQ(storage.checkpoint(advanced), MavlinkSigningStorage::Result::Updated);
	EXPECT_EQ(storage.load(restored), MavlinkSigningStorage::Result::Loaded);
	EXPECT_EQ(restored.timestamp, advanced.timestamp);
}

TEST_F(MavlinkSigningStorageTest, SameKeyReloadNeverRollsBack)
{
	const auto current = state_with(0x22, 300);
	auto stored = state_with(0x22, 100);

	auto reconciled = MavlinkSigningStorage::reconcile(current, true, stored, 200);
	EXPECT_EQ(reconciled.timestamp, 300);

	stored.timestamp = 400;
	reconciled = MavlinkSigningStorage::reconcile(current, true, stored, 200);
	EXPECT_EQ(reconciled.timestamp, 400);

	reconciled = MavlinkSigningStorage::reconcile(current, true, stored, 500);
	EXPECT_EQ(reconciled.timestamp, 500);
}

TEST_F(MavlinkSigningStorageTest, MultipleInstancesKeepHighestCheckpoint)
{
	MavlinkSigningStorage storage_a(_path);
	MavlinkSigningStorage storage_b(_path);
	EXPECT_EQ(storage_a.replace(state_with(0x33, 100)), MavlinkSigningStorage::Result::Updated);
	EXPECT_EQ(storage_a.checkpoint(state_with(0x33, 250)), MavlinkSigningStorage::Result::Updated);
	EXPECT_EQ(storage_b.checkpoint(state_with(0x33, 150)), MavlinkSigningStorage::Result::Unchanged);
	EXPECT_EQ(storage_b.checkpoint(state_with(0x33, 350)), MavlinkSigningStorage::Result::Updated);

	MavlinkSigningStorage::State restored{};
	EXPECT_EQ(storage_a.load(restored), MavlinkSigningStorage::Result::Loaded);
	EXPECT_EQ(restored.timestamp, 350);
}

TEST_F(MavlinkSigningStorageTest, ChangedOrDisabledKeyCannotBeRestoredByStaleCheckpoint)
{
	MavlinkSigningStorage storage(_path);
	const auto old_state = state_with(0x44, 100);
	const auto new_state = state_with(0x55, 200);
	MavlinkSigningStorage::State disabled{};

	const auto reconciled = MavlinkSigningStorage::reconcile(old_state, true, new_state, 150);
	EXPECT_TRUE(MavlinkSigningStorage::keys_equal(reconciled, new_state));
	EXPECT_EQ(reconciled.timestamp, new_state.timestamp);

	EXPECT_EQ(storage.replace(old_state), MavlinkSigningStorage::Result::Updated);
	EXPECT_EQ(storage.replace(new_state), MavlinkSigningStorage::Result::Updated);
	EXPECT_EQ(storage.checkpoint(state_with(0x44, 500)), MavlinkSigningStorage::Result::KeyMismatch);

	MavlinkSigningStorage::State restored{};
	EXPECT_EQ(storage.load(restored), MavlinkSigningStorage::Result::Loaded);
	EXPECT_TRUE(MavlinkSigningStorage::keys_equal(restored, new_state));
	EXPECT_EQ(restored.timestamp, new_state.timestamp);

	EXPECT_EQ(storage.replace(disabled), MavlinkSigningStorage::Result::Updated);
	EXPECT_EQ(storage.checkpoint(state_with(0x55, 600)), MavlinkSigningStorage::Result::KeyMismatch);
	EXPECT_EQ(storage.load(restored), MavlinkSigningStorage::Result::Loaded);
	EXPECT_FALSE(MavlinkSigningStorage::is_enabled(restored));
}

TEST_F(MavlinkSigningStorageTest, DisabledStateDoesNotCreateAFileAndRepeatedCheckpointIsStable)
{
	MavlinkSigningStorage storage(_path);
	MavlinkSigningStorage::State disabled{};

	EXPECT_EQ(storage.load(disabled), MavlinkSigningStorage::Result::NotFound);
	EXPECT_EQ(storage.checkpoint(disabled), MavlinkSigningStorage::Result::NotFound);
	EXPECT_NE(access(_path, F_OK), 0);

	const auto enabled = state_with(0x66, 100);
	EXPECT_EQ(storage.replace(enabled), MavlinkSigningStorage::Result::Updated);
	EXPECT_EQ(storage.checkpoint(enabled), MavlinkSigningStorage::Result::Unchanged);
}

TEST_F(MavlinkSigningStorageTest, PartialWritesAreCompleted)
{
	MavlinkSigningStorage storage(_path, &test_file_operations);
	failure_mode = FailureMode::PartialWrite;

	EXPECT_EQ(storage.replace(state_with(0x77, 100)), MavlinkSigningStorage::Result::Updated);

	MavlinkSigningStorage::State restored{};
	EXPECT_EQ(storage.load(restored), MavlinkSigningStorage::Result::Loaded);
	EXPECT_EQ(restored.timestamp, 100);
}

TEST_F(MavlinkSigningStorageTest, FailedAtomicWritesPreserveLastGoodState)
{
	MavlinkSigningStorage normal_storage(_path);
	MavlinkSigningStorage failing_storage(_path, &test_file_operations);
	const auto initial = state_with(0x7a, 100);
	EXPECT_EQ(normal_storage.replace(initial), MavlinkSigningStorage::Result::Updated);

	const FailureMode failures[] {
		FailureMode::PartialThenFail,
		FailureMode::Fsync,
		FailureMode::Close,
		FailureMode::Rename,
	};

	for (const FailureMode failure : failures) {
		failure_mode = failure;
		write_call_count = 0;
		EXPECT_EQ(failing_storage.checkpoint(state_with(0x7a, 200)), MavlinkSigningStorage::Result::IoError);

		failure_mode = FailureMode::None;
		MavlinkSigningStorage::State restored{};
		EXPECT_EQ(normal_storage.load(restored), MavlinkSigningStorage::Result::Loaded);
		EXPECT_TRUE(MavlinkSigningStorage::keys_equal(restored, initial));
		EXPECT_EQ(restored.timestamp, initial.timestamp);
		EXPECT_NE(access(_temporary_path, F_OK), 0);
	}
}

TEST_F(MavlinkSigningStorageTest, TimestampConversionUsesMavlinkEpochAndUnits)
{
	EXPECT_EQ(MavlinkSigningStorage::timestamp_from_unix_time(1420070399, 999999999), 0);
	EXPECT_EQ(MavlinkSigningStorage::timestamp_from_unix_time(1420070400, 0), 0);
	EXPECT_EQ(MavlinkSigningStorage::timestamp_from_unix_time(1420070400, 10000), 1);
	EXPECT_EQ(MavlinkSigningStorage::timestamp_from_unix_time(1420070401, 0), 100000);
}

} // namespace
