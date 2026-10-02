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

#include "odid_bid_storage.hpp"

#include <crc32.h>
#include <fcntl.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>

#ifdef __PX4_NUTTX
#include <px4_platform_common/px4_manifest.h>
#endif

namespace open_drone_id
{
namespace
{

static constexpr uint32_t ODID_UASID_MAGIC = 0x4449444f; // "ODID"
static constexpr uint8_t ODID_UASID_RECORD_VERSION = 1;

struct __attribute__((packed)) UasIdRecord {
	uint32_t magic;
	uint8_t version;
	uint8_t id_type;
	uint8_t ua_type;
	uint8_t reserved;
	uint8_t uas_id[UAS_ID_SIZE];
	uint32_t crc;
};

class BidStorageBackend
{
public:
	BidLockInfo get();
	BidStoreResult checkOrStore(uint8_t ua_type, const uint8_t uas_id[UAS_ID_SIZE]);

private:
	static bool uasIdIsEmpty(const uint8_t uas_id[UAS_ID_SIZE])
	{
		for (unsigned i = 0; i < UAS_ID_SIZE; ++i) {
			if (uas_id[i] != 0) {
				return false;
			}
		}

		return true;
	}

	static uint32_t recordCrc(const UasIdRecord &record)
	{
		return crc32(reinterpret_cast<const uint8_t *>(&record.version),
			     offsetof(UasIdRecord, crc) - offsetof(UasIdRecord, version));
	}

	static bool recordValid(const UasIdRecord &record)
	{
		return record.magic == ODID_UASID_MAGIC
		       && record.version == ODID_UASID_RECORD_VERSION
		       && record.id_type == UAS_ID_TYPE_SERIAL_NUMBER
		       && !uasIdIsEmpty(record.uas_id)
		       && record.crc == recordCrc(record);
	}

	// Check if the record is all 0xff, which is the erased state of flash.
	// This is needed to distinguish between an unwritten record and a record that was written with all bytes set to 0 (which would be invalid but not unwritten).
	static bool recordBlank(const UasIdRecord &record)
	{
		const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&record);

		for (unsigned i = 0; i < sizeof(record); ++i) {
			if (bytes[i] != 0xff) {
				return false;
			}
		}

		return true;
	}

	static bool recordMatches(const UasIdRecord &record, const uint8_t uas_id[UAS_ID_SIZE])
	{
		return memcmp(record.uas_id, uas_id, UAS_ID_SIZE) == 0;
	}

	// Write the entire buffer to the file descriptor, handling short writes.
	static bool writeAll(int fd, const void *buffer, size_t size)
	{
		const uint8_t *bytes = static_cast<const uint8_t *>(buffer);

		while (size > 0) {
			const ssize_t written = write(fd, bytes, size);

			if (written <= 0) {
				return false;
			}

			bytes += written;
			size -= written;
		}

		return true;
	}

};

static_assert(sizeof(UasIdRecord) <= 256, "OpenDroneID UAS ID record must fit in /fs/mtd_id");

// The UAS ID storage is only supported on PX4 targets with NuttX and an MTD partition named "ID".
// On other platforms, the get() method will return StorageUnavailable and the checkOrStore() method will return Unsupported.
BidLockInfo BidStorageBackend::get()
{
#if defined(__PX4_NUTTX)
	const char *path = nullptr;

	if (px4_mtd_query("MTD_ID", nullptr, &path) != 0 || path == nullptr) {
		return {BidLockState::StorageUnavailable, {}};
	}

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		return {BidLockState::StorageUnavailable, {}};
	}

	UasIdRecord record{};

	if (read(fd, &record, sizeof(record)) != sizeof(record)) {
		close(fd);
		return {BidLockState::StorageUnavailable, {}};
	}

	close(fd);

	if (recordValid(record)) {
		BidLockInfo info{BidLockState::Locked, {}};
		info.id.ua_type = record.ua_type;
		memcpy(info.id.uas_id, record.uas_id, UAS_ID_SIZE);
		return info;
	}

	if (recordBlank(record)) {
		return {BidLockState::Empty, {}};
	}

	return {BidLockState::InvalidRecord, {}};
#else
	return {BidLockState::Unsupported, {}};
#endif
}

// Check if the given UAS ID matches the stored one, or store it if the storage is empty.
// The storage is write-once, so once a UAS ID is stored it cannot be changed (except by erasing the flash).
BidStoreResult BidStorageBackend::checkOrStore(uint8_t ua_type, const uint8_t uas_id[UAS_ID_SIZE])
{
	if (uasIdIsEmpty(uas_id)) {
		return BidStoreResult::InvalidBasicId;
	}

#if defined(__PX4_NUTTX)
	const char *path = nullptr;

	if (px4_mtd_query("MTD_ID", nullptr, &path) != 0 || path == nullptr) {
		return BidStoreResult::StorageUnavailable;
	}

	int fd = open(path, O_RDWR);

	if (fd < 0) {
		return BidStoreResult::StorageUnavailable;
	}

	UasIdRecord existing{};

	if (read(fd, &existing, sizeof(existing)) != sizeof(existing)) {
		close(fd);
		return BidStoreResult::StorageUnavailable;
	}

	if (recordValid(existing)) {
		const BidStoreResult result = recordMatches(existing, uas_id)
					      ? BidStoreResult::AlreadyLocked
					      : BidStoreResult::AlreadyLockedDifferent;
		close(fd);
		return result;
	}

	if (!recordBlank(existing)) {
		close(fd);
		return BidStoreResult::InvalidRecord;
	}

	UasIdRecord record{};
	record.magic = ODID_UASID_MAGIC;
	record.version = ODID_UASID_RECORD_VERSION;
	record.id_type = UAS_ID_TYPE_SERIAL_NUMBER;
	record.ua_type = ua_type;
	memcpy(record.uas_id, uas_id, UAS_ID_SIZE);
	record.crc = recordCrc(record);

	const uint32_t magic = record.magic;
	record.magic = 0xffffffff;

	const bool written = lseek(fd, sizeof(record.magic), SEEK_SET) == static_cast<off_t>(sizeof(record.magic))
			     && writeAll(fd, reinterpret_cast<const uint8_t *>(&record) + sizeof(record.magic),
					 sizeof(record) - sizeof(record.magic))
			     && fsync(fd) == 0
			     && lseek(fd, 0, SEEK_SET) == 0
			     && writeAll(fd, &magic, sizeof(magic))
			     && fsync(fd) == 0;

	close(fd);

	return written ? BidStoreResult::Stored : BidStoreResult::WriteError;
#else
	return BidStoreResult::Unsupported;
#endif
}

} // namespace

// Public interface that uses the backend implementation.
BidLockInfo BidStorage::get()
{
	BidStorageBackend storage;
	return storage.get();
}

BidStoreResult BidStorage::checkOrStore(uint8_t ua_type, const uint8_t uas_id[UAS_ID_SIZE])
{
	BidStorageBackend storage;
	return storage.checkOrStore(ua_type, uas_id);
}

//
BidStoreResult BidLock::resolve(uint8_t id_type, uint8_t ua_type, const uint8_t uas_id[UAS_ID_SIZE],
				UasId &locked_id)
{
	if (id_type != UAS_ID_TYPE_SERIAL_NUMBER) {
		return BidStoreResult::InvalidBasicId;
	}

	if (!_loaded) {
		_info = BidStorage::get();
		_loaded = true;
	}

	if (_info.locked()) {
		locked_id = _info.id;
		return BidStoreResult::AlreadyLocked;
	}

	if (_info.state != BidLockState::Empty) {
		switch (_info.state) {
		case BidLockState::InvalidRecord:
			return BidStoreResult::InvalidRecord;

		case BidLockState::StorageUnavailable:
			return BidStoreResult::StorageUnavailable;

		case BidLockState::WriteError:
			return BidStoreResult::WriteError;

		case BidLockState::Unsupported:
			return BidStoreResult::Unsupported;

		case BidLockState::Empty:
		case BidLockState::Locked:
			break;
		}
	}

	const BidStoreResult result = BidStorage::checkOrStore(ua_type, uas_id);

	switch (result) {
	case BidStoreResult::Stored:
		_info.state = BidLockState::Locked;
		_info.id.ua_type = ua_type;
		memcpy(_info.id.uas_id, uas_id, UAS_ID_SIZE);
		locked_id = _info.id;
		return result;

	case BidStoreResult::AlreadyLocked:
	case BidStoreResult::AlreadyLockedDifferent: {
			_info = BidStorage::get();

			if (_info.locked()) {
				locked_id = _info.id;
				return BidStoreResult::AlreadyLocked;
			}

			return BidStoreResult::InvalidRecord;
		}

	case BidStoreResult::InvalidBasicId:
	case BidStoreResult::StorageUnavailable:
	case BidStoreResult::InvalidRecord:
	case BidStoreResult::WriteError:
	case BidStoreResult::Unsupported:
		return result;
	}

	return BidStoreResult::InvalidRecord;
}

} // namespace open_drone_id
