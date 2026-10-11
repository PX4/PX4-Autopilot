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

#include "odid_identity.hpp"

#include <string.h>

#ifdef __PX4_NUTTX
#include <nuttx/crc32.h>
#else
#include <crc32.h>
#endif

namespace open_drone_id
{
namespace
{

static constexpr size_t MANUFACTURER_CODE_SIZE = 4;
static constexpr size_t SERIAL_PREFIX_SIZE = MANUFACTURER_CODE_SIZE + 1;

// Character set also used by the RID identity proposal in PX4 PR #27555.
bool serialCharacterValid(uint8_t c)
{
	return (c >= '0' && c <= '9')
	       || (c >= 'A' && c <= 'H')
	       || (c >= 'J' && c <= 'N')
	       || (c >= 'P' && c <= 'Z');
}

uint32_t recordCrc(const UasIdRecord &record)
{
	return crc32(reinterpret_cast<const uint8_t *>(&record.version),
		     offsetof(UasIdRecord, crc) - offsetof(UasIdRecord, version));
}

} // namespace

bool serialNumberValid(const uint8_t *serial, size_t size)
{
	if (serial == nullptr || size <= SERIAL_PREFIX_SIZE || size > UAS_ID_SIZE) {
		return false;
	}

	size_t length = 0;

	while (length < size && serial[length] != 0) {
		if (!serialCharacterValid(serial[length])) {
			return false;
		}

		++length;
	}

	if (length <= SERIAL_PREFIX_SIZE) {
		return false;
	}

	for (size_t i = length; i < size; ++i) {
		if (serial[i] != 0) {
			return false;
		}
	}

	const uint8_t length_code = serial[MANUFACTURER_CODE_SIZE];
	const size_t suffix_length = (length_code >= '1' && length_code <= '9') ? length_code - '0'
				     : (length_code >= 'A' && length_code <= 'F') ? length_code - 'A' + 10 : 0;
	return suffix_length == length - SERIAL_PREFIX_SIZE;
}

bool uidValid(const uint8_t *uid, size_t size)
{
	if (uid == nullptr || size == 0 || size > UID_MAX_SIZE) {
		return false;
	}

	bool all_zero = true;
	bool all_erased = true;

	for (size_t i = 0; i < size; ++i) {
		all_zero &= uid[i] == 0;
		all_erased &= uid[i] == 0xff;
	}

	return !all_zero && !all_erased;
}

bool recordCreate(UasIdRecord &record, uint8_t ua_type, const uint8_t *serial, size_t serial_size,
		  const uint8_t *uid, size_t uid_size)
{
	if (!serialNumberValid(serial, serial_size) || !uidValid(uid, uid_size)) {
		return false;
	}

	record = {};
	record.magic = UAS_ID_RECORD_MAGIC;
	record.version = UAS_ID_RECORD_VERSION;
	record.id_type = UAS_ID_TYPE_SERIAL_NUMBER;
	record.ua_type = ua_type;
	record.uid_size = static_cast<uint8_t>(uid_size);
	memcpy(record.uas_id, serial, serial_size);
	memcpy(record.uid, uid, uid_size);
	record.crc = recordCrc(record);
	return true;
}

bool recordValid(const UasIdRecord &record, const uint8_t *uid, size_t uid_size)
{
	if (!uidValid(uid, uid_size)
	    || record.magic != UAS_ID_RECORD_MAGIC
	    || record.version != UAS_ID_RECORD_VERSION
	    || record.id_type != UAS_ID_TYPE_SERIAL_NUMBER
	    || !serialNumberValid(record.uas_id, sizeof(record.uas_id))
	    || record.uid_size != uid_size
	    || memcmp(record.uid, uid, uid_size) != 0
	    || record.crc != recordCrc(record)) {
		return false;
	}

	for (size_t i = uid_size; i < sizeof(record.uid); ++i) {
		if (record.uid[i] != 0) {
			return false;
		}
	}

	return true;
}

bool recordBlank(const UasIdRecord &record)
{
	const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&record);

	for (size_t i = 0; i < sizeof(record); ++i) {
		if (bytes[i] != 0xff) {
			return false;
		}
	}

	return true;
}

} // namespace open_drone_id
