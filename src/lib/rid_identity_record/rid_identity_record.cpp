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

#include "rid_identity_record.h"

#include "sha256.h"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <errno.h>
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static constexpr unsigned RID_UID_BYTE_LENGTH = 12;

namespace
{
static constexpr uint8_t kRidMagic[4] = {'R', 'I', 'D', '1'};
static constexpr uint8_t kRidRecordVersion = 1;
static constexpr uint8_t kRidFlagLocked = 0x01;
static constexpr char kRidIdentityMtdPath[] = "/fs/mtd_id";

static bool g_initialized{false};
static bool g_valid{false};
static bool g_charset_valid{false};
static rid_identity g_identity {};

static bool constant_time_memeq(const uint8_t *a, const uint8_t *b, size_t len)
{
	uint8_t diff = 0;

	for (size_t i = 0; i < len; i++) {
		diff |= a[i] ^ b[i];
	}

	return diff == 0;
}

static bool is_allowed_rid_char(uint8_t c)
{
	return (c >= '0' && c <= '9')
	       || (c >= 'A' && c <= 'H')
	       || (c >= 'J' && c <= 'N')
	       || (c >= 'P' && c <= 'Z');
}

static void ensure_initialized()
{
	if (!g_initialized) {
		rid_identity_init();
	}
}

#if defined(CONFIG_BOARD_ODID_RID_IDENTITY)
static const char *verify_error_to_string(rid_identity_verify_error err)
{
	switch (err) {
	case RID_VERIFY_OK:
		return "ok";

	case RID_VERIFY_NULL:
		return "null";

	case RID_VERIFY_LENGTH:
		return "length";

	case RID_VERIFY_MAGIC:
		return "magic";

	case RID_VERIFY_VERSION:
		return "version";

	case RID_VERIFY_UID_MISSING:
		return "uid_missing";

	case RID_VERIFY_UID_HASH_MISMATCH:
		return "uid_hash_mismatch";

	case RID_VERIFY_SIGNATURE_REQUIRED:
		return "signature_required";

	case RID_VERIFY_SIGNATURE_INVALID:
		return "signature_invalid";

	case RID_VERIFY_SN_CHARSET_INVALID:
		return "sn_charset_invalid";

	default:
		return "unknown";
	}
}
#endif

} // namespace

extern "C" {

#if defined(CONFIG_BOARD_ODID_RID_IDENTITY)
	__attribute__((weak)) int board_get_mfguid(uint8_t *mfgid);
#endif

	bool rid_identity_compute_uid_hash(const uint8_t *sn, size_t sn_len,
					   const uint8_t *uid, size_t uid_len,
					   uint8_t out_hash[RID_IDENTITY_UID_HASH_LEN])
	{
		if (sn == nullptr || uid == nullptr || out_hash == nullptr) {
			return false;
		}

		uint8_t hash_input[RID_IDENTITY_SN_LEN + RID_UID_BYTE_LENGTH] {};
		const size_t copy_sn_len = sn_len > RID_IDENTITY_SN_LEN ? RID_IDENTITY_SN_LEN : sn_len;
		const size_t copy_uid_len = uid_len > RID_UID_BYTE_LENGTH ? RID_UID_BYTE_LENGTH : uid_len;

		memcpy(hash_input, sn, copy_sn_len);
		memcpy(hash_input + RID_IDENTITY_SN_LEN, uid, copy_uid_len);

		sha256(hash_input, sizeof(hash_input), out_hash);
		return true;
	}

	bool rid_identity_verify_sn_charset(const uint8_t *sn, size_t sn_len)
	{
		if (sn == nullptr) {
			return false;
		}

		for (size_t i = 0; i < sn_len; i++) {
			const uint8_t c = sn[i];

			if (c == 0) {
				continue;
			}

			if (!is_allowed_rid_char(c)) {
				return false;
			}
		}

		return true;
	}

	bool rid_identity_parse_record(const uint8_t *data, size_t len, struct rid_identity_record *out_record)
	{
		if (data == nullptr || out_record == nullptr || len < sizeof(*out_record)) {
			return false;
		}

		memcpy(out_record, data, sizeof(*out_record));
		return true;
	}

	rid_identity_verify_error rid_identity_verify_record(const struct rid_identity_record *record,
			const uint8_t *uid, size_t uid_len,
			bool require_signature,
			rid_identity_signature_verify_cb signature_verify,
			void *signature_verify_context)
	{
		if (record == nullptr) {
			return RID_VERIFY_NULL;
		}

		if (!constant_time_memeq(record->magic, kRidMagic, sizeof(record->magic))) {
			return RID_VERIFY_MAGIC;
		}

		if (record->version != kRidRecordVersion) {
			return RID_VERIFY_VERSION;
		}

		if (!rid_identity_verify_sn_charset(record->sn, RID_IDENTITY_SN_LEN)) {
			return RID_VERIFY_SN_CHARSET_INVALID;
		}

		if (uid == nullptr || uid_len == 0) {
			return RID_VERIFY_UID_MISSING;
		}

		uint8_t expected_hash[RID_IDENTITY_UID_HASH_LEN] {};

		if (!rid_identity_compute_uid_hash(record->sn, RID_IDENTITY_SN_LEN, uid, uid_len, expected_hash)) {
			return RID_VERIFY_UID_MISSING;
		}

		if (!constant_time_memeq(expected_hash, record->uid_hash, sizeof(expected_hash))) {
			return RID_VERIFY_UID_HASH_MISMATCH;
		}

		if (require_signature || (record->flags & kRidFlagLocked)) {
			if (signature_verify == nullptr) {
				return RID_VERIFY_SIGNATURE_REQUIRED;
			}

			const uint8_t *message = reinterpret_cast<const uint8_t *>(record);
			const size_t message_len = offsetof(rid_identity_record, signature);

			if (!signature_verify(message, message_len,
					      record->signature, sizeof(record->signature),
					      signature_verify_context)) {
				return RID_VERIFY_SIGNATURE_INVALID;
			}
		}

		return RID_VERIFY_OK;
	}

	bool rid_identity_init(void)
	{
		if (g_initialized) {
			return g_valid;
		}

		g_initialized = true;
		g_valid = false;
		g_charset_valid = false;
		memset(&g_identity, 0, sizeof(g_identity));

#if defined(CONFIG_BOARD_ODID_RID_IDENTITY)
		FILE *mtd_file = fopen(kRidIdentityMtdPath, "rb");

		if (mtd_file == nullptr) {
			PX4_WARN("RID identity mtd read open failed: %s (%d)", kRidIdentityMtdPath, errno);
			return false;
		}

		uint8_t record_bytes[sizeof(rid_identity_record)] {};
		const size_t bytes_read = fread(record_bytes, 1, sizeof(record_bytes), mtd_file);
		const int close_result = fclose(mtd_file);

		if (close_result != 0) {
			PX4_WARN("RID identity mtd close failed: %s (%d)", kRidIdentityMtdPath, errno);
		}

		if (bytes_read < sizeof(record_bytes)) {
			PX4_WARN("RID identity mtd read short: %zu/%zu", bytes_read, sizeof(record_bytes));
			return false;
		}

		rid_identity_record record {};

		if (!rid_identity_parse_record(record_bytes, sizeof(record_bytes), &record)) {
			PX4_WARN("RID identity parse failed");
			return false;
		}

		uint8_t mfguid[RID_UID_BYTE_LENGTH] {};

		if (board_get_mfguid == nullptr) {
			PX4_WARN("RID identity board mfguid API unavailable on this target");
			return false;
		}

		const int mfguid_len = board_get_mfguid(mfguid);

		if (mfguid_len < static_cast<int>(RID_UID_BYTE_LENGTH)) {
			PX4_WARN("RID identity board mfguid unavailable: len=%d", mfguid_len);
			return false;
		}

		const rid_identity_verify_error verify_error = rid_identity_verify_record(&record,
				mfguid,
				sizeof(mfguid),
				false,
				nullptr,
				nullptr);

		if (verify_error != RID_VERIFY_OK) {
			PX4_WARN("RID identity verify failed: %s", verify_error_to_string(verify_error));
			return false;
		}

		memcpy(g_identity.sn, record.sn, RID_IDENTITY_SN_LEN);
		g_identity.sn[RID_IDENTITY_SN_LEN] = '\0';
		g_charset_valid = true;
		g_valid = true;
#endif

		return g_valid;
	}

	bool rid_identity_is_valid(void)
	{
		ensure_initialized();
		return g_valid;
	}

	bool rid_identity_charset_is_valid(void)
	{
		ensure_initialized();
		return g_valid && g_charset_valid;
	}

	const struct rid_identity *rid_identity_get(void)
	{
		ensure_initialized();

		if (!rid_identity_is_valid()) {
			return nullptr;
		}

		return &g_identity;
	}

	bool rid_identity_mavlink_stream_stop_denied(const char *stream_name)
	{
		if (stream_name == nullptr) {
			return false;
		}

		static const char *const protected_streams[] = {
			"OPEN_DRONE_ID_BASIC_ID",
			"OPEN_DRONE_ID_LOCATION",
			"OPEN_DRONE_ID_SYSTEM",
			"OPEN_DRONE_ID_ARM_STATUS",
		};

		for (const char *name : protected_streams) { // NOLINT(readability-use-anyofallof): avoid <algorithm> dependency for constrained toolchains.
			if (strcmp(stream_name, name) == 0) {
				return true;
			}
		}

		return false;
	}

	bool rid_identity_mavlink_msg_stop_denied(uint16_t msg_id)
	{
		switch (msg_id) {
		case 12900: // OPEN_DRONE_ID_BASIC_ID
		case 12901: // OPEN_DRONE_ID_LOCATION
		case 12904: // OPEN_DRONE_ID_SYSTEM
		case 12918: // OPEN_DRONE_ID_ARM_STATUS
			return true;

		default:
			return false;
		}
	}

} // extern "C"
