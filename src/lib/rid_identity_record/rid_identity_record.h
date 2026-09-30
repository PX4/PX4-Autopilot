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

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RID_IDENTITY_SN_LEN 20
#define RID_IDENTITY_UID_HASH_LEN 32
#define RID_IDENTITY_SIGNATURE_LEN 64

/**
 * Runtime identity published to modules that consume Open Drone ID.
 */
struct rid_identity {
	char sn[RID_IDENTITY_SN_LEN + 1];
};

/**
 * Backend-independent serialized RID identity record.
 */
struct rid_identity_record {
	uint8_t magic[4];      // "RID1"
	uint8_t version;       // RID identity record format version, independent of PX4 firmware version.
	uint8_t flags;         // bit0: locked
	uint8_t reserved[2];
	uint8_t sn[RID_IDENTITY_SN_LEN];
	uint8_t uid_hash[RID_IDENTITY_UID_HASH_LEN]; // SHA256(SN||UID)
	uint8_t signature[RID_IDENTITY_SIGNATURE_LEN];
};

typedef enum {
	RID_VERIFY_OK = 0,
	RID_VERIFY_NULL,
	RID_VERIFY_LENGTH,
	RID_VERIFY_MAGIC,
	RID_VERIFY_VERSION,
	RID_VERIFY_UID_MISSING,
	RID_VERIFY_UID_HASH_MISMATCH,
	RID_VERIFY_SIGNATURE_REQUIRED,
	RID_VERIFY_SIGNATURE_INVALID,
	RID_VERIFY_SN_CHARSET_INVALID
} rid_identity_verify_error;

typedef bool (*rid_identity_signature_verify_cb)(
	const uint8_t *message, size_t message_len,
	const uint8_t *signature, size_t signature_len,
	void *context);

bool rid_identity_compute_uid_hash(const uint8_t *sn, size_t sn_len,
				   const uint8_t *uid, size_t uid_len,
				   uint8_t out_hash[RID_IDENTITY_UID_HASH_LEN]);

bool rid_identity_verify_sn_charset(const uint8_t *sn, size_t sn_len);

bool rid_identity_parse_record(const uint8_t *data, size_t len, struct rid_identity_record *out_record);

rid_identity_verify_error rid_identity_verify_record(const struct rid_identity_record *record,
		const uint8_t *uid, size_t uid_len,
		bool require_signature,
		rid_identity_signature_verify_cb signature_verify,
		void *signature_verify_context);

/**
 * Runtime API used by current call sites.
 */
bool rid_identity_init(void);
bool rid_identity_is_valid(void);
bool rid_identity_charset_is_valid(void);
const struct rid_identity *rid_identity_get(void);
bool rid_identity_mavlink_stream_stop_denied(const char *stream_name);
bool rid_identity_mavlink_msg_stop_denied(uint16_t msg_id);

#ifdef __cplusplus
}
#endif
