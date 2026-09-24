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

#include <gtest/gtest.h>

#include "rid_identity_record.h"

#include <cstdint>
#include <cstring>

namespace
{

constexpr uint8_t RID_MAGIC[4] = {'R', 'I', 'D', '1'};
constexpr uint8_t RID_RECORD_VERSION = 1;

rid_identity_record makeValidRecord(const uint8_t *uid, size_t uid_len)
{
	rid_identity_record record {};
	memcpy(record.magic, RID_MAGIC, sizeof(record.magic));
	record.version = RID_RECORD_VERSION;
	record.flags = 0;

	constexpr char sn[] = "ABCD1234EFGH5678JKLM";
	static_assert(sizeof(sn) - 1 == RID_IDENTITY_SN_LEN, "RID SN must be 20 bytes");
	memcpy(record.sn, sn, RID_IDENTITY_SN_LEN);

	const bool hash_ok = rid_identity_compute_uid_hash(record.sn, RID_IDENTITY_SN_LEN,
			     uid, uid_len, record.uid_hash);
	EXPECT_TRUE(hash_ok);

	return record;
}

} // namespace

TEST(RidIdentityRecordTest, VerifyRecordAcceptsValidRecord)
{
	const uint8_t uid_ok[12] = {0x10, 0x11, 0x12, 0x13, 0x20, 0x21, 0x22, 0x23, 0x30, 0x31, 0x32, 0x33};
	const rid_identity_record record = makeValidRecord(uid_ok, sizeof(uid_ok));

	const rid_identity_verify_error err = rid_identity_verify_record(
			&record,
			uid_ok,
			sizeof(uid_ok),
			false,
			nullptr,
			nullptr);

	EXPECT_EQ(err, RID_VERIFY_OK);
}

TEST(RidIdentityRecordTest, VerifyRecordRejectsUidMismatch)
{
	const uint8_t uid_ok[12] = {0x10, 0x11, 0x12, 0x13, 0x20, 0x21, 0x22, 0x23, 0x30, 0x31, 0x32, 0x33};
	const uint8_t uid_bad[12] = {0xAA, 0x11, 0x12, 0x13, 0x20, 0x21, 0x22, 0x23, 0x30, 0x31, 0x32, 0x33};
	const rid_identity_record record = makeValidRecord(uid_ok, sizeof(uid_ok));

	const rid_identity_verify_error err = rid_identity_verify_record(
			&record,
			uid_bad,
			sizeof(uid_bad),
			false,
			nullptr,
			nullptr);

	EXPECT_EQ(err, RID_VERIFY_UID_HASH_MISMATCH);
}

TEST(RidIdentityRecordTest, VerifyRecordRejectsUnsupportedVersion)
{
	const uint8_t uid_ok[12] = {0x10, 0x11, 0x12, 0x13, 0x20, 0x21, 0x22, 0x23, 0x30, 0x31, 0x32, 0x33};
	rid_identity_record record = makeValidRecord(uid_ok, sizeof(uid_ok));
	record.version = 0xFF;

	const rid_identity_verify_error err = rid_identity_verify_record(
			&record,
			uid_ok,
			sizeof(uid_ok),
			false,
			nullptr,
			nullptr);

	EXPECT_EQ(err, RID_VERIFY_VERSION);
}

TEST(RidIdentityRecordTest, VerifyRecordRejectsMagicMismatch)
{
	const uint8_t uid_ok[12] = {0x10, 0x11, 0x12, 0x13, 0x20, 0x21, 0x22, 0x23, 0x30, 0x31, 0x32, 0x33};
	rid_identity_record record = makeValidRecord(uid_ok, sizeof(uid_ok));
	record.magic[0] ^= 0x01;

	const rid_identity_verify_error err = rid_identity_verify_record(
			&record,
			uid_ok,
			sizeof(uid_ok),
			false,
			nullptr,
			nullptr);

	EXPECT_EQ(err, RID_VERIFY_MAGIC);
}

TEST(RidIdentityRecordTest, VerifyRecordRejectsInvalidSnCharset)
{
	const uint8_t uid_ok[12] = {0x10, 0x11, 0x12, 0x13, 0x20, 0x21, 0x22, 0x23, 0x30, 0x31, 0x32, 0x33};
	rid_identity_record record = makeValidRecord(uid_ok, sizeof(uid_ok));
	record.sn[0] = 'I';

	const rid_identity_verify_error err = rid_identity_verify_record(
			&record,
			uid_ok,
			sizeof(uid_ok),
			false,
			nullptr,
			nullptr);

	EXPECT_EQ(err, RID_VERIFY_SN_CHARSET_INVALID);
}
