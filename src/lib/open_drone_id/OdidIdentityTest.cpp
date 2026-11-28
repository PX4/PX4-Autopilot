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

#include <gtest/gtest.h>
#include <crc32.h>
#include <stddef.h>
#include <string.h>

using namespace open_drone_id;

namespace
{

constexpr uint8_t SERIAL[] = "ABCD512345";
constexpr uint8_t CN_PRODUCT[] = "ABCD0001000000000001";
constexpr uint8_t UID[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0x10, 0x32, 0x54, 0x76};
constexpr uint8_t UA_TYPE = 2;

void updateCrc(UasIdRecord &record)
{
	record.crc = crc32(reinterpret_cast<const uint8_t *>(&record.version),
			   offsetof(UasIdRecord, crc) - offsetof(UasIdRecord, version));
}

class OdidIdentityTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		ASSERT_TRUE(recordCreate(_record, UAS_ID_TYPE_SERIAL_NUMBER, UA_TYPE, SERIAL, sizeof(SERIAL) - 1, UID, sizeof(UID)));
	}

	UasIdRecord _record{};
};

} // namespace

TEST(OdidSerialNumberTest, AcceptsMinimumMaximumAndZeroPaddedSerials)
{
	const uint8_t minimum[] = "ABCD1A";
	const uint8_t maximum[] = "ABCDF123456789ABCDEF";
	uint8_t padded[UAS_ID_SIZE] {};
	memcpy(padded, minimum, sizeof(minimum) - 1);

	EXPECT_TRUE(serialNumberValid(minimum, sizeof(minimum) - 1));
	EXPECT_TRUE(serialNumberValid(minimum, sizeof(minimum)));
	EXPECT_TRUE(serialNumberValid(maximum, sizeof(maximum) - 1));
	EXPECT_TRUE(serialNumberValid(padded, sizeof(padded)));
}

TEST(OdidSerialNumberTest, AcceptsEveryAllowedManufacturerAndSerialCharacter)
{
	const char allowed[] = "0123456789ABCDEFGHJKLMNPQRSTUVWXYZ";
	uint8_t serial[] = "ABCD1A";

	for (size_t position = 0; position < sizeof(serial) - 1; ++position) {
		if (position == 4) {
			continue; // The fifth character encodes the serial suffix length.
		}

		const uint8_t original = serial[position];

		for (size_t character = 0; character < sizeof(allowed) - 1; ++character) {
			serial[position] = allowed[character];
			EXPECT_TRUE(serialNumberValid(serial, sizeof(serial) - 1)) << position << ": " << allowed[character];
		}

		serial[position] = original;
	}
}

TEST(OdidSerialNumberTest, AcceptsEveryEncodedSuffixLength)
{
	const char lengths[] = "123456789ABCDEF";

	for (size_t suffix_length = 1; suffix_length < sizeof(lengths); ++suffix_length) {
		uint8_t serial[UAS_ID_SIZE] {};
		memcpy(serial, "ABCD", 4);
		serial[4] = lengths[suffix_length - 1];
		memset(serial + 5, 'A', suffix_length);
		EXPECT_TRUE(serialNumberValid(serial, sizeof(serial))) << suffix_length;
	}
}

TEST(OdidSerialNumberTest, RejectsEmptyNullTruncatedAndOversizedInput)
{
	const uint8_t empty[UAS_ID_SIZE] {};
	const uint8_t short_serial[] = "ABCD1";
	const uint8_t oversized[] = "ABCDF123456789ABCDEFZ";

	EXPECT_FALSE(serialNumberValid(nullptr, UAS_ID_SIZE));
	EXPECT_FALSE(serialNumberValid(empty, 0));
	EXPECT_FALSE(serialNumberValid(empty, sizeof(empty)));
	EXPECT_FALSE(serialNumberValid(short_serial, 1));
	EXPECT_FALSE(serialNumberValid(short_serial, sizeof(short_serial)));
	EXPECT_FALSE(serialNumberValid(SERIAL, sizeof(SERIAL) - 2));
	EXPECT_FALSE(serialNumberValid(oversized, sizeof(oversized) - 1));
}

TEST(OdidSerialNumberTest, RejectsInvalidCharactersInManufacturerAndSuffix)
{
	const uint8_t invalid[] = {'I', 'O', 'a', 'z', ' ', '-', '_', '.', '\n', 0x7f, 0x80, 0xff};
	uint8_t serial[] = "ABCD1A";

	for (size_t position = 0; position < sizeof(serial) - 1; ++position) {
		if (position == 4) {
			continue;
		}

		const uint8_t original = serial[position];

		for (uint8_t character : invalid) {
			serial[position] = character;
			EXPECT_FALSE(serialNumberValid(serial, sizeof(serial) - 1)) << position << ": " << +character;
		}

		serial[position] = original;
	}
}

TEST(OdidSerialNumberTest, RejectsInvalidOrMismatchedEncodedLength)
{
	const uint8_t invalid_lengths[] = {'0', '2', 'F', 'G', 'a', '/', ' '};
	uint8_t serial[] = "ABCD1A";

	for (uint8_t length : invalid_lengths) {
		serial[4] = length;
		EXPECT_FALSE(serialNumberValid(serial, sizeof(serial) - 1)) << +length;
	}

	const uint8_t too_long_suffix[] = "ABCD1AB";
	EXPECT_FALSE(serialNumberValid(too_long_suffix, sizeof(too_long_suffix) - 1));
}

TEST(OdidSerialNumberTest, RejectsEmbeddedNullAndNonzeroAfterPadding)
{
	uint8_t serial[UAS_ID_SIZE] {};
	memcpy(serial, SERIAL, sizeof(SERIAL) - 1);
	serial[6] = '\0';
	EXPECT_FALSE(serialNumberValid(serial, sizeof(serial)));

	memcpy(serial, SERIAL, sizeof(SERIAL) - 1);
	serial[UAS_ID_SIZE - 1] = 'A';
	EXPECT_FALSE(serialNumberValid(serial, sizeof(serial)));

	serial[0] = '\0';
	EXPECT_FALSE(serialNumberValid(serial, sizeof(serial)));
}

TEST(OdidCnProductIdTest, AcceptsFixedManufacturerModelAndSerialFormat)
{
	// Four manufacturer, four model, twelve serial characters; no CTA length byte.
	EXPECT_EQ(UAS_ID_TYPE_CN_PRODUCT_ID, 5);
	ASSERT_EQ(sizeof(CN_PRODUCT) - 1, UAS_ID_SIZE);
	EXPECT_TRUE(cnProductIdValid(CN_PRODUCT, sizeof(CN_PRODUCT) - 1));
	EXPECT_TRUE(uasIdValid(UAS_ID_TYPE_CN_PRODUCT_ID, CN_PRODUCT, sizeof(CN_PRODUCT) - 1));
	EXPECT_FALSE(serialNumberValid(CN_PRODUCT, sizeof(CN_PRODUCT) - 1));
	EXPECT_FALSE(uasIdValid(UAS_ID_TYPE_SERIAL_NUMBER, CN_PRODUCT, sizeof(CN_PRODUCT) - 1));
}

TEST(OdidCnProductIdTest, AcceptsEveryAllowedCharacterAtEveryPosition)
{
	const char allowed[] = "0123456789ABCDEFGHJKLMNPQRSTUVWXYZ";
	uint8_t product[UAS_ID_SIZE];
	memcpy(product, CN_PRODUCT, sizeof(product));

	for (size_t position = 0; position < sizeof(product); ++position) {
		const uint8_t original = product[position];

		for (size_t character = 0; character < sizeof(allowed) - 1; ++character) {
			product[position] = allowed[character];
			EXPECT_TRUE(cnProductIdValid(product, sizeof(product))) << position << ": " << allowed[character];
		}

		product[position] = original;
	}
}

TEST(OdidCnProductIdTest, RejectsNullEmptyShortAndOversizedInput)
{
	const uint8_t empty[UAS_ID_SIZE] {};
	const uint8_t oversized[] = "ABCD0001000000000001Z";
	EXPECT_FALSE(cnProductIdValid(nullptr, UAS_ID_SIZE));
	EXPECT_FALSE(cnProductIdValid(CN_PRODUCT, 0));
	EXPECT_FALSE(cnProductIdValid(empty, sizeof(empty)));
	EXPECT_FALSE(cnProductIdValid(CN_PRODUCT, 1));
	EXPECT_FALSE(cnProductIdValid(CN_PRODUCT, UAS_ID_SIZE - 1));
	EXPECT_FALSE(cnProductIdValid(oversized, sizeof(oversized) - 1));
	// A C string terminator is not part of the fixed 20-byte wire field.
	EXPECT_FALSE(cnProductIdValid(CN_PRODUCT, sizeof(CN_PRODUCT)));
}

TEST(OdidCnProductIdTest, RejectsInvalidCharactersAndNullPaddingAtEveryPosition)
{
	const uint8_t invalid[] = {'I', 'O', 'a', 'z', ' ', '-', '_', '.', '\n', 0, 0x7f, 0x80, 0xff};
	uint8_t product[UAS_ID_SIZE];
	memcpy(product, CN_PRODUCT, sizeof(product));

	for (size_t position = 0; position < sizeof(product); ++position) {
		const uint8_t original = product[position];

		for (uint8_t character : invalid) {
			product[position] = character;
			EXPECT_FALSE(cnProductIdValid(product, sizeof(product))) << position << ": " << +character;
		}

		product[position] = original;
	}
}

TEST(OdidCnProductIdTest, ShortCtaSerialCannotBeInferredAsCnProduct)
{
	uint8_t padded[UAS_ID_SIZE] {};
	memcpy(padded, SERIAL, sizeof(SERIAL) - 1);
	EXPECT_TRUE(uasIdValid(UAS_ID_TYPE_SERIAL_NUMBER, padded, sizeof(padded)));
	EXPECT_FALSE(uasIdValid(UAS_ID_TYPE_CN_PRODUCT_ID, padded, sizeof(padded)));

	UasIdRecord record{};
	EXPECT_FALSE(recordCreate(record, UAS_ID_TYPE_CN_PRODUCT_ID, UA_TYPE, padded, sizeof(padded), UID, sizeof(UID)));
	EXPECT_FALSE(recordCreate(record, UAS_ID_TYPE_SERIAL_NUMBER, UA_TYPE, CN_PRODUCT, sizeof(CN_PRODUCT) - 1, UID,
				  sizeof(UID)));
}

TEST(OdidCnProductIdTest, OverlappingSyntaxPreservesExplicitWireType)
{
	const uint8_t overlapping[] = "ABCDF123456789ABCDEF";
	EXPECT_TRUE(uasIdValid(UAS_ID_TYPE_SERIAL_NUMBER, overlapping, sizeof(overlapping) - 1));
	EXPECT_TRUE(uasIdValid(UAS_ID_TYPE_CN_PRODUCT_ID, overlapping, sizeof(overlapping) - 1));
	UasIdRecord cta{};
	UasIdRecord cn{};
	ASSERT_TRUE(recordCreate(cta, UAS_ID_TYPE_SERIAL_NUMBER, UA_TYPE, overlapping, sizeof(overlapping) - 1, UID, sizeof(UID)));
	ASSERT_TRUE(recordCreate(cn, UAS_ID_TYPE_CN_PRODUCT_ID, UA_TYPE, overlapping, sizeof(overlapping) - 1, UID, sizeof(UID)));
	EXPECT_EQ(cta.id_type, UAS_ID_TYPE_SERIAL_NUMBER);
	EXPECT_EQ(cn.id_type, UAS_ID_TYPE_CN_PRODUCT_ID);
	EXPECT_TRUE(recordValid(cta, UID, sizeof(UID)));
	EXPECT_TRUE(recordValid(cn, UID, sizeof(UID)));
	EXPECT_EQ(memcmp(cta.uas_id, cn.uas_id, UAS_ID_SIZE), 0);
	EXPECT_NE(cta.crc, cn.crc);

	// Changing only the type must fail CRC even when both syntaxes match.
	cn.id_type = UAS_ID_TYPE_SERIAL_NUMBER;
	EXPECT_FALSE(recordValid(cn, UID, sizeof(UID)));
}

TEST(OdidCnProductIdTest, UnsupportedWireTypesCannotProvisionIdentity)
{
	const uint8_t unsupported[] = {0, 2, 3, 4, 255};
	const uint8_t overlapping[] = "ABCDF123456789ABCDEF";

	for (uint8_t type : unsupported) {
		UasIdRecord record{};
		EXPECT_FALSE(uasIdValid(type, overlapping, sizeof(overlapping) - 1)) << +type;
		EXPECT_FALSE(recordCreate(record, type, UA_TYPE, overlapping, sizeof(overlapping) - 1, UID, sizeof(UID))) << +type;
	}
}

TEST(OdidUidTest, AcceptsEverySupportedUidLength)
{
	uint8_t uid[UID_MAX_SIZE];
	memset(uid, 0x12, sizeof(uid));

	for (size_t length = 1; length <= sizeof(uid); ++length) {
		EXPECT_TRUE(uidValid(uid, length)) << length;
	}

	const uint8_t mixed[] = {0x00, 0xff};
	EXPECT_TRUE(uidValid(mixed, sizeof(mixed)));
}

TEST(OdidUidTest, RejectsMissingOversizedAndSentinelUids)
{
	uint8_t uid[UID_MAX_SIZE + 1] {};
	EXPECT_FALSE(uidValid(nullptr, sizeof(UID)));
	EXPECT_FALSE(uidValid(UID, 0));
	EXPECT_TRUE(uidValid(UID, sizeof(UID)));
	EXPECT_FALSE(uidValid(uid, UID_MAX_SIZE));
	memset(uid, 0xff, sizeof(uid));
	EXPECT_FALSE(uidValid(uid, UID_MAX_SIZE));
	memset(uid, 0x12, sizeof(uid));
	EXPECT_FALSE(uidValid(uid, sizeof(uid)));
}

TEST_F(OdidIdentityTest, RecordRoundTripPreservesIdentityAndClearsPadding)
{
	memset(&_record, 0xff, sizeof(_record));
	ASSERT_TRUE(recordCreate(_record, UAS_ID_TYPE_SERIAL_NUMBER, UA_TYPE, SERIAL, sizeof(SERIAL) - 1, UID, sizeof(UID)));
	EXPECT_TRUE(recordValid(_record, UID, sizeof(UID)));
	EXPECT_EQ(_record.magic, UAS_ID_RECORD_MAGIC);
	EXPECT_EQ(_record.version, UAS_ID_RECORD_VERSION);
	EXPECT_EQ(_record.id_type, UAS_ID_TYPE_SERIAL_NUMBER);
	EXPECT_EQ(_record.ua_type, UA_TYPE);
	EXPECT_EQ(_record.uid_size, sizeof(UID));
	EXPECT_EQ(memcmp(_record.uas_id, SERIAL, sizeof(SERIAL) - 1), 0);
	EXPECT_EQ(memcmp(_record.uid, UID, sizeof(UID)), 0);

	for (size_t index = sizeof(SERIAL) - 1; index < sizeof(_record.uas_id); ++index) {
		EXPECT_EQ(_record.uas_id[index], 0);
	}

	for (size_t index = sizeof(UID); index < sizeof(_record.uid); ++index) {
		EXPECT_EQ(_record.uid[index], 0);
	}
}

TEST_F(OdidIdentityTest, RecordRoundTripSupportsMaximumSerialAndUid)
{
	const uint8_t serial[] = "ABCDF123456789ABCDEF";
	uint8_t uid[UID_MAX_SIZE];
	memset(uid, 0x12, sizeof(uid));
	ASSERT_TRUE(recordCreate(_record, UAS_ID_TYPE_SERIAL_NUMBER, UA_TYPE, serial, sizeof(serial) - 1, uid, sizeof(uid)));
	EXPECT_TRUE(recordValid(_record, uid, sizeof(uid)));
	EXPECT_EQ(memcmp(_record.uas_id, serial, UAS_ID_SIZE), 0);
	EXPECT_EQ(memcmp(_record.uid, uid, sizeof(uid)), 0);
}

TEST_F(OdidIdentityTest, RecordRoundTripSupportsEveryUidLength)
{
	uint8_t uid[UID_MAX_SIZE];
	memset(uid, 0x12, sizeof(uid));

	for (size_t length = 1; length <= sizeof(uid); ++length) {
		ASSERT_TRUE(recordCreate(_record, UAS_ID_TYPE_SERIAL_NUMBER, UA_TYPE, SERIAL, sizeof(SERIAL) - 1, uid, length));
		EXPECT_TRUE(recordValid(_record, uid, length)) << length;
	}
}

TEST_F(OdidIdentityTest, RecordCreationRejectsInvalidIdentity)
{
	const uint8_t invalid_serial[] = "ABCD1I";
	const uint8_t empty_uid[sizeof(UID)] {};
	EXPECT_FALSE(recordCreate(_record, UAS_ID_TYPE_SERIAL_NUMBER, UA_TYPE, invalid_serial, sizeof(invalid_serial) - 1, UID,
				  sizeof(UID)));
	EXPECT_FALSE(recordCreate(_record, UAS_ID_TYPE_SERIAL_NUMBER, UA_TYPE, nullptr, UAS_ID_SIZE, UID, sizeof(UID)));
	EXPECT_FALSE(recordCreate(_record, UAS_ID_TYPE_SERIAL_NUMBER, UA_TYPE, SERIAL, sizeof(SERIAL) - 1, empty_uid,
				  sizeof(empty_uid)));
	EXPECT_FALSE(recordCreate(_record, UAS_ID_TYPE_SERIAL_NUMBER, UA_TYPE, SERIAL, sizeof(SERIAL) - 1, nullptr, sizeof(UID)));
}

TEST_F(OdidIdentityTest, RejectsRecordFromAnotherBoard)
{
	uint8_t foreign_uid[sizeof(UID)];
	memcpy(foreign_uid, UID, sizeof(UID));
	foreign_uid[sizeof(foreign_uid) - 1] ^= 1;
	EXPECT_FALSE(recordValid(_record, foreign_uid, sizeof(foreign_uid)));
	EXPECT_FALSE(recordValid(_record, UID, sizeof(UID) - 1));

	uint8_t longer_uid[sizeof(UID) + 1] {};
	memcpy(longer_uid, UID, sizeof(UID));
	EXPECT_FALSE(recordValid(_record, longer_uid, sizeof(longer_uid)));
}

TEST_F(OdidIdentityTest, RejectsMissingOrInvalidBoardUid)
{
	const uint8_t missing_uid[UID_MAX_SIZE] {};
	EXPECT_FALSE(recordValid(_record, nullptr, sizeof(UID)));
	EXPECT_FALSE(recordValid(_record, UID, 0));
	EXPECT_FALSE(recordValid(_record, missing_uid, sizeof(missing_uid)));
	EXPECT_FALSE(recordValid(_record, UID, UID_MAX_SIZE + 1));
}

TEST_F(OdidIdentityTest, RejectsUnsupportedHeaderWithCorrectCrc)
{
	UasIdRecord invalid = _record;
	invalid.magic ^= 1;
	updateCrc(invalid);
	EXPECT_FALSE(recordValid(invalid, UID, sizeof(UID)));

	invalid = _record;
	invalid.version = UAS_ID_RECORD_VERSION + 1;
	updateCrc(invalid);
	EXPECT_FALSE(recordValid(invalid, UID, sizeof(UID)));

	invalid = _record;
	invalid.id_type = 0;
	updateCrc(invalid);
	EXPECT_FALSE(recordValid(invalid, UID, sizeof(UID)));
}

TEST_F(OdidIdentityTest, RejectsLegacyUnboundVersion)
{
	_record.version = 1;
	updateCrc(_record);
	EXPECT_FALSE(recordValid(_record, UID, sizeof(UID)));
	EXPECT_FALSE(recordBlank(_record));
}

TEST_F(OdidIdentityTest, RejectsInvalidRecordedUidSizeWithCorrectCrc)
{
	const uint8_t invalid_sizes[] = {0, UID_MAX_SIZE + 1, 0xff};

	for (uint8_t size : invalid_sizes) {
		_record.uid_size = size;
		updateCrc(_record);
		EXPECT_FALSE(recordValid(_record, UID, sizeof(UID))) << +size;
	}
}

TEST_F(OdidIdentityTest, RejectsNonzeroUidPaddingWithCorrectCrc)
{
	_record.uid[sizeof(UID)] = 1;
	updateCrc(_record);
	EXPECT_FALSE(recordValid(_record, UID, sizeof(UID)));
}

TEST_F(OdidIdentityTest, RejectsMalformedSerialWithCorrectCrc)
{
	const uint8_t invalid_characters[] = {'I', 'O', 'a', '\0'};

	for (uint8_t character : invalid_characters) {
		UasIdRecord invalid = _record;
		invalid.uas_id[6] = character;
		updateCrc(invalid);
		EXPECT_FALSE(recordValid(invalid, UID, sizeof(UID))) << +character;
	}

	memset(_record.uas_id, 0, sizeof(_record.uas_id));
	updateCrc(_record);
	EXPECT_FALSE(recordValid(_record, UID, sizeof(UID)));
}

TEST_F(OdidIdentityTest, DetectsEverySingleByteRecordCorruption)
{
	for (size_t index = 0; index < sizeof(_record); ++index) {
		UasIdRecord corrupted = _record;
		reinterpret_cast<uint8_t *>(&corrupted)[index] ^= 1;
		EXPECT_FALSE(recordValid(corrupted, UID, sizeof(UID))) << index;
	}
}

TEST_F(OdidIdentityTest, OnlyCompletelyErasedRecordIsBlank)
{
	EXPECT_FALSE(recordBlank(_record));
	UasIdRecord blank{};
	EXPECT_FALSE(recordBlank(blank));
	memset(&blank, 0xff, sizeof(blank));
	EXPECT_TRUE(recordBlank(blank));
	EXPECT_FALSE(recordValid(blank, UID, sizeof(UID)));

	for (size_t index = 0; index < sizeof(blank); ++index) {
		UasIdRecord partial = blank;
		reinterpret_cast<uint8_t *>(&partial)[index] = 0;
		EXPECT_FALSE(recordBlank(partial)) << index;
	}
}

TEST_F(OdidIdentityTest, CnProductRecordRoundTripPreservesWireTypeAndUid)
{
	ASSERT_TRUE(recordCreate(_record, UAS_ID_TYPE_CN_PRODUCT_ID, UA_TYPE, CN_PRODUCT, sizeof(CN_PRODUCT) - 1, UID,
				 sizeof(UID)));
	EXPECT_TRUE(recordValid(_record, UID, sizeof(UID)));
	EXPECT_EQ(_record.version, 2);
	EXPECT_EQ(_record.id_type, UAS_ID_TYPE_CN_PRODUCT_ID);
	EXPECT_EQ(_record.ua_type, UA_TYPE);
	EXPECT_EQ(_record.uid_size, sizeof(UID));
	EXPECT_EQ(memcmp(_record.uas_id, CN_PRODUCT, UAS_ID_SIZE), 0);
	EXPECT_EQ(memcmp(_record.uid, UID, sizeof(UID)), 0);

	for (size_t index = sizeof(UID); index < sizeof(_record.uid); ++index) {
		EXPECT_EQ(_record.uid[index], 0);
	}
}

TEST_F(OdidIdentityTest, CnProductRecordRejectsForeignUidAndCorruption)
{
	ASSERT_TRUE(recordCreate(_record, UAS_ID_TYPE_CN_PRODUCT_ID, UA_TYPE, CN_PRODUCT, sizeof(CN_PRODUCT) - 1, UID,
				 sizeof(UID)));
	uint8_t foreign_uid[sizeof(UID)];
	memcpy(foreign_uid, UID, sizeof(UID));
	foreign_uid[sizeof(foreign_uid) - 1] ^= 1;
	EXPECT_FALSE(recordValid(_record, foreign_uid, sizeof(foreign_uid)));
	EXPECT_FALSE(recordValid(_record, UID, sizeof(UID) - 1));

	for (size_t index = 0; index < sizeof(_record); ++index) {
		UasIdRecord corrupted = _record;
		reinterpret_cast<uint8_t *>(&corrupted)[index] ^= 1;
		EXPECT_FALSE(recordValid(corrupted, UID, sizeof(UID))) << index;
	}
}

TEST_F(OdidIdentityTest, CnProductRecordRejectsWrongTypeAndInvalidTextWithCorrectCrc)
{
	ASSERT_TRUE(recordCreate(_record, UAS_ID_TYPE_CN_PRODUCT_ID, UA_TYPE, CN_PRODUCT, sizeof(CN_PRODUCT) - 1, UID,
				 sizeof(UID)));
	UasIdRecord invalid = _record;
	invalid.id_type = UAS_ID_TYPE_SERIAL_NUMBER;
	updateCrc(invalid);
	EXPECT_FALSE(recordValid(invalid, UID, sizeof(UID)));

	const uint8_t invalid_characters[] = {'I', 'O', 'a', 0};

	for (uint8_t character : invalid_characters) {
		invalid = _record;
		invalid.uas_id[UAS_ID_SIZE - 1] = character;
		updateCrc(invalid);
		EXPECT_FALSE(recordValid(invalid, UID, sizeof(UID))) << +character;
	}
}

TEST_F(OdidIdentityTest, RecordRejectsUnsupportedWireTypesWithCorrectCrc)
{
	const uint8_t unsupported[] = {0, 2, 3, 4, 255};

	for (uint8_t type : unsupported) {
		_record.id_type = type;
		updateCrc(_record);
		EXPECT_FALSE(recordValid(_record, UID, sizeof(UID))) << +type;
	}
}

TEST(OdidRecordCompatibilityTest, ReadsExistingVersionTwoCtaRecord)
{
	// Existing little-endian v2 record for SERIAL/UID, before CN product support.
	// Keep these bytes fixed so recordCreate cannot silently migrate the fixture.
	const uint8_t stored[] = {
		0x4f, 0x44, 0x49, 0x44, 0x02, 0x01, 0x02, 0x0c, 0x41, 0x42,
		0x43, 0x44, 0x35, 0x31, 0x32, 0x33, 0x34, 0x35, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x23,
		0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0x10, 0x32, 0x54, 0x76,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb8, 0xc7, 0xda, 0xf3,
	};
	UasIdRecord record{};
	ASSERT_EQ(sizeof(record), sizeof(stored));
	memcpy(&record, stored, sizeof(record));
	EXPECT_TRUE(recordValid(record, UID, sizeof(UID)));
	EXPECT_EQ(record.id_type, UAS_ID_TYPE_SERIAL_NUMBER);
	EXPECT_EQ(record.ua_type, UA_TYPE);
	EXPECT_EQ(memcmp(record.uas_id, SERIAL, sizeof(SERIAL) - 1), 0);
}
