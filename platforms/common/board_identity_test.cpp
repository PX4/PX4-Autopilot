/****************************************************************************
 *
 *   Copyright (C) 2022 PX4 Development Team. All rights reserved.
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
#include <px4_platform_common/px4_config.h>
#include <string.h>

/**
 * @file board_identity_test.cpp
 * @author Junwoo Hwang <junwoo091400@gmail.com>
 *
 * Unit test for checking if board identification string retrieval function works correctly,
 * and without any unexpected buffer overflows
 */

/**
 * @brief Test `board_get_uuid32_formated` function for buffer overflows & correct formatting
 *
 * This is a deprecated function that gets used on exceptional cases where "BOARD_OVERRIDE_UUID" is defined.
 * Which is the case in SITL target as well as few others. In that case, the generic function `board_get_px4_guid_formated`
 * calls this function to fill out the buffer.
 *
 * It should write 16 bytes of UUID for SITL (PX4_CPU_UUID_BYTE_LENGTH depends on target!) in specified format, and if the
 * buffer has size less than that, the buffer shouldn't overflow, respecting the size argument.
 *
 * For example, with the format "02x", which writes each byte in 2-character hexadecimal value, the 16 bytes of UUID
 * will be written across '33' bytes of buffer, as each UUID byte will use 2 bytes, and NULL terminator will use 1 byte.
 *
 * If the buffer overflow occurs, the GCC compiler level will terminate the test with '*** stack smashing detected ***'
 * message, which should cancel the test
 *
 * TODO: Make the test fail properly, instead of getting canceled in stack smashing case
 */
TEST(BoardIdentityTest, UUID32BufferOverflow)
{
	// Get groundtruth UUID value. Without separator, should require 33 bytes
	char groundtruth_uuid_no_sep[50];
	board_get_uuid32_formated(groundtruth_uuid_no_sep, sizeof(groundtruth_uuid_no_sep), "%02x", NULL); // No separator
	EXPECT_EQ(32, strlen(groundtruth_uuid_no_sep));
	// Get groundtruth UUID value. With separator, should be 36 bytes
	char groundtruth_uuid_with_sep[50];
	board_get_uuid32_formated(groundtruth_uuid_with_sep, sizeof(groundtruth_uuid_with_sep), "%02x", "-"); // With separator
	EXPECT_EQ(35, strlen(groundtruth_uuid_with_sep));

	// 12 bytes : Shouldn't be enough, but shouldn't also cause buffer overflow
	// We check : 1. If the function causes buffer overflow | 2. If the buffer gets filled with correct data
	// Note: Since for `char buf[N]`, the maimum string length is N-1, we give N-1 for `strncmp` function.
	char buf_12[12];
	board_get_uuid32_formated(buf_12, sizeof(buf_12), "%02x", NULL); // No separator
	EXPECT_EQ(0, strncmp(groundtruth_uuid_no_sep, buf_12, sizeof(buf_12) - 1));
	board_get_uuid32_formated(buf_12, sizeof(buf_12), "%02x", "-"); // With separator
	EXPECT_EQ(0, strncmp(groundtruth_uuid_with_sep, buf_12, sizeof(buf_12) - 1));

	// 33 bytes: Should be enough for UUID with no separator, but not enough for UUID with separator
	char buf_33[33];
	board_get_uuid32_formated(buf_33, sizeof(buf_33), "%02x", NULL); // No separator
	EXPECT_EQ(0, strncmp(groundtruth_uuid_no_sep, buf_33, sizeof(buf_33) - 1));
	board_get_uuid32_formated(buf_33, sizeof(buf_33), "%02x", "-"); // With separator
	EXPECT_EQ(0, strncmp(groundtruth_uuid_with_sep, buf_33, sizeof(buf_33) - 1));

	// 40 bytes: Should be enough for UUID for all cases
	char buf_40[40];
	board_get_uuid32_formated(buf_40, sizeof(buf_40), "%02x", NULL); // No separator
	EXPECT_EQ(0, strncmp(groundtruth_uuid_no_sep, buf_40, sizeof(buf_40) - 1));
	board_get_uuid32_formated(buf_40, sizeof(buf_40), "%02x", "-"); // With separator
	EXPECT_EQ(0, strncmp(groundtruth_uuid_with_sep, buf_40, sizeof(buf_40) - 1));
}

/**
 * @brief Test `board_get_mfguid_formated` function for buffer overflows & correct formatting
 *
 * The size that should be written is PX4_CPU_MFGUID_FORMAT_SIZE, which is usually 33 (like UUID)
 */
TEST(BoardIdentityTest, MFGUIDBufferOverflow)
{
	// Get groundtruth MFGUID value
	char groundtruth_mfg_uid[50];
	board_get_mfguid_formated(groundtruth_mfg_uid, sizeof(groundtruth_mfg_uid));

	// Expect buffer of size PX4_CPU_MFGUID_FORMAT_SIZE, which means string length will be -1 of that.
	EXPECT_EQ(PX4_CPU_MFGUID_FORMAT_SIZE - 1, strlen(groundtruth_mfg_uid));

	// 12 bytes : Shouldn't be enough, but shouldn't also cause buffer overflow
	// We check : 1. If the function causes buffer overflow | 2. If the buffer gets filled with correct data
	// Note: Since for `char buf[N]`, the maimum string length is N-1, we give N-1 for `strncmp` function.
	char buf_12[12];
	board_get_mfguid_formated(buf_12, sizeof(buf_12));
	EXPECT_EQ(0, strncmp(groundtruth_mfg_uid, buf_12, sizeof(buf_12) - 1));

	// 33 bytes: Should be just enough for MFGUID
	char buf_33[33];
	board_get_mfguid_formated(buf_33, sizeof(buf_33));
	EXPECT_EQ(0, strncmp(groundtruth_mfg_uid, buf_33, sizeof(buf_33) - 1));

	// 40 bytes: Should be enough for MFGUID
	char buf_40[40];
	board_get_mfguid_formated(buf_40, sizeof(buf_40));
	EXPECT_EQ(0, strncmp(groundtruth_mfg_uid, buf_40, sizeof(buf_40) - 1));
}

/**
 * @brief Test `board_get_px4_guid_formated` function for buffer overflows & correct formatting
 *
 * The size that needs to be written is PX4_GUID_FORMAT_SIZE, which includes the NULL
 * termination character into account, which is usually 33.
 */
TEST(BoardIdentityTest, PX4GUIDBufferOverflow)
{
	// Get groundtruth PX4 GUID value with enough buffer (minimum 32 + 1)
	char groundtruth_px4_guid[50];
	board_get_px4_guid_formated(groundtruth_px4_guid, sizeof(groundtruth_px4_guid));

	// Expect buffer of size PX4_GUID_FORMAT_SIZE, which means string length will be -1 of that.
	EXPECT_EQ(PX4_GUID_FORMAT_SIZE - 1, strlen(groundtruth_px4_guid));

	// 12 bytes : Shouldn't be enough, but shouldn't also cause buffer overflow
	// We check : 1. If the function causes buffer overflow | 2. If the buffer gets filled with correct data
	// Note: Since for `char buf[N]`, the maimum string length is N-1, we give N-1 for `strncmp` function.
	char buf_12[12];
	board_get_px4_guid_formated(buf_12, sizeof(buf_12));
	EXPECT_EQ(0, strncmp(groundtruth_px4_guid, buf_12, sizeof(buf_12) - 1));

	// 33 bytes: Should be just enough for PX4 GUID
	char buf_33[33];
	board_get_px4_guid_formated(buf_33, sizeof(buf_33));
	EXPECT_EQ(0, strncmp(groundtruth_px4_guid, buf_33, sizeof(buf_33) - 1));

	// 40 bytes: Should be enough for PX4 GUID
	char buf_40[40];
	board_get_px4_guid_formated(buf_40, sizeof(buf_40));
	EXPECT_EQ(0, strncmp(groundtruth_px4_guid, buf_40, sizeof(buf_40) - 1));
}
