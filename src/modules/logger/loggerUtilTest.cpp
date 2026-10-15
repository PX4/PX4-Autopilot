/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

/**
 * Test code for the logger utility parsing functions
 * Run this test using: build/px4_sitl_test/unit-util
 */

#include <gtest/gtest.h>
#include "util_parse.h"

using namespace px4::logger::util;

TEST(LoggerUtilTest, ProcessDirEntryMixedSessAndDate)
{
	LogDirInfo info{};

	process_dir_entry("sess001", info);
	process_dir_entry("2024-03-15", info);
	process_dir_entry("sess005", info);
	process_dir_entry("2023-12-01", info);

	EXPECT_EQ(info.num_sess, 2);
	EXPECT_EQ(info.sess_idx_min, 1);
	EXPECT_EQ(info.sess_idx_max, 5);

	EXPECT_EQ(info.num_dates, 2);
	EXPECT_EQ(info.oldest_year, 2023);
	EXPECT_EQ(info.oldest_month, 12);
	EXPECT_EQ(info.oldest_day, 1);
}

TEST(LoggerUtilTest, ProcessDirEntryIgnoresInvalid)
{
	LogDirInfo info{};

	// These should be ignored
	process_dir_entry(".", info);
	process_dir_entry("..", info);
	process_dir_entry("log001", info);
	process_dir_entry("session001", info);
	process_dir_entry("2024-01", info);
	process_dir_entry("random_dir", info);

	EXPECT_EQ(info.num_sess, 0);
	EXPECT_EQ(info.num_dates, 0);
	EXPECT_EQ(info.sess_idx_max, -1);       // unchanged from default
	EXPECT_EQ(info.sess_idx_min, INT_MAX);  // unchanged from default
}

TEST(LoggerUtilTest, ProcessDirEntryDateOldestTracking)
{
	LogDirInfo info{};

	// Start with a date in the middle
	process_dir_entry("2024-06-15", info);
	EXPECT_EQ(info.oldest_year, 2024);
	EXPECT_EQ(info.oldest_month, 6);
	EXPECT_EQ(info.oldest_day, 15);

	// Add older year
	process_dir_entry("2023-12-31", info);
	EXPECT_EQ(info.oldest_year, 2023);
	EXPECT_EQ(info.oldest_month, 12);
	EXPECT_EQ(info.oldest_day, 31);

	// Add same year, older month
	process_dir_entry("2023-01-15", info);
	EXPECT_EQ(info.oldest_year, 2023);
	EXPECT_EQ(info.oldest_month, 1);
	EXPECT_EQ(info.oldest_day, 15);

	// Add same year/month, older day
	process_dir_entry("2023-01-01", info);
	EXPECT_EQ(info.oldest_year, 2023);
	EXPECT_EQ(info.oldest_month, 1);
	EXPECT_EQ(info.oldest_day, 1);

	// Add newer date (oldest should not change)
	process_dir_entry("2025-01-01", info);
	EXPECT_EQ(info.oldest_year, 2023);
	EXPECT_EQ(info.oldest_month, 1);
	EXPECT_EQ(info.oldest_day, 1);
}
