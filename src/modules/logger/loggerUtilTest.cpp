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

// Session directory parsing tests
TEST(LoggerUtilTest, ParseSessDirValid)
{
	int idx;
	EXPECT_TRUE(parse_sess_dir_name("sess000", idx));
	EXPECT_EQ(idx, 0);

	EXPECT_TRUE(parse_sess_dir_name("sess001", idx));
	EXPECT_EQ(idx, 1);

	EXPECT_TRUE(parse_sess_dir_name("sess123", idx));
	EXPECT_EQ(idx, 123);

	EXPECT_TRUE(parse_sess_dir_name("sess999", idx));
	EXPECT_EQ(idx, 999);
}

TEST(LoggerUtilTest, ParseSessDirInvalid)
{
	int idx;
	EXPECT_FALSE(parse_sess_dir_name("session001", idx));
	EXPECT_FALSE(parse_sess_dir_name("2024-01-15", idx));
	EXPECT_FALSE(parse_sess_dir_name(".", idx));
	EXPECT_FALSE(parse_sess_dir_name("..", idx));
	EXPECT_FALSE(parse_sess_dir_name("log001", idx));
	EXPECT_FALSE(parse_sess_dir_name("", idx));
}

// Date directory parsing tests
TEST(LoggerUtilTest, ParseDateDirValid)
{
	int y, m, d;
	EXPECT_TRUE(parse_date_dir_name("2024-01-15", y, m, d));
	EXPECT_EQ(y, 2024);
	EXPECT_EQ(m, 1);
	EXPECT_EQ(d, 15);

	EXPECT_TRUE(parse_date_dir_name("2023-12-31", y, m, d));
	EXPECT_EQ(y, 2023);
	EXPECT_EQ(m, 12);
	EXPECT_EQ(d, 31);

	EXPECT_TRUE(parse_date_dir_name("2025-06-01", y, m, d));
	EXPECT_EQ(y, 2025);
	EXPECT_EQ(m, 6);
	EXPECT_EQ(d, 1);
}

TEST(LoggerUtilTest, ParseDateDirInvalid)
{
	int y, m, d;
	EXPECT_FALSE(parse_date_dir_name("sess001", y, m, d));
	EXPECT_FALSE(parse_date_dir_name("2024-01", y, m, d));
	EXPECT_FALSE(parse_date_dir_name("2024", y, m, d));
	EXPECT_FALSE(parse_date_dir_name(".", y, m, d));
	EXPECT_FALSE(parse_date_dir_name("..", y, m, d));
	EXPECT_FALSE(parse_date_dir_name("", y, m, d));
}

// Date comparison tests
TEST(LoggerUtilTest, IsDateOlderYearDifference)
{
	// Earlier year is older
	EXPECT_TRUE(is_date_older(2023, 12, 31, 2024, 1, 1));
	EXPECT_FALSE(is_date_older(2024, 1, 1, 2023, 12, 31));
}

TEST(LoggerUtilTest, IsDateOlderMonthDifference)
{
	// Same year, earlier month is older
	EXPECT_TRUE(is_date_older(2024, 1, 15, 2024, 2, 1));
	EXPECT_FALSE(is_date_older(2024, 2, 1, 2024, 1, 15));
}

TEST(LoggerUtilTest, IsDateOlderDayDifference)
{
	// Same year and month, earlier day is older
	EXPECT_TRUE(is_date_older(2024, 1, 1, 2024, 1, 15));
	EXPECT_FALSE(is_date_older(2024, 1, 15, 2024, 1, 1));
}

TEST(LoggerUtilTest, IsDateOlderSameDate)
{
	// Same date is not older
	EXPECT_FALSE(is_date_older(2024, 1, 15, 2024, 1, 15));
}

TEST(LoggerUtilTest, IsDateOlderEdgeCases)
{
	// Year boundary
	EXPECT_TRUE(is_date_older(2023, 12, 31, 2024, 1, 1));

	// Month boundary
	EXPECT_TRUE(is_date_older(2024, 1, 31, 2024, 2, 1));

	// Large year difference
	EXPECT_TRUE(is_date_older(2000, 6, 15, 2024, 6, 15));
}

// process_dir_entry tests - combined sess vs date logic
TEST(LoggerUtilTest, ProcessDirEntrySessionOnly)
{
	LogDirInfo info{};

	process_dir_entry("sess000", info);
	EXPECT_EQ(info.num_sess, 1);
	EXPECT_EQ(info.sess_idx_min, 0);
	EXPECT_EQ(info.sess_idx_max, 0);
	EXPECT_EQ(info.num_dates, 0);

	process_dir_entry("sess005", info);
	EXPECT_EQ(info.num_sess, 2);
	EXPECT_EQ(info.sess_idx_min, 0);
	EXPECT_EQ(info.sess_idx_max, 5);

	process_dir_entry("sess003", info);
	EXPECT_EQ(info.num_sess, 3);
	EXPECT_EQ(info.sess_idx_min, 0);
	EXPECT_EQ(info.sess_idx_max, 5);
}

TEST(LoggerUtilTest, ProcessDirEntryDateOnly)
{
	LogDirInfo info{};

	process_dir_entry("2024-06-15", info);
	EXPECT_EQ(info.num_dates, 1);
	EXPECT_EQ(info.oldest_year, 2024);
	EXPECT_EQ(info.oldest_month, 6);
	EXPECT_EQ(info.oldest_day, 15);
	EXPECT_EQ(info.num_sess, 0);

	// Add older date
	process_dir_entry("2024-01-10", info);
	EXPECT_EQ(info.num_dates, 2);
	EXPECT_EQ(info.oldest_year, 2024);
	EXPECT_EQ(info.oldest_month, 1);
	EXPECT_EQ(info.oldest_day, 10);

	// Add newer date (oldest should not change)
	process_dir_entry("2024-12-25", info);
	EXPECT_EQ(info.num_dates, 3);
	EXPECT_EQ(info.oldest_year, 2024);
	EXPECT_EQ(info.oldest_month, 1);
	EXPECT_EQ(info.oldest_day, 10);
}

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

TEST(LoggerUtilTest, ProcessDirEntrySessMinMaxTracking)
{
	LogDirInfo info{};

	// Add in non-sequential order
	process_dir_entry("sess050", info);
	EXPECT_EQ(info.sess_idx_min, 50);
	EXPECT_EQ(info.sess_idx_max, 50);

	process_dir_entry("sess010", info);
	EXPECT_EQ(info.sess_idx_min, 10);
	EXPECT_EQ(info.sess_idx_max, 50);

	process_dir_entry("sess100", info);
	EXPECT_EQ(info.sess_idx_min, 10);
	EXPECT_EQ(info.sess_idx_max, 100);

	process_dir_entry("sess025", info);
	EXPECT_EQ(info.sess_idx_min, 10);
	EXPECT_EQ(info.sess_idx_max, 100);
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
