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

#pragma once

#include <climits>

namespace px4
{
namespace logger
{
namespace util
{

/**
 * Information about log directories gathered by scan_log_directories()
 */
struct LogDirInfo {
	int sess_idx_max{-1};        ///< highest sess index found (-1 if none)
	int sess_idx_min{INT_MAX};   ///< lowest sess index found
	int num_sess{0};             ///< count of sess directories
	int oldest_year{INT_MAX};    ///< oldest date directory year
	int oldest_month{INT_MAX};   ///< oldest date directory month
	int oldest_day{INT_MAX};     ///< oldest date directory day
	int num_dates{0};            ///< count of date directories
};

/**
 * Process a single directory entry and update LogDirInfo accordingly.
 * Tries to parse as session dir first, then as date dir.
 * @param name directory entry name to process
 * @param info LogDirInfo struct to update
 */
void process_dir_entry(const char *name, LogDirInfo &info);

/**
 * Parse a session directory name (e.g., "sess001", "sess123")
 * @param name directory name to parse
 * @param sess_idx output: session index if parsing succeeds
 * @return true if name matches "sess%d" pattern
 */
bool parse_sess_dir_name(const char *name, int &sess_idx);

/**
 * Parse a date directory name (e.g., "2024-01-15")
 * @param name directory name to parse
 * @param year output: year if parsing succeeds
 * @param month output: month if parsing succeeds
 * @param day output: day if parsing succeeds
 * @return true if name matches "%d-%d-%d" pattern
 */
bool parse_date_dir_name(const char *name, int &year, int &month, int &day);

/**
 * Compare two dates
 * @return true if (y1,m1,d1) < (y2,m2,d2)
 */
bool is_date_older(int y1, int m1, int d1, int y2, int m2, int d2);

} //namespace util
} //namespace logger
} //namespace px4
