/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include <stdint.h>
#include <climits>
#include <time.h>

#include <uORB/uORB.h>

#ifdef __PX4_NUTTX
#define LOG_DIR_LEN 64
#else
#define LOG_DIR_LEN 256
#endif

// Include parsing utilities (separate file for testability)
#include "util_parse.h"

namespace px4
{
namespace logger
{
namespace util
{

/**
 * Recursively remove a directory
 * @return 0 on success, <0 otherwise
 */
int remove_directory(const char *dir);

/**
 * check if a file exists
 */
bool file_exist(const char *filename);

/**
 * Get available and total storage space for a path.
 * @param path path to check
 * @param avail_bytes available bytes (output)
 * @param total_bytes total bytes (output), can be nullptr if not needed
 * @return true on success, false on error
 */
bool get_free_space(const char *path, uint64_t &avail_bytes, uint64_t *total_bytes);

/**
 * Scan log directory and gather information about subdirectories.
 * @param log_root_dir log root directory to scan
 * @param info output: populated with directory information
 * @return true on success, false if directory cannot be opened
 */
bool scan_log_directories(const char *log_root_dir, LogDirInfo &info);

/**
 * Cleanup old logs to ensure sufficient free space. Deletes oldest files
 * using "other type first" logic (sess dirs if we have time, date dirs if not).
 * @param log_root_dir log root directory
 * @param mavlink_log_pub mavlink log publisher
 * @param target_free_mb target free space in MB (0 = use default minimum)
 * @param max_log_dirs_to_keep maximum log directories to keep (0 = unlimited)
 * @return 0 on success, 1 if not enough space even after cleanup
 */
int cleanup_old_logs(const char *log_root_dir, orb_advert_t &mavlink_log_pub,
		     uint32_t target_free_mb, int32_t max_log_dirs_to_keep = 0);

/**
 * Utility for fetching UTC time in microseconds from sensor_gps or CLOCK_REALTIME
 * @param utc_time_usec returned microseconds
 * @param utc_offset_sec UTC time offset [s]
 * @param boot_time use time when booted instead of current time
 * @return true on success, false otherwise (eg. if no gps)
 */
bool get_log_time(uint64_t &utc_time_usec, int utc_offset_sec, bool boot_time);

/**
 * Get the time for log file name
 * @param tt returned time
 * @param utc_offset_sec UTC time offset [s]
 * @param boot_time use time when booted instead of current time
 * @return true on success, false otherwise (eg. if no gps)
 */
bool get_log_time(struct tm *tt, int utc_offset_sec = 0, bool boot_time = false);

} //namespace util
} //namespace logger
} //namespace px4
