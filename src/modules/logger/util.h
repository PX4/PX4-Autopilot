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
#include <time.h>

#include <uORB/uORB.h>

#ifdef __PX4_NUTTX
#define LOG_DIR_LEN 64
#else
#define LOG_DIR_LEN 256
#endif

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
 * Check if there is enough free space left on the SD Card.
 * It will remove old log files if there is not enough space,
 * and if that fails return 1, and send a user message
 * @param log_root_dir log root directory: it's expected to contain directories in the form of sess%i or %d-%d-%d (year, month, day)
 * @param max_log_dirs_to_keep maximum log directories to keep (set to 0 for unlimited)
 * @param mavlink_log_pub
 * @param sess_dir_index output argument: will be set to the next free directory sess%i index.
 * @return 0 on success, 1 if not enough space, <0 on error
 */
int check_free_space(const char *log_root_dir, int32_t max_log_dirs_to_keep, orb_advert_t &mavlink_log_pub,
		     int &sess_dir_index);

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
