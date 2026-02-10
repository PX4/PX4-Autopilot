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

#include "util.h"

#include <dirent.h>
#include <inttypes.h>
#include <sys/stat.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <systemlib/mavlink_log.h>

#if defined(__PX4_DARWIN)
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif

#define GPS_EPOCH_SECS ((time_t)1234567890ULL)

namespace px4
{
namespace logger
{
namespace util
{

bool file_exist(const char *filename)
{
	struct stat buffer;
	return stat(filename, &buffer) == 0;
}

bool get_free_space(const char *path, uint64_t *avail_bytes, uint64_t *total_bytes)
{
	struct statfs statfs_buf;

	if (statfs(path, &statfs_buf) != 0) {
		return false;
	}

	if (avail_bytes != nullptr) {
		*avail_bytes = (uint64_t)statfs_buf.f_bavail * statfs_buf.f_bsize;
	}

	if (total_bytes != nullptr) {
		*total_bytes = (uint64_t)statfs_buf.f_blocks * statfs_buf.f_bsize;
	}

	return true;
}

bool get_log_time(uint64_t &utc_time_usec, int utc_offset_sec, bool boot_time)
{
	struct timespec ts = {};
	px4_clock_gettime(CLOCK_REALTIME, &ts);
	utc_time_usec = ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;

	if (utc_time_usec < (uint64_t) GPS_EPOCH_SECS * 1000000ULL) {
		return false;
	}

	/* strip the time elapsed since boot */
	if (boot_time) {
		utc_time_usec -= hrt_absolute_time();
	}

	/* apply utc offset */
	utc_time_usec += (int64_t) utc_offset_sec * 1000000LL;

	return true;
}

bool get_log_time(struct tm *tt, int utc_offset_sec, bool boot_time)
{
	uint64_t utc_time_usec;
	bool result = get_log_time(utc_time_usec, utc_offset_sec, boot_time);
	time_t utc_time_sec = static_cast<time_t>(utc_time_usec / 1000000ULL);
	return result && gmtime_r(&utc_time_sec, tt) != nullptr;
}

bool scan_log_directories(const char *log_root_dir, LogDirInfo &info)
{
	DIR *dp = opendir(log_root_dir);

	if (dp == nullptr) {
		return false;
	}

	// Reset info to defaults
	info = LogDirInfo{};

	struct dirent *result = nullptr;

	while ((result = readdir(dp))) {
		process_dir_entry(result->d_name, info);
	}

	closedir(dp);
	return true;
}

int cleanup_old_logs(const char *log_root_dir, orb_advert_t &mavlink_log_pub,
		     uint32_t target_free_mb, int32_t max_log_dirs_to_keep)
{
	uint64_t avail_bytes;
	uint64_t total_bytes;

	if (!get_free_space(log_root_dir, &avail_bytes, &total_bytes)) {
		return PX4_ERROR;
	}

	// Calculate cleanup threshold
	uint64_t cleanup_threshold;

	if (target_free_mb > 0) {
		cleanup_threshold = (uint64_t)target_free_mb * 1024ULL * 1024ULL;

	} else {
		// Default: 300 MiB or 10% of disk, whichever is smaller
		cleanup_threshold = 300ULL * 1024ULL * 1024ULL;

		if (total_bytes / 10 < cleanup_threshold) {
			cleanup_threshold = total_bytes / 10;
		}
	}

	// Early out if we have enough space and no directory limit
	bool need_space_cleanup = avail_bytes < cleanup_threshold;

	if (!need_space_cleanup && max_log_dirs_to_keep <= 0) {
		return PX4_OK;
	}

	// Scan directories for cleanup
	LogDirInfo info;

	if (!scan_log_directories(log_root_dir, info)) {
		return PX4_OK; // ignore if we cannot access the log directory
	}

	int total_dirs = info.num_sess + info.num_dates;
	bool need_count_cleanup = (max_log_dirs_to_keep > 0) && (total_dirs > max_log_dirs_to_keep);

	if (!need_space_cleanup && !need_count_cleanup) {
		return PX4_OK;
	}

	PX4_INFO("Log cleanup: %u MiB free, threshold %u MiB, %d dirs (max %" PRId32 ")",
		 (unsigned)(avail_bytes / 1024U / 1024U), (unsigned)(cleanup_threshold / 1024U / 1024U),
		 total_dirs, max_log_dirs_to_keep > 0 ? max_log_dirs_to_keep : -1);

	// Determine if we currently have valid time (using date dirs) or not (using sess dirs)
	// Delete from the "other" scheme first to avoid deleting current log
	uint64_t utc_time_usec;
	bool have_time = get_log_time(utc_time_usec, 0, false);

	// Cleanup oldest .ulg files one by one until conditions are met
	int empty_dir_failures = 0;

	while (need_space_cleanup || need_count_cleanup) {
		char oldest_file[LOG_DIR_LEN] = "";
		char oldest_dir_name[64] = "";

		if (!scan_log_directories(log_root_dir, info)) {
			break;
		}

		total_dirs = info.num_sess + info.num_dates;
		bool found_sess = info.num_sess > 0;
		bool found_date = info.num_dates > 0;

		if (!found_sess && !found_date) {
			PX4_WARN("No log directories found to clean up");
			break; // no log directories found
		}

		// Delete from the "other" naming scheme first (it's old/stale)
		// - Have time (using date dirs): delete sess dirs first
		// - No time (using sess dirs): delete date dirs first, then sess dirs
		char oldest_dir[LOG_DIR_LEN];

		if (have_time && found_sess) {
			// Using date dirs, delete old sess dirs first
			snprintf(oldest_dir, sizeof(oldest_dir), "%s/sess%03u", log_root_dir, info.sess_idx_min);
			snprintf(oldest_dir_name, sizeof(oldest_dir_name), "sess%03u", info.sess_idx_min);

		} else if (!have_time && found_date) {
			// Using sess dirs, delete old date dirs first
			snprintf(oldest_dir, sizeof(oldest_dir), "%s/%04u-%02u-%02u", log_root_dir,
				 info.oldest_year, info.oldest_month, info.oldest_day);
			snprintf(oldest_dir_name, sizeof(oldest_dir_name), "%04u-%02u-%02u",
				 info.oldest_year, info.oldest_month, info.oldest_day);

		} else if (found_sess) {
			// Delete from oldest sess dir (including current - old files are ok to delete)
			snprintf(oldest_dir, sizeof(oldest_dir), "%s/sess%03u", log_root_dir, info.sess_idx_min);
			snprintf(oldest_dir_name, sizeof(oldest_dir_name), "sess%03u", info.sess_idx_min);

		} else if (found_date) {
			// Delete from oldest date dir
			snprintf(oldest_dir, sizeof(oldest_dir), "%s/%04u-%02u-%02u", log_root_dir,
				 info.oldest_year, info.oldest_month, info.oldest_day);
			snprintf(oldest_dir_name, sizeof(oldest_dir_name), "%04u-%02u-%02u",
				 info.oldest_year, info.oldest_month, info.oldest_day);

		} else {
			// Nothing left to delete
			break;
		}

		PX4_DEBUG("Checking directory %s for old logs", oldest_dir_name);

		// Find oldest .ulg file in that directory
		DIR *dp = opendir(oldest_dir);

		if (dp == nullptr) {
			PX4_WARN("Cannot open directory %s", oldest_dir_name);
			break;
		}

		char oldest_ulg[64] = "";
		struct dirent *result = nullptr;

		while ((result = readdir(dp))) {
			size_t len = strlen(result->d_name);

			if (len > 4 && strcmp(result->d_name + len - 4, ".ulg") == 0) {
				if (oldest_ulg[0] == '\0' || strcmp(result->d_name, oldest_ulg) < 0) {
					strncpy(oldest_ulg, result->d_name, sizeof(oldest_ulg) - 1);
					oldest_ulg[sizeof(oldest_ulg) - 1] = '\0';
				}
			}
		}

		closedir(dp);

		if (oldest_ulg[0] == '\0') {
			// No .ulg files, try to remove directory
			if (remove_directory(oldest_dir) == 0) {
				PX4_INFO("removed directory %s (no .ulg files)", oldest_dir_name);
				empty_dir_failures = 0;

			} else {
				// Removal failed (littlefs may report "not empty" for empty dirs)
				// Toggle have_time to try the other naming scheme next iteration
				empty_dir_failures++;

				if (empty_dir_failures >= 3) {
					PX4_WARN("Cannot remove empty directories, giving up");
					break;
				}

				have_time = !have_time;
				PX4_DEBUG("Cannot remove %s, trying other scheme", oldest_dir_name);
			}

			continue;
		}

		// Build full path and delete the file
		snprintf(oldest_file, sizeof(oldest_file), "%s/%s", oldest_dir, oldest_ulg);
		PX4_INFO("removing old log %s/%s", oldest_dir_name, oldest_ulg);

		if (unlink(oldest_file) != 0) {
			PX4_ERR("Failed to delete %s", oldest_file);
			break;
		}

		// Re-check conditions
		if (!get_free_space(log_root_dir, &avail_bytes, nullptr)) {
			break;
		}

		need_space_cleanup = avail_bytes < cleanup_threshold;
		need_count_cleanup = (max_log_dirs_to_keep > 0) && (total_dirs > max_log_dirs_to_keep);
	}

	// Final check: if still not enough space, refuse to log
	if (avail_bytes < 10ULL * 1024ULL * 1024ULL) {  // Less than 10 MiB is critical
		mavlink_log_critical(&mavlink_log_pub, "[logger] Storage full: %u MiB free\t",
				     (unsigned)(avail_bytes / 1024U / 1024U));
		return 1;
	}

	return PX4_OK;
}

int remove_directory(const char *dir)
{
	DIR *d = opendir(dir);
	size_t dir_len = strlen(dir);
	struct dirent *p;
	int ret = 0;

	if (!d) {
		return -1;
	}

	while (!ret && (p = readdir(d))) {
		int ret2 = -1;
		char *buf;
		size_t len;

		if (!strcmp(p->d_name, ".") || !strcmp(p->d_name, "..")) {
			continue;
		}

		len = dir_len + strlen(p->d_name) + 2;
		buf = new char[len];

		if (buf) {
			struct stat statbuf;

			snprintf(buf, len, "%s/%s", dir, p->d_name);

			if (!stat(buf, &statbuf)) {
				if (S_ISDIR(statbuf.st_mode)) {
					ret2 = remove_directory(buf);

				} else {
					ret2 = unlink(buf);
				}
			}

			delete[] buf;
		}

		ret = ret2;
	}

	closedir(d);

	if (!ret) {
		ret = rmdir(dir);
	}

	return ret;
}

} //namespace util
} //namespace logger
} //namespace px4
