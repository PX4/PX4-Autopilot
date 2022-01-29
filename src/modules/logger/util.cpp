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
#include <sys/stat.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_gps_position.h>

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

typedef decltype(statfs::f_bavail) px4_statfs_buf_f_bavail_t;

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

bool get_log_time(struct tm *tt, int utc_offset_sec, bool boot_time)
{
	uORB::Subscription vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};

	time_t utc_time_sec;
	bool use_clock_time = true;

	/* Get the latest GPS publication */
	vehicle_gps_position_s gps_pos;

	if (vehicle_gps_position_sub.copy(&gps_pos)) {
		utc_time_sec = gps_pos.time_utc_usec / 1e6;

		if (gps_pos.fix_type >= 2 && utc_time_sec >= GPS_EPOCH_SECS) {
			use_clock_time = false;
		}
	}

	if (use_clock_time) {
		/* take clock time if there's no fix (yet) */
		struct timespec ts = {};
		px4_clock_gettime(CLOCK_REALTIME, &ts);
		utc_time_sec = ts.tv_sec + (ts.tv_nsec / 1e9);

		if (utc_time_sec < GPS_EPOCH_SECS) {
			return false;
		}
	}

	/* strip the time elapsed since boot */
	if (boot_time) {
		utc_time_sec -= hrt_absolute_time() / 1e6;
	}

	/* apply utc offset */
	utc_time_sec += utc_offset_sec;

	return gmtime_r(&utc_time_sec, tt) != nullptr;
}

int check_free_space(const char *log_root_dir, int32_t max_log_dirs_to_keep, orb_advert_t &mavlink_log_pub,
		     int &sess_dir_index)
{
	struct statfs statfs_buf;

	if (max_log_dirs_to_keep == 0) {
		max_log_dirs_to_keep = INT32_MAX;
	}

	// remove old logs if the free space falls below a threshold
	do {
		if (statfs(log_root_dir, &statfs_buf) != 0) {
			return PX4_ERROR;
		}

		DIR *dp = opendir(log_root_dir);

		if (dp == nullptr) {
			break; // ignore if we cannot access the log directory
		}

		struct dirent *result = nullptr;

		int num_sess = 0, num_dates = 0;

		// There are 2 directory naming schemes: sess<i> or <year>-<month>-<day>.
		// For both we find the oldest and then remove the one which has more directories.
		int year_min = 10000, month_min = 99, day_min = 99, sess_idx_min = 99999999, sess_idx_max = 99;

		while ((result = readdir(dp))) {
			int year, month, day, sess_idx;

			if (sscanf(result->d_name, "sess%d", &sess_idx) == 1) {
				++num_sess;

				if (sess_idx > sess_idx_max) {
					sess_idx_max = sess_idx;
				}

				if (sess_idx < sess_idx_min) {
					sess_idx_min = sess_idx;
				}

			} else if (sscanf(result->d_name, "%d-%d-%d", &year, &month, &day) == 3) {
				++num_dates;

				if (year < year_min) {
					year_min = year;
					month_min = month;
					day_min = day;

				} else if (year == year_min) {
					if (month < month_min) {
						month_min = month;
						day_min = day;

					} else if (month == month_min) {
						if (day < day_min) {
							day_min = day;
						}
					}
				}
			}
		}

		closedir(dp);

		sess_dir_index = sess_idx_max + 1;


		uint64_t min_free_bytes = 300ULL * 1024ULL * 1024ULL;
		uint64_t total_bytes = (uint64_t)statfs_buf.f_blocks * statfs_buf.f_bsize;

		if (total_bytes / 10 < min_free_bytes) { // reduce the minimum if it's larger than 10% of the disk size
			min_free_bytes = total_bytes / 10;
		}

		if (num_sess + num_dates <= max_log_dirs_to_keep &&
		    statfs_buf.f_bavail >= (px4_statfs_buf_f_bavail_t)(min_free_bytes / statfs_buf.f_bsize)) {
			break; // enough free space and limit not reached
		}

		if (num_sess == 0 && num_dates == 0) {
			break; // nothing to delete
		}

		char directory_to_delete[LOG_DIR_LEN];
		int n;

		if (num_sess >= num_dates) {
			n = snprintf(directory_to_delete, sizeof(directory_to_delete), "%s/sess%03u", log_root_dir, sess_idx_min);

		} else {
			n = snprintf(directory_to_delete, sizeof(directory_to_delete), "%s/%04u-%02u-%02u", log_root_dir, year_min, month_min,
				     day_min);
		}

		if (n >= (int)sizeof(directory_to_delete)) {
			PX4_ERR("log path too long (%i)", n);
			break;
		}

		PX4_INFO("removing log directory %s to get more space (left=%u MiB)", directory_to_delete,
			 (unsigned int)(statfs_buf.f_bavail * statfs_buf.f_bsize / 1024U / 1024U));

		if (remove_directory(directory_to_delete)) {
			PX4_ERR("Failed to delete directory");
			break;
		}

	} while (true);


	/* use a threshold of 50 MiB: if below, do not start logging */
	if (statfs_buf.f_bavail < (px4_statfs_buf_f_bavail_t)(50 * 1024 * 1024 / statfs_buf.f_bsize)) {
		mavlink_log_critical(&mavlink_log_pub,
				     "[logger] Not logging; SD almost full: %u MiB\t",
				     (unsigned int)(statfs_buf.f_bavail * statfs_buf.f_bsize / 1024U / 1024U));
		/* EVENT
		 * @description Either manually free up some space, or enable automatic log rotation
		 * via <param>SDLOG_DIRS_MAX</param>.
		 */
		events::send<uint32_t>(events::ID("logger_storage_full"), events::Log::Error,
				       "Not logging, storage is almost full: {1} MiB", (uint32_t)(statfs_buf.f_bavail * statfs_buf.f_bsize / 1024U / 1024U));
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
