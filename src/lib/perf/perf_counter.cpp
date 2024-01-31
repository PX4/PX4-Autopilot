/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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
 * @file perf_counter.c
 *
 * @brief Performance measuring tools.
 */

#ifndef MODULE_NAME
#define MODULE_NAME "perf"
#endif

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <systemlib/err.h>
#include <dirent.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <px4_platform_common/mmap.h>

#include "perf_counter.h"

#define PERF_SHMNAME_PREFIX	"_perf_"
#define PERF_SHMNAME_STR	PERF_SHMNAME_PREFIX "%s"
#define PERF_SHMNAME_MAX	NAME_MAX + 1

#ifndef CONFIG_FS_SHMFS_VFS_PATH
#define CONFIG_FS_SHMFS_VFS_PATH "/dev/shm"
#endif

/**
 * Header common to all counters.
 */
struct perf_ctr_header {
	enum perf_counter_type	type;	/**< counter type */
#ifdef CONFIG_BUILD_FLAT
	const char		*name;	/**< counter name */
#else
	char 			name[PERF_SHMNAME_MAX]; /**< counter name */
#endif
};

/**
 * PC_EVENT counter.
 */
struct perf_ctr_count : public perf_ctr_header {
	uint64_t		event_count{0};
};

/**
 * PC_ELAPSED counter.
 */
struct perf_ctr_elapsed : public perf_ctr_header {
	uint64_t		event_count{0};
	uint64_t		time_start{0};
	uint64_t		time_total{0};
	uint32_t		time_least{0};
	uint32_t		time_most{0};
	float			mean{0.0f};
	float			M2{0.0f};
};

/**
 * PC_INTERVAL counter.
 */
struct perf_ctr_interval : public perf_ctr_header {
	uint64_t		event_count{0};
	uint64_t		time_event{0};
	uint64_t		time_first{0};
	uint64_t		time_last{0};
	uint32_t		time_least{0};
	uint32_t		time_most{0};
	float			mean{0.0f};
	float			M2{0.0f};
};

static size_t perf_size(enum perf_counter_type type)
{
	switch (type) {
	case PC_COUNT:
		return sizeof(perf_ctr_count);

	case PC_ELAPSED:
		return sizeof(perf_ctr_elapsed);

	case PC_INTERVAL:
		return sizeof(perf_ctr_interval);

	default:
		break;
	}

	return 0;
}

static size_t perf_shmsize(int fd)
{
	struct stat sb;

	if (fstat(fd, &sb) < 0) {
		PX4_ERR("Cannot stat %d", fd);
	}

	return sb.st_size;
}

static int perf_shmname(char *shmname, const char *name, size_t size)
{
	int ret = snprintf(shmname, size, PERF_SHMNAME_STR, name);

	if ((size_t)ret >= size) {
		return -ENAMETOOLONG;
	}

	return OK;
}

static void perf_foreach(perf_callback cb, void *arg = NULL)
{
	DIR *dir = opendir(CONFIG_FS_SHMFS_VFS_PATH);
	struct dirent *perf;

	while ((perf = readdir(dir)) != nullptr) {
		if (!strncmp(perf->d_name, PERF_SHMNAME_PREFIX, sizeof(PERF_SHMNAME_PREFIX) - 1)) {
			/* found the counter, map it for us */
			int fd = shm_open(perf->d_name, O_RDWR, 0666);

			if (fd >= 0) {
				size_t size = perf_shmsize(fd);
				void *p = px4_mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

				/* the file can be closed now */
				close(fd);

				if (p != MAP_FAILED) {
					cb((perf_counter_t)p, arg);
					px4_munmap(p, size);
				}
			}
		}
	}
}

static void perf_print_cb(perf_counter_t handle, void *arg)
{
	perf_print_counter(handle);
}

static void perf_reset_cb(perf_counter_t handle, void *arg)
{
	perf_reset(handle);
}

perf_counter_t
perf_alloc(enum perf_counter_type type, const char *name)
{
	perf_counter_t ctr;
	char shmname[PERF_SHMNAME_MAX];
	int ret;
	int fd;

	/* generate name for shmfs file */
	ret = perf_shmname(shmname, name, sizeof(shmname));

	if (ret < 0) {
		PX4_ERR("failed to allocate perf counter %s", name);
		return nullptr;
	}

	/* try to allocate new shm object and map it */
	ctr = nullptr;
	fd = shm_open(shmname, O_CREAT | O_RDWR | O_EXCL, 0666);

	if (fd >= 0) {
		ret = ftruncate(fd, perf_size(type));

		if (ret == 0) {
			void *p = px4_mmap(0, perf_size(type), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

			if (p != MAP_FAILED) {
				ctr = (perf_counter_t)p;
			}
		}
	}

	close(fd);

	if (ctr) {
		ctr->type = type;
#ifdef CONFIG_BUILD_FLAT
		ctr->name = name;
#else
		strncpy(ctr->name, name, PERF_SHMNAME_MAX);
#endif
	}

	return ctr;
}

perf_counter_t
perf_alloc_once(enum perf_counter_type type, const char *name)
{
	DIR *dir = opendir(CONFIG_FS_SHMFS_VFS_PATH);
	struct dirent *perf;
	char shmname[PERF_SHMNAME_MAX];
	int ret;

	/* generate name for shmfs file */
	ret = perf_shmname(shmname, name, sizeof(shmname));

	if (ret < 0) {
		PX4_ERR("failed to allocate perf counter %s", name);
		return nullptr;
	}

	while ((perf = readdir(dir)) != nullptr) {
		if (!strncmp(perf->d_name, shmname, sizeof(shmname))) {
			/* found the counter, map it for us */
			int fd = shm_open(perf->d_name, O_RDWR, 0666);

			if (fd >= 0) {
				void *p = px4_mmap(0, perf_size(type), PROT_READ, MAP_SHARED, fd, 0);

				/* the file can be closed now */
				close(fd);

				if (p == MAP_FAILED) {
					return nullptr;
				}

				return (perf_counter_t)p;
			}
		}
	}

	/* if the execution reaches here, no existing counter of that name was found */
	return perf_alloc(type, name);
}

void
perf_free(perf_counter_t handle)
{
	if (handle) {
		char shmname[PERF_SHMNAME_MAX];

		/* should not fail, if object creation succeeded ? */
		if (perf_shmname(shmname, handle->name, sizeof(shmname) >= 0)) {
			shm_unlink(shmname);
		}

		px4_munmap(handle, perf_size(handle->type));
	}
}

void
perf_count(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT:
		((struct perf_ctr_count *)handle)->event_count++;
		break;

	case PC_INTERVAL:
		perf_count_interval(handle, hrt_absolute_time());
		break;

	default:
		break;
	}
}

void
perf_begin(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED:
		((struct perf_ctr_elapsed *)handle)->time_start = hrt_absolute_time();
		break;

	default:
		break;
	}
}

void
perf_end(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;

			if (pce->time_start != 0) {
				perf_set_elapsed(handle, hrt_elapsed_time(&pce->time_start));
			}
		}
		break;

	default:
		break;
	}
}

void
perf_set_elapsed(perf_counter_t handle, int64_t elapsed)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;

			if (elapsed >= 0) {
				pce->event_count++;
				pce->time_total += elapsed;

				if ((pce->time_least > (uint32_t)elapsed) || (pce->time_least == 0)) {
					pce->time_least = elapsed;
				}

				if (pce->time_most < (uint32_t)elapsed) {
					pce->time_most = elapsed;
				}

				// maintain mean and variance of the elapsed time in seconds
				// Knuth/Welford recursive mean and variance of update intervals (via Wikipedia)
				float dt = elapsed / 1e6f;
				float delta_intvl = dt - pce->mean;
				pce->mean += delta_intvl / pce->event_count;
				pce->M2 += delta_intvl * (dt - pce->mean);

				pce->time_start = 0;
			}
		}
		break;

	default:
		break;
	}
}

void
perf_count_interval(perf_counter_t handle, hrt_abstime now)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;

			switch (pci->event_count) {
			case 0:
				pci->time_first = now;
				break;

			case 1:
				pci->time_least = (uint32_t)(now - pci->time_last);
				pci->time_most = (uint32_t)(now - pci->time_last);
				pci->mean = pci->time_least / 1e6f;
				pci->M2 = 0;
				break;

			default: {
					hrt_abstime interval = now - pci->time_last;

					if ((uint32_t)interval < pci->time_least) {
						pci->time_least = (uint32_t)interval;
					}

					if ((uint32_t)interval > pci->time_most) {
						pci->time_most = (uint32_t)interval;
					}

					// maintain mean and variance of interval in seconds
					// Knuth/Welford recursive mean and variance of update intervals (via Wikipedia)
					float dt = interval / 1e6f;
					float delta_intvl = dt - pci->mean;
					pci->mean += delta_intvl / pci->event_count;
					pci->M2 += delta_intvl * (dt - pci->mean);
					break;
				}
			}

			pci->time_last = now;
			pci->event_count++;
			break;
		}

	default:
		break;
	}
}

void
perf_set_count(perf_counter_t handle, uint64_t count)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT: {
			((struct perf_ctr_count *)handle)->event_count = count;
		}
		break;

	default:
		break;
	}

}

void
perf_cancel(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;

			pce->time_start = 0;
		}
		break;

	default:
		break;
	}
}

void
perf_reset(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT:
		((struct perf_ctr_count *)handle)->event_count = 0;
		break;

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			pce->event_count = 0;
			pce->time_start = 0;
			pce->time_total = 0;
			pce->time_least = 0;
			pce->time_most = 0;
			break;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			pci->event_count = 0;
			pci->time_event = 0;
			pci->time_first = 0;
			pci->time_last = 0;
			pci->time_least = 0;
			pci->time_most = 0;
			break;
		}
	}
}

void
perf_print_counter(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT:
		PX4_INFO_RAW("%s: %" PRIu64 " events\n",
			     handle->name,
			     ((struct perf_ctr_count *)handle)->event_count);
		break;

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			float rms = sqrtf(pce->M2 / (pce->event_count - 1));
			PX4_INFO_RAW("%s: %" PRIu64 " events, %" PRIu64 "us elapsed, %.2fus avg, min %" PRIu32 "us max %" PRIu32
				     "us %5.3fus rms\n",
				     handle->name,
				     pce->event_count,
				     pce->time_total,
				     (pce->event_count == 0) ? 0 : (double)pce->time_total / (double)pce->event_count,
				     pce->time_least,
				     pce->time_most,
				     (double)(1e6f * rms));
			break;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			float rms = sqrtf(pci->M2 / (pci->event_count - 1));

			PX4_INFO_RAW("%s: %" PRIu64 " events, %.2fus avg, min %" PRIu32 "us max %" PRIu32 "us %5.3fus rms\n",
				     handle->name,
				     pci->event_count,
				     (pci->event_count == 0) ? 0 : (double)(pci->time_last - pci->time_first) / (double)pci->event_count,
				     pci->time_least,
				     pci->time_most,
				     (double)(1e6f * rms));
			break;
		}

	default:
		break;
	}
}


int
perf_print_counter_buffer(char *buffer, int length, perf_counter_t handle)
{
	int num_written = 0;

	if (handle == nullptr) {
		return 0;
	}

	switch (handle->type) {
	case PC_COUNT:
		num_written = snprintf(buffer, length, "%s: %" PRIu64 " events",
				       handle->name,
				       ((struct perf_ctr_count *)handle)->event_count);
		break;

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			float rms = sqrtf(pce->M2 / (pce->event_count - 1));
			num_written = snprintf(buffer, length,
					       "%s: %" PRIu64 " events, %" PRIu64 "us elapsed, %.2fus avg, min %" PRIu32 "us max %" PRIu32 "us %5.3fus rms",
					       handle->name,
					       pce->event_count,
					       pce->time_total,
					       (pce->event_count == 0) ? 0 : (double)pce->time_total / (double)pce->event_count,
					       pce->time_least,
					       pce->time_most,
					       (double)(1e6f * rms));
			break;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			float rms = sqrtf(pci->M2 / (pci->event_count - 1));

			num_written = snprintf(buffer, length,
					       "%s: %" PRIu64 " events, %.2f avg, min %" PRIu32 "us max %" PRIu32 "us %5.3fus rms",
					       handle->name,
					       pci->event_count,
					       (pci->event_count == 0) ? 0 : (double)(pci->time_last - pci->time_first) / (double)pci->event_count,
					       pci->time_least,
					       pci->time_most,
					       (double)(1e6f * rms));
			break;
		}

	default:
		break;
	}

	buffer[length - 1] = 0; // ensure 0-termination
	return num_written;
}

uint64_t
perf_event_count(perf_counter_t handle)
{
	if (handle == nullptr) {
		return 0;
	}

	switch (handle->type) {
	case PC_COUNT:
		return ((struct perf_ctr_count *)handle)->event_count;

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			return pce->event_count;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			return pci->event_count;
		}

	default:
		break;
	}

	return 0;
}

float
perf_mean(perf_counter_t handle)
{
	if (handle == nullptr) {
		return 0;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			return pce->mean;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			return pci->mean;
		}

	default:
		break;
	}

	return 0.0f;
}

void
perf_iterate_all(perf_callback cb, void *user)
{
	perf_foreach(cb, user);
}

void
perf_print_all(void)
{
	perf_foreach(perf_print_cb);
}

void
perf_print_latency(void)
{
	latency_info_t latency;
	PX4_INFO_RAW("bucket [us] : events\n");

	for (int i = 0; i < get_latency_bucket_count(); i++) {
		latency = get_latency(i, i);
		PX4_INFO_RAW("       %4i : %li\n", latency.bucket, (long int)latency.counter);
	}

	// print the overflow bucket value
	latency = get_latency(get_latency_bucket_count() - 1, get_latency_bucket_count());
	PX4_INFO_RAW(" >%4" PRIu16 " : %" PRIu32 "\n", latency.bucket, latency.counter);
}

void
perf_reset_all(void)
{
	perf_foreach(perf_reset_cb);
	reset_latency_counters();
}
