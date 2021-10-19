/****************************************************************************
 *
 *   Copyright (c) 2016, 2021 PX4 Development Team. All rights reserved.
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
 * @file sd_bench.c
 *
 * SD Card benchmarking
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <drivers/drv_hrt.h>

typedef struct sdb_config {
	int num_runs; ///< number of runs
	int run_duration; ///< duration of a single run [ms]
	bool synchronized; ///< call fsync after each block?
	bool aligned;
	unsigned int total_blocks_written;
} sdb_config_t;

static void	usage(void);

/** sequential write speed test */
static void write_test(int fd, sdb_config_t *cfg, uint8_t *block, int block_size);
/** sequential read speed test */
static int read_test(int fd, sdb_config_t *cfg, uint8_t *block, int block_size);

/**
 * Measure the time for fsync.
 * @param fd
 * @return time in ms
 */
static inline unsigned int time_fsync(int fd);

__EXPORT int	sd_bench_main(int argc, char *argv[]);

static const char *BENCHMARK_FILE = PX4_STORAGEDIR"/benchmark.tmp";

static void
usage()
{
	PRINT_MODULE_DESCRIPTION("Test the speed of an SD Card");

	PRINT_MODULE_USAGE_NAME_SIMPLE("sd_bench", "command");
	PRINT_MODULE_USAGE_PARAM_INT('b', 4096, 1, 1000000, "Block size for each read/write", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 5, 1, 1000, "Number of runs", true);
	PRINT_MODULE_USAGE_PARAM_INT('d', 2000, 1, 100000, "Duration of a run in ms", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('s', "Call fsync after each block (default=at end of each run)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('u', "Test performance with unaligned data", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('v', "Verify data and block number", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('k', "Keep the test file", true);
}


int
sd_bench_main(int argc, char *argv[])
{
	int block_size = 4096;
	bool verify = false;
	bool keep = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = NULL;
	sdb_config_t cfg;
	cfg.synchronized = false;
	cfg.num_runs = 5;
	cfg.run_duration = 2000;
	cfg.aligned = true;
	uint8_t *block =  NULL;

	while ((ch = px4_getopt(argc, argv, "b:r:d:suv", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			block_size = strtol(myoptarg, NULL, 0);
			break;

		case 'r':
			cfg.num_runs = strtol(myoptarg, NULL, 0);
			break;

		case 'd':
			cfg.run_duration = strtol(myoptarg, NULL, 0);
			break;

		case 'k':
			keep = true;
			break;

		case 's':
			cfg.synchronized = true;
			break;

		case 'u':
			cfg.aligned = false;
			break;

		case 'v':
			verify = true;
			break;

		default:
			usage();
			return -1;
			break;
		}
	}

	if (block_size <= 0 || cfg.num_runs <= 0) {
		PX4_ERR("invalid argument");
		return -1;
	}

	int bench_fd = open(BENCHMARK_FILE, O_CREAT | (verify ? O_RDWR : O_WRONLY) | O_TRUNC, PX4_O_MODE_666);

	if (bench_fd < 0) {
		PX4_ERR("Can't open benchmark file %s", BENCHMARK_FILE);
		return -1;
	}

	//create some data block
	if (cfg.aligned) {
		block = (uint8_t *)px4_cache_aligned_alloc(block_size);

	} else {
		block = (uint8_t *)malloc(block_size);
	}

	if (!block) {
		PX4_ERR("Failed to allocate memory block");
		close(bench_fd);
		return -1;
	}

	for (int i = 0; i < block_size; ++i) {
		block[i] = (uint8_t)i;
	}

	PX4_INFO("Using block size = %i bytes, sync=%i", block_size, (int)cfg.synchronized);
	write_test(bench_fd, &cfg, block, block_size);

	if (verify) {
		fsync(bench_fd);
		lseek(bench_fd, 0, SEEK_SET);
		read_test(bench_fd, &cfg, block, block_size);
	}

	free(block);
	close(bench_fd);

	if (!keep) {
		unlink(BENCHMARK_FILE);
	}

	return 0;
}

unsigned int time_fsync(int fd)
{
	hrt_abstime fsync_start = hrt_absolute_time();
	fsync(fd);
	return hrt_elapsed_time(&fsync_start) / 1000;
}

void write_test(int fd, sdb_config_t *cfg, uint8_t *block, int block_size)
{
	PX4_INFO("");
	PX4_INFO("Testing Sequential Write Speed...");
	double total_elapsed = 0.;
	unsigned int total_blocks = 0;
	cfg->total_blocks_written = 0;
	unsigned int *blocknumber = (unsigned int *)(void *)&block[0];

	for (int run = 0; run < cfg->num_runs; ++run) {
		hrt_abstime start = hrt_absolute_time();
		unsigned int num_blocks = 0;
		unsigned int max_write_time = 0;
		unsigned int fsync_time = 0;

		while ((int64_t)hrt_elapsed_time(&start) < cfg->run_duration * 1000) {

			hrt_abstime write_start = hrt_absolute_time();
			*blocknumber =  total_blocks + num_blocks;
			size_t written = write(fd, block, block_size);
			unsigned int write_time = hrt_elapsed_time(&write_start) / 1000;

			if (write_time > max_write_time) {
				max_write_time = write_time;
			}

			if ((int)written != block_size) {
				PX4_ERR("Write error");
				return;
			}

			if (cfg->synchronized) {
				fsync_time += time_fsync(fd);
			}

			++num_blocks;
		}

		//Note: if testing a slow device (SD Card) and the OS buffers a lot (eg. Linux),
		//fsync can take really long, and it looks like the process hangs. But it does
		//not and the reported result will still be correct.
		fsync_time += time_fsync(fd);

		//report
		double elapsed = hrt_elapsed_time(&start) / 1.e6;
		PX4_INFO("  Run %2i: %8.2lf KB/s, max write time: %i ms (=%7.2lf KB/s), fsync: %i ms", run,
			 (double)block_size * num_blocks / elapsed / 1024.,
			 max_write_time, (double)block_size / max_write_time * 1000. / 1024., fsync_time);

		total_elapsed += elapsed;
		total_blocks += num_blocks;
	}

	cfg->total_blocks_written = total_blocks;
	PX4_INFO("  Avg   : %8.2lf KB/s", (double)block_size * total_blocks / total_elapsed / 1024.);
}

int read_test(int fd, sdb_config_t *cfg, uint8_t *block, int block_size)
{
	uint8_t *read_block =  NULL;

	PX4_INFO("");
	PX4_INFO("Testing Sequential Read Speed of %d blocks", cfg->total_blocks_written);

	if (cfg->aligned) {
		read_block = (uint8_t *)px4_cache_aligned_alloc(block_size);

	} else {
		read_block = (uint8_t *)malloc(block_size);
	}

	if (!read_block) {
		PX4_ERR("Failed to allocate memory block");
		return -1;
	}

	double total_elapsed = 0.;
	unsigned int total_blocks = 0;
	unsigned int *blocknumber = (unsigned int *)(void *) &read_block[0];

	for (int run = 0; run < cfg->num_runs  && total_blocks < cfg->total_blocks_written; ++run) {
		hrt_abstime start = hrt_absolute_time();
		unsigned int num_blocks = 0;
		unsigned int max_read_time = 0;

		while ((int64_t)hrt_elapsed_time(&start) < cfg->run_duration * 1000
		       && total_blocks + num_blocks < cfg->total_blocks_written) {

			hrt_abstime read_start = hrt_absolute_time();
			size_t nread = read(fd, read_block, block_size);
			unsigned int read_time = hrt_elapsed_time(&read_start) / 1000;

			if (read_time > max_read_time) {
				max_read_time = read_time;
			}

			if ((int)nread != block_size) {
				PX4_ERR("Read error");
				return -1;
			}


			if (*blocknumber !=  total_blocks + num_blocks) {
				PX4_ERR("Read data error at block: %d wrote:0x%04x read:0x%04x", (total_blocks + num_blocks),
					total_blocks + num_blocks, *blocknumber);

			}


			for (unsigned int i = sizeof(*blocknumber); i < (block_size - sizeof(*blocknumber)); ++i) {
				if (block[i] != read_block[i]) {
					PX4_ERR("Read data error at offset: %d wrote:0x%02x read:0x%02x", total_blocks + num_blocks + i, block[i],
						read_block[i]);
				}
			}

			++num_blocks;
		}

		//report
		double elapsed = hrt_elapsed_time(&start) / 1.e6;
		PX4_INFO("  Run %2i: %8.2lf KB/s, max read/verify time: %i ms (=%7.2lf KB/s)", run,
			 (double)block_size * num_blocks / elapsed / 1024.,
			 max_read_time, (double)block_size / max_read_time * 1000. / 1024.);

		total_elapsed += elapsed;
		total_blocks += num_blocks;
	}

	PX4_INFO("  Avg   : %8.2lf KB/s %d blocks read and verified", (double)block_size * total_blocks / total_elapsed / 1024.,
		 total_blocks);
	return 0;
}
