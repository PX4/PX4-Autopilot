/****************************************************************************
 * px4/sensors/test_dataman.c
 *
 *  Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include <drivers/drv_led.h>
#include <systemlib/systemlib.h>
#include <drivers/drv_hrt.h>
#include <semaphore.h>


#include "tests.h"

#include "dataman/dataman.h"

static px4_sem_t *sems;

static int
task_main(int argc, char *argv[])
{
	char buffer[DM_MAX_DATA_SIZE];
	hrt_abstime wstart, wend, rstart, rend;

	warnx("Starting dataman test task %s", argv[1]);
	/* try to read an invalid item */
	int my_id = atoi(argv[1]);

	/* try to read an invalid item */
	if (dm_read(DM_KEY_NUM_KEYS, 0, buffer, sizeof(buffer)) >= 0) {
		warnx("%d read an invalid item failed", my_id);
		goto fail;
	}

	/* try to read an invalid index */
	if (dm_read(DM_KEY_SAFE_POINTS, DM_KEY_SAFE_POINTS_MAX, buffer, sizeof(buffer)) >= 0) {
		warnx("%d read an invalid index failed", my_id);
		goto fail;
	}

	srand(hrt_absolute_time() ^ my_id);
	unsigned hit = 0, miss = 0;
	wstart = hrt_absolute_time();

	for (unsigned i = 0; i < NUM_MISSIONS_SUPPORTED; i++) {
		memset(buffer, my_id, sizeof(buffer));
		buffer[1] = i;
		unsigned hash = i ^ my_id;
		unsigned len = (hash & 63) + 2;

		int ret = dm_write(DM_KEY_WAYPOINTS_OFFBOARD_1, hash, DM_PERSIST_IN_FLIGHT_RESET, buffer, len);
		warnx("ret: %d", ret);

		if (ret != len) {
			warnx("%d write failed, index %d, length %d", my_id, hash, len);
			goto fail;
		}

		usleep(rand() & ((64 * 1024) - 1));
	}

	rstart = hrt_absolute_time();
	wend = rstart;

	for (unsigned i = 0; i < NUM_MISSIONS_SUPPORTED; i++) {
		unsigned hash = i ^ my_id;
		unsigned len2, len = (hash & 63) + 2;

		if ((len2 = dm_read(DM_KEY_WAYPOINTS_OFFBOARD_1, hash, buffer, sizeof(buffer))) < 2) {
			warnx("%d read failed length test, index %d", my_id, hash);
			goto fail;
		}

		if (buffer[0] == my_id) {
			hit++;

			if (len2 != len) {
				warnx("%d read failed length test, index %d, wanted %d, got %d", my_id, hash, len, len2);
				goto fail;
			}

			if (buffer[1] != i) {
				warnx("%d data verification failed, index %d, wanted %d, got %d", my_id, hash, my_id, buffer[1]);
				goto fail;
			}

		} else {
			miss++;
		}
	}

	rend = hrt_absolute_time();
	warnx("Test %d pass, hit %d, miss %d, io time read %llums. write %llums.",
	      my_id, hit, miss, (rend - rstart) / NUM_MISSIONS_SUPPORTED / 1000, (wend - wstart) / NUM_MISSIONS_SUPPORTED / 1000);
	px4_sem_post(sems + my_id);
	return 0;
fail:
	warnx("Test %d fail, buffer %02x %02x %02x %02x %02x %02x",
	      my_id, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
	px4_sem_post(sems + my_id);
	return -1;
}

int test_dataman(int argc, char *argv[])
{
	int i, num_tasks = 4;
	char buffer[DM_MAX_DATA_SIZE];

	if (argc > 1) {
		num_tasks = atoi(argv[1]);
	}

	sems = (px4_sem_t *)malloc(num_tasks * sizeof(px4_sem_t));
	warnx("Running %d tasks", num_tasks);

	for (i = 0; i < num_tasks; i++) {
		int task;
		char a[16];
		sprintf(a, "%d", i);
		const char *av[2];
		av[0] = a;
		av[1] = 0;
		px4_sem_init(sems + i, 1, 0);

		/* start the task */
		if ((task = px4_task_spawn_cmd("dataman", SCHED_DEFAULT, SCHED_PRIORITY_MAX - 5, 2048, task_main, av)) <= 0) {
			warn("task start failed");
		}
	}

	for (i = 0; i < num_tasks; i++) {
		px4_sem_wait(sems + i);
		px4_sem_destroy(sems + i);
	}

	free(sems);
	dm_restart(DM_INIT_REASON_IN_FLIGHT);

	for (i = 0; i < NUM_MISSIONS_SUPPORTED; i++) {
		if (dm_read(DM_KEY_WAYPOINTS_OFFBOARD_1, i, buffer, sizeof(buffer)) != 0) {
			break;
		}
	}

	if (i >= NUM_MISSIONS_SUPPORTED) {
		warnx("Restart in-flight failed");
		return -1;

	}

	dm_restart(DM_INIT_REASON_POWER_ON);

	for (i = 0; i < NUM_MISSIONS_SUPPORTED; i++) {
		if (dm_read(DM_KEY_WAYPOINTS_OFFBOARD_1, i, buffer, sizeof(buffer)) != 0) {
			warnx("Restart power-on failed");
			return -1;
		}
	}

	return 0;
}
