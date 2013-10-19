/****************************************************************************
 * px4/sensors/test_gpio.c
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

#include <nuttx/config.h>

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


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Public Functions
 ****************************************************************************/
static sem_t *sems;

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
		return -1;
	}
	/* try to read an invalid index */
	if (dm_read(DM_KEY_RTL_POINT, DM_KEY_RTL_POINT_MAX, buffer, sizeof(buffer)) >= 0) {
		warnx("%d read an invalid index failed", my_id);
		return -1;
	}
	srand(2323748);
	unsigned hit = 0, miss = 0;
	wstart = hrt_absolute_time();
	for (unsigned i = 0; i < 256; i++) {
		buffer[0] = my_id;
		buffer[1] = i;
		unsigned hash = i ^ my_id;
		unsigned len = (hash & 63) + 2;
		if (dm_write(DM_KEY_WAY_POINTS, hash, DM_PERSIST_IN_FLIGHT_RESET, buffer, len) != len) {
			warnx("%d write failed, index %d, length %d", my_id, hash, len);
			return -1;
		}
		usleep(rand() & ((64 * 1024) - 1));
	}
	rstart = hrt_absolute_time();
	wend = rstart;

	for (unsigned i = 0; i < 256; i++) {
		unsigned hash = i ^ my_id;
		unsigned len2, len = (hash & 63) + 2;
		if ((len2 = dm_read(DM_KEY_WAY_POINTS, hash, buffer, sizeof(buffer))) < 2) {
			warnx("%d read failed length test, index %d", my_id, hash);
			return -1;
		}
		if (buffer[0] == my_id) {
			hit++;
			if (len2 != len) {
				warnx("%d read failed length test, index %d, wanted %d, got %d", my_id, hash, len, len2);
				return -1;
			}
			if (buffer[1] != i) {
				warnx("%d data verification failed, index %d, wanted %d, got %d", my_id, hash, my_id, buffer[1]);
				return -1;
			}
		}
		else
			miss++;
	}
	rend = hrt_absolute_time();
	warnx("Test %d, hit %d, miss %d, io time read %llums. write %llums.",
		my_id, hit, miss, (rend - rstart) / 256000, (wend - wstart) / 256000);
	sem_post(sems + my_id);
	return 0;
}

int test_dataman(int argc, char *argv[])
{
	int num_tasks = 4;
	char buffer[DM_MAX_DATA_SIZE];

	if (argc > 1)
		num_tasks = atoi(argv[1]);
	
	sems = (sem_t *)malloc(num_tasks * sizeof(sem_t));
	warnx("Running %d tasks", num_tasks);
	dm_clear(DM_KEY_WAY_POINTS);
	for (unsigned i = 0; i < 256; i++) {
		if (dm_read(DM_KEY_WAY_POINTS, i, buffer, sizeof(buffer)) != 0) {
			warnx("read failed clear test, index %d", i);
			return -1;
		}
	}
	for (int i = 0; i < num_tasks; i++) {
		int task;
		char a[16];
		sprintf(a, "%d", i);
		const char *av[2];
		av[0] = a;
		av[1] = 0;
		sem_init(sems + i, 1, 0);
		/* start the task */
		if ((task = task_spawn_cmd("dataman", SCHED_DEFAULT, SCHED_PRIORITY_MAX - 5, 2048, task_main, av)) <= 0) {
			warn("task start failed");
		}
	}
	for (int i = 0; i < num_tasks; i++) {
		sem_wait(sems + i);
		sem_destroy(sems + i);
	}
	free(sems);
	return 0;
}
