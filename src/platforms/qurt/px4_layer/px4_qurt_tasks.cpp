/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *   Author: @author Mark Charlebois <charlebm#gmail.com>
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
 * @file px4_linux_tasks.c
 * Implementation of existing task API for Linux
 */

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdbool.h>
#include <signal.h>
#include <fcntl.h>
#include <sched.h>
#include <unistd.h>
#include <string.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <string>

#include <px4_tasks.h>
#include <hexagon_standalone.h>

#define MAX_CMD_LEN 100

#define PX4_MAX_TASKS 100
struct task_entry
{
	int pid;
	std::string name;
	bool isused;
	task_entry() : isused(false) {}
	void *sp;
};

static task_entry taskmap[PX4_MAX_TASKS];

typedef struct 
{
	px4_main_t entry;
	int argc;
	char *argv[];
	// strings are allocated after the 
} pthdata_t;

static void entry_adapter ( void *ptr )
{
	printf("entry_adapter\n");
	pthdata_t *data = (pthdata_t *) ptr;

	data->entry(data->argc, data->argv);
	free(ptr);
	printf("after entry\n");
	printf("Before px4_task_exit\n");
	px4_task_exit(0); 
	printf("After px4_task_exit\n");
} 

void
px4_systemreset(bool to_bootloader)
{
	printf("Called px4_system_reset\n");
}

px4_task_t px4_task_spawn_cmd(const char *name, int scheduler, int priority, int stack_size, px4_main_t entry, char * const *argv)
{
	int rv;
	int argc = 0;
	int i;
	unsigned int len = 0;
	unsigned long offset;
	unsigned long structsize;
	char * p = (char *)argv;

	// Calculate argc
	while (p != (char *)0) {
		p = argv[argc];
		if (p == (char *)0)
			break;
		++argc;
		len += strlen(p)+1;
	}
	printf("arg %d %p\n", argc, argv);
        structsize = sizeof(pthdata_t)+(argc+1)*sizeof(char *);
	pthdata_t *taskdata;
    
	// not safe to pass stack data to the thread creation
	taskdata = (pthdata_t *)malloc(structsize+len);
	offset = ((unsigned long)taskdata)+structsize;

    	taskdata->entry = entry;
	taskdata->argc = argc;

	for (i=0; i<argc; i++) {
		printf("TEST\n");
		printf("arg %d %s\n", i, argv[i]);
		taskdata->argv[i] = (char *)offset;
		strcpy((char *)offset, argv[i]);
		offset+=strlen(argv[i])+1;
	}
	// Must add NULL at end of argv
	taskdata->argv[argc] = (char *)0;

	for (i=0; i<PX4_MAX_TASKS; ++i) {
		if (taskmap[i].isused == false) {
			taskmap[i].pid = i+1;
			taskmap[i].name = name;
			taskmap[i].isused = true;
			taskmap[i].sp = malloc(stack_size);
			break;
		}
	}
	printf("TEST2\n");
	thread_create(entry_adapter, taskmap[i].sp, i+1, (void *) taskdata);

        return i+1;
}

int px4_task_delete(px4_task_t id)
{
	printf("Called px4_task_delete\n");
	return -EINVAL;
}

void px4_task_exit(int ret)
{
	thread_stop();

	// Free stack
}

int px4_task_kill(px4_task_t id, int sig)
{
	printf("Called px4_task_kill\n");
	return -EINVAL;
}

void px4_show_tasks()
{
	int idx;
	int count = 0;

	printf("Active Tasks:\n");
	for (idx=0; idx < PX4_MAX_TASKS; idx++)
	{
		if (taskmap[idx].isused) {
			printf("   %-10s %d\n", taskmap[idx].name.c_str(), taskmap[idx].pid);
			count++;
		}
	}
	if (count == 0)
		printf("   No running tasks\n");

}

// STUBS

extern "C" {
void hrt_sleep(unsigned long)
{
}
}
