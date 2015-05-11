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
#include <px4_log.h>
#include <hexagon_standalone.h>

#define MAX_CMD_LEN 100

#define PX4_MAX_TASKS 5

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
	char * argv[];
	// strings are allocated after  
} pthdata_t;

static void entry_adapter ( void *ptr )
{
	pthdata_t *data = (pthdata_t *) ptr;
	PX4_DEBUG("entry_adapter %p %p entry %p %d %p\n", ptr, data, data->entry, data->argc, data->argv[0]);

	PX4_DEBUG("data->entry = %p\n", data->entry);
	data->entry(data->argc, data->argv);
	free(ptr);
	PX4_DEBUG("after entry\n");
	PX4_DEBUG("Before px4_task_exit\n");
	px4_task_exit(0); 
	PX4_DEBUG("After px4_task_exit\n");
} 

void
px4_systemreset(bool to_bootloader)
{
	PX4_DEBUG("Called px4_system_reset\n");
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

	PX4_DEBUG("px4_task_spawn_cmd entry = %p %p %s\n", entry, argv, argv[0]);
	// Calculate argc
	while (p != (char *)0) {
		p = argv[argc];
		if (p == (char *)0)
			break;
		++argc;
		len += strlen(p)+1;
	}
	PX4_DEBUG("arg %d %p\n", argc, argv);
        structsize = sizeof(pthdata_t)+(argc+1)*sizeof(char *);
	pthdata_t *taskdata;
    
	PX4_DEBUG("arg %d %p\n", argc, argv);
	// not safe to pass stack data to the thread creation
	taskdata = (pthdata_t *)malloc(structsize+len);
	PX4_DEBUG("arg %d %p\n", argc, argv);
	offset = ((unsigned long)taskdata)+structsize;
	PX4_DEBUG("arg %d %p\n", argc, argv);

    	taskdata->entry = entry;
	taskdata->argc = argc;

	PX4_DEBUG("arg %d %p\n", argc, argv);
	for (i=0; i<argc; i++) {
		PX4_DEBUG("TEST\n");
		PX4_DEBUG("arg %d %s\n", i, argv[i]);
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
			taskmap[i].sp = malloc(2048);
			break;
		}
	}
	PX4_DEBUG("taskdata %p entry %p %d %p\n", taskdata, taskdata->entry, taskdata->argc, taskdata->argv[0]);
	thread_create(entry_adapter, taskmap[i].sp, i+1, (void *) taskdata);

        return i+1;
}

int px4_task_delete(px4_task_t id)
{
	PX4_DEBUG("Called px4_task_delete\n");
	return -EINVAL;
}

void px4_task_exit(int ret)
{
	thread_stop();

	// Free stack
}

int px4_task_kill(px4_task_t id, int sig)
{
	PX4_DEBUG("Called px4_task_kill\n");
	return -EINVAL;
}

void px4_show_tasks()
{
	int idx;
	int count = 0;

	PX4_DEBUG("Active Tasks:\n");
	for (idx=0; idx < PX4_MAX_TASKS; idx++)
	{
		if (taskmap[idx].isused) {
			PX4_DEBUG("   %-10s %d\n", taskmap[idx].name.c_str(), taskmap[idx].pid);
			count++;
		}
	}
	if (count == 0)
		PX4_DEBUG("   No running tasks\n");

}

// STUBS

extern "C" {
void hrt_sleep(unsigned long)
{
}

}
int ioctl(int d, int request, unsigned long foo) { return 0; }
int write(int a, char const*b, int c) { return c; }
