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
#include <fcntl.h>
#include <sched.h>
#include <signal.h>
#include <unistd.h>
#include <float.h>
#include <string.h>

#include <sys/stat.h>
#include <sys/types.h>

//#include <systemlib/systemlib.h>

#include <px4_tasks.h>

#define MAX_CMD_LEN 100

#define PX4_MAX_TASKS 100
static pthread_t taskmap[PX4_MAX_TASKS] = {};

typedef struct 
{
	px4_main_t entry;
	int argc;
	char *argv[];
	// strings are allocated after the 
} pthdata_t;

void entry_adapter ( void *ptr );
void entry_adapter ( void *ptr )
{
	pthdata_t *data;            
	data = (pthdata_t *) ptr;  

	data->entry(data->argc, data->argv);
	free(ptr);
	pthread_exit(0); 
} 

void
px4_systemreset(bool to_bootloader)
{
	printf("Called px4_system_reset\n");
}

px4_task_t px4_task_spawn_cmd(const char *name, int scheduler, int priority, int stack_size, px4_main_t entry, char * const argv[])
{
	int rv;
	int argc = 0;
	int i;
	unsigned int len = 0;
	unsigned long offset;
	unsigned long structsize;
	char * p = (char *)argv;

        px4_task_t task;

	// Calculate argc
	while (p != (char *)0) {
		p = argv[argc];
		if (p == (char *)0)
			break;
		++argc;
		len += strlen(p)+1;
	}
        structsize = sizeof(pthdata_t)+(argc+1)*sizeof(char *);
	pthdata_t *taskdata;
    
	// not safe to pass stack data to the thread creation
	taskdata = (pthdata_t *)malloc(structsize+len);
	offset = ((unsigned long)taskdata)+structsize;

    	taskdata->entry = entry;
	taskdata->argc = argc;

	for (i=0; i<argc; i++) {
		printf("arg %d %s\n", i, argv[i]);
		taskdata->argv[i] = (char *)offset;
		strcpy((char *)offset, argv[i]);
		offset+=strlen(argv[i])+1;
	}
	// Must add NULL at end of argv
	taskdata->argv[argc] = (char *)0;

	//printf("Called px4_task_spawn_cmd\n");
	// FIXME - add handling for scheduler and priority
        rv = pthread_create (&task, NULL, (void *)&entry_adapter, (void *) taskdata);

	if (rv != 0)
	{
		return (rv < 0) ? rv : -rv;
	}
	//printf("pthread_create task=%d rv=%d\n",(int)task, rv);

	for (i=0; i<PX4_MAX_TASKS; ++i) {
		// FIXME - precludes pthread task to have an ID of 0
		if (taskmap[i] == 0) {
			taskmap[i] = task;
			break;
		}
	}
	if (i>=PX4_MAX_TASKS) {
		return -ENOSPC;
	}
        return i;
}

int px4_task_delete(px4_task_t id)
{
	int rv = 0;
	pthread_t pid;
	//printf("Called px4_task_delete\n");

	if (id < PX4_MAX_TASKS && taskmap[id] != 0)
		pid = taskmap[id];
	else
		return -EINVAL;

	// If current thread then exit, otherwise cancel
        if (pthread_self() == pid) {
		pthread_exit(0);
	} else {
		rv = pthread_cancel(pid);
	}

	taskmap[id] = 0;

	return rv;
}

void px4_task_exit(int ret)
{
	int i; 
	pthread_t pid = pthread_self();

	// Get pthread ID from the opaque ID
	for (i=0; i<PX4_MAX_TASKS; ++i) {
		// FIXME - precludes pthread task to have an ID of 0
		if (taskmap[i] == pid) {
			taskmap[i] = 0;
			break;
		}
	}
	if (i>=PX4_MAX_TASKS) 
		printf("px4_task_exit: self task not found!\n");

	pthread_exit((void *)(unsigned long)ret);
}

void px4_killall(void)
{
	//printf("Called px4_killall\n");
	for (int i=0; i<PX4_MAX_TASKS; ++i) {
		// FIXME - precludes pthread task to have an ID of 0
		if (taskmap[i] != 0) {
			px4_task_delete(i);
		}
	}
}

