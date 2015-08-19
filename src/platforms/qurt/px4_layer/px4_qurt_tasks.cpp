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
 * @file px4_posix_tasks.c
 * Implementation of existing task API for Linux
 */

#include <px4_log.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdbool.h>

#if !defined(__PX4_QURT)
#include <signal.h>
#endif

#include <fcntl.h>
#include <sched.h>
#include <unistd.h>
#include <string.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <string>

#include <px4_tasks.h>

#define MAX_CMD_LEN 100

#define PX4_MAX_TASKS 50
#define SHELL_TASK_ID (PX4_MAX_TASKS+1)

pthread_t _shell_task_id = 0;

struct task_entry
{
	pthread_t pid;
	std::string name;
	bool isused;
	task_entry() : isused(false) {}
};

static task_entry taskmap[PX4_MAX_TASKS];

typedef struct 
{
	px4_main_t entry;
	int argc;
	char *argv[];
	// strings are allocated after the 
} pthdata_t;

static void *entry_adapter ( void *ptr )
{
	pthdata_t *data;            
	data = (pthdata_t *) ptr;  

	data->entry(data->argc, data->argv);
        PX4_WARN( "Before waiting infinte busy loop" );
        //for( ;; )
        //{
        //   volatile int x = 0;
        //   ++x;
       // }
	free(ptr);
	px4_task_exit(0); 

	return NULL;
} 

void
px4_systemreset(bool to_bootloader)
{
	PX4_WARN("Called px4_system_reset");
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

	PX4_DEBUG("Creating %s\n", name);
        pthread_t task;
	pthread_attr_t attr;
	struct sched_param param;

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
		PX4_DEBUG("arg %d %s\n", i, argv[i]);
		taskdata->argv[i] = (char *)offset;
		strcpy((char *)offset, argv[i]);
		offset+=strlen(argv[i])+1;
	}
	// Must add NULL at end of argv
	taskdata->argv[argc] = (char *)0;

	rv = pthread_attr_init(&attr);
	if (rv != 0) {
		PX4_WARN("px4_task_spawn_cmd: failed to init thread attrs");
		return (rv < 0) ? rv : -rv;
	}
#if 0
	rv = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	if (rv != 0) {
		PX4_WARN("px4_task_spawn_cmd: failed to set inherit sched");
		return (rv < 0) ? rv : -rv;
	}
	rv = pthread_attr_setschedpolicy(&attr, scheduler);
	if (rv != 0) {
		PX4_WARN("px4_task_spawn_cmd: failed to set sched policy");
		return (rv < 0) ? rv : -rv;
	}
#endif
        size_t fixed_stacksize = -1;
        pthread_attr_getstacksize(&attr, &fixed_stacksize);
        PX4_WARN("stack size: %d passed stacksize(%d)", fixed_stacksize, stack_size );
        fixed_stacksize = 8 * 1024;
        fixed_stacksize = ( fixed_stacksize < (size_t)stack_size )? (size_t)stack_size:fixed_stacksize;

        PX4_WARN("setting the thread[%s] stack size to[%d]", name, fixed_stacksize );
        pthread_attr_setstacksize(&attr, fixed_stacksize);
        //pthread_attr_setstacksize(&attr, stack_size);

        
	param.sched_priority = priority;

	rv = pthread_attr_setschedparam(&attr, &param);
	if (rv != 0) {
		PX4_WARN("px4_task_spawn_cmd: failed to set sched param");
		return (rv < 0) ? rv : -rv;
	}

        rv = pthread_create (&task, &attr, &entry_adapter, (void *) taskdata);
	if (rv != 0) {

		return (rv < 0) ? rv : -rv;
	}

	for (i=0; i<PX4_MAX_TASKS; ++i) {
		if (taskmap[i].isused == false) {
			taskmap[i].pid = task;
			taskmap[i].name = name;
			taskmap[i].isused = true;
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
	PX4_WARN("Called px4_task_delete");

	if (id < PX4_MAX_TASKS && taskmap[id].isused)
		pid = taskmap[id].pid;
	else
		return -EINVAL;

	// If current thread then exit, otherwise cancel
        if (pthread_self() == pid) {
		taskmap[id].isused = false;
		pthread_exit(0);
	} else {
		rv = pthread_cancel(pid);
	}

	taskmap[id].isused = false;

	return rv;
}

void px4_task_exit(int ret)
{
	int i; 
	pthread_t pid = pthread_self();

	// Get pthread ID from the opaque ID
	for (i=0; i<PX4_MAX_TASKS; ++i) {
		if (taskmap[i].pid == pid) {
			taskmap[i].isused = false;
			break;
		}
	}
	if (i>=PX4_MAX_TASKS)  {
		PX4_ERR("px4_task_exit: self task not found!");
	}
	else {
		PX4_DEBUG("px4_task_exit: %s", taskmap[i].name.c_str());
	}

	//pthread_exit((void *)(unsigned long)ret);
}

int px4_task_kill(px4_task_t id, int sig)
{
	int rv = 0;
	pthread_t pid;
	PX4_DEBUG("Called px4_task_kill %d, taskname %s", sig, taskmap[id].name.c_str());

	if (id < PX4_MAX_TASKS && taskmap[id].pid != 0)
		pid = taskmap[id].pid;
	else
		return -EINVAL;

	// If current thread then exit, otherwise cancel
	rv = pthread_kill(pid, sig);

	return rv;
}

void px4_show_tasks()
{
	int idx;
	int count = 0;

	PX4_INFO("Active Tasks:");
	for (idx=0; idx < PX4_MAX_TASKS; idx++)
	{
		if (taskmap[idx].isused) {
			PX4_INFO("   %-10s %d", taskmap[idx].name.c_str(), taskmap[idx].pid);
			count++;
		}
	}
	if (count == 0)
		PX4_INFO("   No running tasks");

}

__BEGIN_DECLS

int px4_getpid()
{
	pthread_t pid = pthread_self();
//
//	if (pid == _shell_task_id)
//		return SHELL_TASK_ID;

	// Get pthread ID from the opaque ID
	for (int i=0; i<PX4_MAX_TASKS; ++i) {
		if (taskmap[i].isused && taskmap[i].pid == pid) {
			return i;
		}
	}
	PX4_ERR("px4_getpid() called from unknown thread context!");
	return -EINVAL;
}


const char *getprogname();
const char *getprogname()
{
        pthread_t pid = pthread_self();
	for (int i=0; i<PX4_MAX_TASKS; i++)
	{
		if (taskmap[i].isused && taskmap[i].pid == pid)
		{
			return taskmap[i].name.c_str();
		}
	}
	return "Unknown App";
}
__END_DECLS

