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
#include <px4_defines.h>
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
#include <pthread.h>
#include <limits.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <string>

#include <px4_tasks.h>
#include <px4_posix.h>

#define MAX_CMD_LEN 100

#define PX4_MAX_TASKS 50
#define SHELL_TASK_ID (PX4_MAX_TASKS+1)

pthread_t _shell_task_id = 0;
pthread_mutex_t task_mutex = PTHREAD_MUTEX_INITIALIZER;

struct task_entry {
	pthread_t pid;
	std::string name;
	bool isused;
	task_entry() : isused(false) {}
};

static task_entry taskmap[PX4_MAX_TASKS] = {};

typedef struct {
	px4_main_t entry;
	const char *name;
	int argc;
	char *argv[];
	// strings are allocated after the
} pthdata_t;

static void *entry_adapter(void *ptr)
{
	pthdata_t *data = (pthdata_t *) ptr;

	int rv;
	// set the threads name
#ifdef __PX4_DARWIN
	rv = pthread_setname_np(data->name);
#else
	rv = pthread_setname_np(pthread_self(), data->name);
#endif

	if (rv) {
		PX4_ERR("px4_task_spawn_cmd: failed to set name of thread %d %d\n", rv, errno);
	}

	data->entry(data->argc, data->argv);
	free(ptr);
	PX4_DEBUG("Before px4_task_exit");
	px4_task_exit(0);
	PX4_DEBUG("After px4_task_exit");

	return NULL;
}

void
px4_systemreset(bool to_bootloader)
{
	PX4_WARN("Called px4_system_reset");
	exit(0);
}

px4_task_t px4_task_spawn_cmd(const char *name, int scheduler, int priority, int stack_size, px4_main_t entry,
			      char *const argv[])
{

	int rv;
	int argc = 0;
	int i;
	unsigned int len = 0;
	unsigned long offset;
	unsigned long structsize;
	char *p = (char *)argv;

	pthread_attr_t attr;
	struct sched_param param = {};

	// Calculate argc
	while (p != (char *)0) {
		p = argv[argc];

		if (p == (char *)0) {
			break;
		}

		++argc;
		len += strlen(p) + 1;
	}

	structsize = sizeof(pthdata_t) + (argc + 1) * sizeof(char *);

	// not safe to pass stack data to the thread creation
	pthdata_t *taskdata = (pthdata_t *)malloc(structsize + len);
	memset(taskdata, 0, structsize + len);
	offset = ((unsigned long)taskdata) + structsize;

	taskdata->name = name;
	taskdata->entry = entry;
	taskdata->argc = argc;

	for (i = 0; i < argc; i++) {
		PX4_DEBUG("arg %d %s\n", i, argv[i]);
		taskdata->argv[i] = (char *)offset;
		strcpy((char *)offset, argv[i]);
		offset += strlen(argv[i]) + 1;
	}

	// Must add NULL at end of argv
	taskdata->argv[argc] = (char *)0;

	PX4_DEBUG("starting task %s", name);

	rv = pthread_attr_init(&attr);

	if (rv != 0) {
		PX4_WARN("px4_task_spawn_cmd: failed to init thread attrs");
		return (rv < 0) ? rv : -rv;
	}

#ifndef __PX4_DARWIN

	if (stack_size < PTHREAD_STACK_MIN) {
		stack_size = PTHREAD_STACK_MIN;
	}

	rv = pthread_attr_setstacksize(&attr, stack_size);

	if (rv != 0) {
		PX4_ERR("pthread_attr_setstacksize to %d returned error (%d)", stack_size, rv);
		return (rv < 0) ? rv : -rv;
	}

#endif

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

	param.sched_priority = priority;

	rv = pthread_attr_setschedparam(&attr, &param);

	if (rv != 0) {
		PX4_WARN("px4_task_spawn_cmd: failed to set sched param");
		return (rv < 0) ? rv : -rv;
	}

	pthread_mutex_lock(&task_mutex);

	int taskid = 0;

	for (i = 0; i < PX4_MAX_TASKS; ++i) {
		if (taskmap[i].isused == false) {
			taskmap[i].name = name;
			taskmap[i].isused = true;
			taskid = i;
			break;
		}
	}

	rv = pthread_create(&taskmap[taskid].pid, &attr, &entry_adapter, (void *) taskdata);

	if (rv != 0) {

		if (rv == EPERM) {
			//printf("WARNING: NOT RUNING AS ROOT, UNABLE TO RUN REALTIME THREADS\n");
			rv = pthread_create(&taskmap[taskid].pid, NULL, &entry_adapter, (void *) taskdata);

			if (rv != 0) {
				PX4_ERR("px4_task_spawn_cmd: failed to create thread %d %d\n", rv, errno);
				taskmap[taskid].isused = false;
				return (rv < 0) ? rv : -rv;
			}

		} else {
			return (rv < 0) ? rv : -rv;
		}
	}

	pthread_mutex_unlock(&task_mutex);

	if (i >= PX4_MAX_TASKS) {
		return -ENOSPC;
	}

	return i;
}

int px4_task_delete(px4_task_t id)
{
	int rv = 0;
	pthread_t pid;
	PX4_DEBUG("Called px4_task_delete");

	if (id < PX4_MAX_TASKS && taskmap[id].isused) {
		pid = taskmap[id].pid;

	} else {
		return -EINVAL;
	}

	pthread_mutex_lock(&task_mutex);

	// If current thread then exit, otherwise cancel
	if (pthread_self() == pid) {
		taskmap[id].isused = false;
		pthread_mutex_unlock(&task_mutex);
		pthread_exit(0);

	} else {
		rv = pthread_cancel(pid);
	}

	taskmap[id].isused = false;
	pthread_mutex_unlock(&task_mutex);

	return rv;
}

void px4_task_exit(int ret)
{
	int i;
	pthread_t pid = pthread_self();

	// Get pthread ID from the opaque ID
	for (i = 0; i < PX4_MAX_TASKS; ++i) {
		if (taskmap[i].pid == pid) {
			pthread_mutex_lock(&task_mutex);
			taskmap[i].isused = false;
			break;
		}
	}

	if (i >= PX4_MAX_TASKS)  {
		PX4_ERR("px4_task_exit: self task not found!");

	} else {
		PX4_DEBUG("px4_task_exit: %s", taskmap[i].name.c_str());
	}

	pthread_mutex_unlock(&task_mutex);

	pthread_exit((void *)(unsigned long)ret);
}

int px4_task_kill(px4_task_t id, int sig)
{
	int rv = 0;
	pthread_t pid;
	PX4_DEBUG("Called px4_task_kill %d", sig);

	if (id < PX4_MAX_TASKS && taskmap[id].isused && taskmap[id].pid != 0) {
		pthread_mutex_lock(&task_mutex);
		pid = taskmap[id].pid;
		pthread_mutex_unlock(&task_mutex);

	} else {
		return -EINVAL;
	}

	// If current thread then exit, otherwise cancel
	rv = pthread_kill(pid, sig);

	return rv;
}

void px4_show_tasks()
{
	int idx;
	int count = 0;

	PX4_INFO("Active Tasks:");

	for (idx = 0; idx < PX4_MAX_TASKS; idx++) {
		if (taskmap[idx].isused) {
			PX4_INFO("   %-10s %lu", taskmap[idx].name.c_str(), (unsigned long)taskmap[idx].pid);
			count++;
		}
	}

	if (count == 0) {
		PX4_INFO("   No running tasks");
	}

}

bool px4_task_is_running(const char *taskname)
{
	int idx;

	for (idx = 0; idx < PX4_MAX_TASKS; idx++) {
		if (taskmap[idx].isused && (strcmp(taskmap[idx].name.c_str(), taskname) == 0)) {
			return true;
		}
	}

	return false;
}
__BEGIN_DECLS

unsigned long px4_getpid()
{
	return (unsigned long)pthread_self();
}

const char *getprogname();
const char *getprogname()
{
	pthread_t pid = pthread_self();

	for (int i = 0; i < PX4_MAX_TASKS; i++) {
		if (taskmap[i].isused && taskmap[i].pid == pid) {
			pthread_mutex_lock(&task_mutex);
			return taskmap[i].name.c_str();
			pthread_mutex_unlock(&task_mutex);
		}
	}

	return "Unknown App";
}

int px4_prctl(int option, const char *arg2, unsigned pid)
{
	int rv;

	switch (option) {
	case PR_SET_NAME:
		// set the threads name
#ifdef __PX4_DARWIN
		rv = pthread_setname_np(arg2);
#else
		rv = pthread_setname_np(pthread_self(), arg2);
#endif
		break;

	default:
		rv = -1;
		PX4_WARN("FAILED SETTING TASK NAME");
		break;
	}

	return rv;
}

__END_DECLS
