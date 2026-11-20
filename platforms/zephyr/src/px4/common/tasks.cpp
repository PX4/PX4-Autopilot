/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file tasks.cpp
 * Implementation of the task API for Zephyr
 */

#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_zephyr.h>

#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define PX4_MAX_TASKS CONFIG_POSIX_THREAD_THREADS_MAX

// a task id is the slot of the task in taskmap, and the generation of the slot above it, so that the id of a task
// that was released doesn't name the task that reuses its slot
static constexpr int TASK_SLOT_BITS = 8;
static constexpr int TASK_SLOT_MASK = (1 << TASK_SLOT_BITS) - 1;
static constexpr uint32_t TASK_GENERATION_MASK = INT32_MAX >> TASK_SLOT_BITS;
static_assert(PX4_MAX_TASKS <= TASK_SLOT_MASK + 1, "too many tasks for the task ids");

// Zephyr thread attributes own the thread stack, so they are only destroyed after the thread has been joined
enum class TaskState : uint8_t {
	Unused,
	Running,
	Exited,
	Released, // the thread was joined by the next spawn, the exit code waits for px4_task_join()
};

struct task_entry {
	pthread_t pid;
	pthread_attr_t attr;
	void *data; // the arguments, freed with the task, as px4_task_delete() may stop it at any point
	char name[16];
	TaskState state;
	bool joining; // the thread joining the task releases it
	bool releasing; // a spawn is releasing the task, which then keeps its exit code
	uint32_t generation;
	int exit_code; // of a released task
};

static task_entry taskmap[PX4_MAX_TASKS] {};
static pthread_mutex_t task_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t task_released = PTHREAD_COND_INITIALIZER;

static px4_task_t task_id(int slot)
{
	return (px4_task_t)((taskmap[slot].generation << TASK_SLOT_BITS) | slot);
}

// the slot of a task that is not unused, or -1, with task_mutex held
static int task_slot(px4_task_t id)
{
	const int slot = id & TASK_SLOT_MASK;

	if ((id < 0) || (slot >= PX4_MAX_TASKS) || (taskmap[slot].state == TaskState::Unused) || (task_id(slot) != id)) {
		return -1;
	}

	return slot;
}

typedef struct {
	px4_main_t entry;
	int slot; // in taskmap
	int argc;
	char *argv[];
	// strings are allocated after the struct data
} pthdata_t;

static void *entry_adapter(void *ptr)
{
	pthdata_t *data = (pthdata_t *)ptr;

	// px4_task_delete() is expected to stop the task immediately, as task_delete() on NuttX, see px4_task_delete()
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, nullptr);
	pthread_setname_np(pthread_self(), taskmap[data->slot].name);

	px4_task_exit(data->entry(data->argc, data->argv));

	return nullptr;
}

// join the thread of a task and free its resources, then keep its exit code for px4_task_join(), or free the slot
static int task_release(task_entry &task, bool keep_exit_code, int *exit_code)
{
	void *retval = nullptr;
	const int rv = pthread_join(task.pid, &retval);
	pthread_attr_destroy(&task.attr);
	free(task.data);

	// a task stopped by px4_task_delete() returns PTHREAD_CANCELED, -1
	const int code = (rv == 0) ? (int)(intptr_t)retval : PX4_ERROR;

	pthread_mutex_lock(&task_mutex);
	task.data = nullptr;
	task.joining = false;
	task.releasing = false;
	task.exit_code = code;
	task.state = keep_exit_code ? TaskState::Released : TaskState::Unused;
	pthread_cond_broadcast(&task_released);
	pthread_mutex_unlock(&task_mutex);

	if ((rv == 0) && (exit_code != nullptr)) {
		*exit_code = code;
	}

	return -rv;
}

static void task_release_exited()
{
	for (task_entry &task : taskmap) {
		pthread_mutex_lock(&task_mutex);
		const bool exited = (task.state == TaskState::Exited) && !task.joining && !task.releasing;

		if (exited) {
			task.releasing = true;
		}

		pthread_mutex_unlock(&task_mutex);

		if (exited) {
			task_release(task, true, nullptr);
		}
	}
}

px4_task_t px4_task_spawn_cmd(const char *name, int scheduler, int priority, int stack_size, px4_main_t entry,
			      char *const argv[])
{
	task_release_exited();

	int argc = 0;
	size_t len = strlen(name) + 1;

	while (argv != nullptr && argv[argc] != nullptr) {
		len += strlen(argv[argc]) + 1;
		argc++;
	}

	const size_t structsize = sizeof(pthdata_t) + (argc + 2) * sizeof(char *);

	pthdata_t *taskdata = (pthdata_t *)calloc(1, structsize + len);

	if (taskdata == nullptr) {
		return -ENOMEM;
	}

	taskdata->entry = entry;
	taskdata->argc = argc + 1;

	// argv[0] is the task name, as on NuttX
	char *offset = (char *)taskdata + structsize;
	taskdata->argv[0] = offset;
	strcpy(offset, name);
	offset += strlen(name) + 1;

	for (int i = 0; i < argc; i++) {
		taskdata->argv[i + 1] = offset;
		strcpy(offset, argv[i]);
		offset += strlen(argv[i]) + 1;
	}

	pthread_mutex_lock(&task_mutex);

	int slot = -1;

	for (int i = 0; i < PX4_MAX_TASKS; i++) {
		if (taskmap[i].state == TaskState::Unused) {
			slot = i;
			break;
		}
	}

	// then the slot of a task that was released and never joined, whose id then names no task
	for (int i = 0; (slot < 0) && (i < PX4_MAX_TASKS); i++) {
		if (taskmap[i].state == TaskState::Released) {
			slot = i;
		}
	}

	if (slot < 0) {
		pthread_mutex_unlock(&task_mutex);
		free(taskdata);
		return -ENOSPC;
	}

	task_entry &task = taskmap[slot];
	task.generation = (task.generation + 1) & TASK_GENERATION_MASK;
	const px4_task_t id = task_id(slot);
	taskdata->slot = slot;
	task.data = taskdata;
	task.joining = false;
	task.releasing = false;
	strncpy(task.name, name, sizeof(task.name) - 1);
	task.name[sizeof(task.name) - 1] = '\0';

	if (stack_size < (int)PTHREAD_STACK_MIN) {
		stack_size = PTHREAD_STACK_MIN;
	}

	sched_param param{};
	param.sched_priority = priority;

	int rv = pthread_attr_init(&task.attr);

	if (rv == 0) {
		rv = pthread_attr_setstacksize(&task.attr, stack_size);

		if (rv == 0) {
			rv = pthread_attr_setinheritsched(&task.attr, PTHREAD_EXPLICIT_SCHED);
		}

		if (rv == 0) {
			rv = pthread_attr_setschedpolicy(&task.attr, scheduler);
		}

		if (rv == 0) {
			rv = pthread_attr_setschedparam(&task.attr, &param);
		}

		if (rv == 0) {
			task.state = TaskState::Running;
			rv = pthread_create(&task.pid, &task.attr, &entry_adapter, taskdata);
		}

		if (rv != 0) {
			pthread_attr_destroy(&task.attr);
		}
	}

	if (rv != 0) {
		PX4_ERR("failed to create thread for %s (%i)", name, rv);
		task.data = nullptr;
		task.state = TaskState::Unused;
		pthread_mutex_unlock(&task_mutex);
		free(taskdata);
		return -rv;
	}

	pthread_mutex_unlock(&task_mutex);

	return id;
}

int px4_zephyr_task_join(int id, int *exit_code)
{
	pthread_mutex_lock(&task_mutex);

	int slot = task_slot(id);

	// a task that exited can be released by a spawn before it is joined: wait for its exit code
	while ((slot >= 0) && taskmap[slot].releasing) {
		pthread_cond_wait(&task_released, &task_mutex);
		slot = task_slot(id);
	}

	if (slot < 0) {
		pthread_mutex_unlock(&task_mutex);
		return -ESRCH;
	}

	task_entry &task = taskmap[slot];

	if (task.state == TaskState::Released) {
		if (exit_code != nullptr) {
			*exit_code = task.exit_code;
		}

		task.state = TaskState::Unused;
		pthread_mutex_unlock(&task_mutex);
		return 0;
	}

	// another thread joins the task
	if (task.joining) {
		pthread_mutex_unlock(&task_mutex);
		return -EINVAL;
	}

	// the task keeps running while it is joined: it can still be listed and deleted
	task.joining = true;
	pthread_mutex_unlock(&task_mutex);

	return task_release(task, false, exit_code);
}

int px4_task_join(px4_task_t id)
{
	return px4_zephyr_task_join(id, nullptr);
}

int px4_task_delete(px4_task_t id)
{
	pthread_mutex_lock(&task_mutex);

	const int slot = task_slot(id);

	if ((slot < 0) || (taskmap[slot].state != TaskState::Running)) {
		pthread_mutex_unlock(&task_mutex);
		return -EINVAL;
	}

	const pthread_t pid = taskmap[slot].pid;

	if (pthread_equal(pid, pthread_self())) {
		pthread_mutex_unlock(&task_mutex);
		px4_task_exit(0);
	}

	// The task cancels asynchronously: Zephyr aborts its thread wherever it runs, as Zephyr's blocking calls are not
	// cancellation points. A lock the task holds then, such as the heap's or that of a uORB topic, is never released,
	// and the threads that wait for it block forever. MAVLink deletes its task only once it did not stop when asked to.
	const int rv = pthread_cancel(pid);
	char name[sizeof(taskmap[slot].name)];
	strcpy(name, taskmap[slot].name);

	if (rv == 0) {
		taskmap[slot].state = TaskState::Exited;
	}

	pthread_mutex_unlock(&task_mutex);

	if (rv == 0) {
		PX4_WARN("task %s deleted", name);
	}

	return -rv;
}

void px4_task_exit(int ret)
{
	const pthread_t pid = pthread_self();

	pthread_mutex_lock(&task_mutex);

	for (task_entry &task : taskmap) {
		if ((task.state == TaskState::Running) && pthread_equal(task.pid, pid)) {
			task.state = TaskState::Exited;
			break;
		}
	}

	pthread_mutex_unlock(&task_mutex);

	pthread_exit((void *)(intptr_t)ret);
}

int px4_task_kill(px4_task_t id, int sig)
{
	return -ENOTSUP;
}

void px4_show_tasks()
{
	int count = 0;

	PX4_INFO("Active Tasks:");

	pthread_mutex_lock(&task_mutex);

	for (const task_entry &task : taskmap) {
		if (task.state == TaskState::Running) {
			PX4_INFO("   %-10s %lu", task.name, (unsigned long)task.pid);
			count++;
		}
	}

	pthread_mutex_unlock(&task_mutex);

	if (count == 0) {
		PX4_INFO("   No running tasks");
	}
}

bool px4_task_is_running(const char *taskname)
{
	bool running = false;

	pthread_mutex_lock(&task_mutex);

	for (const task_entry &task : taskmap) {
		if ((task.state == TaskState::Running) && (strcmp(task.name, taskname) == 0)) {
			running = true;
			break;
		}
	}

	pthread_mutex_unlock(&task_mutex);

	return running;
}

px4_task_t px4_getpid()
{
	const pthread_t pid = pthread_self();
	px4_task_t ret = -1;

	pthread_mutex_lock(&task_mutex);

	// released tasks have no thread, whose pthread_t Zephyr gives to new threads
	for (int i = 0; i < PX4_MAX_TASKS; i++) {
		if (((taskmap[i].state == TaskState::Running) || (taskmap[i].state == TaskState::Exited))
		    && pthread_equal(taskmap[i].pid, pid)) {
			ret = task_id(i);
			break;
		}
	}

	pthread_mutex_unlock(&task_mutex);

	return ret;
}

const char *px4_get_taskname()
{
	const px4_task_t id = px4_getpid();
	return (id >= 0) ? taskmap[id & TASK_SLOT_MASK].name : "UnknownApp";
}

int px4_prctl(int option, const char *arg2, px4_task_t pid)
{
	if (option == PR_SET_NAME) {
		return pthread_setname_np(pthread_self(), arg2);
	}

	PX4_WARN("FAILED SETTING TASK NAME");
	return -1;
}
