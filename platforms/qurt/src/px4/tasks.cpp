/****************************************************************************
 *
 * Copyright (C) 2022-2026 ModalAI, Inc. All rights reserved.
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

#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/workqueue.h>

#include <drivers/drv_hrt.h>
#include <limits.h>
#include <pthread.h>
#include "hrt_work.h"

#define MODULE_NAME "tasks"

#define PX4_TASK_STACK_SIZE 8192
#define PX4_TASK_MAX_NAME_LENGTH 32
#define PX4_TASK_MAX_ARGC 32
#define PX4_TASK_MAX_ARGV_LENGTH 32
#define PX4_MAX_TASKS 24

typedef struct task_entry {
	pthread_t tid;
	px4_task_t id;
	char name[PX4_TASK_MAX_NAME_LENGTH + 4];
	char stack[PX4_TASK_STACK_SIZE];
	pthread_attr_t attr;
	px4_main_t main_entry;
	int argc;
	char argv_storage[PX4_TASK_MAX_ARGC][PX4_TASK_MAX_ARGV_LENGTH];
	char *argv[PX4_TASK_MAX_ARGC];
	bool isused;
	bool exiting;
	bool joining;
	bool waiting;

	task_entry() : tid(0), id(-1), isused(false), exiting(false), joining(false), waiting(false) {}
} task_entry_t;

static task_entry_t taskmap[PX4_MAX_TASKS];

static pthread_mutex_t task_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t task_cond = PTHREAD_COND_INITIALIZER;

// Task handles must not alias a new task when a table slot is reused. Every
// successfully issued handle below next_task_id is either still in taskmap or
// has already been joined. This also lets a late px4_task_join() succeed after
// automatic reaping, without retaining an unbounded list of completed tasks.
static unsigned int next_task_id = 0;

// All taskmap helpers require task_mutex to be held.
static task_entry_t *find_task(px4_task_t id)
{
	for (task_entry_t &task : taskmap) {
		if (task.isused && task.id == id) {
			return &task;
		}
	}

	return nullptr;
}

static task_entry_t *find_current_task()
{
	const pthread_t tid = pthread_self();

	for (task_entry_t &task : taskmap) {
		// A joined pthread's native ID can be reused before its joiner gets
		// task_mutex back. Do not mistake that old entry for the new thread.
		if (task.isused && !task.exiting && task.tid == tid) {
			return &task;
		}
	}

	return nullptr;
}

// Construct and destroy with task_mutex held. Cancelling a task while it owns
// a join or waits on task_cond could strand a reserved slot or a locked mutex.
// QuRT does not expose pthread cleanup handlers, so deletion returns EBUSY
// during these waits instead. Cooperative task shutdown is unaffected.
class TaskWaitGuard
{
public:
	TaskWaitGuard() : _task(find_current_task())
	{
		if (_task) { _task->waiting = true; }
	}

	~TaskWaitGuard()
	{
		if (_task) { _task->waiting = false; }
	}

private:
	task_entry_t *_task;
};

static int join_task(task_entry_t &task)
{
	// Keep the slot reserved while the mutex is released. Only one caller may
	// join this pthread, and it must terminate before its argv/attributes can
	// be reused. In particular, never hold task_mutex across pthread_join().
	task.joining = true;
	const pthread_t tid = task.tid;
	TaskWaitGuard wait_guard;
	pthread_mutex_unlock(&task_mutex);
	const int ret = pthread_join(tid, nullptr);
	pthread_mutex_lock(&task_mutex);

	if (ret == 0) {
		pthread_attr_destroy(&task.attr);
		task.tid = 0;
		task.isused = false;
	}

	task.joining = false;
	pthread_cond_broadcast(&task_cond);
	return ret;
}

static int wait_for_task(px4_task_t id)
{
	if (id < 0 || static_cast<unsigned int>(id) >= next_task_id) {
		return -EINVAL;
	}

	while (task_entry_t *task = find_task(id)) {
		if (!task->exiting && task->tid == pthread_self()) {
			return -EINVAL;
		}

		if (!task->joining) {
			return join_task(*task);
		}

		// Another join/delete/spawn owns the pthread_join(). Recheck the
		// handle after waking because its slot may already have been reused.
		TaskWaitGuard wait_guard;
		const int ret = pthread_cond_wait(&task_cond, &task_mutex);

		if (ret != 0) {
			return ret;
		}
	}

	return 0; // This handle was already reaped.
}

static void reap_exiting_tasks()
{
	for (task_entry_t &task : taskmap) {
		if (task.isused && task.exiting && !task.joining && task.tid != pthread_self()) {
			const int ret = join_task(task);

			if (ret != 0) {
				PX4_ERR("Couldn't join exited task %d (%d)", task.id, ret);
			}
		}
	}
}

// These are some Qurt pthread stubs needed for compilation and linking
extern "C" {

	int pthread_setname_np(pthread_t __target_thread, const char *__name) {	return 0; }

	// This function is in pthread.h but is not, apparently, defined in the
	// Qurt system image. So, it is being defined here. Unfortunately it doesn't
	// seem to work. It looks like the Qurt pthread implementation likes to allocate
	// the stack itself and not give us that ability.
	// int pthread_attr_setstackaddr(pthread_attr_t *attr, void * stackaddr) {
	// if (attr == NULL) return -1;
	// if (stackaddr == NULL) return -1;
	// attr->stackaddr = stackaddr;
	// attr->internal_stack = 0;
	// return 0;
	// }

	// This function is in pthread.h but is not, apparently, defined in the
	// Qurt system image. So, it is being defined here.
	int pthread_attr_setthreadname(pthread_attr_t *attr, const char *name)
	{
		if (attr == NULL) { return -1; }

		if (&attr->name[0] == NULL) { return -1; }

		if (name == NULL) { return -1; }

		size_t name_len = strlen(name);

		if (name_len > PX4_TASK_MAX_NAME_LENGTH) { name_len = PX4_TASK_MAX_NAME_LENGTH; }

		memcpy(attr->name, name, name_len);
		attr->name[name_len] = 0;

		return 0;
	}

	// Qurt only has one scheduling policy so this just returns a success
	int pthread_attr_setschedpolicy(pthread_attr_t *attr, int policy) { return 0; }

}

static void *entry_adapter(void *ptr)
{
	task_entry_t *data;
	data = (task_entry_t *) ptr;
	int ret = -1;

	if (data->main_entry) { ret = data->main_entry(data->argc, data->argv); }

	else { PX4_ERR("No valid task entry points"); }

	px4_task_exit(ret);
	return nullptr;

}

static px4_task_t px4_task_spawn_internal(const char *name, int priority, px4_main_t main_entry, char *const argv[])
{
	int retcode = 0;
	int i = 0;
	int task_index = 0;
	char *p = (char *)argv;

	PX4_INFO("Creating pthread %s", name);

	pthread_mutex_lock(&task_mutex);

	// Normal module shutdown does not call px4_task_join(). Reap completed
	// threads here so both their slots and pthread resources can be reused.
	reap_exiting_tasks();

	// Do not wrap task handles and risk operating on a replacement task.
	if (next_task_id > static_cast<unsigned int>(INT_MAX)) {
		pthread_mutex_unlock(&task_mutex);
		PX4_ERR("Task handles exhausted");
		return -1;
	}

	for (task_index = 0; task_index < PX4_MAX_TASKS; task_index++) {
		if (taskmap[task_index].isused == false) { break; }
	}

	if (task_index == PX4_MAX_TASKS) {
		pthread_mutex_unlock(&task_mutex);
		PX4_ERR("Hit maximum number of threads");
		return -1;
	}

	taskmap[task_index].argc = 0;

	while (p) {
		taskmap[task_index].argc++;
		p = argv[taskmap[task_index].argc];
	}

	if (taskmap[task_index].argc >= PX4_TASK_MAX_ARGC) {
		pthread_mutex_unlock(&task_mutex);
		PX4_ERR("Too many arguments for thread %d", taskmap[task_index].argc);
		return -1;
	}

	char *charPointer = const_cast<char *>(name);
	taskmap[task_index].argv[0] = charPointer;

	for (i = 0; i < PX4_TASK_MAX_ARGC; i++) {
		if (i < taskmap[task_index].argc) {
			int argument_length = strlen(argv[i]);

			if (argument_length >= PX4_TASK_MAX_ARGV_LENGTH) {
				pthread_mutex_unlock(&task_mutex);
				PX4_ERR("Argument %d is too long %d", i, argument_length);
				return -1;

			} else {
				strcpy(taskmap[task_index].argv_storage[i], argv[i]);
				taskmap[task_index].argv[i + 1] = taskmap[task_index].argv_storage[i];
			}

		} else {
			// Must add NULL at end of argv
			taskmap[task_index].argv[i + 1] = nullptr;
			taskmap[task_index].argc = i + 1;
			break;
		}
	}

	taskmap[task_index].main_entry = main_entry;

	if ((priority > 255) || (priority < 0)) {
		pthread_mutex_unlock(&task_mutex);
		PX4_ERR("Invalid priority %d", priority);
		return -1;
	}

	// Qurt threads have different priority numbers. 1 is the highest
	// priority and 254 is the lowest. But we are using the pthread
	// implementation on Qurt which returns 255 when you call sched_get_priority_max.
	// So, the big assumption is that the Qurt pthread implementation deals with
	// this properly when creating the underlying Qurt task.
	// TODO: Needs to be verified!

	if (strlen(name) >= PX4_TASK_MAX_NAME_LENGTH) {
		pthread_mutex_unlock(&task_mutex);
		PX4_ERR("Task name is too long %s", name);
		return -1;
	}

	strcpy(taskmap[task_index].name, "PX4_");
	strcpy(&taskmap[task_index].name[4], name);

	struct sched_param param;

	// Qurt threads have different priority numbers. 1 is the highest
	// priority and 255 is the lowest. But we are using the pthread
	// implementation on Qurt which returns 255 when you call sched_get_priority_max.
	// However, the Qurt pthread implementation deals with this properly when
	// creating the underlying Qurt task by mapping the high number into a low number.

	// For high priorities bump everything down a little so that critical Qurt
	// threads are not impacted.
	if (priority > 128) {
		priority -= 16;
	}

	// Likewise, for low priorities, bump everything up a little.
	else if (priority < 128) {
		priority += 10;
	}

	param.sched_priority = priority;

	pthread_attr_init(&taskmap[task_index].attr);
	pthread_attr_setthreadname(&taskmap[task_index].attr, taskmap[task_index].name);
	// See note above about the pthread_attr_setstackaddr function
	// pthread_attr_setstackaddr(&taskmap[task_index].attr, taskmap[task_index].stack);
	pthread_attr_setstacksize(&taskmap[task_index].attr, PX4_TASK_STACK_SIZE);
	pthread_attr_setschedparam(&taskmap[task_index].attr, &param);

	retcode = pthread_create(&taskmap[task_index].tid, &taskmap[task_index].attr, entry_adapter,
				 (void *) &taskmap[task_index]);

	if (retcode != PX4_OK) {
		pthread_attr_destroy(&taskmap[task_index].attr);
		pthread_mutex_unlock(&task_mutex);
		PX4_ERR("Couldn't create pthread %s", name);
		return -1;

	} else {
		PX4_INFO("Successfully created px4 task %s with tid %u",
			 taskmap[task_index].name,
			 (unsigned int) taskmap[task_index].tid);
	}

	taskmap[task_index].isused = true;
	taskmap[task_index].exiting = false;
	taskmap[task_index].joining = false;
	taskmap[task_index].waiting = false;
	const px4_task_t task_id = static_cast<px4_task_t>(next_task_id++);
	taskmap[task_index].id = task_id;

	pthread_mutex_unlock(&task_mutex);

	return task_id;
}

px4_task_t px4_task_spawn_cmd(const char *name, int scheduler, int priority, int stack_size, px4_main_t entry,
			      char *const argv[])
{
	if (entry == nullptr) {
		PX4_ERR("Entry function pointer is null");
		return -1;
	}

	return px4_task_spawn_internal(name, priority, entry, argv);
}

int px4_task_join(px4_task_t id)
{
	pthread_mutex_lock(&task_mutex);
	const int ret = wait_for_task(id);
	pthread_mutex_unlock(&task_mutex);
	return ret;
}

int px4_task_delete(px4_task_t id)
{
	pthread_mutex_lock(&task_mutex);

	if (id < 0 || static_cast<unsigned int>(id) >= next_task_id) {
		pthread_mutex_unlock(&task_mutex);
		return -EINVAL;
	}

	task_entry_t *task = find_task(id);

	if (task == nullptr) {
		pthread_mutex_unlock(&task_mutex);
		return 0; // Already reaped; never cancel a replacement task.
	}

	if (!task->exiting && pthread_self() == task->tid) {
		pthread_mutex_unlock(&task_mutex);
		px4_task_exit(0);
		return 0;
	}

	if (task->waiting) {
		pthread_mutex_unlock(&task_mutex);
		return -EBUSY;
	}

	if (!task->exiting) {
		const int ret = pthread_cancel(task->tid);

		if (ret != 0) {
			pthread_mutex_unlock(&task_mutex);
			return ret;
		}

		// Cancellation may bypass px4_task_exit(). Prevent another caller
		// from cancelling/signalling a native ID that has already been joined.
		task->exiting = true;
	}

	const int ret = wait_for_task(id);
	pthread_mutex_unlock(&task_mutex);
	return ret;
}

void px4_task_exit(int ret)
{
	pthread_mutex_lock(&task_mutex);
	task_entry_t *task = find_current_task();

	if (task) {
		// Retain the slot until another thread has joined us. Clearing it
		// here would permit reuse before pthread_exit() has completed.
		task->exiting = true;
	}

	pthread_mutex_unlock(&task_mutex);
	pthread_exit((void *)(unsigned long)ret);
}

int px4_task_kill(px4_task_t id, int sig)
{
	pthread_mutex_lock(&task_mutex);
	task_entry_t *task = find_task(id);
	const int ret = (task != nullptr && !task->exiting) ? pthread_kill(task->tid, sig) : -EINVAL;
	pthread_mutex_unlock(&task_mutex);
	return ret;
}

void px4_show_tasks()
{
	int idx = 0;
	int count = 0;

	PX4_INFO("Active Tasks:");
	pthread_mutex_lock(&task_mutex);

	for (; idx < PX4_MAX_TASKS; idx++) {
		if (taskmap[idx].isused && !taskmap[idx].exiting) {
			PX4_INFO("   %-10s %u", taskmap[idx].name,
				 (unsigned int) taskmap[idx].tid);
			count++;
		}
	}

	pthread_mutex_unlock(&task_mutex);

	if (count == 0) {
		PX4_INFO("No running tasks");
	}
}

px4_task_t px4_getpid()
{
	pthread_mutex_lock(&task_mutex);
	task_entry_t *task = find_current_task();
	const px4_task_t ret = task ? task->id : -1;
	pthread_mutex_unlock(&task_mutex);
	return ret;

}


const char *px4_get_taskname()
{
	pthread_mutex_lock(&task_mutex);
	task_entry_t *task = find_current_task();
	const char *prog_name = task ? task->name : "UnknownApp";
	pthread_mutex_unlock(&task_mutex);

	return prog_name;

}

static void timer_cb(void *data)
{
	px4_sem_t *sem = reinterpret_cast<px4_sem_t *>(data);

	sem_post(sem);
}

int px4_sem_timedwait(px4_sem_t *sem, const struct timespec *ts)
{
	work_s _hpwork = {};

	struct timespec ts_now;
	px4_clock_gettime(CLOCK_MONOTONIC, &ts_now);

	hrt_abstime timeout_us = ts_to_abstime((struct timespec *)ts) - ts_to_abstime(&ts_now);

	hrt_work_queue(&_hpwork, (worker_t)&timer_cb, (void *)sem, timeout_us);
	sem_wait(sem);
	hrt_work_cancel(&_hpwork);
	return 0;
}

int px4_prctl(int option, const char *arg2, px4_task_t pid)
{
	int rv = -1;

	if (option != PR_SET_NAME) { return rv; }

	pthread_mutex_lock(&task_mutex);

	task_entry_t *task = find_task(pid);

	if (task != nullptr && !task->exiting) {
		rv = pthread_attr_setthreadname(&task->attr, arg2);
	}

	pthread_mutex_unlock(&task_mutex);

	return rv;
}
