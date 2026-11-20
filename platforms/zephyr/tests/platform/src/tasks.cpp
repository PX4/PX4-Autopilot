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

#include "px4_test.h"

#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <px4_platform_common/tasks.h>
#include <px4_sched.h>
#include <px4_zephyr.h>

#include <climits>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/ztest.h>

static struct {
	int argc;
	char name[16];
	int value;
	int priority;
	px4_task_t pid;
} result;

static int task_main(int argc, char *argv[])
{
	result.argc = argc;
	strncpy(result.name, argv[0], sizeof(result.name) - 1);
	result.value = (argc > 1) ? atoi(argv[1]) : -1;
	result.priority = k_thread_priority_get(k_current_get());
	result.pid = px4_getpid();
	return 0;
}

static int run_task(int priority, char *const argv[])
{
	const px4_task_t task = px4_task_spawn_cmd("test_task", SCHED_DEFAULT, priority, 2048, task_main, argv);
	zassert_true(task >= 0, "spawn failed (%d)", task);
	zassert_ok(px4_task_join(task));
	return task;
}

ZTEST(px4_tasks, test_spawn_join)
{
	char value[] = "42";
	char *const argv[] = {value, nullptr};

	memset(&result, 0, sizeof(result));
	const px4_task_t task = run_task(SCHED_PRIORITY_DEFAULT, argv);

	zassert_equal(result.argc, 2);
	zassert_str_equal(result.name, "test_task");
	zassert_equal(result.value, 42);
	zassert_equal(result.pid, task);
	zassert_false(px4_task_is_running("test_task"));
}

static bool running_while_joined;

static int joined_main(int argc, char *argv[])
{
	// leave the test time to start joining this task
	k_sleep(K_MSEC(10));
	running_while_joined = px4_task_is_running("test_joined");
	return 3;
}

ZTEST(px4_tasks, test_join_exit_code)
{
	running_while_joined = false;
	const px4_task_t task = px4_task_spawn_cmd("test_joined", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 2048, joined_main,
				nullptr);
	zassert_true(task >= 0, "spawn failed (%d)", task);

	int exit_code = 0;
	zassert_ok(px4_zephyr_task_join(task, &exit_code));
	zassert_equal(exit_code, 3);

	// a task that is joined still runs
	zassert_true(running_while_joined);
	zassert_false(px4_task_is_running("test_joined"));
}

static px4_sem_t blocked_sem;

static int blocked_main(int argc, char *argv[])
{
	px4_sem_wait(&blocked_sem);
	return 0;
}

static void *delete_thread(void *arg)
{
	// leave the test time to start joining the task
	k_sleep(K_MSEC(10));
	return (void *)(intptr_t)px4_task_delete(*(px4_task_t *)arg);
}

ZTEST(px4_tasks, test_delete_joined)
{
	px4_sem_init(&blocked_sem, 0, 0);
	px4_task_t task = px4_task_spawn_cmd("test_blocked", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 2048, blocked_main,
					     nullptr);
	zassert_true(task >= 0, "spawn failed (%d)", task);

	pthread_t thread;
	zassert_ok(pthread_create(&thread, nullptr, delete_thread, &task));

	// a deleted task exits with PTHREAD_CANCELED
	int exit_code = 0;
	zassert_ok(px4_zephyr_task_join(task, &exit_code));
	zassert_equal(exit_code, (int)(intptr_t)PTHREAD_CANCELED);

	void *deleted = nullptr;
	zassert_ok(pthread_join(thread, &deleted));
	zassert_equal((intptr_t)deleted, 0);
	px4_sem_destroy(&blocked_sem);
}

static int exit_main(int argc, char *argv[])
{
	return (argc > 1) ? atoi(argv[1]) : 0;
}

ZTEST(px4_tasks, test_join_released)
{
	char seven[] = "7";
	char *const argv[] = {seven, nullptr};

	// a task that exits before it is joined, then released by the next spawn
	const px4_task_t exited = px4_task_spawn_cmd("test_exited", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 2048, exit_main,
				  argv);
	zassert_true(exited >= 0, "spawn failed (%d)", exited);
	k_sleep(K_MSEC(10));

	const px4_task_t next = px4_task_spawn_cmd("test_next", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 2048, exit_main,
				nullptr);
	zassert_true(next >= 0, "spawn failed (%d)", next);

	// it is still joined, with its exit code
	int exit_code = 0;
	zassert_ok(px4_zephyr_task_join(exited, &exit_code));
	zassert_equal(exit_code, 7);
	zassert_equal(px4_zephyr_task_join(exited, &exit_code), -ESRCH);
	zassert_ok(px4_task_join(next));

	// the id of a joined task doesn't name the task that reuses its slot
	const px4_task_t reused = px4_task_spawn_cmd("test_reused", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 2048,
				  exit_main, nullptr);
	zassert_true(reused >= 0, "spawn failed (%d)", reused);
	zassert_not_equal(reused, exited);
	zassert_not_equal(reused, next);
	zassert_equal(px4_task_join(next), -ESRCH);
	zassert_ok(px4_task_join(reused));
}

ZTEST(px4_tasks, test_priority)
{
	run_task(SCHED_PRIORITY_MAX, nullptr);
	const int high = result.priority;

	run_task(SCHED_PRIORITY_MIN, nullptr);
	const int low = result.priority;

	// preemptible threads, a lower Zephyr priority value runs first
	zassert_true(high >= 0);
	zassert_true(high < low, "high %d, low %d", high, low);
	zassert_true(low < K_IDLE_PRIO);
}

ZTEST(px4_tasks, test_priority_order)
{
	// PX4 priorities keep their order, and the work queues one Zephyr priority each
	for (int priority = SCHED_PRIORITY_MIN; priority < SCHED_PRIORITY_MAX; priority++) {
		const int posix = px4_zephyr_posix_priority(priority);
		const int next = px4_zephyr_posix_priority(priority + 1);

		zassert_true(posix <= next, "priority %d", priority);

		if (priority >= SCHED_PRIORITY_MAX - PX4_ZEPHYR_PRIORITY_BAND) {
			zassert_true(posix < next, "priority %d", priority);
			zassert_equal(next, posix + 1, "priority %d", priority);
		}
	}

	zassert_equal(px4_zephyr_posix_priority(SCHED_PRIORITY_MIN), 0);
	zassert_equal(px4_zephyr_posix_priority(SCHED_PRIORITY_MAX), CONFIG_NUM_PREEMPT_PRIORITIES - 1);

	// the commander, at the default priority + 40, below the controllers and the estimator, as on NuttX
	const int commander = px4_zephyr_posix_priority(SCHED_PRIORITY_DEFAULT + 40);
	zassert_true(commander < px4_zephyr_posix_priority(SCHED_PRIORITY_MAX
			+ px4::wq_configurations::nav_and_controllers.relative_priority));
	zassert_true(commander < px4_zephyr_posix_priority(SCHED_PRIORITY_MAX
			+ px4::wq_configurations::INS0.relative_priority));
	zassert_true(commander < px4_zephyr_posix_priority(SCHED_PRIORITY_MAX
			+ px4::wq_configurations::lp_default.relative_priority));
}

static void *thread_main(void *arg)
{
	*(int *)arg = k_thread_priority_get(k_current_get());
	return nullptr;
}

static int pthread_priority(int policy, int priority)
{
	pthread_attr_t attr;
	zassert_ok(pthread_attr_init(&attr));
	zassert_ok(pthread_attr_setstacksize(&attr, 2048));
	zassert_ok(pthread_attr_setschedpolicy(&attr, policy));

	sched_param param{};
	param.sched_priority = priority;
	zassert_ok(pthread_attr_setschedparam(&attr, &param));

	// PX4 reads back the priority of Zephyr
	sched_param read{};
	zassert_ok(pthread_attr_getschedparam(&attr, &read));
	zassert_equal(read.sched_priority, px4_zephyr_posix_priority(priority));

	int zephyr_priority = INT_MIN;
	pthread_t thread;
	zassert_ok(pthread_create(&thread, &attr, thread_main, &zephyr_priority));
	zassert_ok(pthread_join(thread, nullptr));
	pthread_attr_destroy(&attr);

	return zephyr_priority;
}

ZTEST(px4_tasks, test_pthread_priority)
{
	// threads that set a priority run at it, not at the priority of the thread that creates them
	const int priority = SCHED_PRIORITY_MAX - 10;
	const int expected = CONFIG_NUM_PREEMPT_PRIORITIES - 1 - px4_zephyr_posix_priority(priority);

	zassert_equal(pthread_priority(SCHED_RR, priority), expected);

	// SCHED_FIFO threads are preemptible, as on NuttX
	zassert_equal(pthread_priority(SCHED_FIFO, priority), expected);
}

ZTEST(px4_tasks, test_resources_released)
{
	for (int i = 0; i < 2 * CONFIG_POSIX_THREAD_THREADS_MAX; i++) {
		run_task(SCHED_PRIORITY_DEFAULT, nullptr);
	}

	// exited tasks that are not joined are released by the next spawn
	px4_task_t task = -1;

	for (int i = 0; i < 2 * CONFIG_POSIX_THREAD_THREADS_MAX; i++) {
		task = px4_task_spawn_cmd("test_task", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 2048, task_main, nullptr);
		zassert_true(task >= 0, "spawn %d failed (%d)", i, task);
		k_sleep(K_MSEC(1));
	}

	// no spawn releases the last one
	zassert_ok(px4_task_join(task));
}

static void *wait_thread(void *arg)
{
	px4_sem_wait(static_cast<px4_sem_t *>(arg));
	return nullptr;
}

ZTEST(px4_tasks, test_attributes_destroyed)
{
	pthread_attr_t attr;
	zassert_ok(pthread_attr_init(&attr));
	zassert_ok(pthread_attr_setstacksize(&attr, 2048));

	void *stack;
	size_t size;
	zassert_ok(pthread_attr_getstack(&attr, &stack, &size));

	px4_sem_t sem;
	px4_sem_init(&sem, 0, 0);
	pthread_t thread;
	zassert_ok(pthread_create(&thread, &attr, wait_thread, &sem));

	// the thread keeps running on its stack once its attributes are destroyed
	zassert_ok(pthread_attr_destroy(&attr));

	k_thread_stack_t *other = k_thread_stack_alloc(size, 0);
	zassert_not_null(other);
	zassert_true(((uintptr_t)other >= (uintptr_t)stack + size) || ((uintptr_t)other + size <= (uintptr_t)stack),
		     "stack of a running thread allocated again");
	zassert_ok(k_thread_stack_free(other));

	px4_sem_post(&sem);
	zassert_ok(pthread_join(thread, nullptr));
	px4_sem_destroy(&sem);
}

ZTEST(px4_tasks, test_run_command)
{
	char ver[] = "ver";
	char git[] = "git";
	char unknown[] = "unknown";
	char *ver_argv[] = {ver, git};
	char *unknown_argv[] = {unknown};

	zassert_ok(px4_zephyr_run_command(2, ver_argv));

	// the exit code of the command, ver prints its usage and fails without arguments
	zassert_equal(px4_zephyr_run_command(1, ver_argv), 1);
	zassert_equal(px4_zephyr_run_command(1, unknown_argv), -ENOENT);
}

ZTEST_SUITE(px4_tasks, NULL, px4_test_setup, NULL, NULL, NULL);
