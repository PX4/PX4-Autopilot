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
 * @file px4_pthread.c
 *
 * A thread of the Zephyr POSIX API runs on the stack of the attributes it was
 * created with, which pthread_attr_destroy() frees. PX4 destroys the
 * attributes once the thread is created, as POSIX allows, so PX4 code calls
 * px4_pthread_attr_destroy() instead, through the pthread.h of the platform,
 * to keep the stack of a thread that still runs on it, and to free it once
 * the thread has exited.
 *
 * Zephyr only refuses to free the stack of a running thread, and leaks it then.
 * Its check misses a thread whose stack starts after an MPU or FPU guard, as on
 * Cortex-M7, and frees the stack while the thread runs on it (Zephyr issue
 * 103856).
 */

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

struct kept_stack {
	void *stack;
	size_t size;
};

struct stack_use {
	uintptr_t start;
	uintptr_t end;
	bool used;
};

static struct kept_stack kept_stacks[CONFIG_POSIX_THREAD_THREADS_MAX];
static K_MUTEX_DEFINE(kept_stacks_lock);

static void find_thread(const struct k_thread *thread, void *user_data)
{
	struct stack_use *use = user_data;

	// the stack starts after the guard that the MPU carves out of it
	if ((thread->stack_info.start >= use->start) && (thread->stack_info.start < use->end)) {
		use->used = true;
	}
}

static bool stack_used(void *stack, size_t size)
{
	struct stack_use use = {
		.start = (uintptr_t)stack,
		.end = (uintptr_t)stack + size,
	};

	k_thread_foreach(find_thread, &use);

	return use.used;
}

int px4_pthread_attr_destroy(pthread_attr_t *attr)
{
	void *stack = NULL;
	size_t size = 0;
	bool keep = false;

	k_mutex_lock(&kept_stacks_lock, K_FOREVER);

	for (size_t i = 0; i < ARRAY_SIZE(kept_stacks); i++) {
		if ((kept_stacks[i].stack != NULL) && !stack_used(kept_stacks[i].stack, kept_stacks[i].size)) {
			k_thread_stack_free(kept_stacks[i].stack);
			kept_stacks[i].stack = NULL;
		}
	}

	if ((pthread_attr_getstack(attr, &stack, &size) == 0) && (stack != NULL) && stack_used(stack, size)) {
		keep = true;

		// a stack that can't be freed later is still kept for its thread
		for (size_t i = 0; i < ARRAY_SIZE(kept_stacks); i++) {
			if (kept_stacks[i].stack == NULL) {
				kept_stacks[i].stack = stack;
				kept_stacks[i].size = size;
				break;
			}
		}
	}

	k_mutex_unlock(&kept_stacks_lock);

	if (keep) {
		memset(attr, 0, sizeof(*attr));
		return 0;
	}

	return pthread_attr_destroy(attr);
}
