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
 * @file px4_zephyr.h
 *
 * Entry points of PX4 for Zephyr applications.
 */

#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*px4_zephyr_main_t)(int argc, char *argv[]);

struct px4_zephyr_builtin {
	const char *name;
	px4_zephyr_main_t main;
	int stack_size;
	int priority;
};

extern const struct px4_zephyr_builtin px4_zephyr_builtins[];
extern const size_t px4_zephyr_builtins_count;

extern const char *const px4_zephyr_startup_script[];

/**
 * Initialize the PX4 platform: hrt, work queues, parameters, uORB and logging.
 */
int px4_zephyr_init(void);

/**
 * Run a PX4 command in a task of its own and wait for it to exit.
 * argv[0] is the command name.
 */
int px4_zephyr_run_command(int argc, char *argv[]);

/**
 * Wait for a PX4 task to exit, as px4_task_join(), and get the value its main
 * function returned in exit_code, if not NULL.
 * @return 0, or a negative error code if the task could not be joined
 */
int px4_zephyr_task_join(int id, int *exit_code);

/**
 * Initialize PX4 and run the board startup script.
 */
int px4_zephyr_start(void);

#ifdef __cplusplus
}
#endif
