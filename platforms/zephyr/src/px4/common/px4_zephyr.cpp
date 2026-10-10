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

#include <px4_zephyr.h>

#include <parameters/param.h>
#include <px4_platform_common/init.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/workqueue.h>

#include <errno.h>
#include <string.h>

static constexpr int MAX_ARGS = 16;
static constexpr size_t MAX_LINE_LENGTH = 128;

int px4_zephyr_init(void)
{
	static bool initialized = false;

	if (initialized) {
		return PX4_OK;
	}

	initialized = true;

	work_queues_init();

	const int ret = px4_platform_init();

	// parameters are not persistent yet
	param_control_autosave(false);

	return ret;
}

int px4_zephyr_run_command(int argc, char *argv[])
{
	if (argc < 1) {
		return -EINVAL;
	}

	for (size_t i = 0; i < px4_zephyr_builtins_count; i++) {
		const px4_zephyr_builtin &builtin = px4_zephyr_builtins[i];

		if (strcmp(builtin.name, argv[0]) == 0) {
			char *args[MAX_ARGS + 1] {};

			for (int arg = 1; arg < argc && arg <= MAX_ARGS; arg++) {
				args[arg - 1] = argv[arg];
			}

			px4_task_t task = px4_task_spawn_cmd(builtin.name, SCHED_DEFAULT, builtin.priority,
							     PX4_STACK_ADJUSTED(builtin.stack_size),
							     builtin.main, args);

			if (task < 0) {
				PX4_ERR("failed to start %s (%d)", builtin.name, task);
				return task;
			}

			int exit_code = PX4_ERROR;
			const int ret = px4_zephyr_task_join(task, &exit_code);

			return (ret == 0) ? exit_code : ret;
		}
	}

	PX4_ERR("unknown command: %s", argv[0]);
	return -ENOENT;
}

static int run_line(const char *line)
{
	char buffer[MAX_LINE_LENGTH];
	strncpy(buffer, line, sizeof(buffer) - 1);
	buffer[sizeof(buffer) - 1] = '\0';

	char *argv[MAX_ARGS + 1] {};
	int argc = 0;
	char *saveptr = nullptr;

	for (char *token = strtok_r(buffer, " \t", &saveptr); token != nullptr && argc < MAX_ARGS;
	     token = strtok_r(nullptr, " \t", &saveptr)) {
		argv[argc++] = token;
	}

	if (argc == 0) {
		return PX4_OK;
	}

	return px4_zephyr_run_command(argc, argv);
}

int px4_zephyr_start(void)
{
	int ret = px4_zephyr_init();

	if (ret != PX4_OK) {
		PX4_ERR("platform init failed (%d)", ret);
		return ret;
	}

	for (const char *const *line = px4_zephyr_startup_script; *line != nullptr; line++) {
		run_line(*line);
	}

	return PX4_OK;
}
