/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
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
 * Implementation of existing task API for NuttX
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>

#include <px4_platform/task.h>

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <nuttx/kthread.h>

#include <sys/wait.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <sched.h>
#include <errno.h>
#include <stdbool.h>
#include <spawn.h>
#include <libgen.h>
#include <fcntl.h>

#include <builtin/builtin.h>

int px4_task_spawn_cmd(const char *name, int scheduler, int priority, int stack_size, main_t entry, char *const argv[])
{
#if !defined(CONFIG_DISABLE_ENVIRON) && !defined(CONFIG_BUILD_KERNEL)
	/* None of the modules access the environment variables (via getenv() for instance), so delete them
	 * all. They are only used within the startup script, and NuttX automatically exports them to the children
	 * tasks.
	 * This frees up a considerable amount of RAM.
	 */
	clearenv();
#endif

#if !defined(__KERNEL__)
	/* create the task */
	int pid = task_create(name, priority, stack_size, entry, argv);
#else
	int pid = kthread_create(name, priority, stack_size, entry, argv);
#endif

	return pid;
}

int px4_task_delete(int pid)
{
#if !defined(__KERNEL__)
	return task_delete(pid);
#else
	return kthread_delete(pid);
#endif
}

const char *px4_get_taskname(void)
{
	return getprogname();
}

int px4_exec(const char *appname, char *const *argv, const char *redirfile, int oflags)
{
#ifdef CONFIG_BUILTIN
	const struct nsh_param_s param = {
		.fd_in = -1,
		.fd_out = -1,
		.oflags_in = O_RDONLY,
		.oflags_out = oflags,
		.file_in = NULL,
		.file_out = redirfile,
	};
	return exec_builtin(appname, argv, &param);
#else
	char path[CONFIG_PATH_MAX];
	posix_spawn_file_actions_t file_actions;
	posix_spawnattr_t attr;
	pid_t pid;
	int ret;

	/* We launch processes from the /bin/ folder only */

	sprintf(path, "/bin/");
	strcat(path, basename((char *)appname));

	/* Initialize the attributes */

	ret = posix_spawnattr_init(&attr);

	if (ret != 0) {
		goto errout;
	}

	/* Initialize the file actions structure */

	ret = posix_spawn_file_actions_init(&file_actions);

	if (ret != 0) {
		goto errout_with_attrs;
	}

	/* Redirect output if instructed to do so */

	if (redirfile) {
		ret = posix_spawn_file_actions_addopen(&file_actions, 1, redirfile, oflags, 0644);

		if (ret != 0) {
			goto errout_with_actions;
		}
	}

	/* Attempt to load the executable */

	ret = posix_spawnp(&pid, path, &file_actions, &attr, argv, environ);

	if (ret != 0) {
		goto errout_with_actions;
	}

	posix_spawn_file_actions_destroy(&file_actions);
	posix_spawnattr_destroy(&attr);
	return pid;

errout_with_actions:
	posix_spawn_file_actions_destroy(&file_actions);

errout_with_attrs:
	posix_spawnattr_destroy(&attr);

errout:
	return ERROR;
#endif
}
