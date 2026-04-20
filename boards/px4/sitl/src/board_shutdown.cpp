/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file board_shutdown.cpp
 *
 * sitl board shutdown and reboot backend.
 */

#ifndef MODULE_NAME
#define MODULE_NAME "board_shutdown"
#endif

#include <px4_platform_common/tasks.h>
#include <px4_platform_common/log.h>
#include <board_config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <limits.h>

#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

#ifndef PATH_MAX
#define PATH_MAX 1024
#endif

#if defined(BOARD_HAS_POWER_CONTROL)
int board_register_power_state_notification_cb(power_button_state_notification_t cb)
{
	return 0;
}

int board_power_off(int status)
{
	printf("Exiting NOW.\n");
	fflush(stdout);
	system_exit(0);
	return 0;
}
#endif // BOARD_HAS_POWER_CONTROL

#if defined(CONFIG_BOARDCTL_RESET)

#include <sys/boardctl.h>

extern "C" int boardctl(unsigned int cmd, uintptr_t arg)
{
	if (cmd == BOARDIOC_RESET) {

		char exe_path[PATH_MAX];

#ifdef __linux__
		ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);

		if (len == -1) {
			PX4_ERR("readlink /proc/self/exe failed: %s", strerror(errno));
			system_exit(1);
			return -1;
		}

		exe_path[len] = '\0';

#elif defined(__APPLE__)
		uint32_t size = sizeof(exe_path);

		if (_NSGetExecutablePath(exe_path, &size) != 0) {
			PX4_ERR("_NSGetExecutablePath failed");
			system_exit(1);
			return -1;
		}

#else
		PX4_ERR("SITL reboot not supported on this platform");
		system_exit(1);
		return -1;
#endif

		// Read the original command line arguments from /proc/self/cmdline (Linux)
		// or _NSGetArgv (macOS) and re-exec the process.
#ifdef __linux__
		int fd = open("/proc/self/cmdline", O_RDONLY);

		if (fd < 0) {
			PX4_ERR("open /proc/self/cmdline failed: %s", strerror(errno));
			system_exit(1);
			return -1;
		}

		char cmdline[4096];
		ssize_t bytes = read(fd, cmdline, sizeof(cmdline) - 1);
		close(fd);

		if (bytes <= 0) {
			PX4_ERR("read /proc/self/cmdline failed");
			system_exit(1);
			return -1;
		}

		cmdline[bytes] = '\0';

		// Parse NUL-separated arguments into argv array
		char *argv[64];
		int argc = 0;
		char *p = cmdline;

		while (p < cmdline + bytes && argc < 63) {
			argv[argc++] = p;
			p += strlen(p) + 1;
		}

		argv[argc] = nullptr;

#elif defined(__APPLE__)
		extern int *_NSGetArgc(void);
		extern char ***_NSGetArgv(void);
		char **argv = *_NSGetArgv();
#endif

		PX4_INFO_RAW("Rebooting SITL: %s\n", exe_path);
		fflush(stdout);
		fflush(stderr);

		// Close all file descriptors except stdin/stdout/stderr before exec.
		// Without this, sockets (MAVLink UDP ports, etc.) survive across execv
		// and the new process fails to bind them ("Address already in use").
		for (int i = 3; i < 1024; i++) {
			close(i);
		}

		execv(exe_path, argv);

		// execv only returns on failure
		PX4_ERR("execv failed: %s", strerror(errno));
		system_exit(1);
	}

	return -1;
}

#endif // CONFIG_BOARDCTL_RESET
