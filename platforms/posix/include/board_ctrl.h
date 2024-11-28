/****************************************************************************
 *
 *   Copyright (C) 2015-2022 PX4 Development Team. All rights reserved.
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
 * @file main.cpp
 *
 * This is the main() of PX4 for POSIX.
 *
 * The application is designed as a daemon/server app with multiple clients.
 * Both, the server and the client is started using this main() function.
 *
 * If the executable is called with its usual name 'px4', it will start the
 * server. However, if it is started with an executable name (symlink) starting
 * with 'px4-' such as 'px4-navigator', it will start as a client and try to
 * connect to the server.
 *
 * The symlinks for all modules are created using the build system.
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 * @author Roman Bapst <bapstroman@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include <algorithm>
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#if (_POSIX_MEMLOCK > 0)
#include <sys/mman.h>
#endif

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/init.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>

#include "hrt_work.h"
#include <drivers/drv_hrt.h>
#include <errno.h>
#include <parameters/param.h>
#include <pthread.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/init.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/workqueue.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#ifndef PATH_MAX
#define PATH_MAX 1024
#endif

struct BoardParameters {
	int argc;
	char **argv;
	int server_instance;
	bool pxh_off;
	std::string commands_file;
	std::string absolute_binary_paths;
};

class Board
{
private:
	Board() {}

public:
	static Board *instance();
	static void init(const BoardParameters &parameters)
	{
		auto x = instance();
		x->parameters = parameters;
	}
	bool _reset_req = false;

	BoardParameters parameters;

	void shutdown(bool reset);
	void run();
	void handle_sigint();
	int run_once();
private:
	void lockstep_cleanup();
	void lockstep_threadfunc();
	void lockstep_start();
};

