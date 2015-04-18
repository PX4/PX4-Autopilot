/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
 * @file px4_linux_impl.cpp
 *
 * PX4 Middleware Wrapper Linux Implementation
 */

#include <px4_defines.h>
#include <px4_middleware.h>
#include <px4_workqueue.h>
#include <stdint.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include "systemlib/param/param.h"

__BEGIN_DECLS

// FIXME - sysconf(_SC_CLK_TCK) not supported
long PX4_TICKS_PER_SEC = 1000;

__END_DECLS

extern struct wqueue_s gwork[NWORKERS];

namespace px4
{

void init(int argc, char *argv[], const char *app_name)
{
	printf("App name: %s\n", app_name);

	// Create high priority worker thread
	g_work[HPWORK].pid = px4_task_spawn_cmd("wkr_high",
			       SCHED_DEFAULT,
			       SCHED_PRIORITY_MAX,
			       2000,
			       work_hpthread,
			       (char* const*)NULL);

	// Create low priority worker thread
	g_work[LPWORK].pid = px4_task_spawn_cmd("wkr_low",
			       SCHED_DEFAULT,
			       SCHED_PRIORITY_MIN,
			       2000,
			       work_lpthread,
			       (char* const*)NULL);

}

}

