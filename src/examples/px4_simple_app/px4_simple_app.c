/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 *
 * Simple app to test SLIP utility.
 * Partial copy of platforms/nuttx/NuttX/apps/examples/thttpd/thttpd_main.c
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
// #include <unistd.h>
// #include <stdio.h>
// #include <poll.h>
// #include <string.h>
// #include <math.h>

// #include "netutils/netlib.h"

/* Include SLIP related libraries / macro defs */
#ifdef CONFIG_NET_SLIP
/* TTY device to use */

#pragma message "NET_SLIP is enabled, adding the libraries!"

// #  include <nuttx/net/net.h>
#  include <nuttx/net/slip.h>

#  ifndef CONFIG_NET_SLIPTTY
#    define CONFIG_NET_SLIPTTY "/dev/ttyS4"
#  endif

#  define SLIP_DEVNO 0

#  ifndef NET_DEVNAME
#    define NET_DEVNAME "sl0"
#  endif

#endif



__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("PX4 Simple SLIP Testing App!");

	/* Sanity Checks */
	// static_assert(CONFIG_NET_SLIPTTY == "/dev/ttyS4");

	/* Configure SLIP */

#ifdef CONFIG_NET_SLIP
	int ret = slip_initialize(SLIP_DEVNO, CONFIG_NET_SLIPTTY);

	if (ret < 0) {
		printf("ERROR: SLIP initialization failed: %d\n", ret);
		exit(1);
	}

#endif

	// Idle by just sleeping for 1 second continuously
	while (true) {
		px4_usleep(1E6);
	}

	PX4_INFO("exiting");

	return 0;
}
