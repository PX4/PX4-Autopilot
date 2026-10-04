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
 * @file remote_core.cpp
 *
 * Drive the rptun remote core: load and release it, print the board's view
 * of it, and exchange messages on the "rpmsg-hello" endpoint. Vring dumps
 * and the driver ping are the NuttX nsh "rpmsg" and "rptun" commands.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <board_config.h>

#include "remote_link.h"

namespace
{

int report(int ret, const char *what, int timeout_ms)
{
	if (ret == -ENOTCONN) {
		PX4_ERR("rpmsg-hello endpoint not bound (remote not started or no announce)");

	} else if (ret == -ETIMEDOUT) {
		PX4_ERR("%s: no reply within %d ms", what, timeout_ms);

	} else if (ret < 0) {
		PX4_ERR("%s: send failed: %d", what, ret);
	}

	return ret;
}

/* Round trip over the hello endpoint, timed with hrt. */
int do_ping(int times, int len)
{
	uint8_t payload[256];

	if (times <= 0 || len <= 0 || len > static_cast<int>(sizeof(payload))) {
		PX4_ERR("count must be > 0 and len 1..%zu", sizeof(payload));
		return 1;
	}

	uint64_t min = UINT64_MAX;
	uint64_t max = 0;
	uint64_t total = 0;
	int ok = 0;

	for (int i = 0; i < times; i++) {
		memset(payload, 'A' + (i % 26), len - 1);
		payload[len - 1] = '\0';

		uint64_t rtt = 0;
		int ret = report(remote_link_xfer(payload, len, nullptr, 0, 1000, &rtt), "ping", 1000);

		if (ret == -ENOTCONN) {
			return 1;
		}

		if (ret < 0) {
			continue;
		}

		ok++;
		total += rtt;
		min = rtt < min ? rtt : min;
		max = rtt > max ? rtt : max;
	}

	if (ok == 0) {
		PX4_ERR("no replies");
		return 1;
	}

	PX4_INFO("%d/%d replies, %d byte payload", ok, times, len);
	PX4_INFO("rtt min %" PRIu64 " us, avg %" PRIu64 " us, max %" PRIu64 " us", min, total / ok, max);
	return ok == times ? 0 : 1;
}

int do_hello(const char *text)
{
	char reply[128];

	if (report(remote_link_xfer(text, strlen(text) + 1, reply, sizeof(reply), 1000, nullptr), "hello", 1000) < 0) {
		return 1;
	}

	PX4_INFO("cm7 -> %s: \"%s\"", BOARD_RPMSG_CPUNAME, text);
	PX4_INFO("%s -> cm7: \"%s\"", BOARD_RPMSG_CPUNAME, reply);
	return 0;
}

/* Fault injection on the remote (boards/px4/fmu-v6xrt/cm4/fault.c): the kind
 * travels as "!fault <kind>" over the hello endpoint. Lethal kinds never
 * answer; the board status afterwards shows what the remote did.
 */
int do_fault(const char *kind)
{
	char text[64];
	char reply[128];

	snprintf(text, sizeof(text), "!fault %s", kind);
	int ret = report(remote_link_xfer(text, strlen(text) + 1, reply, sizeof(reply), 500, nullptr), kind, 500);

	if (ret == -ENOTCONN || (ret < 0 && ret != -ETIMEDOUT)) {
		return 1;
	}

	if (ret == 0) {
		PX4_INFO("%s: %s", BOARD_RPMSG_CPUNAME, reply);
	}

	return board_rpmsg_status();
}

void usage()
{
	PRINT_MODULE_DESCRIPTION("Load, start and exercise the rptun remote core.");
	PRINT_MODULE_USAGE_NAME("remote_core", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Load the remote ELF and release the core");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Hold the remote core in reset");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Reset state and status block, from the board");
	PRINT_MODULE_USAGE_COMMAND_DESCR("ping", "Round-trip test timed in the shell (hello endpoint)");
	PRINT_MODULE_USAGE_ARG("<count> <len>", "Iterations and payload bytes (default 100 64)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("hello", "Send a text message and print the reply");
	PRINT_MODULE_USAGE_ARG("<text>", "Message (default \"hello from cm7\")", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("fault", "Inject a fault on the remote, then print status");
	PRINT_MODULE_USAGE_ARG("<kind>", "Kind, or \"list\" (default)", true);
}

} // namespace

extern "C" __EXPORT int remote_core_main(int argc, char *argv[]);

int remote_core_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return 1;
	}

	remote_link_register(BOARD_RPMSG_CPUNAME);

	if (strcmp(argv[1], "start") == 0 || strcmp(argv[1], "stop") == 0) {
		int ret = remote_link_rptun(argv[1][2] == 'a');

		if (ret < 0) {
			PX4_ERR("%s failed: %d", argv[1], ret);
			return 1;
		}

		return 0;
	}

	if (strcmp(argv[1], "status") == 0) {
		return board_rpmsg_status();
	}

	if (strcmp(argv[1], "ping") == 0) {
		return do_ping(argc > 2 ? atoi(argv[2]) : 100, argc > 3 ? atoi(argv[3]) : 64);
	}

	if (strcmp(argv[1], "hello") == 0) {
		return do_hello(argc > 2 ? argv[2] : "hello from cm7");
	}

	if (strcmp(argv[1], "fault") == 0) {
		return do_fault(argc > 2 ? argv[2] : "list");
	}

	usage();
	return 1;
}
