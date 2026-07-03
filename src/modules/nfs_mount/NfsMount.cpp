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

#include "NfsMount.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>
#include <cerrno>

#ifdef __PX4_NUTTX
#include <nuttx/config.h>
#include <arpa/inet.h>
#include <cstring>
#include <netinet/in.h>
#include <sys/mount.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <nuttx/fs/nfs.h>
#endif

using namespace time_literals;

static constexpr const char *NFS_MOUNT_POINT = CONFIG_NFS_MOUNT_MOUNT_POINT;
static constexpr const char *NFS_SERVER_PATH = CONFIG_NFS_MOUNT_SERVER_PATH;

ModuleBase::Descriptor NfsMount::desc{task_spawn, custom_command, print_usage};

NfsMount::NfsMount() : ModuleParams(nullptr) {}

#ifdef __PX4_NUTTX

/* Send a minimal RPC NULL call to portmapper and wait for a reply.
 * recv() blocks on the socket semaphore (not net_lock), so no priority
 * inheritance occurs while waiting.  Returns true when any reply arrives. */
static bool portmapper_up(in_addr_t ip)
{
	/* XDR: xid=1, CALL, rpcver=2, prog=100000, ver=2, proc=0,
	 * cred/verf = AUTH_NONE length 0 */
	static const uint8_t null_call[] = {
		0, 0, 0, 1,  0, 0, 0, 0,  0, 0, 0, 2,
		0, 1, 0x86, 0xa0,  0, 0, 0, 2,  0, 0, 0, 0,
		0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,
	};
	struct sockaddr_in dst {};
	dst.sin_family = AF_INET;
	dst.sin_port   = htons(NFS_PMAPPORT);
	dst.sin_addr.s_addr = ip;

	int s = socket(AF_INET, SOCK_DGRAM, 0);

	if (s < 0) { return false; }

	const struct timeval tv = {0, 200000}; /* 200 ms */
	setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

	sendto(s, null_call, sizeof(null_call), 0,
	       reinterpret_cast<const struct sockaddr *>(&dst), sizeof(dst));

	uint8_t reply[32];
	const bool up = recv(s, reply, sizeof(reply), 0) > 0;

	close(s);
	return up;
}

#endif /* __PX4_NUTTX */

void NfsMount::run()
{
	updateParams();

#ifdef __PX4_NUTTX
	const int32_t ip_int = _param_nfs_ip.get();
	char ip_str[16];
	snprintf(ip_str, sizeof(ip_str), "%u.%u.%u.%u",
		 static_cast<uint8_t>(ip_int >> 24),
		 static_cast<uint8_t>(ip_int >> 16),
		 static_cast<uint8_t>(ip_int >>  8),
		 static_cast<uint8_t>(ip_int));

	struct sockaddr_in sin {};
	sin.sin_family = AF_INET;
	sin.sin_port   = htons(NFS_PMAPPORT);
	inet_pton(AF_INET, ip_str, &sin.sin_addr);

	struct nfs_args args {};
	memcpy(&args.addr, &sin, sizeof(sin));
	args.addrlen = sizeof(sin);
	args.sotype  = SOCK_DGRAM;
	args.flags   = NFSMNT_SOFT | NFSMNT_TIMEO | NFSMNT_RETRANS;
	args.timeo   = 10;  /* 1 s per RPC attempt (deciseconds) */
	args.retrans = 3;
	args.path    = const_cast<char *>(NFS_SERVER_PATH);

	while (!should_exit()) {
		/* Probe portmapper via socket API before calling mount(): mount() acquires
		 * net_lock internally, which can cause priority-inheritance starvation if
		 * the server is unreachable.  recv() on a plain socket blocks on the socket
		 * semaphore (not net_lock), so we use it to confirm the server is responsive
		 * before handing control to mount(). */
		if (portmapper_up(sin.sin_addr.s_addr)) {
			if (mount(nullptr, NFS_MOUNT_POINT, "nfs", 0, &args) == 0) {
				PX4_INFO("mounted %s:%s at %s", ip_str, NFS_SERVER_PATH, NFS_MOUNT_POINT);

				nfs_up_s msg{};
				msg.timestamp = hrt_absolute_time();
				// Use orb_advertise instead of publish to ensure the topic remains visible
				// after this one-shot module exits.
				orb_advertise(ORB_ID(nfs_up), &msg);

				return;
			}
		}

		px4_usleep(5_s);
	}

#endif
}

int NfsMount::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int, char **) -> ModuleBase * {
		return new NfsMount();
	}, argc, argv);
}

int NfsMount::task_spawn(int argc, char *argv[])
{
	desc.task_id = px4_task_spawn_cmd("nfs_mount",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  PX4_STACK_ADJUSTED(2000),
					  (px4_main_t)&run_trampoline,
					  (char *const *)argv);

	if (desc.task_id < 0) {
		desc.task_id = -1;
		return -errno;
	}

	return 0;
}

int NfsMount::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int NfsMount::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Mounts an NFS filesystem from NFS_IP on /fs/nfs.
Probes portmapper with a raw UDP socket (no net_lock starvation) and
only calls mount() once the server is confirmed responsive.
Started automatically by rcS when NFS_EN is set.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("nfs_mount", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int nfs_mount_main(int argc, char *argv[])
{
	return ModuleBase::main(NfsMount::desc, argc, argv);
}
