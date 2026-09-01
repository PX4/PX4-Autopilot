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
 * @file serial_autostart.cpp
 * Walk SER_*_PROTO / SER_*_BAUD and start the matching driver.
 */

#include "serial_autostart_config.h"

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <lib/parameters/param.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/time.h>

#ifdef __PX4_NUTTX
__BEGIN_DECLS
#include <builtin/builtin.h>
#include <sched.h>
#include <sys/wait.h>
__END_DECLS
#elif defined(__PX4_POSIX) || defined(__PX4_QURT)
#include "apps.h"
#endif

static constexpr unsigned kCmdBuf = 320;
static constexpr unsigned kMaxArgs = 16;
static constexpr unsigned kMavlinkCap = 3;

static bool get_int32(const char *name, int32_t *value)
{
	const param_t p = param_find(name);

	if (p == PARAM_INVALID) {
		return false;
	}

	return param_get(p, value) == PX4_OK;
}

static int run_argv(char *argv[], int argc)
{
	if (argc <= 0 || argv[0] == nullptr) {
		return 0;
	}

#ifdef __PX4_NUTTX
	sched_lock();
	const int pid = exec_builtin(argv[0], argv, nullptr, 0);
	sched_unlock();

	if (pid < 0) {
		PX4_ERR("failed to start %s", argv[0]);
		return pid;
	}

#  ifdef CONFIG_SCHED_WAITPID
	int status = 0;

	if (waitpid(pid, &status, 0) < 0) {
		return -errno;
	}

	if (WIFEXITED(status)) {
		return WEXITSTATUS(status);
	}

	return -1;
#  else
	return 0;
#  endif

#elif defined(__PX4_POSIX) || defined(__PX4_QURT)
	static apps_map_type apps;
	static bool apps_ready = false;

	if (!apps_ready) {
		init_app_map(apps);
		apps_ready = true;
	}

	const auto it = apps.find(argv[0]);

	if (it == apps.end()) {
		PX4_ERR("unknown command %s", argv[0]);
		return -1;
	}

	return it->second(argc, argv);

#else
	PX4_ERR("serial_autostart: no command runner");
	return -1;
#endif
}

static int run_tokens(char *line)
{
	char *argv[kMaxArgs + 1];
	int argc = 0;
	char *cursor = line;

	while (*cursor != '\0') {
		while (*cursor == ' ' || *cursor == '\t') {
			cursor++;
		}

		if (*cursor == '\0') {
			break;
		}

		if (argc >= static_cast<int>(kMaxArgs)) {
			PX4_ERR("too many args");
			return -1;
		}

		argv[argc++] = cursor;

		while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t') {
			cursor++;
		}

		if (*cursor != '\0') {
			*cursor++ = '\0';
		}
	}

	argv[argc] = nullptr;
	return run_argv(argv, argc);
}

static int run_line(const char *line);

static int run_line_skip_prefix(const char *line)
{
	const char *semi = strchr(line, ';');

	if (semi == nullptr) {
		return 0;
	}

	return run_line(semi + 1);
}

static int run_line(const char *line)
{
	while (*line == ' ' || *line == '\t') {
		line++;
	}

	if (*line == '\0' || *line == '#') {
		return 0;
	}

	unsigned delay = 0;

	if (sscanf(line, "sleep %u", &delay) == 1) {
		px4_usleep(delay * 1000000ull);
		return run_line_skip_prefix(line);
	}

	if (sscanf(line, "usleep %u", &delay) == 1) {
		px4_usleep(delay);
		return run_line_skip_prefix(line);
	}

	char buf[kCmdBuf];
	const int n = snprintf(buf, sizeof(buf), "%s", line);

	if (n < 0 || n >= static_cast<int>(sizeof(buf))) {
		PX4_ERR("command too long");
		return -1;
	}

	return run_tokens(buf);
}

static bool append_str(char **out, const char *end, const char *s)
{
	if (s == nullptr) {
		s = "";
	}

	while (*s != '\0') {
		if (*out >= end) {
			return false;
		}

		*(*out)++ = *s++;
	}

	return true;
}

static bool expand_command(const char *tmpl, const char *device, const char *baud,
			   const char *instance, const char *dual, char *out, size_t out_len)
{
	char *cursor = out;
	char *end = out + out_len - 1;

	while (*tmpl != '\0') {
		const char *insert = nullptr;
		size_t skip = 0;

		if (strncmp(tmpl, "${SERIAL_DEV}", 13) == 0) {
			insert = device;
			skip = 13;

		} else if (strncmp(tmpl, "${BAUD_PARAM}", 13) == 0) {
			insert = baud;
			skip = 13;

		} else if (strncmp(tmpl, "${DUAL_GPS_ARGS}", 16) == 0) {
			insert = dual;
			skip = 16;

		} else if (strncmp(tmpl, "${i}", 4) == 0) {
			insert = instance;
			skip = 4;
		}

		if (insert != nullptr) {
			if (!append_str(&cursor, end, insert)) {
				return false;
			}

			tmpl += skip;

		} else {
			if (cursor >= end) {
				return false;
			}

			*cursor++ = *tmpl++;
		}
	}

	*cursor = '\0';
	return true;
}

static int start_expanded(const char *tmpl, const char *device, const char *baud,
			  const char *instance, const char *dual)
{
	char expanded[kCmdBuf];

	if (!expand_command(tmpl, device, baud, instance, dual, expanded, sizeof(expanded))) {
		PX4_ERR("expand failed");
		return -1;
	}

	return run_line(expanded);
}

static int start_protocol_on_port(const SerialProtocolConfig &protocol, const char *device,
				  const char *baud, const char *dual)
{
	const int ret = start_expanded(protocol.command, device, baud, nullptr, dual);

	if (ret == 0 && protocol.success_command != nullptr) {
		run_line(protocol.success_command);

	} else if (ret != 0 && protocol.fail_command != nullptr) {
		run_line(protocol.fail_command);
	}

	return ret;
}

static bool param_is_one(const char *fmt, int instance)
{
	char name[24];
	snprintf(name, sizeof(name), fmt, instance);
	int32_t value = 0;
	return get_int32(name, &value) && value == 1;
}

static int32_t param_int(const char *fmt, int instance, int32_t fallback)
{
	char name[24];
	snprintf(name, sizeof(name), fmt, instance);
	int32_t value = fallback;
	get_int32(name, &value);
	return value;
}

static int start_mavlink(int instance, const char *device, const char *baud_param, bool ethernet)
{
	// Instance flags come from MAV_${i}_*.
	char *argv[24];
	int argc = 0;
	char b_buf[24];
	char m_buf[24];
	char r_buf[24];
	char u_buf[24];
	char o_buf[24];
	char f_buf[24];

	argv[argc++] = (char *)"mavlink";
	argv[argc++] = (char *)"start";

	if (ethernet) {
		snprintf(u_buf, sizeof(u_buf), "p:MAV_%d_UDP_PRT", instance);
		snprintf(o_buf, sizeof(o_buf), "p:MAV_%d_REMOTE_PRT", instance);
		argv[argc++] = (char *)"-u";
		argv[argc++] = u_buf;
		argv[argc++] = (char *)"-o";
		argv[argc++] = o_buf;

		const int32_t broadcast = param_int("MAV_%d_BROADCAST", instance, 0);

		if (broadcast == 1) {
			argv[argc++] = (char *)"-p";

		} else if (broadcast == 2) {
			argv[argc++] = (char *)"-c";
		}

	} else {
		argv[argc++] = (char *)"-d";
		argv[argc++] = (char *)device;
		argv[argc++] = (char *)"-b";
		snprintf(b_buf, sizeof(b_buf), "p:%s", baud_param);
		argv[argc++] = b_buf;
	}

	snprintf(m_buf, sizeof(m_buf), "p:MAV_%d_MODE", instance);
	argv[argc++] = (char *)"-m";
	argv[argc++] = m_buf;
	snprintf(r_buf, sizeof(r_buf), "p:MAV_%d_RATE", instance);
	argv[argc++] = (char *)"-r";
	argv[argc++] = r_buf;

	if (param_is_one("MAV_%d_FORWARD", instance)) {
		argv[argc++] = (char *)"-f";
	}

	if (param_is_one("MAV_%d_RADIO_CTL", instance)) {
		argv[argc++] = (char *)"-s";
	}

	const int32_t flow = param_int("MAV_%d_FLOW_CTRL", instance, 2);

	if (flow == 0) {
		argv[argc++] = (char *)"-Z";

	} else if (flow == 1) {
		argv[argc++] = (char *)"-z";
	}

	if (param_int("MAV_%d_MODE", instance, -1) == 6) {
		snprintf(f_buf, sizeof(f_buf), "p:MAV_%d_HL_FREQ", instance);
		argv[argc++] = (char *)"-F";
		argv[argc++] = f_buf;
	}

	argv[argc++] = (char *)"-x";
	argv[argc] = nullptr;

#if !defined(CONSTRAINED_FLASH)

	if (ethernet) {
		PX4_INFO("Starting MAVLink %d on ethernet", instance);

	} else {
		PX4_INFO("Starting MAVLink %d on %s", instance, device);
	}

#endif

	return run_argv(argv, argc);
}

static const SerialProtocolConfig *protocol_by_id(int32_t id)
{
	// Signed bound: unsigned i < constexpr 0 is always-false under -Wtype-limits
	// when the board has no serial_config protocols.
	for (int i = 0; i < static_cast<int>(kSerialProtocolCount); i++) {
		if (kSerialProtocols[i].id == id) {
			return &kSerialProtocols[i];
		}
	}

	return nullptr;
}

static unsigned protocol_index(const SerialProtocolConfig *protocol)
{
	return static_cast<unsigned>(protocol - kSerialProtocols);
}

static bool ethernet_on(const SerialProtocolConfig &protocol)
{
	if (protocol.ethernet_param == nullptr) {
		return false;
	}

	int32_t value = 0;
	return get_int32(protocol.ethernet_param, &value) && value == 1;
}

static int ethernet_mav_instance(int mav_next)
{
	if (mav_next >= static_cast<int>(kMavlinkCap)) {
		return -1;
	}

	return mav_next;
}

static void start_ports()
{
	int mav_next = 0;
	const int mav_lim = static_cast<int>(kMavlinkCap);

	constexpr unsigned taken_len = kSerialProtocolCount > 0 ? kSerialProtocolCount : 1;
	bool taken[taken_len] {};
	uint8_t collect_n[taken_len] {};
	const char *collect_dev[taken_len][2] {};
	const char *collect_baud[taken_len][2] {};
	uint8_t collect_rank[taken_len][2] {};

	for (int p = 0; p < static_cast<int>(kSerialPortCount); p++) {
		const SerialPortConfig &port = kSerialPorts[p];
		int32_t prot = 0;

		if (port.proto_param == nullptr || !get_int32(port.proto_param, &prot) || prot == 0) {
			continue;
		}

		const SerialProtocolConfig *protocol = protocol_by_id(prot);

		if (protocol == nullptr) {
			PX4_ERR("%s=%" PRId32 " is not a known protocol", port.proto_param, prot);
			continue;
		}

		const unsigned idx = protocol_index(protocol);

		if (protocol->kind == kSerialKindInstance) {
			if (mav_next >= mav_lim) {
#if !defined(CONSTRAINED_FLASH)
				PX4_WARN("%s instance limit", protocol->name);
#endif
				continue;
			}

			start_mavlink(mav_next, port.device, port.baud_param, false);
			mav_next++;
			continue;
		}

		if (protocol->kind == kSerialKindCollect) {
			if (collect_n[idx] >= 2) {
#if !defined(CONSTRAINED_FLASH)
				PX4_WARN("%s already assigned to two ports", protocol->name);
#endif
				continue;
			}

			// GPS1/GPS2/GPS3 before any other UART with the same protocol.
			unsigned n = collect_n[idx];
			unsigned at = n;

			while (at > 0 && port.collect_rank < collect_rank[idx][at - 1]) {
				collect_dev[idx][at] = collect_dev[idx][at - 1];
				collect_baud[idx][at] = collect_baud[idx][at - 1];
				collect_rank[idx][at] = collect_rank[idx][at - 1];
				at--;
			}

			collect_dev[idx][at] = port.device;
			collect_baud[idx][at] = port.baud_param;
			collect_rank[idx][at] = port.collect_rank;
			collect_n[idx]++;
			continue;
		}

		if (taken[idx]) {
#if !defined(CONSTRAINED_FLASH)
			PX4_WARN("%s already started", protocol->name);
#endif
			continue;
		}

#if !defined(CONSTRAINED_FLASH)
		PX4_INFO("Starting %s on %s", protocol->name, port.device);
#endif
		start_protocol_on_port(*protocol, port.device, port.baud_param, "");
		taken[idx] = true;
	}

	for (int i = 0; i < static_cast<int>(kSerialProtocolCount); i++) {
		if (kSerialProtocols[i].kind != kSerialKindCollect || collect_n[i] == 0) {
			continue;
		}

		const char *primary_dev = collect_dev[i][0];
		const char *primary_baud = collect_baud[i][0];

		if (primary_dev == nullptr || primary_baud == nullptr) {
			continue;
		}

		char dual[96] {};

		if (collect_n[i] > 1 && kSerialProtocols[i].secondary_command != nullptr) {
			if (!expand_command(kSerialProtocols[i].secondary_command, collect_dev[i][1], collect_baud[i][1],
					    nullptr, "", dual, sizeof(dual))) {
				PX4_ERR("dual GPS expand failed");
				dual[0] = '\0';
			}
		}

#if !defined(CONSTRAINED_FLASH)
		PX4_INFO("Starting %s on %s", kSerialProtocols[i].name, primary_dev);
#endif
		start_protocol_on_port(kSerialProtocols[i], primary_dev, primary_baud, dual);
	}

	for (int i = 0; i < static_cast<int>(kSerialProtocolCount); i++) {
		if (!ethernet_on(kSerialProtocols[i])) {
			continue;
		}

		if (kSerialProtocols[i].kind == kSerialKindInstance) {
			const int inst = ethernet_mav_instance(mav_next);

			if (inst < 0) {
#if !defined(CONSTRAINED_FLASH)
				PX4_WARN("%s instance limit", kSerialProtocols[i].name);
#endif
				continue;
			}

			start_mavlink(inst, nullptr, nullptr, true);
			continue;
		}

		if (taken[i]) {
#if !defined(CONSTRAINED_FLASH)
			PX4_WARN("%s already started", kSerialProtocols[i].name);
#endif
			continue;
		}

		const char *cmd = kSerialProtocols[i].ethernet_command;

		if (cmd == nullptr) {
			continue;
		}

#if !defined(CONSTRAINED_FLASH)
		PX4_INFO("Starting %s on ethernet", kSerialProtocols[i].name);
#endif
		run_line(cmd);
		taken[i] = true;
	}
}

static void print_usage()
{
	PRINT_MODULE_DESCRIPTION("Start serial drivers from SER_*_PROTO / SER_*_BAUD.\n");
	PRINT_MODULE_USAGE_NAME_SIMPLE("serial_autostart", "command");
}

extern "C" __EXPORT int serial_autostart_main(int argc, char *argv[])
{
	if (argc > 1) {
		print_usage();
		return 0;
	}

	start_ports();
	return 0;
}
