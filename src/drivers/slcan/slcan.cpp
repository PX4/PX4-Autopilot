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
 * @file slcan.cpp
 *
 * LAWICEL SLCAN (SocketCAN <-> serial/USB) for DroneCAN GUI Tool.
 *
 * Opens a second PF_CAN socket on the NuttX SocketCAN iface that UAVCAN
 * already uses. Does not reconfigure bitrate.
 */

#include "slcan.hpp"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <parameters/param.h>

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <cstring>
#include <termios.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <nuttx/config.h>
#include <nuttx/can.h>
#include <nuttx/net/ioctl.h>
#include <netpacket/can.h>

#ifndef CAN_EFF_FLAG
#  define CAN_EFF_FLAG 0x80000000U
#endif
#ifndef CAN_RTR_FLAG
#  define CAN_RTR_FLAG 0x40000000U
#endif
#ifndef CAN_SFF_MASK
#  define CAN_SFF_MASK 0x000007FFU
#endif
#ifndef CAN_EFF_MASK
#  define CAN_EFF_MASK 0x1FFFFFFFU
#endif

ModuleBase::Descriptor Slcan::desc{task_spawn, custom_command, print_usage};

static constexpr int SLCAN_STACK_SIZE = PX4_STACK_ADJUSTED(3072);

static constexpr uint8_t kDlcLen[16] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
};

static int hex_nibble(char c)
{
	if (c >= '0' && c <= '9') {
		return c - '0';
	}

	if (c >= 'A' && c <= 'F') {
		return c - 'A' + 10;
	}

	if (c >= 'a' && c <= 'f') {
		return c - 'a' + 10;
	}

	return -1;
}

static int parse_hex(const char *s, int nchars, uint32_t *out)
{
	uint32_t v = 0;

	for (int i = 0; i < nchars; i++) {
		const int nib = hex_nibble(s[i]);

		if (nib < 0) {
			return -1;
		}

		v = (v << 4) | static_cast<uint32_t>(nib);
	}

	*out = v;
	return 0;
}

static uint8_t len_to_dlc(uint8_t len)
{
	for (uint8_t i = 0; i < 16; i++) {
		if (kDlcLen[i] >= len) {
			return i;
		}
	}

	return 15;
}

Slcan::Slcan(const char *serial_dev, const char *can_iface)
{
	strlcpy(_serial_dev, serial_dev, sizeof(_serial_dev));
	strlcpy(_can_iface, can_iface, sizeof(_can_iface));
}

Slcan::~Slcan()
{
	close_fds();
}

void Slcan::close_can()
{
	if (_can_sock >= 0) {
		close(_can_sock);
		_can_sock = -1;
	}

	_open = false;
}

void Slcan::close_fds()
{
	if (_serial_fd >= 0) {
		px4_close(_serial_fd);
		_serial_fd = -1;
	}

	close_can();
}

int Slcan::open_can()
{
	_can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	if (_can_sock < 0) {
		PX4_ERR("socket PF_CAN: %d", errno);
		return -1;
	}

	struct ifreq ifr {};

	strlcpy(ifr.ifr_name, _can_iface, IFNAMSIZ);

	const int ifindex = if_nametoindex(ifr.ifr_name);

	if (ifindex == 0) {
		PX4_ERR("no iface %s (is NuttX SocketCAN initialized?)", _can_iface);
		close_can();
		return -1;
	}

	// Bring the iface up at UAVCAN_BITRATE if UAVCAN has not already.
	if (ioctl(_can_sock, SIOCGIFFLAGS, &ifr) == 0) {
		if ((ifr.ifr_flags & IFF_UP) == 0) {
			int32_t bitrate = 1000000;
			param_t p = param_find("UAVCAN_BITRATE");

			if (p == PARAM_INVALID) {
				p = param_find("CANNODE_BITRATE");
			}

			if (p != PARAM_INVALID) {
				(void)param_get(p, &bitrate);
			}

#ifdef CONFIG_NETDEV_CAN_BITRATE_IOCTL
			const bool can_fd = bitrate > 1000000;
			const uint16_t kbps = static_cast<uint16_t>(bitrate / 1000);
			ifr.ifr_ifru.ifru_can_data.arbi_bitrate = can_fd ? 1000 : kbps;
			ifr.ifr_ifru.ifru_can_data.arbi_samplep = 80;
			/* data_bitrate 0 makes NuttX FDCAN bittiming fail and
			 * leaves RX interrupts off (DNA never completes). */
			ifr.ifr_ifru.ifru_can_data.data_bitrate = kbps;
			ifr.ifr_ifru.ifru_can_data.data_samplep = 80;
			(void)ioctl(_can_sock, SIOCSCANBITRATE, &ifr);
			strlcpy(ifr.ifr_name, _can_iface, IFNAMSIZ);
#endif
			ifr.ifr_flags |= IFF_UP;
			(void)ioctl(_can_sock, SIOCSIFFLAGS, &ifr);
		}
	}

#ifdef CONFIG_NET_CAN_CANFD
	const int on = 1;

	if (setsockopt(_can_sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)) < 0) {
		PX4_WARN("CAN_RAW_FD_FRAMES not available");
	}

#endif

	struct sockaddr_can addr {};
	addr.can_family = AF_CAN;
	addr.can_ifindex = static_cast<int16_t>(ifindex);

	if (bind(_can_sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
		PX4_ERR("bind %s: %d", _can_iface, errno);
		close_can();
		return -1;
	}

	return 0;
}

int Slcan::open_serial()
{
	_serial_fd = px4_open(_serial_dev, O_RDWR | O_NONBLOCK);

	if (_serial_fd < 0) {
		PX4_ERR("open %s: %d", _serial_dev, errno);
		return -1;
	}

	struct termios uart_config {};

	if (tcgetattr(_serial_fd, &uart_config) == 0) {
		cfsetspeed(&uart_config, B115200);
		uart_config.c_lflag &= ~(ECHO | ECHOK | ICANON);
		uart_config.c_oflag &= ~ONLCR;
		(void)tcsetattr(_serial_fd, TCSANOW, &uart_config);
	}

	return 0;
}

void Slcan::ack()
{
	const char cr = '\r';
	(void)px4_write(_serial_fd, &cr, 1);
}

void Slcan::nack()
{
	const char bel = '\a';
	(void)px4_write(_serial_fd, &bel, 1);
}

void Slcan::send_ascii(const char *s)
{
	(void)px4_write(_serial_fd, s, strlen(s));
}

void Slcan::apply_armed_state()
{
	if (_armed) {
		if (_can_sock >= 0) {
			PX4_INFO("SLCAN disabled (armed)");
			close_can();
		}

	} else if (_can_sock < 0) {
		if (open_can() == 0) {
			PX4_INFO("SLCAN enabled (disarmed)");
		}
	}
}

void Slcan::send_can_frame(uint32_t can_id, const uint8_t *data, uint8_t len, bool ext, bool fd, bool brs)
{
#ifdef CONFIG_NET_CAN_CANFD
	struct canfd_frame frame {};
	frame.can_id = can_id | (ext ? CAN_EFF_FLAG : 0);
	frame.len = len;
	frame.flags = (fd && brs) ? CANFD_BRS : 0;
	memcpy(frame.data, data, len);
	const size_t mtu = fd ? CANFD_MTU : CAN_MTU;
	const ssize_t n = write(_can_sock, &frame, mtu);
#else
	(void)fd;
	(void)brs;
	struct can_frame frame {};
	frame.can_id = can_id | (ext ? CAN_EFF_FLAG : 0);
	frame.can_dlc = len;
	memcpy(frame.data, data, len > 8 ? 8 : len);
	const ssize_t n = write(_can_sock, &frame, CAN_MTU);
#endif

	if (n > 0) {
		_tx_count++;
	}
}

void Slcan::handle_command(const char *cmd, int len)
{
	if (len < 1) {
		return;
	}

	switch (cmd[0]) {
	case 'S': // bitrate index — accepted, bitrate is owned by UAVCAN
	case 's':
	case 'Y': // FD data bitrate
		ack();
		break;

	case 'O':
		if (_armed || _can_sock < 0) {
			nack();
			break;
		}

		_open = true;
		ack();
		break;

	case 'C':
		_open = false;
		ack();
		break;

	case 'F':
		send_ascii("F00\r");
		break;

	case 'V':
		send_ascii("V1011\r");
		break;

	case 'v':
		send_ascii("vPX4\r");
		break;

	case 'N':
		send_ascii("N0001\r");
		break;

	case 'Z':
		ack();
		break;

	case 't':
	case 'T':
	case 'd':
	case 'D': {
			const bool ext = (cmd[0] == 'T' || cmd[0] == 'D');
			const bool fd = (cmd[0] == 'd' || cmd[0] == 'D');
			const int id_chars = ext ? 8 : 3;

			if (len < id_chars + 2) {
				nack();
				return;
			}

			uint32_t id = 0;

			if (parse_hex(cmd + 1, id_chars, &id) < 0) {
				nack();
				return;
			}

			const int dlc_nib = hex_nibble(cmd[1 + id_chars]);

			if (dlc_nib < 0 || dlc_nib > 15) {
				nack();
				return;
			}

			const uint8_t dlen = fd ? kDlcLen[dlc_nib] : static_cast<uint8_t>(dlc_nib > 8 ? 8 : dlc_nib);
			const int data_off = 2 + id_chars;

			if (len < data_off + static_cast<int>(dlen) * 2) {
				nack();
				return;
			}

			uint8_t data[64] {};

			for (uint8_t i = 0; i < dlen; i++) {
				uint32_t b = 0;

				if (parse_hex(cmd + data_off + i * 2, 2, &b) < 0) {
					nack();
					return;
				}

				data[i] = static_cast<uint8_t>(b);
			}

			if (_armed || _can_sock < 0 || !_open) {
				nack();
				return;
			}

			send_can_frame(id, data, dlen, ext, fd, fd);
			ack();
			break;
		}

	default:
		ack();
		break;
	}
}

void Slcan::handle_serial()
{
	char buf[64];
	const ssize_t n = px4_read(_serial_fd, buf, sizeof(buf));

	if (n <= 0) {
		return;
	}

	for (ssize_t i = 0; i < n; i++) {
		const char c = buf[i];

		if (c == '\r' || c == '\n') {
			if (_line_len > 0) {
				_line[_line_len] = '\0';
				handle_command(_line, _line_len);
				_line_len = 0;
			}

			continue;
		}

		if (_line_len < static_cast<int>(sizeof(_line)) - 1) {
			_line[_line_len++] = c;

		} else {
			_line_len = 0;
			nack();
		}
	}
}

void Slcan::handle_can()
{
#ifdef CONFIG_NET_CAN_CANFD
	struct canfd_frame frame {};
	const ssize_t nbytes = recv(_can_sock, &frame, sizeof(frame), MSG_DONTWAIT);
	const bool fd = (nbytes == CANFD_MTU);
	const uint8_t len = frame.len;
#else
	struct can_frame frame {};
	const ssize_t nbytes = recv(_can_sock, &frame, sizeof(frame), MSG_DONTWAIT);
	const bool fd = false;
	const uint8_t len = frame.can_dlc;
#endif

	if (nbytes <= 0 || !_open) {
		return;
	}

	const bool ext = (frame.can_id & CAN_EFF_FLAG) != 0;
	const uint32_t id = frame.can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK);
	const uint8_t dlc = len_to_dlc(len);
	char out[8 + 1 + 1 + 64 * 2 + 2];
	int pos = 0;

	if (fd) {
		out[pos++] = ext ? 'D' : 'd';

	} else {
		out[pos++] = ext ? 'T' : 't';
	}

	if (ext) {
		pos += snprintf(out + pos, sizeof(out) - pos, "%08" PRIX32, id);

	} else {
		pos += snprintf(out + pos, sizeof(out) - pos, "%03" PRIX32, id);
	}

	pos += snprintf(out + pos, sizeof(out) - pos, "%X", dlc);

	for (uint8_t i = 0; i < len && i < 64; i++) {
		pos += snprintf(out + pos, sizeof(out) - pos, "%02X", frame.data[i]);
	}

	out[pos++] = '\r';
	(void)px4_write(_serial_fd, out, pos);
	_rx_count++;
}

void Slcan::poll_loop()
{
	uORB::Subscription armed_sub{ORB_ID(actuator_armed)};

	while (!should_exit()) {
		actuator_armed_s armed{};

		if (armed_sub.update(&armed)) {
			_armed = armed.armed;
		}

		apply_armed_state();

		struct pollfd fds[2] {};
		int nfds = 1;
		fds[0].fd = _serial_fd;
		fds[0].events = POLLIN;

		if (_can_sock >= 0) {
			fds[1].fd = _can_sock;
			fds[1].events = POLLIN;
			nfds = 2;
		}

		const int pret = ::poll(fds, nfds, 200);

		if (pret < 0) {
			if (errno == EINTR) {
				continue;
			}

			PX4_ERR("poll: %d", errno);
			break;
		}

		if (fds[0].revents & POLLIN) {
			handle_serial();
		}

		if (nfds > 1 && (fds[1].revents & POLLIN)) {
			handle_can();
		}
	}
}

void Slcan::run()
{
	if (open_can() < 0 || open_serial() < 0) {
		close_fds();
		return;
	}

	PX4_INFO("SLCAN on %s <-> %s (bitrate owned by UAVCAN)", _serial_dev, _can_iface);
	poll_loop();
	close_fds();
}

int Slcan::print_status()
{
	PX4_INFO("serial: %s", _serial_dev);
	PX4_INFO("can: %s", _can_iface);
	PX4_INFO("open: %s", _open ? "yes" : "no");
	PX4_INFO("armed: %s", _armed ? "yes" : "no");
	PX4_INFO("rx: %" PRIu32 "  tx: %" PRIu32, _rx_count, _tx_count);
	return 0;
}

int Slcan::task_spawn(int argc, char *argv[])
{
	int task_id = px4_task_spawn_cmd("slcan", SCHED_DEFAULT,
					 SCHED_PRIORITY_SLOW_DRIVER, SLCAN_STACK_SIZE,
					 (px4_main_t)&run_trampoline, (char *const *)argv);

	if (task_id < 0) {
		desc.task_id = -1;
		return -errno;
	}

	desc.task_id = task_id;
	return 0;
}

int Slcan::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return Slcan::instantiate(ac, av);
	}, argc, argv);
}

Slcan *Slcan::instantiate(int argc, char *argv[])
{
	const char *device = "/dev/ttyACM0";
	const char *iface = "can0";
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:i:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		case 'i':
			iface = myoptarg;
			break;

		default:
			print_usage("unrecognized flag");
			return nullptr;
		}
	}

	if (device == nullptr || iface == nullptr) {
		print_usage("missing -d or -i");
		return nullptr;
	}

	return new Slcan(device, iface);
}

int Slcan::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Slcan::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
LAWICEL SLCAN adapter on a NuttX SocketCAN interface.

Opens a second CAN socket so DroneCAN GUI Tool can talk to the bus while
UAVCAN keeps running. Bitrate is not changed (`S` is ACKed only).
SLCAN is fully disabled while armed (CAN socket closed, no RX/TX).

### Examples
Set `SYS_USB_AUTO` to SLCAN and plug USB, or:
$ slcan start -d /dev/ttyACM0 -i can0
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("slcan", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyACM0", nullptr, "Serial/USB device", true);
	PRINT_MODULE_USAGE_PARAM_STRING('i', "can0", "can0|can1", "SocketCAN iface", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int slcan_main(int argc, char *argv[])
{
	return ModuleBase::main(Slcan::desc, argc, argv);
}
