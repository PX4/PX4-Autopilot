/****************************************************************************
 *
 *   Copyright (c) 2024 Technology Innovation Institute. All rights reserved.
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

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <drivers/drv_hrt.h>

constexpr int CHAIN_UART_COUNT = 3;

static void usage(const char *reason)
{
	if (reason != nullptr) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION("Telemetry UART test tool");

	PRINT_MODULE_USAGE_NAME("telem_test", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("flush", "Flush UART");
	PRINT_MODULE_USAGE_ARG("<dev>", "Uart device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("chain_loopback", "Loopback test for max. 3 uarts");
	PRINT_MODULE_USAGE_ARG("<dev1>", "First UART device", false);
	PRINT_MODULE_USAGE_ARG("<dev2>", "Second UART device", false);
	PRINT_MODULE_USAGE_ARG("<dev3>", "Third UART device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("send", "Send string to UART");
	PRINT_MODULE_USAGE_ARG("<dev>", "Uart device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("recv", "Receive string from UART");
	PRINT_MODULE_USAGE_ARG("<dev>", "UART device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("set_port", "Set UART port to 115200 baud");
	PRINT_MODULE_USAGE_ARG("<dev>", "UART device", false);
}

static int set_port_param(int fd)
{
	struct termios uart_config = {0};

	if (tcgetattr(fd, &uart_config) < 0) {
		PX4_ERR("tcgetattr");
		return 1;
	}

	if (cfsetspeed(&uart_config, 115200) < 0) {
		PX4_ERR("cfsetspeed");
		return 1;
	}

	if (tcsetattr(fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("tcsetattr");
		return 1;
	}

	return 0;
}

static int test_recv(int uart_recv, char *recv_buf, size_t buf_len)
{
	if (read(uart_recv, recv_buf, buf_len) < 0) {
		PX4_ERR("    ERROR read: %d", errno);
		return 1;
	}

	return 0;
}

static int test_send(int uart_send, const char *send_buf)
{
	if (write(uart_send, send_buf, strlen(send_buf)) < 0) {
		PX4_ERR("    ERROR write: %d", errno);
		return 1;
	}

	px4_usleep(200000);

	return 0;
}

static int send_receive_verify(int uart_send, int uart_recv, const char *send_buf)
{
	char recv_buf[15] = {0};

	if (test_send(uart_send, send_buf)) {
		return 1;
	}

	if (test_recv(uart_recv, recv_buf, sizeof(recv_buf) - 1)) {
		return 1;
	}

	if (strncmp(send_buf, recv_buf, strlen(send_buf)) != 0) {
		PX4_ERR("    ERROR: send != recv");
		PX4_ERR("        send '%s'", send_buf);
		PX4_ERR("        recv '%s'", recv_buf);
		return 1;
	}

	return 0;
}

static int test_chain_loopback(int uart[CHAIN_UART_COUNT])
{
	int last_idx = -1;

	if (uart[1] < 0) {
		usage("ERROR: uart loopback configuration");
		return 1;
	}

	char send_str[] = "send_uart_0";

	for (int i = 0; i < CHAIN_UART_COUNT - 1; i++) {
		if (uart[i + 1] < 0) {
			break;
		}

		// change send string content
		send_str[strlen(send_str) - 1] = '0' + i;

		PX4_INFO("send: %d. UART -> %d. UART", i + 1, i + 2);

		if (send_receive_verify(uart[i], uart[i + 1], send_str)) {
			return 1;
		}

		last_idx = i + 1;
	}

	// Complete loop: last -> first
	send_str[strlen(send_str) - 1] = '0' + last_idx;

	PX4_INFO("send: %d. UART -> %d. UART", last_idx + 1, 1);

	if (send_receive_verify(uart[last_idx], uart[0], send_str)) {
		return 1;
	}

	return 0;
}

static int open_uarts(const char *dev[CHAIN_UART_COUNT], int uart[CHAIN_UART_COUNT])
{
	for (int i = 0; i < CHAIN_UART_COUNT; i++) {
		if (dev[i] == nullptr) {
			uart[i] = -1;
			continue;
		}

		uart[i] = open(dev[i], O_RDWR | O_NONBLOCK | O_NOCTTY);

		if (uart[i] < 0) {
			PX4_ERR("ERROR: failed to open %s: %d", dev[i], errno);
			return 1;
		}

		if (set_port_param(uart[i])) {
			PX4_ERR("%s set_port_param", dev[i]);
			return 1;
		}

		if (tcflush(uart[i], TCIOFLUSH) < 0) {
			PX4_ERR("%s tcflush: %d", dev[i], errno);
			return 1;
		}

		PX4_INFO("%d. UART %s opened [%d]", i + 1, dev[i], uart[i]);
	}

	return 0;
}

static void cleanup(int uart[CHAIN_UART_COUNT])
{
	for (int i = 0; i < CHAIN_UART_COUNT; i++) {
		if (uart[i] < 0) {
			continue;
		}

		close(uart[i]);
	}
}

extern "C" __EXPORT int telem_test_main(int argc, char *argv[])
{
	int res = 1;

	const char *test_type = NULL;

	const char *uart_dev[CHAIN_UART_COUNT] = { nullptr };
	int uart_fd[CHAIN_UART_COUNT] = { 0 };

	switch (argc) {
	case 5:
		uart_dev[2] = argv[4];

	// fall through

	case 4:
		uart_dev[1] = argv[3];

	// fall through

	case 3:
		test_type = argv[1];
		uart_dev[0] = argv[2];
		break;

	default:
		usage(nullptr);
		return 1;
	}

	PX4_INFO("test type: %s", test_type);

	if (open_uarts(uart_dev, uart_fd)) {
		return 1;
	}

	// At minimum single UART must be defined
	if (uart_fd[0] < 0) {
		usage("ERROR: no UARTs defined");
		return 1;
	}

	if (strcmp(test_type, "chain_loopback") == 0) {
		res = test_chain_loopback(uart_fd);

		cleanup(uart_fd);

	} else if (strcmp(test_type, "flush") == 0) {
		PX4_INFO("flush uart: %s", uart_dev[0]);

		res = tcflush(uart_fd[0], TCIOFLUSH);

		if (res < 0) {
			PX4_ERR("ERROR tcflush: %d", errno);
		}

	} else if (strcmp(test_type, "send") == 0) {
		const char *send_str = "send_uart";
		res = test_send(uart_fd[0], send_str);

		PX4_INFO("sent: '%s'", send_str);

	} else if (strcmp(test_type, "recv") == 0) {
		char recv_buf[20] = {0};
		res = test_recv(uart_fd[0], recv_buf, sizeof(recv_buf) - 1);

		PX4_INFO("recv: '%s'", recv_buf);

	} else if (strcmp(test_type, "set_port") == 0) {
		res = set_port_param(uart_fd[0]);

	} else {
		usage("ERROR: test type not supported");
	}

	close(uart_fd[0]);
	return res;
}
