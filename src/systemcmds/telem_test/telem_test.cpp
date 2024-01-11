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

static void usage(const char *reason)
{
	if (reason != nullptr) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION("Telemetry UART test tool");

	PRINT_MODULE_USAGE_NAME("telem_test", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("flush", "Flush UART");
	PRINT_MODULE_USAGE_ARG("<dev>", "Uart device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("loopback", "Loopback test between uarts");
	PRINT_MODULE_USAGE_ARG("<dev1>", "First UART device", false);
	PRINT_MODULE_USAGE_ARG("<dev2>", "Second UART device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("send", "Send string to UART");
	PRINT_MODULE_USAGE_ARG("<dev>", "Uart device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("recv", "Receive string from UART");
	PRINT_MODULE_USAGE_ARG("<dev>", "UART device", false);
}

static int test_recv(int uart_recv, char *recv_buf, size_t buf_len)
{
	if (read(uart_recv, recv_buf, buf_len) < 0) {
		printf("ERROR read: %d\n", errno);
		return 1;
	}

	return 0;
}

static int test_send(int uart_send, const char *send_buf)
{
	if (write(uart_send, send_buf, strlen(send_buf)) < 0) {
		printf("ERROR write: %d\n", errno);
		return 1;
	}

	px4_usleep(200000);

	return 0;
}

static int loopback(int uart_send, int uart_recv, const char *send_buf)
{
	char recv_buf[15] = {0};

	if (test_send(uart_send, send_buf)) {
		return 1;
	}

	if (test_recv(uart_recv, recv_buf, sizeof(recv_buf) - 1)) {
		return 1;
	}

	if (strncmp(send_buf, recv_buf, strlen(send_buf)) != 0) {
		printf("ERROR: send != recv\n");
		printf("    send '%s'\n", send_buf);
		printf("    recv '%s'\n", recv_buf);
		return 1;
	}

	return 0;
}

static int test_loopback(int uart1, int uart2)
{
	const char *send_str1 = "send_uart_1";
	const char *send_str2 = "send_uart_2";

	printf("send: first -> second\n");

	if (loopback(uart1, uart2, send_str1)) {
		return 1;
	}

	printf("send: second -> first\n");

	if (loopback(uart2, uart1, send_str2)) {
		return 1;
	}

	return 0;
}

extern "C" __EXPORT int telem_test_main(int argc, char *argv[])
{
	int res = 1;

	if (argc < 3) {
		usage(nullptr);
		return 1;
	}

	const char *test_type = argv[1];
	const char *uart1_dev = argv[2];
	const char *uart2_dev = NULL;

	if (argc > 3) {
		uart2_dev = argv[3];
	}

	printf("test type: %s\n", test_type);

	int uart1 = open(uart1_dev, O_RDWR | O_NONBLOCK | O_NOCTTY);

	if (uart1 < 0) {
		printf("ERROR: %s open\n", uart1_dev);
		return 1;
	}

	if (strcmp(test_type, "loopback") == 0) {
		printf("first uart: %s\n", uart1_dev);

		if (!uart2_dev) {
			usage("ERROR: second uart not specified");
			close(uart1);
			return 1;
		}

		int uart2 = open(uart2_dev, O_RDWR | O_NONBLOCK | O_NOCTTY);

		if (uart2 < 0) {
			printf("ERROR: %s open\n", uart2_dev);
			close(uart1);
			return 1;
		}

		printf("second uart: %s\n", uart2_dev);

		res = test_loopback(uart1, uart2);

		close(uart2);

	} else if (strcmp(test_type, "flush") == 0) {
		printf("flush uart: %s\n", uart1_dev);

		res = tcflush(uart1, TCIOFLUSH);

		if (res < 0) {
			printf("ERROR tcflush: %d\n", errno);
		}

	} else if (strcmp(test_type, "send") == 0) {
		printf("uart: %s\n", uart1_dev);

		const char *send_str = "send_uart";
		res = test_send(uart1, send_str);

		printf("sent: '%s'\n", send_str);

	} else if (strcmp(test_type, "recv") == 0) {
		printf("uart: %s\n", uart1_dev);

		char recv_buf[20] = {0};
		res = test_recv(uart1, recv_buf, sizeof(recv_buf) - 1);

		printf("recv: '%s'\n", recv_buf);

	} else {
		usage("ERROR: test type not supported");
	}

	close(uart1);
	return res;
}
