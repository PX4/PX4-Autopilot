/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Simon Wilks <sjwilks@gmail.com>
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
 * @file hott_telemetry_main.c
 *
 * Graupner HoTT Telemetry implementation.
 *
 * The HoTT receiver polls each device at a regular interval at which point
 * a data packet can be returned if necessary.
 *
 * TODO: Add support for at least the vario and GPS sensor data.
 *
 */

#include <assert.h>
#include <fcntl.h>
#include <nuttx/config.h>
#include <poll.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <systemlib/systemlib.h>

#include "messages.h"

/* The following are equired for UART direct manipulation. */
#include <arch/board/board.h>
#include "up_arch.h"
#include "chip.h"
#include "stm32_internal.h"

static volatile bool thread_should_exit = false;        /**< Deamon exit flag */
static volatile bool thread_running = false;            /**< Deamon status flag */
static int daemon_task;                                 /**< Handle of deamon task / thread */
static const char daemon_name[] = "hott_telemetry";
static const char commandline_usage[] = "usage: hott_telemetry start|status|stop [-d <device>]";

#define STATIC_ASSERT_EXPR(c) (sizeof(struct { int:-!(c); }))
/* &a[0] degrades to a pointer: a different type from an array */
#define ASSERT_ARRAY_EXPR(a) STATIC_ASSERT_EXPR(!__builtin_types_compatible_p(typeof(a), typeof(&(a)[0])))
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]) + ASSERT_ARRAY_EXPR(a))

/* A little console messaging experiment - console helper macro */
#define FATAL_MSG(_msg)		fprintf(stderr, "[%s] %s\n", daemon_name, _msg); exit(1);
#define ERROR_MSG(_msg)		fprintf(stderr, "[%s] %s\n", daemon_name, _msg);
#define INFO_MSG(_msg)		printf("[%s] %s\n", daemon_name, _msg);
/**
 * Deamon management function.
 */
__EXPORT int hott_telemetry_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int hott_telemetry_thread_main(int argc, char *argv[]);

static int recv_request_id(int uart, uint8_t *id);
static int send_data(int uart, uint8_t const *buffer, size_t size);

static int open_uart(const char *device, struct termios *uart_config_original)
{
	/* baud rate */
	static const speed_t speed = B19200;

	/* open uart */
	const int uart = open(device, O_RDWR | O_NOCTTY);

	if (uart < 0) {
		char msg[80];
		sprintf(msg, "Error opening port: %s\n", device);
		FATAL_MSG(msg);
	}

	/* Try to set baud rate */
	int termios_state;

	/* Back up the original uart configuration to restore it after exit */
	char msg[80];

	if ((termios_state = tcgetattr(uart, uart_config_original)) < 0) {
		sprintf(msg, "Error getting baudrate / termios config for %s: %d\n", device, termios_state);
		close(uart);
		FATAL_MSG(msg);
	}

	/* Fill the struct for the new configuration */
	struct termios uart_config = *uart_config_original;

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* Set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		sprintf(msg, "Error setting baudrate / termios config for %s: %d (cfsetispeed, cfsetospeed)\n",
			device, termios_state);
		close(uart);
		FATAL_MSG(msg);
	}

	if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0) {
		sprintf(msg, "Error setting baudrate / termios config for %s (tcsetattr)\n", device);
		close(uart);
		FATAL_MSG(msg);
	}

	/* Activate single wire mode */
	ioctl(uart, TIOCSSINGLEWIRE, SER_SINGLEWIRE_ENABLED);

	return uart;
}

int recv_request_id(int uart, uint8_t *id)
{
	static const int timeout = 1000 /* ms */;
	struct pollfd fds[] = { { .fd = uart, .events = POLLIN } };

	if (poll(fds, ARRAY_SIZE(fds), timeout) > 0) {
		/* Get the mode: binary or text  */
		uint8_t mode;
		read(uart, &mode, sizeof(mode));
		/* Read the device ID being polled */
		read(uart, id, sizeof(*id));

		/* if we have a binary mode request */
		if (mode != BINARY_MODE_REQUEST_ID) {
			return ERROR;
		}
	} else {
		ERROR_MSG("UART timeout on TX/RX port");
		return ERROR;
	}

	return OK;
}

int send_data(int uart, uint8_t const *buffer, size_t size)
{
	usleep(POST_READ_DELAY_IN_USECS);

	uint8_t checksum = 0;
	assert(size >= 1 && "checksum placeholder byte required as last place in buffer");

	/* Skipping the last octect, as we're assuming it's a placeholder for the checksum. */
	for (size_t i = 0; i < (size - 1); ++i) {
		write(uart, &buffer[i], sizeof(buffer[i]));
		checksum += buffer[i];

		/* Sleep before sending the next byte. */
		usleep(POST_WRITE_DELAY_IN_USECS);
	}

	/* Write the checksum: the last uint8_t in the stream is taken as the checksum. */
	write(uart, &checksum, sizeof(checksum));

	/* A hack the reads out what was written so the next read from the receiver doesn't get it. */
	/* TODO: Fix this!! */
	uint8_t dummy[size];
	read(uart, &dummy, size);

	return OK;
}

int hott_telemetry_thread_main(int argc, char *argv[])
{
	INFO_MSG("starting");

	thread_running = true;

	const char* device = "/dev/ttyS1";		/**< Default telemetry port: USART2 */

	/* read commandline arguments */
	for (int i = 0; i < argc && argv[i]; i++) {
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) { //device set
			if (argc > i + 1) {
				device = argv[i + 1];

			} else {
				thread_running = false;
				ERROR_MSG("missing parameter to -d");
				ERROR_MSG(commandline_usage);
				exit(1);
			}
		}
	}

	/* enable UART, writes potentially an empty buffer, but multiplexing is disabled */
	struct termios uart_config_original;
	const int uart = open_uart(device, &uart_config_original);

	if (uart < 0) {
		ERROR_MSG("Failed opening HoTT UART, exiting.");
		thread_running = false;
		exit(ERROR);
	}

	messages_init();

	while (!thread_should_exit) {
		uint8_t id = 0;
		if (recv_request_id(uart, &id) == OK) {
			uint8_t buffer[MESSAGE_BUFFER_SIZE];
			size_t size = sizeof(buffer);

			switch (id) {
			case ELECTRIC_AIR_MODULE:
				build_eam_response(buffer, &size);
				break;

			default:
				continue;	// Not a module we support.
			}

			send_data(uart, buffer, size);
		}
	}

	INFO_MSG("exiting");

	close(uart);

	thread_running = false;

	return 0;
}

/**
 * Process command line arguments and tart the daemon.
 */
int hott_telemetry_main(int argc, char *argv[])
{
	if (argc < 1) {
		ERROR_MSG("missing command");
		ERROR_MSG(commandline_usage);
		exit(1);
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			INFO_MSG("deamon already running");
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn("hott_telemetry",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 40,
					 2048,
					 hott_telemetry_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			INFO_MSG("daemon is running");

		} else {
			INFO_MSG("daemon not started");
		}

		exit(0);
	}

	ERROR_MSG("unrecognized command");
	ERROR_MSG(commandline_usage);
	exit(1);
}



