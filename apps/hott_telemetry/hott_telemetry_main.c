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
 */

#include <fcntl.h>
#include <nuttx/config.h>
#include <poll.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <systemlib/systemlib.h>

#include "messages.h"

// The following are equired for UART direct manipulation.
#include <arch/board/board.h>
#include "up_arch.h"
#include "chip.h"
#include "stm32_internal.h"

static int thread_should_exit = false;		/**< Deamon exit flag */
static int thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

/**
 * Deamon management function.
 */
__EXPORT int hott_telemetry_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int hott_telemetry_thread_main(int argc, char *argv[]);

static int read_data(int uart);
static int send_data(int uart, const struct eam_module_msg *msg);
static void uart_disable_rx(void);
static void uart_disable_tx(void);

static int open_uart(const char *uart_name, struct termios *uart_config_original)
{
	/* baud rate */
	int speed = B19200;
	int uart;

	/* open uart */
	uart = open(uart_name, O_RDWR | O_NOCTTY);

	if (uart < 0) {
		fprintf(stderr, "[hott_telemetry] ERROR opening port: %s\n", uart_name);
		return ERROR;
	}

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(uart, uart_config_original)) < 0) {
		fprintf(stderr, "[hott_telemetry] ERROR getting baudrate / termios config for %s: %d\n", uart_name, termios_state);
		close(uart);
		return ERROR;
	}

	/* Fill the struct for the new configuration */
	tcgetattr(uart, &uart_config);

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* Set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		fprintf(stderr, "[hott_telemetry] ERROR setting baudrate / termios config for %s: %d (cfsetispeed, cfsetospeed)\n", uart_name, termios_state);
		close(uart);
		return ERROR;
	}

	if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0) {
		fprintf(stderr, "[hott_telemetry] ERROR setting baudrate / termios config for %s (tcsetattr)\n", uart_name);
		close(uart);
		return ERROR;
	}

	return uart;
}

int hott_telemetry_thread_main(int argc, char *argv[]) 
{
	printf("[hott_telemetry] starting\n");

	thread_running = true;

	char *device = "/dev/ttyS2";  // UART5
	char *commandline_usage = "\tusage: hott_telemetry start|status|stop [-d device]\n";

	/* read commandline arguments */
	for (int i = 0; i < argc && argv[i]; i++) {
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) { //device set
			if (argc > i + 1) {
				device = argv[i + 1];

			} else {
				thread_running = false;
				errx(1, "missing parameter to -m 1..4\n %s", commandline_usage);
			}
		}
	}

	/* enable UART, writes potentially an empty buffer, but multiplexing is disabled */
	struct termios uart_config_original;
	int uart = open_uart(device, &uart_config_original);

	if (uart < 0) {
		fprintf(stderr, "[hott_telemetry] Failed opening HoTT UART, exiting.\n");
		thread_running = false;
		exit(ERROR);
	}

	messages_init();

	struct eam_module_msg msg;
	while (!thread_should_exit) {
		build_eam_response(&msg);
		if (read_data(uart) == OK) {
			send_data(uart, &msg);
		}
	}

	printf("[hott_telemetry] exiting.\n");

	/* close uarts */
	close(uart);

	thread_running = false;

	return 0;
}

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: deamon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_create().
 */
int hott_telemetry_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("deamon already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn("hott_telemetry",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 20,
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
			printf("\thott_telemetry is running\n");
		} else {
			printf("\thott_telemetry not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int read_data(int uart)
{
        uart_disable_tx();

	const int timeout = 1000;
	struct pollfd fds[] = { { .fd = uart, .events = POLLIN } };

	if (poll(fds, 1, timeout) > 0) {
		// get the mode: binary or text
		char mode;
		read(uart, &mode, 1);
		
		// read the poll ID (device ID being targetted)
		char id;
		read(uart, &id, 1);

		//printf("Reading: mode='%x' id='%x'\n", mode, id);

		// if we have a binary mode request for our sensor ID let's run with it
		if (mode != BINARY_MODE_REQUEST_ID || id != ELECTRIC_AIR_MODULE) {
			return ERROR;	// not really an error, rather uninteresting.
		}
	} else {
		printf("Timeout\n");
		return ERROR;
	}
	return OK;
}

int send_data(int uart, const struct eam_module_msg *msg)
{
	usleep(POST_READ_DELAY_IN_USECS);

	uart_disable_rx();

	uint16_t checksum = 0;
  	int size = sizeof(*msg);
  	char buffer[size];

	memcpy(buffer, msg, size);

	for(int i = 0; i < size; i++) {
		if (i == size - 1) {
			// Set the checksum: the first uint8_t is taken as the checksum.
			buffer[i] = checksum & 0xff;
		} else {
			checksum += buffer[i];
		}

		//printf("%x ", buffer[i]);
		write(uart, &buffer[i], 1);
		
		// Sleep before sending the next uint8_t.
		usleep(POST_WRITE_DELAY_IN_USECS);
	}

	return OK;
}

void uart_disable_rx()
{
	uint32_t cr;
	cr  = getreg32(STM32_UART5_CR1);
	cr &= ~(USART_CR1_RE);	// turn off RX
	cr |= (USART_CR1_TE);	// turn on TX
        putreg32(cr, STM32_UART5_CR1);
}

void uart_disable_tx()
{
	uint32_t cr;
	cr  = getreg32(STM32_UART5_CR1);
	cr |= (USART_CR1_RE);	// turn on RX
	cr &= ~(USART_CR1_TE);	// turn off TX
        putreg32(cr, STM32_UART5_CR1);
}


