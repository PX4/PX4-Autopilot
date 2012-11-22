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
 * NOTE: Since HoTT telemetry works half-duplex over a single wire the wire
 * is connected to both the UART TX and RX port. In order to send and receive
 * we need to be able to disable one of these ports at a time. This level of
 * control is currently not provided by Nuttx (yet) so we need to do this 
 * at the hardware level for now.
 *
 * TODO: Add support for at least the vario and GPS sensor data.  
 *
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

/* The following are equired for UART direct manipulation. */
#include <arch/board/board.h>
#include "up_arch.h"
#include "chip.h"
#include "stm32_internal.h"

static int thread_should_exit = false;		/**< Deamon exit flag */
static int thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static uint32_t uart_addr;			/**< The regsitry address of the UART for direct access */
static char *daemon_name = "hott_telemetry";
static char *commandline_usage = "usage: hott_telemetry start|status|stop [-d <device>]";

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

static int read_data(int uart, int *id);
static int send_data(int uart, char *buffer, int size);
static void uart_disable_rx(void);
static void uart_disable_tx(void);
static uint32_t get_uart_address(const char *device);

static int open_uart(const char *uart_name, struct termios *uart_config_original)
{
	/* baud rate */
	int speed = B19200;
	int uart;

	/* open uart */
	uart = open(uart_name, O_RDWR | O_NOCTTY);

	if (uart < 0) {
		char msg[80];
		sprintf(msg, "Error opening port: %s\n", uart_name);
		FATAL_MSG(msg);
	}

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(uart, uart_config_original)) < 0) {
		char msg[80];
		sprintf(msg, "Error getting baudrate / termios config for %s: %d\n", uart_name, termios_state);
		close(uart);
		FATAL_MSG(msg);
	}

	/* Fill the struct for the new configuration */
	tcgetattr(uart, &uart_config);

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* Set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		char msg[80];
		sprintf(msg, "Error setting baudrate / termios config for %s: %d (cfsetispeed, cfsetospeed)\n", uart_name, termios_state);
		close(uart);
		FATAL_MSG(msg);
	}

	if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0) {
		char msg[80];
		sprintf(msg, "Error setting baudrate / termios config for %s (tcsetattr)\n", uart_name);
		close(uart);
		FATAL_MSG(msg);
	}

	return uart;
}

int read_data(int uart, int *id)
{
        uart_disable_tx();

	const int timeout = 1000;
	struct pollfd fds[] = { { .fd = uart, .events = POLLIN } };

	if (poll(fds, 1, timeout) > 0) {
		/* get the mode: binary or text  */
		char mode;
		read(uart, &mode, 1);
		
		/* read the poll ID (device ID being targetted) */
		read(uart, id, 1);

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

int send_data(int uart, char *buffer, int size)
{
	usleep(POST_READ_DELAY_IN_USECS);

	uart_disable_rx();

	uint16_t checksum = 0;
	for(int i = 0; i < size; i++) {
		if (i == size - 1) {
			/* Set the checksum: the first uint8_t is taken as the checksum. */
			buffer[i] = checksum & 0xff;
		} else {
			checksum += buffer[i];
		}

		write(uart, &buffer[i], 1);
		
		/* Sleep before sending the next byte. */
		usleep(POST_WRITE_DELAY_IN_USECS);
	}

	return OK;
}

void uart_disable_rx(void)
{
	uint32_t cr;
	cr  = getreg32(uart_addr);
	cr &= ~(USART_CR1_RE);	// turn off RX
	cr |= (USART_CR1_TE);	// turn on TX
        putreg32(cr, uart_addr);
}

void uart_disable_tx(void)
{
	uint32_t cr;
	cr  = getreg32(uart_addr);
	cr |= (USART_CR1_RE);	// turn on RX
	cr &= ~(USART_CR1_TE);	// turn off TX
        putreg32(cr, uart_addr);
}

uint32_t get_uart_address(const char *device)
{
	/* Map the tty device number to the UART address */
	switch(device[strlen(device) - 1]) {
		case '0':  return STM32_USART1_CR1;
		case '1':  return STM32_USART2_CR1;
		case '2':  return STM32_UART5_CR1;
		case '3':  return STM32_USART6_CR1;
		default:   return STM32_UART5_CR1; 
	}	
}

int hott_telemetry_thread_main(int argc, char *argv[]) 
{
	INFO_MSG("starting");

	thread_running = true;

	char *device = "/dev/ttyS2";		/**< Default telemetry port: UART5 */

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
	int uart = open_uart(device, &uart_config_original);

	if (uart < 0) {
		ERROR_MSG("Failed opening HoTT UART, exiting.");
		thread_running = false;
		exit(ERROR);
	}

	/* Since we need to enable/disable both TX and RX on the UART at the device level
	 * we need to know the register address of the UART we are working with. Making it
	 * global so it's easy to remove later when TX/RX control is provided by Nuttx. */
	uart_addr = get_uart_address(device);

	messages_init();

	char *buffer;
	int size = 0;
	int id = 0;
	while (!thread_should_exit) {
		if (read_data(uart, &id) == OK) {
			switch(id) {
				case ELECTRIC_AIR_MODULE:
					build_eam_response(&buffer, &size);
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
		deamon_task = task_spawn("hott_telemetry",
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



