/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
 *   Author: Stefan Rado <px4@sradonia.net>
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
 * @file frsky_telemetry.c
 * @author Stefan Rado <px4@sradonia.net>
 * @author Mark Whitehorn <kd0aij@github.com>
 *
 * FrSky D8 mode and SmartPort (D16 mode) telemetry implementation.
 *
 * This daemon emulates the FrSky Sensor Hub for D8 mode.
 * For X series receivers (D16 mode) it emulates SmartPort sensors by responding to polling
 * packets received from an attached FrSky X series receiver.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <termios.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/sensor_baro.h>

#include "sPort_data.h"
#include "frsky_data.h"


/* thread state */
static volatile bool thread_should_exit = false;
static volatile bool thread_running = false;
static int frsky_task;
typedef enum { IDLE, SPORT, DTYPE } frsky_state_t;
static frsky_state_t frsky_state = IDLE;

/* functions */
static int sPort_open_uart(const char *uart_name, struct termios *uart_config, struct termios *uart_config_original);
static int set_uart_speed(int uart, struct termios *uart_config, speed_t speed);
static void usage(void);
static int frsky_telemetry_thread_main(int argc, char *argv[]);
__EXPORT int frsky_telemetry_main(int argc, char *argv[]);


/**
 * Opens the UART device and sets all required serial parameters.
 */
static int sPort_open_uart(const char *uart_name, struct termios *uart_config, struct termios *uart_config_original)
{
	/* Open UART */
	const int uart = open(uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (uart < 0) {
		err(1, "Error opening port: %s", uart_name);
	}

	/* Back up the original UART configuration to restore it after exit */
	int termios_state;

	if ((termios_state = tcgetattr(uart, uart_config_original)) < 0) {
		warnx("ERR: tcgetattr%s: %d\n", uart_name, termios_state);
		close(uart);
		return -1;
	}

	/* Fill the struct for the new configuration */
	tcgetattr(uart, uart_config);

	/* Disable output post-processing */
	uart_config->c_oflag &= ~OPOST;

	/* Set baud rate */
	static const speed_t speed = B57600;

	if (cfsetispeed(uart_config, speed) < 0 || cfsetospeed(uart_config, speed) < 0) {
		warnx("ERR: %s: %d (cfsetispeed, cfsetospeed)\n", uart_name, termios_state);
		close(uart);
		return -1;
	}

	if ((termios_state = tcsetattr(uart, TCSANOW, uart_config)) < 0) {
		warnx("ERR: %s (tcsetattr)\n", uart_name);
		close(uart);
		return -1;
	}

	return uart;
}

static int set_uart_speed(int uart, struct termios *uart_config, speed_t speed)
{

	if (cfsetispeed(uart_config, speed) < 0) {
		return -1;
	}

	if (tcsetattr(uart, TCSANOW, uart_config) < 0) {
		return -1;
	}

	return uart;
}

/**
 * Print command usage information
 */
static void usage()
{
	fprintf(stderr,
		"usage: frsky_telemetry start [-d <devicename>]\n"
		"       frsky_telemetry stop\n"
		"       frsky_telemetry status\n");
	exit(1);
}

/**
 * The daemon thread.
 */
static int frsky_telemetry_thread_main(int argc, char *argv[])
{
	/* Default values for arguments */
	char *device_name = "/dev/ttyS6"; /* USART8 */

	/* Work around some stupidity in task_create's argv handling */
	argc -= 2;
	argv += 2;

	int ch;

	while ((ch = getopt(argc, argv, "d:")) != EOF) {
		switch (ch) {
		case 'd':
			device_name = optarg;
			break;

		default:
			usage();
			break;
		}
	}

	/* Open UART assuming SmartPort telemetry */
	struct termios uart_config_original;
	struct termios uart_config;
	const int uart = sPort_open_uart(device_name, &uart_config, &uart_config_original);

	if (uart < 0) {
		warnx("could not open %s", device_name);
		err(1, "could not open %s", device_name);
	}

	/* poll descriptor */
	struct pollfd fds[1];
	fds[0].fd = uart;
	fds[0].events = POLLIN;

	thread_running = true;

	/* Main thread loop */
	char sbuf[20];
	frsky_state = IDLE;

	while (!thread_should_exit && frsky_state == IDLE) {
		/* 2 byte polling frames indicate SmartPort telemetry
		 * 11 byte packets indicate D type telemetry
		 */
		int status = poll(fds, sizeof(fds) / sizeof(fds[0]), 3000);

		if (status > 0) {
			/* traffic on the port, D type is 11 bytes per frame, SmartPort is only 2
			 * allow a little time to receive the entire packet
			 */
			usleep(5000);
			status = read(uart, &sbuf[0], sizeof(sbuf));
		}

		/* received some data; check size of packet */
		if (status > 0 && status < 3) {
			frsky_state = SPORT;

		} else if (status > 0) {
			frsky_state = DTYPE;
		}
	}

	if (frsky_state == SPORT) {
		/* Subscribe to topics */
		if (!sPort_init()) {
			err(1, "could not allocate memory");
		}

		warnx("sending FrSky SmartPort telemetry");

		struct sensor_baro_s *sensor_baro = malloc(sizeof(struct sensor_baro_s));

		if (sensor_baro == NULL) {
			err(1, "could not allocate memory");
		}

		static float filtered_alt = NAN;
		int sensor_sub = orb_subscribe(ORB_ID(sensor_baro));

		/* send S.port telemetry */
		while (!thread_should_exit) {

			/* wait for poll frame starting with value 0x7E
			* note that only the bus master is supposed to put a 0x7E on the bus.
			* slaves use byte stuffing to send 0x7E and 0x7D.
			* we expect a poll frame every 12msec
			*/
			int status = poll(fds, sizeof(fds) / sizeof(fds[0]), 50);

			if (status < 1) { continue; }

			// read 1 byte
			int newBytes = read(uart, &sbuf[0], 1);

			if (newBytes < 1 || sbuf[0] != 0x7E) { continue; }

			/* wait for ID byte */
			status = poll(fds, sizeof(fds) / sizeof(fds[0]), 50);

			if (status < 1) { continue; }

			hrt_abstime now = hrt_absolute_time();

			newBytes = read(uart, &sbuf[1], 1);

			/* get a local copy of the current sensor values
			 * in order to apply a lowpass filter to baro pressure.
			 */
			static float last_baro_alt = 0;
			bool sensor_updated;
			orb_check(sensor_sub, &sensor_updated);

			if (sensor_updated) {
				orb_copy(ORB_ID(sensor_baro), sensor_sub, sensor_baro);

				if (isnan(filtered_alt)) {
					filtered_alt = sensor_baro->altitude;

				} else {
					filtered_alt = .05f * sensor_baro->altitude + .95f * filtered_alt;
				}
			}

			// allow a minimum of 500usec before reply
			usleep(500);

			static hrt_abstime lastBATV = 0;
			static hrt_abstime lastCUR = 0;
			static hrt_abstime lastALT = 0;
			static hrt_abstime lastSPD = 0;
			static hrt_abstime lastFUEL = 0;
			static hrt_abstime lastVSPD = 0;
			static hrt_abstime lastGPS = 0;

			switch (sbuf[1]) {

			case SMARTPORT_POLL_1:

				/* report BATV at 1Hz */
				if (now - lastBATV > 1000 * 1000) {
					lastBATV = now;
					/* send battery voltage */
					sPort_send_BATV(uart);
				}

				break;


			case SMARTPORT_POLL_2:

				/* report battery current at 5Hz */
				if (now - lastCUR > 200 * 1000) {
					lastCUR = now;
					/* send battery current */
					sPort_send_CUR(uart);
				}

				break;


			case SMARTPORT_POLL_3:

				/* report altitude at 5Hz */
				if (now - lastALT > 200 * 1000) {
					lastALT = now;
					/* send altitude */
					sPort_send_ALT(uart);
				}

				break;


			case SMARTPORT_POLL_4:

				/* report speed at 5Hz */
				if (now - lastSPD > 200 * 1000) {
					lastSPD = now;
					/* send speed */
					sPort_send_SPD(uart);
				}

				break;

			case SMARTPORT_POLL_5:

				/* report fuel at 1Hz */
				if (now - lastFUEL > 1000 * 1000) {
					lastFUEL = now;
					/* send fuel */
					sPort_send_FUEL(uart);
				}

				break;

			case SMARTPORT_POLL_6:

				/* report vertical speed at 10Hz */
				if (now - lastVSPD > 100 * 1000) {
					/* estimate vertical speed using first difference and delta t */
					uint64_t dt = now - lastVSPD;
					float speed  = (filtered_alt - last_baro_alt) / (1e-6f * (float)dt);

					/* save current alt and timestamp */
					last_baro_alt = filtered_alt;
					lastVSPD = now;

					sPort_send_VSPD(uart, speed);
				}

				break;

			case SMARTPORT_POLL_7:

				/* report GPS data elements at 2*5Hz */
				if (now - lastGPS > 100 * 1000) {
					static int elementCount = 0;

					switch (elementCount) {

					case 0:
						sPort_send_GPS_LON(uart);
						elementCount++;
						break;

					case 1:
						sPort_send_GPS_LAT(uart);
						elementCount++;
						break;

					case 2:
						sPort_send_GPS_COG(uart);
						elementCount++;
						break;

					case 3:
						sPort_send_GPS_ALT(uart);
						elementCount++;
						break;

					case 4:
						sPort_send_GPS_SPD(uart);
						elementCount = 0;
						break;
					}

				}

				break;
			}
		}

		warnx("freeing sPort memory");
		sPort_deinit();
		free(sensor_baro);

		/* either no traffic on the port (0=>timeout), or D type packet */

	} else if (frsky_state == DTYPE) {
		/* detected D type telemetry: reconfigure UART */
		warnx("sending FrSky D type telemetry");
		int status = set_uart_speed(uart, &uart_config, B9600);

		if (status < 0) {
			warnx("error setting speed for %s, quitting", device_name);
			/* Reset the UART flags to original state */
			tcsetattr(uart, TCSANOW, &uart_config_original);
			close(uart);

			thread_running = false;
			return 0;
		}

		int iteration = 0;

		/* Subscribe to topics */
		if (!frsky_init()) {
			err(1, "could not allocate memory");
		}

		struct adc_linkquality host_frame;

//		uint8_t dbuf[45];

		/* send D8 mode telemetry */
		while (!thread_should_exit) {

			/* Sleep 100 ms */
			usleep(100000);

			/* parse incoming data */
			int nbytes = read(uart, &sbuf[0], sizeof(sbuf));
			bool new_input = frsky_parse_host((uint8_t *)&sbuf[0], nbytes, &host_frame);

			/* the RSSI value could be useful */
			if (false && new_input) {
				warnx("host frame: ad1:%u, ad2: %u, rssi: %u",
				      host_frame.ad1, host_frame.ad2, host_frame.linkq);
			}

			frsky_update_topics();

			/* Send frame 1 (every 200ms): acceleration values, altitude (vario), temperatures, current & voltages, RPM */
			if (iteration % 2 == 0) {
				frsky_send_frame1(uart);
			}

			/* Send frame 2 (every 1000ms): course, latitude, longitude, speed, altitude (GPS), fuel level */
			if (iteration % 10 == 0) {
				frsky_send_frame2(uart);
			}

			/* Send frame 3 (every 5000ms): date, time */
			if (iteration % 50 == 0) {
				frsky_send_frame3(uart);

				iteration = 0;
			}

			iteration++;
		}

//		/* TODO: flush the input buffer if in full duplex mode */
//		read(uart, &sbuf[0], sizeof(sbuf));
		warnx("freeing frsky memory");
		frsky_deinit();

	}

	/* Reset the UART flags to original state */
	tcsetattr(uart, TCSANOW, &uart_config_original);
	close(uart);

	thread_running = false;
	return 0;
}

/**
 * The main command function.
 * Processes command line arguments and starts the daemon.
 */
int frsky_telemetry_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("missing command");
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		/* this is not an error */
		if (thread_running) {
			errx(0, "frsky_telemetry already running");
		}

		thread_should_exit = false;
		frsky_task = px4_task_spawn_cmd("frsky_telemetry",
						SCHED_DEFAULT,
						200,
						1100,
						frsky_telemetry_thread_main,
						(char *const *)argv);

		while (!thread_running) {
			usleep(200);
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {

		/* this is not an error */
		if (!thread_running) {
			errx(0, "frsky_telemetry already stopped");
		}

		thread_should_exit = true;

		while (thread_running) {
			usleep(1000000);
			warnx(".");
		}

		warnx("terminated.");
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			switch (frsky_state) {
			case IDLE:
				errx(0, "running: IDLE");
				break;

			case SPORT:
				errx(0, "running: SPORT");
				break;

			case DTYPE:
				errx(0, "running: DTYPE");
				break;

			}

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	usage();
	/* not getting here */
	return 0;
}
