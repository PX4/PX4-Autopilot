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

#include <px4_tasks.h>
#include <px4_module.h>
#include <systemlib/err.h>
#include <termios.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/sensor_combined.h>
#include <math.h>	// NAN

#include "sPort_data.h"
#include "frsky_data.h"
#include "common.h"


/* thread state */
static volatile bool thread_should_exit = false;
static volatile bool thread_running = false;
static int frsky_task;
typedef enum { SCANNING, SPORT, DTYPE } frsky_state_t;
static frsky_state_t frsky_state = SCANNING;

/* functions */
static int sPort_open_uart(const char *uart_name, struct termios *uart_config, struct termios *uart_config_original);
static int set_uart_speed(int uart, struct termios *uart_config, speed_t speed);
static void usage(void);
static int frsky_telemetry_thread_main(int argc, char *argv[]);
__EXPORT int frsky_telemetry_main(int argc, char *argv[]);


uint16_t get_telemetry_flight_mode(int px4_flight_mode)
{
	// map the flight modes (see https://github.com/ilihack/LuaPilot_Taranis_Telemetry/blob/master/SCRIPTS/TELEMETRY/LuaPil.lua#L790)
	switch (px4_flight_mode) {
	case 0: return 18; // manual

	case 1: return 23; // alt control

	case 2: return 22; // pos control

	case 3: return 27; // mission

	case 4: return 26; // loiter

	case 5:
	case 6:
	case 7: return 28; // rtl

	case 10: return 19; // acro

	case 14: return 24; // offboard

	case 15: return 20; // stabilized

	case 16: return 21; // rattitude

	case 17: return 25; // takeoff

	case 8:
	case 9:
	case 18: return 29; // land

	case 19: return 30; // follow target
	}

	return -1;
}

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
	const speed_t speed = B9600;

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
	PRINT_MODULE_DESCRIPTION("FrSky Telemetry support. Auto-detects D or S.PORT protocol.");

	PRINT_MODULE_USAGE_NAME("frsky_telemetry", "communication");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS6", "<file:dev>", "Select Serial Device", true);
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	exit(1);
}

/**
 * The daemon thread.
 */
static int frsky_telemetry_thread_main(int argc, char *argv[])
{
	/* Default values for arguments */
	const char *device_name = "/dev/ttyS6"; /* USART8 */

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

	/* Open UART assuming D type telemetry */
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
	frsky_state = SCANNING;
	frsky_state_t baudRate = DTYPE;

	while (!thread_should_exit && frsky_state == SCANNING) {
		/* 2 byte polling frames indicate SmartPort telemetry
		 * 11 byte packets indicate D type telemetry
		 */
		int status = poll(fds, sizeof(fds) / sizeof(fds[0]), 3000);

		if (status > 0) {
			/* traffic on the port, D type is 11 bytes per frame, SmartPort is only 2
			 * Wait long enough for 11 bytes at 9600 baud
			 */
			usleep(12000);
			int nbytes = read(uart, &sbuf[0], sizeof(sbuf));
			PX4_DEBUG("frsky input: %d bytes: %x %x, speed: %d", nbytes, sbuf[0], sbuf[1], baudRate);

			// look for valid header byte
			if (nbytes > 10) {
				if (baudRate == DTYPE) {
					// see if we got a valid D-type hostframe
					struct adc_linkquality host_frame;

					if (frsky_parse_host((uint8_t *)&sbuf[0], nbytes, &host_frame)) {
						frsky_state = baudRate;
						break;
					}

				} else {
					// check for alternating S.port start bytes
					int index = 0;

					while (index < 2 && sbuf[index] != 0x7E) { index++; }

					if (index < 2) {

						int success = 1;

						for (int i = index + 2; i < nbytes; i += 2) {
							if (sbuf[i] != 0x7E) { success = 0; break; }
						}

						if (success) {
							frsky_state = baudRate;
							break;
						}
					}
				}

			}

			// alternate between S.port and D-type baud rates
			if (baudRate == SPORT) {
				PX4_DEBUG("setting baud rate to %d", 9600);
				set_uart_speed(uart, &uart_config, B9600);
				baudRate = DTYPE;

			} else {
				PX4_DEBUG("setting baud rate to %d", 57600);
				set_uart_speed(uart, &uart_config, B57600);
				baudRate = SPORT;

			}

			// wait a second
			usleep(1000000);
			// flush buffer
			read(uart, &sbuf[0], sizeof(sbuf));

		}
	}

	if (frsky_state == SPORT) {
		/* Subscribe to topics */
		if (!sPort_init()) {
			err(1, "could not allocate memory");
		}

		PX4_INFO("sending FrSky SmartPort telemetry");

		struct sensor_combined_s sensor_combined = {};

		float filtered_alt = NAN;
		float last_baro_alt = 0.f;
		int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));

		uint32_t lastBATV_ms = 0;
		uint32_t lastCUR_ms = 0;
		uint32_t lastALT_ms = 0;
		uint32_t lastSPD_ms = 0;
		uint32_t lastFUEL_ms = 0;
		uint32_t lastVSPD_ms = 0;
		uint32_t lastGPS_ms = 0;
		uint32_t lastNAV_STATE_ms = 0;
		uint32_t lastGPS_FIX_ms = 0;

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

			uint32_t now_ms = hrt_absolute_time() / 1000;

			newBytes = read(uart, &sbuf[1], 1);

			/* get a local copy of the current sensor values
			 * in order to apply a lowpass filter to baro pressure.
			 */
			bool sensor_updated;
			orb_check(sensor_sub, &sensor_updated);

			if (sensor_updated) {
				orb_copy(ORB_ID(sensor_combined), sensor_sub, &sensor_combined);

				if (isnan(filtered_alt)) {
					filtered_alt = sensor_combined.baro_alt_meter;

				} else {
					filtered_alt = .05f * sensor_combined.baro_alt_meter + .95f * filtered_alt;
				}
			}

			// allow a minimum of 500usec before reply
			usleep(500);

			sPort_update_topics();


			switch (sbuf[1]) {

			case SMARTPORT_POLL_1:

				/* report BATV at 1Hz */
				if (now_ms - lastBATV_ms > 1000) {
					lastBATV_ms = now_ms;
					/* send battery voltage */
					sPort_send_BATV(uart);
				}

				break;


			case SMARTPORT_POLL_2:

				/* report battery current at 5Hz */
				if (now_ms - lastCUR_ms > 200) {
					lastCUR_ms = now_ms;
					/* send battery current */
					sPort_send_CUR(uart);
				}

				break;


			case SMARTPORT_POLL_3:

				/* report altitude at 5Hz */
				if (now_ms - lastALT_ms > 200) {
					lastALT_ms = now_ms;
					/* send altitude */
					sPort_send_ALT(uart);
				}

				break;


			case SMARTPORT_POLL_4:

				/* report speed at 5Hz */
				if (now_ms - lastSPD_ms > 200) {
					lastSPD_ms = now_ms;
					/* send speed */
					sPort_send_SPD(uart);
				}

				break;

			case SMARTPORT_POLL_5:

				/* report fuel at 1Hz */
				if (now_ms - lastFUEL_ms > 1000) {
					lastFUEL_ms = now_ms;
					/* send fuel */
					sPort_send_FUEL(uart);
				}

				break;

			case SMARTPORT_POLL_6:

				/* report vertical speed at 10Hz */
				if (now_ms - lastVSPD_ms > 100) {
					/* estimate vertical speed using first difference and delta t */
					uint32_t dt = now_ms - lastVSPD_ms;
					float speed  = (filtered_alt - last_baro_alt) / (1e-3f * (float)dt);

					/* save current alt and timestamp */
					last_baro_alt = filtered_alt;
					lastVSPD_ms = now_ms;

					sPort_send_VSPD(uart, speed);
				}

				break;

			case SMARTPORT_POLL_7:

				/* report GPS data elements at 5*5Hz */
				if (now_ms - lastGPS_ms > 100) {
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
						sPort_send_GPS_CRS(uart);
						elementCount++;
						break;

					case 3:
						sPort_send_GPS_ALT(uart);
						elementCount++;
						break;

					case 4:
						sPort_send_GPS_SPD(uart);
						elementCount++;
						break;

					case 5:
						sPort_send_GPS_TIME(uart);
						elementCount = 0;
						break;
					}

				}

			/* FALLTHROUGH */

			case SMARTPORT_POLL_8:

				/* report nav_state as DIY_NAVSTATE 2Hz */
				if (now_ms - lastNAV_STATE_ms > 500) {
					lastNAV_STATE_ms = now_ms;
					/* send T1 */
					sPort_send_NAV_STATE(uart);
				}

				/* report satcount and fix as DIY_GPSFIX at 2Hz */
				else if (now_ms - lastGPS_FIX_ms > 500) {
					lastGPS_FIX_ms = now_ms;
					/* send T2 */
					sPort_send_GPS_FIX(uart);
				}

				break;

			case SMARTPORT_SENSOR_ID_SP2UR: {
					static int elementCount = 0;

					switch (elementCount++ % 2) {
					case 0:
						sPort_send_flight_mode(uart);
						break;

					default:
						sPort_send_GPS_info(uart);
						break;
					}
				}

				break;
			}
		}

		PX4_DEBUG("freeing sPort memory");
		sPort_deinit();

		/* either no traffic on the port (0=>timeout), or D type packet */

	} else if (frsky_state == DTYPE) {
		/* detected D type telemetry: reconfigure UART */
		PX4_INFO("sending FrSky D type telemetry");
		int status = set_uart_speed(uart, &uart_config, B9600);

		if (status < 0) {
			PX4_DEBUG("error setting speed for %s, quitting", device_name);
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

		/* send D8 mode telemetry */
		while (!thread_should_exit) {

			/* Sleep 100 ms */
			usleep(100000);

			/* parse incoming data */
			int nbytes = read(uart, &sbuf[0], sizeof(sbuf));
			bool new_input = frsky_parse_host((uint8_t *)&sbuf[0], nbytes, &host_frame);

			/* the RSSI value could be useful */
			if (new_input) {
				PX4_DEBUG("host frame: ad1:%u, ad2: %u, rssi: %u",
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
		PX4_DEBUG("freeing frsky memory");
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
						SCHED_PRIORITY_DEFAULT + 4,
						1268,
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
			case SCANNING:
				errx(0, "running: SCANNING");
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
