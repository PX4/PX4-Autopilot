/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
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

/* @file gps.c
 * GPS app main loop.
 */

#include "gps.h"
#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include "nmealib/nmea/nmea.h" // the nmea library
#include "nmea_helper.h" //header files for interacting with the nmea library
#include "mtk.h" //header files for the custom protocol for the mediatek diydrones chip
#include "ubx.h" //header files for the ubx protocol
#include <termios.h>
#include <signal.h>
#include <pthread.h>
#include <sys/prctl.h>
#include <errno.h>
#include <signal.h>
#include <v1.0/common/mavlink.h>
#include <mavlink/mavlink_log.h>

#include <systemlib/systemlib.h>

static bool thread_should_exit;				/**< Deamon status flag */
static bool thread_running = false;				/**< Deamon status flag */
static int deamon_task;							/**< Handle of deamon task / thread */

/**
 * GPS module readout and publishing.
 * 
 * This function reads the onboard gps and publishes the vehicle_gps_positon topic.
 *
 * @see vehicle_gps_position_s
 * @ingroup apps
 */
__EXPORT int gps_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int gps_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/****************************************************************************
 * Definitions
 ****************************************************************************/
#define IMPORTANT_GPS_BAUD_RATES_N		2
#define RETRY_INTERVAL_SECONDS			10

//gps_bin_ubx_state_t * ubx_state;
bool gps_mode_try_all;
bool gps_baud_try_all;
bool gps_mode_success;
bool terminate_gps_thread;
bool gps_verbose;
int current_gps_speed;

enum GPS_MODES {
	GPS_MODE_START = 0,
	GPS_MODE_UBX = 1,
	GPS_MODE_MTK = 2,
	GPS_MODE_NMEA = 3,
	GPS_MODE_END = 4
};


#define AUTO_DETECTION_COUNT 8
const int autodetection_baudrates[] = {B9600, B38400, B38400, B9600, B9600, B38400, B9600, B38400};
const enum GPS_MODES autodetection_gpsmodes[] = {GPS_MODE_UBX, GPS_MODE_MTK, GPS_MODE_UBX, GPS_MODE_MTK, GPS_MODE_UBX, GPS_MODE_MTK, GPS_MODE_NMEA, GPS_MODE_NMEA}; //nmea is the fall-back if nothing else works, therefore we try the standard modes again before finally trying nmea

/****************************************************************************
 * Private functions
 ****************************************************************************/
int open_port(char *port);

void close_port(int *fd);

void setup_port(char *device, int speed, int *fd);


/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_create().
 */
int gps_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("gps already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn("gps",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT,
					 4096,
					 gps_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		thread_running = true;
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tgps is running\n");
		} else {	
			printf("\tgps not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

/*
 * Main function of gps app.
 */
int gps_thread_main(int argc, char *argv[]) {

	/* welcome message */
	printf("[gps] Initialized. Searching for GPS receiver..\n");

	/* default values */
	const char *commandline_usage = "\tusage: %s {start|stop|status} -d devicename -b baudrate -m mode\n\tmodes are:\n\t\tubx\n\t\tmtkcustom\n\t\tnmea\n\t\tall\n";
	char *device = "/dev/ttyS3";
	char mode[10];
	strcpy(mode, "all");
	int baudrate = -1;
	gps_mode_try_all = false;
	gps_baud_try_all = false;
	gps_mode_success = true;
	terminate_gps_thread = false;
	bool retry = false;
	gps_verbose = false;

	int mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	/* read arguments */
	int i;

	for (i = 0; i < argc; i++) {
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) { //device set
			printf(commandline_usage, argv[0]);
			thread_running = false;
			return 0;
		}

		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) { //device set
			if (argc > i + 1) {
				device = argv[i + 1];

			} else {
				printf(commandline_usage, argv[0]);
				thread_running = false;
				return 0;
			}
		}

		if (strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--retry") == 0) {
			if (argc > i + 1) {
				retry = atoi(argv[i + 1]);

			} else {
				printf(commandline_usage, argv[0]);
				thread_running = false;
				return 0;
			}
		}

		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf(commandline_usage, argv[0]);
				thread_running = false;
				return 0;
			}
		}

		if (strcmp(argv[i], "-m") == 0 || strcmp(argv[i], "--mode") == 0) {
			if (argc > i + 1) {
				strcpy(mode, argv[i + 1]);

			} else {
				printf(commandline_usage, argv[0]);
				thread_running = false;
				return 0;
			}
		}

		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			gps_verbose = true;
		}
	}

	/*
	 * In case a baud rate is set only this baud rate will be tried,
	 * otherwise a array of usual baud rates for gps receivers is used
	 */


//	printf("baudrate = %d\n",baudrate);
	switch (baudrate) {
	case -1:     gps_baud_try_all = true; break;

	case 0:      current_gps_speed = B0;      break;
	case 50:     current_gps_speed = B50;     break;
	case 75:     current_gps_speed = B75;     break;
	case 110:    current_gps_speed = B110;    break;
	case 134:    current_gps_speed = B134;    break;
	case 150:    current_gps_speed = B150;    break;
	case 200:    current_gps_speed = B200;    break;
	case 300:    current_gps_speed = B300;    break;
	case 600:    current_gps_speed = B600;    break;
	case 1200:   current_gps_speed = B1200;   break;
	case 1800:   current_gps_speed = B1800;   break;
	case 2400:   current_gps_speed = B2400;   break;
	case 4800:   current_gps_speed = B4800;   break;
	case 9600:   current_gps_speed = B9600;   break;
	case 19200:  current_gps_speed = B19200;  break;
	case 38400:  current_gps_speed = B38400;  break;
	case 57600:  current_gps_speed = B57600;  break;
	case 115200: current_gps_speed = B115200; break;
	case 230400: current_gps_speed = B230400; break;
	case 460800: current_gps_speed = B460800; break;
	case 921600: current_gps_speed = B921600; break;

	default:
		fprintf(stderr, "[gps] ERROR: Unsupported baudrate: %d\n", baudrate);
		fflush(stdout);
		return -EINVAL;
	}


	enum GPS_MODES current_gps_mode = GPS_MODE_UBX;

	if (strcmp(mode, "ubx") == 0) {
		current_gps_mode = GPS_MODE_UBX;

	} else if (strcmp(mode, "mtkcustom") == 0) {
		current_gps_mode = GPS_MODE_MTK;

	} else if (strcmp(mode, "nmea") == 0) {
		current_gps_mode = GPS_MODE_NMEA;

	} else if (strcmp(mode, "all") == 0) {
		gps_mode_try_all = true;

	} else {
		fprintf(stderr, "\t[gps] Invalid mode argument\n");
		printf(commandline_usage);
		thread_running = false;
		return ERROR;
	}

	/* Declare file descriptor for gps here, open port later in setup_port */
	int fd;

	while (!thread_should_exit) {
		/* Infinite retries or break if retry == false */

		/* Loop over all configurations of baud rate and protocol */
		for (i = 0; i < AUTO_DETECTION_COUNT; i++) {
			if (gps_mode_try_all) {
				current_gps_mode = autodetection_gpsmodes[i];

				if (false == gps_baud_try_all && autodetection_baudrates[i] != current_gps_speed) //there is no need to try modes which are not configured to run with the selcted baud rate
					continue;
			}

			if (gps_baud_try_all) {
				current_gps_speed = autodetection_baudrates[i];

				if (false == gps_mode_try_all && autodetection_gpsmodes[i] != current_gps_mode) //there is no need to try baud rates which are not usual for the selected mode
					continue;
			}


			/*
			 * The watchdog_thread will return and set gps_mode_success to false if no data can be parsed.
			 * if the gps was once running the wtachdog thread will not return but instead try to reconfigure the gps (depending on the mode/protocol)
			 */

			if (current_gps_mode == GPS_MODE_UBX) {

				if (gps_verbose) printf("[gps] Trying UBX mode at %d baud\n", current_gps_speed);

				mavlink_log_info(mavlink_fd, "[gps] trying to connect to a ubx module");
                
				setup_port(device, current_gps_speed, &fd);
                
				/* start ubx thread and watchdog */
				pthread_t ubx_thread;
				pthread_t ubx_watchdog_thread;

				pthread_mutex_t  ubx_mutex_d;
				ubx_mutex = &ubx_mutex_d;
				pthread_mutex_init(ubx_mutex, NULL);
				gps_bin_ubx_state_t ubx_state_d;
				ubx_state = &ubx_state_d;
				ubx_decode_init();

				pthread_attr_t ubx_loop_attr;
				pthread_attr_init(&ubx_loop_attr);
				pthread_attr_setstacksize(&ubx_loop_attr, 3000);

				struct arg_struct args;
				args.fd_ptr = &fd;
				args.thread_should_exit_ptr = &thread_should_exit;

				pthread_create(&ubx_thread, &ubx_loop_attr, ubx_loop, (void *)&args);
				sleep(2); // XXX TODO Check if this is too short, try to lower sleeps in UBX driver
                
				pthread_attr_t ubx_wd_attr;
				pthread_attr_init(&ubx_wd_attr);
				pthread_attr_setstacksize(&ubx_wd_attr, 1400);
				int pthread_create_res = pthread_create(&ubx_watchdog_thread, &ubx_wd_attr, ubx_watchdog_loop, (void *)&args);

				if (pthread_create_res != 0) fprintf(stderr, "[gps] ERROR: could not create ubx watchdog thread, pthread_create =%d\n", pthread_create_res);

				/* wait for threads to complete */
				pthread_join(ubx_watchdog_thread, NULL);

				if (gps_mode_success == false) {
					if (gps_verbose) printf("[gps] no success with UBX mode and %d baud\n", current_gps_speed);

					terminate_gps_thread = true;
					pthread_join(ubx_thread, NULL);

					gps_mode_success = true;
					terminate_gps_thread = false;

				/* maybe the watchdog was stopped through the thread_should_exit flag */
				} else if (thread_should_exit) {
					pthread_join(ubx_thread, NULL);
					if (gps_verbose) printf("[gps] ubx watchdog and ubx loop threads have been terminated, exiting.");
					close(mavlink_fd);
					close_port(&fd);
					thread_running = false;
					return 0;
				}

				close_port(&fd);
			} else if (current_gps_mode == GPS_MODE_MTK) {
				if (gps_verbose) printf("[gps] trying MTK binary mode at %d baud\n", current_gps_speed);

				mavlink_log_info(mavlink_fd, "[gps] trying to connect to a MTK module");

				setup_port(device, current_gps_speed, &fd);

				/* start mtk thread and watchdog */
				pthread_t mtk_thread;
				pthread_t mtk_watchdog_thread;

				pthread_mutex_t  mtk_mutex_d;
				mtk_mutex = &mtk_mutex_d;
				pthread_mutex_init(mtk_mutex, NULL);


				gps_bin_mtk_state_t mtk_state_d;
				mtk_state = &mtk_state_d;
				mtk_decode_init();


				pthread_attr_t mtk_loop_attr;
				pthread_attr_init(&mtk_loop_attr);
				pthread_attr_setstacksize(&mtk_loop_attr, 2048);

				struct arg_struct args;
				args.fd_ptr = &fd;
				args.thread_should_exit_ptr = &thread_should_exit;

				pthread_create(&mtk_thread, &mtk_loop_attr, mtk_loop, (void *)&args);
				sleep(2);
				pthread_create(&mtk_watchdog_thread, NULL, mtk_watchdog_loop, (void *)&args);

				/* wait for threads to complete */
				pthread_join(mtk_watchdog_thread, (void *)&fd);

				if (gps_mode_success == false) {
					if (gps_verbose) printf("[gps] No success with MTK binary mode and %d baud\n", current_gps_speed);

					terminate_gps_thread = true;
					pthread_join(mtk_thread, NULL);

					//if(true == gps_mode_try_all)
					//strcpy(mode, "nmea");

					gps_mode_success = true;
					terminate_gps_thread = false;
				}

				close_port(&fd);

			} else if (current_gps_mode == GPS_MODE_NMEA) {
				if (gps_verbose) printf("[gps] Trying NMEA mode at %d baud\n", current_gps_speed);

				mavlink_log_info(mavlink_fd, "[gps] trying to connect to a NMEA module");


				setup_port(device, current_gps_speed, &fd);

				/* start nmea thread and watchdog */
				pthread_t nmea_thread;
				pthread_t nmea_watchdog_thread;

				pthread_mutex_t  nmea_mutex_d;
				nmea_mutex = &nmea_mutex_d;
				pthread_mutex_init(nmea_mutex, NULL);

				gps_bin_nmea_state_t nmea_state_d;
				nmea_state = &nmea_state_d;

				pthread_attr_t nmea_loop_attr;
				pthread_attr_init(&nmea_loop_attr);
				pthread_attr_setstacksize(&nmea_loop_attr, 4096);

				struct arg_struct args;
				args.fd_ptr = &fd;
				args.thread_should_exit_ptr = &thread_should_exit;

				pthread_create(&nmea_thread, &nmea_loop_attr, nmea_loop, (void *)&args);
				sleep(2);
				pthread_create(&nmea_watchdog_thread, NULL, nmea_watchdog_loop, (void *)&args);

				/* wait for threads to complete */
				pthread_join(nmea_watchdog_thread, (void *)&fd);

				if (gps_mode_success == false) {
					if (gps_verbose) printf("[gps] No success with NMEA mode and %d baud\r\n", current_gps_speed);

					terminate_gps_thread = true;
					pthread_join(nmea_thread, NULL);

					gps_mode_success = true;
					terminate_gps_thread = false;
				}

				close_port(&fd);
			}

			/* exit quickly if stop command has been received */
			if (thread_should_exit) {
				printf("[gps] stopped, exiting.\n");
				close(mavlink_fd);
				thread_running = false;
				return 0;
	        }

			/* if both, mode and baud is set by argument, we only need one loop*/
			if (gps_mode_try_all == false && gps_baud_try_all == false)
				break;
		}

	        
		if (retry) {
			printf("[gps] No configuration was successful, retrying in %d seconds \n", RETRY_INTERVAL_SECONDS);
			mavlink_log_info(mavlink_fd, "[gps] No configuration was successful, retrying...");
			fflush(stdout);

		} else {
			fprintf(stderr, "[gps] No configuration was successful, exiting... \n");
			fflush(stdout);
			mavlink_log_info(mavlink_fd, "[gps] No configuration was successful, exiting...");
			break;
		}

		sleep(RETRY_INTERVAL_SECONDS);
	}

	printf("[gps] exiting.\n");
	close(mavlink_fd);
	thread_running = false;
	return 0;
}


static void usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "\tusage: gps {start|status|stop} -d devicename -b baudrate -m mode\n\tmodes are:\n\t\tubx\n\t\tmtkcustom\n\t\tnmea\n\t\tall\n");
	exit(1);
}

int open_port(char *port)
{
	int fd; /**< File descriptor for the gps port */

	/* Open serial port */
	fd = open(port, O_CREAT | O_RDWR | O_NOCTTY); /* O_RDWR - Read and write O_NOCTTY - Ignore special chars like CTRL-C */
	return (fd);
}


void close_port(int *fd)
{
	/* Close serial port */
	close(*fd);
}

void setup_port(char *device, int speed, int *fd)
{
	/* open port (baud rate is set in defconfig file) */
	*fd = open_port(device);

	if (*fd != -1) {
		if (gps_verbose) printf("[gps] Port opened: %s at %d baud\n", device, speed);

	} else {
		fprintf(stderr, "[gps] Could not open port, exiting gps app!\n");
		fflush(stdout);
	}

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	if ((termios_state = tcgetattr(*fd, &uart_config)) < 0) {
		fprintf(stderr, "[gps] ERROR getting baudrate / termios config for %s: %d\n", device, termios_state);
		close(*fd);
	}
	if (gps_verbose) printf("[gps] Try to set baud rate %d now\n",speed);
	/* Set baud rate */
	cfsetispeed(&uart_config, speed);
	cfsetospeed(&uart_config, speed);
	if ((termios_state = tcsetattr(*fd, TCSANOW, &uart_config)) < 0) {
		fprintf(stderr, "[gps] ERROR setting baudrate / termios config for %s (tcsetattr)\n", device);
		close(*fd);
	}
}
