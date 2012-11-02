/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Julian Oes <joes@student.ethz.ch>
 *           Thomas Gubler <thomasgubler@student.ethz.ch>
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

/* @file NMEA protocol implementation */
#include "gps.h"
#include "nmea_helper.h"
#include <sys/prctl.h>
#include <unistd.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <mavlink/mavlink_log.h>
#include <drivers/drv_hrt.h>

#define NMEA_HEALTH_SUCCESS_COUNTER_LIMIT 2
#define NMEA_HEALTH_FAIL_COUNTER_LIMIT 2

#define NMEA_BUFFER_SIZE 1000

pthread_mutex_t *nmea_mutex;
gps_bin_nmea_state_t *nmea_state;
static struct vehicle_gps_position_s *nmea_gps;

extern bool gps_mode_try_all;
extern bool gps_mode_success;
extern bool terminate_gps_thread;
extern bool gps_baud_try_all;
extern bool gps_verbose;
extern int current_gps_speed;


int read_gps_nmea(int *fd, char *gps_rx_buffer, int buffer_size, nmeaINFO *info, nmeaPARSER *parser)
{
	int ret = 1;
	char c;
	int start_flag = 0;
	int found_cr = 0;
	int rx_count = 0;
	int gpsRxOverflow = 0;

	struct pollfd fds;
	fds.fd = *fd;
	fds.events = POLLIN;

	// NMEA or SINGLE-SENTENCE GPS mode


	while (1) {
		//check if the thread should terminate
		if (terminate_gps_thread == true) {
//			printf("terminate_gps_thread=%u ", terminate_gps_thread);
//			printf("exiting mtk thread\n");
//			fflush(stdout);
			ret = 2;
			break;
		}

		if (poll(&fds, 1, 1000) > 0) {
			if (read(*fd, &c, 1) > 0) {
				// detect start while acquiring stream
				//		printf("Char = %c\n", c);
				if (!start_flag && (c == '$')) {
					start_flag = 1;
					found_cr = 0;
					rx_count = 0;

				} else if (!start_flag) { // keep looking for start sign
					continue;
				}

				if (rx_count >= buffer_size) {
					// The buffer is already full and we haven't found a valid NMEA sentence.
					// Flush the buffer and note the overflow event.
					gpsRxOverflow++;
					start_flag = 0;
					found_cr = 0;
					rx_count = 0;

					if (gps_verbose) printf("\t[gps] Buffer full\n");

				} else {
					// store chars in buffer
					gps_rx_buffer[rx_count] = c;
					rx_count++;
				}

				// look for carriage return CR
				if (start_flag && c == 0x0d) {
					found_cr = 1;
				}

				// and then look for line feed LF
				if (start_flag && found_cr && c == 0x0a) {
					// parse one NMEA line, use buffer up to rx_count
					if (nmea_parse(parser, gps_rx_buffer, rx_count, info) > 0) {
						ret = 0;
					}

					break;
				}

			} else {
				break;
			}

		} else {
			break;
		}
	}



	// As soon as one NMEA message has been parsed, we break out of the loop and end here
	return(ret);
}


/**
 * \brief Convert NDEG (NMEA degree) to fractional degree
 */
float ndeg2degree(float val)
{
	float deg = ((int)(val / 100));
	val = deg + (val - deg * 100) / 60;
	return val;
}

void *nmea_loop(void *args)
{
	/* Set thread name */
	prctl(PR_SET_NAME, "gps nmea read", getpid());

	/* Retrieve file descriptor and thread flag */
	struct arg_struct *arguments = (struct arg_struct *)args;
	int *fd = arguments->fd_ptr;
	bool *thread_should_exit = arguments->thread_should_exit_ptr;

	/* Initialize gps stuff */
	nmeaINFO info_d;
	nmeaINFO *info = &info_d;
	char gps_rx_buffer[NMEA_BUFFER_SIZE];

	/* gps parser (nmea) */
	nmeaPARSER parser;
	nmea_parser_init(&parser);
	nmea_zero_INFO(info);

	/* advertise GPS topic */
	struct vehicle_gps_position_s nmea_gps_d = {.counter=0};
	nmea_gps = &nmea_gps_d;
	orb_advert_t gps_handle = orb_advertise(ORB_ID(vehicle_gps_position), nmea_gps);

	while (!(*thread_should_exit)) {
		/* Parse a message from the gps receiver */
		uint8_t read_res = read_gps_nmea(fd, gps_rx_buffer, NMEA_BUFFER_SIZE, info, &parser);

		if (0 == read_res) {

			/* convert data, ready it for publishing */

			/* convert nmea utc time to usec */
			struct tm timeinfo;
			timeinfo.tm_year = info->utc.year;
			timeinfo.tm_mon = info->utc.mon;
			timeinfo.tm_mday = info->utc.day;
			timeinfo.tm_hour = info->utc.hour;
			timeinfo.tm_min = info->utc.min;
			timeinfo.tm_sec = info->utc.sec;

			time_t epoch = mktime(&timeinfo);

			//			printf("%d.%d.%d %d:%d:%d:%d\n", timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, info->utc.hsec);

			nmea_gps->timestamp = hrt_absolute_time();
			nmea_gps->time_gps_usec = (uint64_t)((epoch)*1000000 + (info->utc.hsec)*10000);
			nmea_gps->fix_type = (uint8_t)info->fix;
			nmea_gps->lat = (int32_t)(ndeg2degree(info->lat) * 1e7f);
			nmea_gps->lon = (int32_t)(ndeg2degree(info->lon) * 1e7f);
			nmea_gps->alt = (int32_t)(info->elv * 1000.0f);
			nmea_gps->eph = (uint16_t)(info->HDOP * 100); //TODO:test scaling
			nmea_gps->epv = (uint16_t)(info->VDOP * 100); //TODO:test scaling
			nmea_gps->vel = (uint16_t)(info->speed * 1000 / 36); //*1000/3600*100
			nmea_gps->cog = 65535;
			nmea_gps->satellites_visible = (uint8_t)info->satinfo.inview;

			int i = 0;

			/* Write info about individual satellites */
			for (i = 0; i < 12; i++) {
				nmea_gps->satellite_prn[i] = (uint8_t)info->satinfo.sat[i].id;
				nmea_gps->satellite_used[i] = (uint8_t)info->satinfo.sat[i].in_use;
				nmea_gps->satellite_elevation[i] = (uint8_t)info->satinfo.sat[i].elv;
				nmea_gps->satellite_azimuth[i] = (uint8_t)info->satinfo.sat[i].azimuth;
				nmea_gps->satellite_snr[i] = (uint8_t)info->satinfo.sat[i].sig;
			}

			if (nmea_gps->satellites_visible > 0) {
				nmea_gps->satellite_info_available = 1;

			} else {
				nmea_gps->satellite_info_available = 0;
			}

			nmea_gps->counter_pos_valid++;

			nmea_gps->timestamp = hrt_absolute_time();
			nmea_gps->counter++;

			pthread_mutex_lock(nmea_mutex);
			nmea_state->last_message_timestamp = hrt_absolute_time();
			pthread_mutex_unlock(nmea_mutex);

			/* publish new GPS position */
			orb_publish(ORB_ID(vehicle_gps_position), gps_handle, nmea_gps);

		} else if (read_res == 2) { //termination
			/* de-advertise */
			close(gps_handle);
			break;
		}

	}

	//destroy gps parser
	nmea_parser_destroy(&parser);
	if(gps_verbose) printf("[gps] nmea loop is going to terminate\n");
	return NULL;

}

void *nmea_watchdog_loop(void *args)
{
	/* Set thread name */
	prctl(PR_SET_NAME, "gps nmea watchdog", getpid());

	bool nmea_healthy = false;

	uint8_t nmea_fail_count = 0;
	uint8_t nmea_success_count = 0;
	bool once_ok = false;

	/* Retrieve file descriptor and thread flag */
	struct arg_struct *arguments = (struct arg_struct *)args;
	//int *fd = arguments->fd_ptr;
	bool *thread_should_exit = arguments->thread_should_exit_ptr;

	int mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	while (!(*thread_should_exit)) {
//		printf("nmea_watchdog_loop : while ");
		/* if we have no update for a long time print warning (in nmea mode there is no reconfigure) */
		pthread_mutex_lock(nmea_mutex);
		uint64_t timestamp_now = hrt_absolute_time();
		bool all_okay = true;

		if (timestamp_now - nmea_state->last_message_timestamp > NMEA_WATCHDOG_CRITICAL_TIME_MICROSECONDS) {
			all_okay = false;
		}

		pthread_mutex_unlock(nmea_mutex);

		if (!all_okay) {
			/* gps error */
			nmea_fail_count++;
//			printf("nmea error, nmea_fail_count=%u\n", nmea_fail_count);
//			fflush(stdout);

			/* If we have too many failures and another mode or baud should be tried, exit... */
			if ((gps_mode_try_all == true  || gps_baud_try_all == true) && (nmea_fail_count >= NMEA_HEALTH_FAIL_COUNTER_LIMIT) && (nmea_healthy == false) && once_ok == false) {
				if (gps_verbose) printf("\t[gps] no NMEA module found\n");

				gps_mode_success = false;
				break;
			}

			if (nmea_healthy && nmea_fail_count >= NMEA_HEALTH_FAIL_COUNTER_LIMIT) {
				printf("\t[gps] ERROR: NMEA GPS module stopped responding\n");
				// global_data_send_subsystem_info(&nmea_present_enabled);
				mavlink_log_critical(mavlink_fd, "[gps] NMEA module stopped responding\n");
				nmea_healthy = false;
				nmea_success_count = 0;

			}



			fflush(stdout);
			sleep(1);

		} else {
			/* gps healthy */
//			printf("\t[gps] nmea success\n");
			nmea_success_count++;

			if (!nmea_healthy && nmea_success_count >= NMEA_HEALTH_SUCCESS_COUNTER_LIMIT) {
				printf("[gps] NMEA module found, status ok (baud=%d)\r\n", current_gps_speed);
				// global_data_send_subsystem_info(&nmea_present_enabled_healthy);
				mavlink_log_info(mavlink_fd, "[gps] NMEA module found, status ok\n");
				nmea_healthy = true;
				nmea_fail_count = 0;
				once_ok = true;
			}

		}

		usleep(NMEA_WATCHDOG_WAIT_TIME_MICROSECONDS);
	}
	if(gps_verbose) printf("[gps] nmea watchdog loop is going to terminate\n");
	close(mavlink_fd);
	return NULL;
}
