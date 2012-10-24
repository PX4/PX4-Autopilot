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

/* @file MTK custom binary (3DR) protocol implementation */

#include "gps.h"
#include "mtk.h"
#include <nuttx/config.h>
#include <unistd.h>
#include <sys/prctl.h>
#include <pthread.h>
#include <poll.h>
#include <fcntl.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <mavlink/mavlink_log.h>

#define MTK_HEALTH_SUCCESS_COUNTER_LIMIT 2
#define MTK_HEALTH_FAIL_COUNTER_LIMIT 2

// XXX decrease this substantially, it should be only a few dozen bytes max.
#warning XXX trying 128 for now
#define MTK_BUFFER_SIZE 128

pthread_mutex_t *mtk_mutex;
gps_bin_mtk_state_t *mtk_state;
static struct vehicle_gps_position_s *mtk_gps;

extern bool gps_mode_try_all;
extern bool gps_mode_success;
extern bool terminate_gps_thread;
extern bool gps_baud_try_all;
extern bool gps_verbose;
extern int current_gps_speed;


void mtk_decode_init(void)
{
	mtk_state->ck_a = 0;
	mtk_state->ck_b = 0;
	mtk_state->rx_count = 0;
	mtk_state->decode_state = MTK_DECODE_UNINIT;
	mtk_state->print_errors = false;
}

void mtk_checksum(uint8_t b, uint8_t *ck_a, uint8_t *ck_b)
{
	*(ck_a) = *(ck_a) + b;
	*(ck_b) = *(ck_b) + *(ck_a);
//	printf("Checksum now: %d\n",*(ck_b));
}



int mtk_parse(uint8_t b,  char *gps_rx_buffer)
{
//	printf("b=%x\n",b);
	// Debug output to telemetry port
	//	PIOS_COM_SendBufferNonBlocking(PIOS_COM_TELEM_RF, &b, 1);
	if (mtk_state->decode_state == MTK_DECODE_UNINIT) {

		if (b == 0xd0) {
			mtk_state->decode_state = MTK_DECODE_GOT_CK_A;
		}

	} else if (mtk_state->decode_state == MTK_DECODE_GOT_CK_A) {
		if (b == 0xdd) {
			mtk_state->decode_state = MTK_DECODE_GOT_CK_B;

		} else {
			// Second start symbol was wrong, reset state machine
			mtk_decode_init();
		}

	} else if (mtk_state->decode_state == MTK_DECODE_GOT_CK_B) {
		// Add to checksum
		if (mtk_state->rx_count < 33) mtk_checksum(b, &(mtk_state->ck_a), &(mtk_state->ck_b));

		// Fill packet buffer
		gps_rx_buffer[mtk_state->rx_count] = b;
		(mtk_state->rx_count)++;
//			printf("Rx count: %d\n",mtk_state->rx_count);
		uint8_t ret = 0;

		/* Packet size minus checksum */
		if (mtk_state->rx_count >= 35) {
			gps_bin_mtk_packet_t *packet = (gps_bin_mtk_packet_t *) gps_rx_buffer;

			/* Check if checksum is valid */
			if (mtk_state->ck_a == packet->ck_a && mtk_state->ck_b == packet->ck_b) {
				mtk_gps->lat = packet->latitude * 10; // mtk: degrees*1e6, mavlink/ubx: degrees*1e7
				mtk_gps->lon = packet->longitude * 10; // mtk: degrees*1e6, mavlink/ubx: degrees*1e7
				mtk_gps->alt = (int32_t)(packet->msl_altitude * 10); // conversion from centimeters to millimeters, and from uint32_t to int16_t
				mtk_gps->fix_type = packet->fix_type;
				mtk_gps->eph = packet->hdop;
				mtk_gps->epv = 65535; //unknown in mtk custom mode
				mtk_gps->vel = packet->ground_speed;
				mtk_gps->cog = 65535; //unknown in mtk custom mode
				mtk_gps->satellites_visible = packet->satellites;

				/* convert time and date information to unix timestamp */
				struct tm timeinfo; //TODO: test this conversion
				uint32_t timeinfo_conversion_temp;

				timeinfo.tm_mday = packet->date * 1e-4;
				timeinfo_conversion_temp = packet->date - timeinfo.tm_mday * 1e4;
				timeinfo.tm_mon = timeinfo_conversion_temp * 1e-2 - 1;
				timeinfo.tm_year = (timeinfo_conversion_temp - (timeinfo.tm_mon + 1) * 1e2) + 100;

				timeinfo.tm_hour = packet->utc_time * 1e-7;
				timeinfo_conversion_temp = packet->utc_time - timeinfo.tm_hour * 1e7;
				timeinfo.tm_min = timeinfo_conversion_temp * 1e-5;
				timeinfo_conversion_temp -= timeinfo.tm_min * 1e5;
				timeinfo.tm_sec = timeinfo_conversion_temp * 1e-3;
				timeinfo_conversion_temp -= timeinfo.tm_sec * 1e3;
				time_t epoch = mktime(&timeinfo);
				mtk_gps->timestamp = hrt_absolute_time();
				mtk_gps->time_gps_usec = epoch * 1e6; //TODO: test this
				mtk_gps->time_gps_usec += timeinfo_conversion_temp * 1e3;

				mtk_gps->counter_pos_valid++;

				mtk_gps->timestamp = hrt_absolute_time();

//					printf("%lu; %lu; %d.%d.%d %d:%d:%d:%d\n", packet->date, packet->utc_time,timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, timeinfo_conversion_temp);

				pthread_mutex_lock(mtk_mutex);
//					printf("Write timestamp /n");
				mtk_state->last_message_timestamp = hrt_absolute_time();
				pthread_mutex_unlock(mtk_mutex);

				ret = 1;
//					printf("found package\n");

			} else {
				if (gps_verbose) printf("[gps] Checksum invalid\r\n");

				ret = 0;
			}

			// Reset state machine to decode next packet
			mtk_decode_init();
//				printf("prepared for next state\n");
			return ret;
		}
	}

	return 0;     // no valid packet found

}

int read_gps_mtk(int *fd, char *gps_rx_buffer, int buffer_size)  // returns 1 if the thread should terminate
{
//	printf("in read_gps_mtk\n");
	uint8_t ret = 0;

	uint8_t c;

	int rx_count = 0;
	int gpsRxOverflow = 0;

	struct pollfd fds;
	fds.fd = *fd;
	fds.events = POLLIN;

	// This blocks the task until there is something on the buffer
	while (1) {
		//check if the thread should terminate
		if (terminate_gps_thread == true) {
//			printf("terminate_gps_thread=%u ", terminate_gps_thread);
//			printf("exiting mtk thread\n");
//			fflush(stdout);
			ret = 1;
			break;
		}

		if (poll(&fds, 1, 1000) > 0) {
			if (read(*fd, &c, 1) > 0) {
//				printf("Read %x\n",c);
				if (rx_count >= buffer_size) {
					// The buffer is already full and we haven't found a valid NMEA sentence.
					// Flush the buffer and note the overflow event.
					gpsRxOverflow++;
					rx_count = 0;
					mtk_decode_init();

					if (gps_verbose) printf("[gps] Buffer full\r\n");

				} else {
					//gps_rx_buffer[rx_count] = c;
					rx_count++;

				}

				int msg_read = mtk_parse(c, gps_rx_buffer);

				if (msg_read > 0) {
					//			printf("Found sequence\n");
					break;
				}

			} else {
				break;
			}

		} else {
			break;
		}

	}

	return ret;
}

int configure_gps_mtk(int *fd)
{
	int success = 0;
	size_t result_write;
	result_write =  write(*fd, MEDIATEK_REFRESH_RATE_10HZ, strlen(MEDIATEK_REFRESH_RATE_10HZ));

	if (result_write != strlen(MEDIATEK_REFRESH_RATE_10HZ)) {
		printf("[gps] Set update speed to 10 Hz failed\r\n");
		success = 1;

	} else {
		if (gps_verbose) printf("[gps] Attempted to set update speed to 10 Hz..\r\n");
	}

	//set custom mode
	result_write =  write(*fd, MEDIATEK_CUSTOM_BINARY_MODE, strlen(MEDIATEK_CUSTOM_BINARY_MODE));

	if (result_write != strlen(MEDIATEK_CUSTOM_BINARY_MODE)) {
		//global_data_send_subsystem_info(&mtk_present);
		printf("[gps] Set MTK custom mode failed\r\n");
		success = 1;

	} else {
		//global_data_send_subsystem_info(&mtk_present_enabled);
		if (gps_verbose) printf("[gps] Attempted to set MTK custom mode..\r\n");
	}

	return success;
}

void *mtk_loop(void *args)
{
//	int oldstate;
//	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, oldstate);
//
//	printf("in mtk loop\n");
	/* Set thread name */
	prctl(PR_SET_NAME, "gps mtk read", getpid());

	/* Retrieve file descriptor and thread flag */
	struct arg_struct *arguments = (struct arg_struct *)args;
	int *fd = arguments->fd_ptr;
	bool *thread_should_exit = arguments->thread_should_exit_ptr;

	/* Initialize gps stuff */
//	int buffer_size = 1000;
//	char * gps_rx_buffer = malloc(buffer_size*sizeof(char));
	char gps_rx_buffer[MTK_BUFFER_SIZE];

	/* set parameters for mtk custom */

	if (configure_gps_mtk(fd) != 0) {
		printf("[gps] Could not write serial port..\r\n");

		/* Write shared variable sys_status */

		//global_data_send_subsystem_info(&mtk_present);

	} else {
		if (gps_verbose) printf("[gps] Configuration finished, awaiting GPS data..\r\n");


		/* Write shared variable sys_status */

		//global_data_send_subsystem_info(&mtk_present_enabled);
	}

	/* advertise GPS topic */
	struct vehicle_gps_position_s mtk_gps_d;
	mtk_gps = &mtk_gps_d;
	orb_advert_t gps_handle = orb_advertise(ORB_ID(vehicle_gps_position), mtk_gps);

	while (!(*thread_should_exit)) {
		/* Parse a message from the gps receiver */
		if (OK == read_gps_mtk(fd, gps_rx_buffer, MTK_BUFFER_SIZE)) {

			/* publish new GPS position */
			orb_publish(ORB_ID(vehicle_gps_position), gps_handle, mtk_gps);

		} else {
			/* de-advertise */
			close(gps_handle);
			break;
		}

	}

	close(gps_handle);
	if(gps_verbose) printf("[gps] mtk loop is going to terminate\n");
	return NULL;
}

void *mtk_watchdog_loop(void *args)
{
//	printf("in mtk watchdog loop\n");
	fflush(stdout);

	/* Set thread name */
	prctl(PR_SET_NAME, "gps mtk watchdog", getpid());

	/* Retrieve file descriptor and thread flag */
	struct arg_struct *arguments = (struct arg_struct *)args;
	int *fd = arguments->fd_ptr;
	bool *thread_should_exit = arguments->thread_should_exit_ptr;

	bool mtk_healthy = false;

	uint8_t mtk_fail_count = 0;
	uint8_t mtk_success_count = 0;
	bool once_ok = false;

	int mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);


	while (!(*thread_should_exit)) {
		fflush(stdout);

		/* if we have no update for a long time reconfigure gps */
		pthread_mutex_lock(mtk_mutex);
		uint64_t timestamp_now = hrt_absolute_time();
		bool all_okay = true;

		if (timestamp_now - mtk_state->last_message_timestamp > MTK_WATCHDOG_CRITICAL_TIME_MICROSECONDS) {
			all_okay = false;
		}

		pthread_mutex_unlock(mtk_mutex);

		if (!all_okay) {
//			printf("mtk unhealthy\n");
			mtk_fail_count++;
			/* gps error */
//			if (err_skip_counter == 0)
//			{
//				printf("[gps] GPS module not connected or not responding..\n");
//				err_skip_counter = 20;
//			}
//			err_skip_counter--;

//			printf("gps_mode_try_all =%u, mtk_fail_count=%u, mtk_healthy=%u, once_ok=%u\n", gps_mode_try_all, mtk_fail_count, mtk_healthy, once_ok);

			/* If we have too many failures and another mode or baud should be tried, exit... */
			if ((gps_mode_try_all == true  || gps_baud_try_all == true) && (mtk_fail_count >= MTK_HEALTH_FAIL_COUNTER_LIMIT) && (mtk_healthy == false) && once_ok == false) {
				if (gps_verbose) printf("[gps] Connection attempt failed, no MTK module found\r\n");

				gps_mode_success = false;
				break;
			}

			if (mtk_healthy && mtk_fail_count >= MTK_HEALTH_FAIL_COUNTER_LIMIT) {
				printf("[gps] ERROR: MTK GPS module stopped responding\r\n");
				// global_data_send_subsystem_info(&mtk_present_enabled);
				mavlink_log_critical(mavlink_fd, "[gps] MTK module stopped responding\n");
				mtk_healthy = false;
				mtk_success_count = 0;

			}

			/* trying to reconfigure the gps configuration */
			configure_gps_mtk(fd);
			fflush(stdout);

		} else {
			/* gps healthy */
			mtk_success_count++;

			if (!mtk_healthy && mtk_success_count >= MTK_HEALTH_SUCCESS_COUNTER_LIMIT) {
				printf("[gps] MTK module found, status ok (baud=%d)\r\n", current_gps_speed);
				/* MTK never has sat info */
				// XXX Check if lock makes sense here
				mtk_gps->satellite_info_available = 0;
				// global_data_send_subsystem_info(&mtk_present_enabled_healthy);
				mavlink_log_info(mavlink_fd, "[gps] MTK custom binary module found, status ok\n");
				mtk_healthy = true;
				mtk_fail_count = 0;
				once_ok = true;
			}
		}

		usleep(MTK_WATCHDOG_WAIT_TIME_MICROSECONDS);
	}
	if(gps_verbose) printf("[gps] mtk watchdog is going to terminate\n");
	close(mavlink_fd);
	return NULL;
}
