/****************************************************************************
 * examples/hello/main.c
 *
 *   Copyright (C) 2008, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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


#include <time.h>
#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdbool.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <time.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/prctl.h>
#include <errno.h>

#include "sdlog.h"
#include "sdlog_generated.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

__EXPORT int sdlog_main(int argc, char *argv[]);

const char *src  = "/dev/mmcsd0";
const char *trgt = "/fs/microsd";
const char *type = "vfat";
const char *logfile_end = ".px4log";
const char *mfile_end = ".m";
char folder_path[64];

#define SDLOG_LED_PRIORITY LED_REQUEST_PRIORITY_MAX
#define BUFFER_BYTES 1000 // length of buffer
#define SAVE_EVERY_BYTES 2000
#define MAX_MOUNT_TRIES 5

static void sdlog_sig_handler(int signo, siginfo_t *info, void *ucontext); // is executed when SIGUSR1 is received
bool sdlog_sigusr1_rcvd; // if this is set to true through SIGUSR1, sdlog will terminate

static pthread_t logbuffer_thread; // thread to copy log values to the buffer
static void *logbuffer_loop(void *arg);


uint8_t *buffer_start; // always points to the very beginning
uint8_t *buffer_end; // always points to the very end
uint8_t *buffer_cursor_start; // points to the start of the current buffer
uint8_t *buffer_cursor_end; // points to the end of the current buffer
size_t buffer_bytes_used; // amount stored in the buffer at the moment

uint32_t bytes_recv; // to count bytes received and written to the sdcard
uint32_t bytes_sent; // to count the bytes written to the buffer

/****************************************************************************
 * user_start
 ****************************************************************************/

int file_exist(const char *filename)
{
	struct stat buffer;
	return (stat(filename, &buffer) == 0);
}


int sdlog_main(int argc, char *argv[])
{
	// print text
	printf("[sdlog] initialized\n");
	usleep(10000);

	sdlog_sigusr1_rcvd = false;

	led_request_t amber_on_request = {.led = LED_AMBER, .priority = SDLOG_LED_PRIORITY, .request_type = LED_REQUEST_ON};
	led_request_t amber_off_request = {.led = LED_AMBER, .priority = SDLOG_LED_PRIORITY, .request_type = LED_REQUEST_OFF};
	amber_on_request.pid = getpid();
	amber_off_request.pid = getpid();

	/* signal handler to abort when low voltage occurs */
	struct sigaction act;
	struct sigaction oact;
	act.sa_sigaction = sdlog_sig_handler;
	(void)sigemptyset(&act.sa_mask);
	(void)sigaddset(&act.sa_mask, SIGUSR1);
	(void)sigaction(SIGUSR1, &act, &oact);

	uint8_t mount_counter = 0;

	commander_state_machine_t current_system_state;

	global_data_send_led_request(&amber_on_request);

	if (file_exist(trgt) == 1) {
		printf("[sdlog] Mount already available at %s\n", trgt);
		global_data_send_led_request(&amber_off_request);

	} else {
		printf("[sdlog] Mount not available yet, trying to mount...\n");

		/* mostly the first mount fails, sometimes it helps to press the card onto the board! */
		while (mount(src, trgt, type, 0, "") != 0) {
			/* abort if kill signal is received */
			if (sdlog_sigusr1_rcvd == true) {
				return 0;
			}

			//make sure were not airborne
			global_data_trylock(&global_data_sys_status->access_conf);
			current_system_state = global_data_sys_status->state_machine;
			global_data_unlock(&global_data_sys_status->access_conf);

			if (current_system_state == SYSTEM_STATE_STANDBY || current_system_state == SYSTEM_STATE_PREFLIGHT || current_system_state == SYSTEM_STATE_GROUND_ERROR) {
				usleep(1000000);
				printf("[sdlog] ERROR: Failed to mount SD card (attempt %d of %d), reason: %s\n", mount_counter + 1, MAX_MOUNT_TRIES, strerror((int)*get_errno_ptr()));
				global_data_send_led_request(&amber_off_request);
				mount_counter++;

			} else {
				printf("[sdlog] WARNING: Not mounting SD card in flight!\n");
				printf("[sdlog] ending now...\n");
				fflush(stdout);
				return 0;
			}

			if (mount_counter >= MAX_MOUNT_TRIES) {
				printf("[sdlog] ERROR: SD card could not be mounted!\n");
				printf("[sdlog] ending now...\n");
				fflush(stdout);
				return 0;
			}
		}

		printf("[sdlog] Mount created at %s...\n", trgt);
		global_data_send_led_request(&amber_off_request);
	}


	/* make folder on sdcard */
	uint16_t foldernumber = 1; // start with folder 0001
	int mkdir_ret;

	/* look for the next folder that does not exist */
	while (foldernumber < MAX_NO_LOGFOLDER) {
		/* set up file path: e.g. /mnt/sdcard/logfile0001.txt */
		sprintf(folder_path, "%s/session%04u", trgt, foldernumber);
		mkdir_ret = mkdir(folder_path, S_IRWXU | S_IRWXG | S_IRWXO);
		/* the result is -1 if the folder exists */

		if (mkdir_ret == 0) {
			/* folder does not exist */
			break;

		} else if (mkdir_ret == -1) {
			/* folder exists already */
			foldernumber++; // to try next time
			continue;

		} else {
			printf("[sdlog] ERROR: Failed creating new folder: %s\n", strerror((int)*get_errno_ptr()));
			return -1;
		}
	}

	if (foldernumber >= MAX_NO_LOGFOLDER) {
		/* we should not end up here, either we have more than MAX_NO_LOGFOLDER on the SD card, or another problem */
		printf("[sdlog] ERROR: all %d possible folders exist already\n", MAX_NO_LOGFOLDER);
		return -1;
	}


	/* write m-file for evaluation in Matlab */
	FILE *mfile;
	char mfile_path[64] = ""; // string to hold the path to the mfile
	const char *mfilename = "sdlog_eval";

	sprintf(mfile_path, "%s/%s%s", folder_path, mfilename, mfile_end);

	if (NULL == (mfile = fopen(mfile_path, "w"))) {
		printf("[sdlog] ERROR: opening %s failed: %s\n", mfile_path, strerror((int)*get_errno_ptr()));
	}

	/* write file contents generated using updatesdlog.sh and mfile.template in nuttx/configs/px4fmu/include */
	fwrite(MFILE_STRING, sizeof(MFILE_STRING), 1, mfile);
	fprintf(mfile, "\n");
	fclose(mfile);
	printf("[sdlog] m-file written to sdcard\n");


	/* create the ringbuffer */
	uint8_t buffer[BUFFER_BYTES];
	buffer_start = buffer;
	buffer_cursor_start = buffer;
	buffer_cursor_end = buffer;
	buffer_end = buffer + BUFFER_BYTES;

	/* create loop to write log data in a ringbuffer */
	pthread_attr_t logbuffer_attr;
	pthread_attr_init(&logbuffer_attr);
	pthread_attr_setstacksize(&logbuffer_attr, 2400);
	pthread_create(&logbuffer_thread, &logbuffer_attr, logbuffer_loop, NULL);

	/* create logfile */
	FILE *logfile;
	char logfile_path[64] = ""; // string to hold the path to the logfile
	const char *logfilename = "all";

	/* set up file path: e.g. /mnt/sdcard/session0001/gpslog.txt */
	sprintf(logfile_path, "%s/%s%s", folder_path, logfilename, logfile_end);


	if (NULL == (logfile = fopen(logfile_path, "wb"))) {
		printf("[sdlog] opening %s failed: %s\n", logfile_path, strerror((int)*get_errno_ptr()));
	}

//	else
//	{
//		printf("[sdlog] opening %s was successful\n",logfile_path);
//	}

	int logfile_no = fileno(logfile);

	bytes_recv = 0;			/**< count all bytes that were received and written to the sdcard */
	size_t ret_write;		/**< last amount written to sdcard */
	size_t target_write;	/**< desired amount to write to sdcard */
	int ret_fsync;			/**< return value of fsync() system call */
	int error_count = 0;	/**< number of continous errors (one successful write resets it) */

	/* Start logging */
	while (1) {
		/* write as soon as content is in the buffer */
		while (buffer_bytes_used > 1) {
			/* case where the data is not wrapped in the buffer */
			if (buffer_cursor_start < buffer_cursor_end) {
				/* write all available data */
				target_write = (size_t)(buffer_cursor_end - buffer_cursor_start);

				if (0 <= (ret_write = fwrite(buffer_cursor_start, 1, target_write, logfile))) {
					/* decrease the amount stored in the buffer, normally to 0 */
					buffer_bytes_used -= ret_write;
					/* set new cursor position: wrap it in case it falls out of the buffer */
					buffer_cursor_start = buffer_start + (buffer_cursor_start - buffer_start + ret_write) % BUFFER_BYTES;
					bytes_recv += ret_write;
					error_count = 0;

				} else {
					error_count++;
					printf("[sdlog] ERROR: Write fail: %d of %d, %s\n", ret_write, target_write, strerror((int)*get_errno_ptr()));
				}
			}

			/* case where the content is wrapped around the buffer */
			else if (buffer_cursor_start > buffer_cursor_end) {
				/* write data until the end of the buffer */
				target_write = (size_t)(buffer_end - buffer_cursor_start);

				if (0 <= (ret_write = fwrite(buffer_cursor_start, 1, target_write, logfile))) {
					/* decrease the amount stored in the buffer */
					buffer_bytes_used -= ret_write;
					/* set new cursor position: wrap it in case it falls out of the buffer */
					buffer_cursor_start = buffer_start + (buffer_cursor_start - buffer_start + ret_write) % BUFFER_BYTES;
					bytes_recv += ret_write;
					error_count = 0;

				} else {
					error_count++;
					printf("[sdlog] ERROR: Write fail: %d of %d, %s\n", ret_write, target_write, strerror((int)*get_errno_ptr()));
				}

				/* write the rest of the data at the beginning of the buffer */
				target_write = (size_t)(buffer_cursor_end - buffer_start);

				if (0 <= (ret_write = fwrite(buffer_start, 1, target_write, logfile))) {
					/* decrease the amount stored in the buffer, now normally to 0 */
					buffer_bytes_used -= ret_write;
					/* set new cursor position: wrap it in case it falls out of the buffer */
					buffer_cursor_start = buffer_start + (buffer_cursor_start - buffer_start + ret_write) % BUFFER_BYTES;
					bytes_recv += ret_write;
					error_count = 0;

				} else {
					error_count++;
					printf("[sdlog] ERROR: write fail: %d of %d, %s\n", ret_write, target_write, strerror((int)*get_errno_ptr()));
				}

			} else {
				error_count++;
				printf("[sdlog] ERROR: dropped data\n");
			}

//			if(bytes_recv>500000)
//			{
//				goto finished;
//			}
		}


		/* save and exit if we received signal 1 or have a permanent error */
		if (sdlog_sigusr1_rcvd == true || error_count > 100) {
			fclose(logfile);
			umount(trgt);

			if (error_count > 100) {
				return ERROR;

			} else {
				return OK;
			}
		}

		/* save file from time to time */
		else if ((bytes_recv > 0 && bytes_recv % SAVE_EVERY_BYTES == 0)) {
			if ((ret_fsync = fsync(logfile_no)) != OK) {
				printf("[sdlog] ERROR: sync fail: #%d, %s\n", ret_fsync, strerror((int)*get_errno_ptr()));
			}
		}

//		else if(bytes_recv > 2*SAVE_EVERY_BYTES)
//		{
//			goto finished;
//		}
		/* sleep, not to block everybody else */
		usleep(1000);
	}

//finished:
//
//	/* at the moment we should not end here */
//	fclose(logfile);
//	printf("[sdlog] logfile saved\n");
//	umount(trgt);
//	printf("[sdlog] ending...\n");
//	fflush(stdout);

	return 0;
}

/* is executed when SIGUSR1 is raised (to terminate the app) */
static void sdlog_sig_handler(int signo, siginfo_t *info, void *ucontext)
{
	sdlog_sigusr1_rcvd = true;
}


static void *logbuffer_loop(void *arg)
{
	/* set name for this pthread */
	prctl(PR_SET_NAME, "sdlog logbuffer", getpid());

	bytes_sent = 0; // count all bytes written to the buffer

	//size_t block_length = sizeof(global_data_sensors_raw_t) - 4*sizeof(access_conf_t);
	size_t first_block_length;
	size_t second_block_length;
	buffer_bytes_used = 0;

	log_block_t log_block = {.check = {'$', '$', '$', '$'}};

	size_t block_length = sizeof(log_block_t);
//	printf("Block length is: %u",(uint16_t)block_length);

//	uint16_t test_counter = 0;

	/* start copying data to buffer */
	while (1) {
		copy_block(&log_block);

		/* no more free space in buffer */
		if (buffer_bytes_used + (uint16_t)block_length > BUFFER_BYTES) {
			printf("[sdlog] buf full, skipping\n");
		}

		/* data needs to be wrapped around ringbuffer*/
		else if (buffer_cursor_end + block_length >= buffer_end) {
			/* first block length is from cursor until the end of the buffer */
			first_block_length = (size_t)(buffer_end - buffer_cursor_end);
			/* second block length is the rest */
			second_block_length = block_length - first_block_length;
			/* copy the data, not the pointer conversion */
			memcpy(buffer_cursor_end, ((uint8_t *) & (log_block)), first_block_length);
			memcpy(buffer_start, ((uint8_t *) & (log_block)) + first_block_length, second_block_length);
			/* set the end position of the cursor */
			buffer_cursor_end = buffer_start + second_block_length;
			/* increase the amount stored in the buffer */
			buffer_bytes_used += block_length;
			bytes_sent += block_length;
		}

		/* data does not need to be wrapped around */
		else {
			/* copy the whole block in one step */
			memcpy(buffer_cursor_end, ((uint8_t *) & (log_block)), block_length);
			/* set the cursor to 0 of the buffer instead of the end */
			buffer_cursor_end = buffer_start + (buffer_cursor_end - buffer_start + block_length) % BUFFER_BYTES;
			/* increase the amount stored in the buffer */
			buffer_bytes_used += block_length;
			bytes_sent += block_length;
		}

		/* also end the pthread when we receive a SIGUSR1 */
		if (sdlog_sigusr1_rcvd == true) {
			break;
		}
	}

	return NULL;
}
