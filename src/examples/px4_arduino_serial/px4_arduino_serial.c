/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file px4_arduino_serial.c
 * Test application, adapted from pxr_simple_app.
 * This program will poll ttyS6 for data and echo all data received on ttyS5.
 * @author Joseph Sullivan <jgs.424112@gmail.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <fcntl.h>
#include <string.h>

__EXPORT int px4_arduino_serial_main(int argc, char *argv[]);

int px4_arduino_serial_main(int argc, char *argv[])
{
	PX4_INFO("[px4_arduino_serial] running _main");
	int exit_code = 0;

	/* open the port which is connected to the arduino for reading only */
	int arduino_fd = px4_open("/dev/ttyS6", O_RDONLY | O_NONBLOCK);

	if(arduino_fd < 0){
		PX4_ERR("[px4_arduino_serial] error opening /dev/ttyS4 for reading");
		PX4_INFO("exiting");

		exit_code = -1;
		return exit_code;
	}

	/* open dev console for writing only */

	int console_fd = px4_open("/dev/ttyS5", O_WRONLY);

	if(console_fd < 0){
		PX4_ERR("[px4_arduino_serial] error opening /dev/ttyS5 for writing");
		PX4_INFO("exiting");

		exit_code = -2;
		return exit_code;
	}

	/* create pollfd struct */
	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = arduino_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	PX4_INFO("[px4_arduino_serial] entering polling loop");

	do{
		/* wait for POLLIN at port for 1000 ms (1 seconds) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("[px4_arduino_serial] Got no data within one seconds");
			error_counter++;

		}

		else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("[px4_arduino_serial] ERROR return value from poll(): %d"
					, poll_ret);
			}

			error_counter++;

		}
		else {

			if (fds[0].revents & POLLIN) {
				/* read the bytes */
				char buff[128];
				ssize_t rc = px4_read(arduino_fd, buff, sizeof(buff));

				if(rc > 0){
					/* echo the data on the dev console */
					rc = write(console_fd, buff, rc);
					if(rc < 0){
						PX4_ERR("[px4_port_echo] ERROR return value from write(): %d", rc);
						error_counter++;
					}
				}
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	} while (error_counter < 10);

	PX4_INFO("exiting");

	px4_close(arduino_fd);

	return 0;
}
