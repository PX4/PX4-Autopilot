
/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * @file px4_eag_example.cpp
 * EAG uORB topic publisher
 * @author Joseph Sullivan <jgs.424112@gmail.com>
 */
#include "px4_eag_publish.h"
#include <px4_middleware.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include "string.h"
#include <uORB/uORB.h>
#include <uORB/topics/eag_raw.h>
#include <mavlink/mavlink_main.h>
#include <v1.0/eag/mavlink.h>

int px4_eag_publish_main(int argc, char *argv[])
{
	int exitcode = 0;

	/* EAG sensor will be connected to ttyS6 (serial port 4) */
	/* Open port */
	int eag_fd = px4_open("/dev/ttyS6", O_RDONLY);

	if(eag_fd < 0) {
		/* return with an exitcode showing that access was not granted */
		PX4_ERR("[px4_eag_publish] failed to open /dev/ttyS6, terminating...");
		exitcode = -1;
		return exitcode;
	}

	/* declare pollfd  struct*/
	px4_pollfd_struct_t pollfd;
	pollfd.fd = eag_fd;
	pollfd.events = POLLIN;

	/* Advertise uORB topic */
	struct eag_raw_s eag_msg;
	memset(&eag_msg, 0, sizeof(eag_msg));
	orb_advert_t eag_pub = orb_advertise(ORB_ID(eag_raw), &eag_msg);

	/* Get handle of Mavlink instance */
	Mavlink* link = Mavlink().get_instance(0);
	link->display_status();

	int error_count = 0;

	while (error_count < 10) {

		/* poll the eag sensor for new data, with 5 second timeout */
		int poll_ret = px4_poll(&pollfd, 1, 5000);

		/* No data received */
		if(0 == poll_ret){
			PX4_ERR("[px4_eag_publish] no data received from eag in 5 seconds");
			error_count++;
		}

		/* Error code returned from poll */
		else if(0 > poll_ret){
			PX4_ERR("px4_eag_publish] poll request returned %d", poll_ret);
			error_count++;
		}

		/* poll returned with POLLIN flag -> data is waiting */
		else if (pollfd.revents & POLLIN){
			char buff[128];
			ssize_t count = px4_read(eag_fd, buff, sizeof(buff));

			/* for each byte read, create a eag message and publish to the eag topic*/
			int i;
			for(i = 0; i < count; i++){

				eag_msg.raw_data = buff[i];
				eag_msg.timestamp = px4::get_time_micros();

				orb_publish(ORB_ID(eag_raw), eag_pub, &eag_msg);

				// link->send_statustext_info("[px4_eag_publish] EAG data received");
			}
		}
	}

	return exitcode;
}
