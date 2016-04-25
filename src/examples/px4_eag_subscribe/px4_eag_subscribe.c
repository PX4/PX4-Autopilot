
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
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include "string.h"
#include <uORB/uORB.h>
#include <uORB/topics/eag_raw.h>


int px4_eag_subscribe_main(int argc, char *argv[]);
__EXPORT int px4_eag_subscribe_main(int argc, char *argv[])
{
	/* Create a uORB subscription */
	struct eag_raw_s msg;
	int eag_sub_fd = orb_subscribe(ORB_ID(eag_raw));

	/* Create poll fd struct  */
	px4_pollfd_struct_t pollfd;
	pollfd.fd = eag_sub_fd;
	pollfd.events = POLLIN;

	int error_counter = 0;

	while(error_counter < 10){
		/* poll for 5 seconds */
		int poll_ret = px4_poll(&pollfd, 1, 5000);

		if(0 > poll_ret){
			/* Bad error */
			PX4_ERR("[px4_eag_subscribe] poll returned %d", poll_ret);
			error_counter++;
		}

		else if (0 == poll_ret){
			/* Poll timed out */
			PX4_ERR("[px4_eag_subscribe] no data receivead in 5 seconds");
			error_counter++;
		}

		else if (pollfd.revents & POLLIN){
			orb_copy(ORB_ID(eag_raw), eag_sub_fd, &msg);
			PX4_INFO("[px4_eag_subscribe] data received from eag topic %u %8.4f",
								msg.raw_data,
								(double)msg.timestamp);
		}
	}

	PX4_INFO("[px4_eag_subscribe] exiting");
	return 0;
}
