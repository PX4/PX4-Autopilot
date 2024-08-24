/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
 * @file camera_gps.c
 * Publish GPS data with camera_feedback data
 *
 * @author Eric Kaynan <kaynanmatos141@gmail.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/topics/camera_capture.h>
#include <uORB/topics/camera_status.h>
#include <uORB/topics/camera_trigger.h>

#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/estimator_gps_status.h>

#include <uORB/topics/vehicle_attitude.h>

#include <uORB/topics/sensor_combined.h>


__EXPORT int camera_gps_main(int argc, char *argv[]);

int camera_gps_main(int argc, char *argv[])
{
	int cam_cap_sub_fd = orb_subscribe(ORB_ID(camera_capture));
	orb_set_interval(cam_cap_sub_fd, 200);



	/* struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));

	orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);
 */
	px4_pollfd_struct_t fds[] = {
		{ .fd = cam_cap_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	for (int i = 0; i < 50; i++) {

		int poll_ret = px4_poll(fds, 1, 1000);


		if (poll_ret == 0) {

			PX4_INFO("Got no data within a second");

		} else if (poll_ret < 0) {

			if (error_counter < 10 || error_counter % 50 == 0) {

				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {

				struct camera_capture_s raw;

				orb_copy(ORB_ID(camera_capture), cam_cap_sub_fd, &raw);
				printf("[camera_capture]: timestamp: %d lat: %f lon: %f seq: %d alt: %f ground_distance: %f x: %f y: %f z: %f ??: %f result: %d \n",
					raw.timestamp,
					(double)raw.lat,
					(double)raw.lon,
					raw.seq,
					(double)raw.alt,
					(double)raw.ground_distance,
					(double)raw.q[0],
					(double)raw.q[1],
					(double)raw.q[2],
					(double)raw.q[3],
					raw.result);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}


	return 0;
}
