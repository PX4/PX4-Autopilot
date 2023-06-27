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
 * @file sr_thruster_controller.c
 * For controlling solenoid cold gas thrusters in space robotics
 *
 * @author Elias Krantz <eliaskra@kth.se>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <stdio.h>
#include <math.h>
#include <poll.h>
#include <time.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/thruster_command.h>
#include <uORB/topics/log_message.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/my_message.h>

/**
 * SR thruster controller app start / stop handling function
 *
 * @ingroup apps
 */
__EXPORT int sr_thruster_controller_main(int argc, char *argv[]);

void delay(unsigned int milliseconds) {
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000;
    nanosleep(&ts, NULL);
}

int sr_thruster_controller_main(int argc, char *argv[])
{
    PX4_INFO("Thruster controller started!");

    // Create publisher for pwm values
    struct actuator_motors_s actuator_motors_msg;
	memset(&actuator_motors_msg, 0, sizeof(actuator_motors_msg));
    orb_advert_t _actuator_motors_pub = orb_advertise(ORB_ID(actuator_motors), &actuator_motors_msg);

    /* For publishing debug messages */
    struct my_message_s my_msg;
	memset(&my_msg, 0, sizeof(my_msg));
    orb_advert_t _my_message_pub = orb_advertise(ORB_ID(my_message), &my_msg);

    int thrustercmd_sub_fd = orb_subscribe(ORB_ID(thruster_command));
    /* limit the update rate to 10 Hz */
	orb_set_interval(thrustercmd_sub_fd, 100);

	PX4_INFO("Subscriber to thruster command created");

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = thrustercmd_sub_fd,   .events = POLLIN },
	};

    int error_counter = 0;

	// for (int i = 0; i < 100; i++) {
    while (true) {
		/* wait for sensor update of 1 file descriptor for 10 s (10 second) */
		int poll_ret = px4_poll(fds, 1, 10000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_INFO("Got no data within a second");

        	} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {
			if (fds[0].revents & POLLIN) {
                /* obtained data for the first file descriptor */
				struct thruster_command_s thruster_cmd;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(thruster_command), thrustercmd_sub_fd, &thruster_cmd);
				PX4_INFO("x1: %8.4f\t x2: %8.4f\t y1: %8.4f\t y2: %8.4f",
					(double)thruster_cmd.x1,
					(double)thruster_cmd.x2,
                    (double)thruster_cmd.y1,
                    (double)thruster_cmd.y2);

                actuator_motors_msg.control[0] = thruster_cmd.x1 > 0 ? thruster_cmd.x1 : 0;
                actuator_motors_msg.control[1] = thruster_cmd.x1 < 0 ? -thruster_cmd.x1 : 0;
                actuator_motors_msg.control[2] = thruster_cmd.x2 > 0 ? thruster_cmd.x2 : 0;
                actuator_motors_msg.control[3] = thruster_cmd.x2 < 0 ? -thruster_cmd.x2 : 0;
                actuator_motors_msg.control[4] = thruster_cmd.y1 > 0 ? thruster_cmd.y1 : 0;
                actuator_motors_msg.control[5] = thruster_cmd.y1 < 0 ? -thruster_cmd.y1 : 0;
                actuator_motors_msg.control[6] = thruster_cmd.y2 > 0 ? thruster_cmd.y2 : 0;
                actuator_motors_msg.control[7] = thruster_cmd.y2 < 0 ? -thruster_cmd.y2 : 0;

                orb_publish(ORB_ID(actuator_motors), _actuator_motors_pub, &actuator_motors_msg);

                strcpy(my_msg.text, "Thruster command received!");
                orb_publish(ORB_ID(my_message), _my_message_pub, &my_msg);
			}
		}
	}

	return OK;
}
