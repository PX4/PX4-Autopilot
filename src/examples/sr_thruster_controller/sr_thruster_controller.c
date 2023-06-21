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

    float values[] = {1,  0,  0,  0,  0,  0,  0,  0, NAN, NAN, NAN, NAN};
    memcpy(actuator_motors_msg.control, values, sizeof(values));

    PX4_INFO("Sending control signal to PWM!");
    for (int i = 0; i < 5000/2.5; i++)
    {
        PX4_INFO("Message #%d\n", i);
        orb_publish(ORB_ID(actuator_motors), _actuator_motors_pub, &actuator_motors_msg);
        delay(2.5);
    }

    // PX4_INFO("Testing GPIO");
    // PX4_INFO("Setting GPIO 0 to high");
    // px4_arch_configgpio(GPIO_GPIO0_OUTPUT);
    // px4_arch_gpiowrite(GPIO_GPIO0_OUTPUT, 1);
    // PX4_INFO("Done");

    // int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
    // /* limit the update rate to 1 Hz */
	// orb_set_interval(sensor_sub_fd, 1000);

	// PX4_INFO("orb_sensor_sub created");

    // int thrustercmd_sub_fd = orb_subscribe(ORB_ID(thruster_command));
    // /* limit the update rate to 1 Hz */
	// orb_set_interval(thrustercmd_sub_fd, 1000);

	// PX4_INFO("Subscriber to thruster command created");


    // /* Log message pub topic */
	// struct log_message_s log_msg;
	// memset(&log_msg, 0, sizeof(log_msg));
	// // orb_advert_t log_msg_pub_fd = orb_advertise(ORB_ID(log_message), &log_msg);
    // PX4_INFO("Publisher of log messages created");

	// /* one could wait for multiple topics with this technique, just using one here */
	// px4_pollfd_struct_t fds[] = {
	// 	{ .fd = thrustercmd_sub_fd,   .events = POLLIN },
	// };

    // int error_counter = 0;

	// // for (int i = 0; i < 100; i++) {
    // while (false) {
	// 	/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
	// 	int poll_ret = px4_poll(fds, 1, 5000);

	// 	/* handle the poll result */
	// 	if (poll_ret == 0) {
	// 		/* this means none of our providers is giving us data */
	// 		PX4_INFO("Got no data within a second");

    //     	} else if (poll_ret < 0) {
	// 		/* this is seriously bad - should be an emergency */
	// 		if (error_counter < 10 || error_counter % 50 == 0) {
	// 			/* use a counter to prevent flooding (and slowing us down) */
	// 			PX4_ERR("ERROR return value from poll(): %d", poll_ret);
	// 		}

	// 		error_counter++;

	// 	} else {
	// 		if (fds[0].revents & POLLIN) {
    //             /* obtained data for the first file descriptor */
	// 			struct thruster_command_s raw;
	// 			/* copy sensors raw data into local buffer */
	// 			orb_copy(ORB_ID(thruster_command), thrustercmd_sub_fd, &raw);
	// 			PX4_INFO("x1: %8.4f\t x2: %8.4f\t y1: %8.4f\t y2: %8.4f",
	// 				(double)raw.x1,
	// 				(double)raw.x2,
    //                 (double)raw.y1,
    //                 (double)raw.y2);


    //             /* set att and publish this information for other apps
	// 			the following does not have any meaning, it's just an example
	// 			*/
    //             // strcpy(log_msg.text, "New thruster command received!");

	// 			// orb_publish(ORB_ID(log_message), log_msg_pub_fd, &log_msg);
	// 		}
	// 	}
	// }

	return OK;
}
