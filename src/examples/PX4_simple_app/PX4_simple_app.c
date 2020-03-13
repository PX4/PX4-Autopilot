/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

__EXPORT int PX4_simple_app_main(int argc, char *argv[]);

int PX4_simple_app_main(int argc, char *argv[])
{
        PX4_INFO("Hello Sky!");

        /* subscribe to sensor_combined topic */
        int sensor_sub_fd = orb_subscribe(ORB_ID(distance_sensor));
        /* limit the update rate to 100 Hz */
        orb_set_interval(sensor_sub_fd, 10);

        /* one could wait for multiple topics with this technique, just using one here */
        px4_pollfd_struct_t fds[] = {
                { .fd = sensor_sub_fd,   .events = POLLIN },
                /* there could be more file descriptors here, in the form like:
                 * { .fd = other_sub_fd,   .events = POLLIN },
                 */
        };

        int error_counter = 0;

        for (int i = 0; i < 5; i++) {
                /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
                int poll_ret = px4_poll(fds, 1, 1000);

                /* handle the poll result */
                if (poll_ret == 0) {
                        /* this means none of our providers is giving us data */
                        PX4_ERR("Got no data within a second");

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
                                struct distance_sensor_s raw;
                                /* copy sensors raw data into local buffer */
                                orb_copy(ORB_ID(distance_sensor), sensor_sub_fd, &raw);
                                PX4_INFO("current_distance:\t%8.4f \n signal_quality:\t%d \n id:\t%d",
                                         (double)raw.current_distance,(int)raw.signal_quality,(int)raw.id);

                        }

                        /* there could be more file descriptors here, in the form like:
                         * if (fds[1..n].revents & POLLIN) {}
                         */
                }
        }

        PX4_INFO("exiting");

        return 0;
}
