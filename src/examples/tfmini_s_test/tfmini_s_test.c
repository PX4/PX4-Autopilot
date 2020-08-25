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
#include <string.h>
#include <math.h>

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

__EXPORT int tfmini_s_test_main(int argc, char *argv[]);

int tfmini_s_test_main(int argc, char *argv[])
{
    PX4_INFO("Hello Sky!");

    /* subscribe to sensor_combined topic */
    int _sub_distance_sensor[5];
    for (int i=0;i<5;i++){
        _sub_distance_sensor[i] = orb_subscribe_multi(ORB_ID(distance_sensor),i);
        /* limit the update rate to 100 Hz */
        orb_set_interval(_sub_distance_sensor[i], 10);
    }
    const hrt_abstime now = hrt_absolute_time();
    int index=0;
    while (hrt_absolute_time()-now<1000000) {

        for (int j=0;j<5;j++){
            bool updated;
            orb_check(_sub_distance_sensor[j], &updated);
            if(updated){
                index++;
                /* obtained data for the first file descriptor */
                struct distance_sensor_s raw;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(distance_sensor), _sub_distance_sensor[j], &raw);
                PX4_INFO("Orientation:%2d \t Distance:%8.4f \t Signal Quality:%2d",
                         (int)raw.orientation,
                         (double)raw.current_distance,
                         (int)raw.signal_quality);
            } else{
                //PX4_INFO("not updated!");
            }
        }
    }

    PX4_INFO("exiting at the index: %d",index/5);

    return 0;
}
