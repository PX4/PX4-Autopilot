/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file land_data.cpp
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
//#include <px4_defines.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/airspeed.h>

extern "C" __EXPORT int land_data_main(int argc, char *argv[]);

int land_data_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

    bool landDetected = false;
    bool flyDetected = false;

    float _velocity_xy_filtered = 0.0;
    float _velocity_z_filtered = 0.0;
    float _accel_horz_lp = 0.0;
    float _airspeed_filtered =0.0;
    float _manual_throttle = 0.0;

    float max_vxy = 0;   //0.105
    float max_vz = 0;    //0.88
    float max_horz = 0;  //7.77

    int sensor_bias_sub_fd = orb_subscribe(ORB_ID(sensor_bias));
    int local_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
    int manual_sub_fd = orb_subscribe(ORB_ID(manual_control_setpoint));
    int airspeed_sub_fd = orb_subscribe(ORB_ID(airspeed));

    orb_set_interval(sensor_bias_sub_fd, 200);
    orb_set_interval(local_sub_fd, 200);
    orb_set_interval(manual_sub_fd, 200);
    orb_set_interval(airspeed_sub_fd, 200);

    struct sensor_bias_s _sensors;
    struct vehicle_local_position_s _local_pos;
    struct airspeed_s _airspeed;

	/* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[4];
    fds[0].fd = manual_sub_fd;
    fds[0].events = POLLIN;
    fds[1].fd = sensor_bias_sub_fd;
    fds[1].events = POLLIN;
    fds[2].fd = local_sub_fd;
    fds[2].events = POLLIN;
    fds[3].fd = airspeed_sub_fd;
    fds[3].events = POLLIN;

	int error_counter = 0;
    int i=0;
    while(1){
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
        int poll_ret = px4_poll(fds, 3, 1000);

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
            i++;

			if (fds[0].revents & POLLIN) {
                struct manual_control_setpoint_s raw;
                orb_copy(ORB_ID(manual_control_setpoint), manual_sub_fd, &raw);
                _manual_throttle = raw.z;
//                PX4_INFO("throle:%.2f",(double)raw.z);
			}
            if (fds[1].revents & POLLIN) {

                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(sensor_bias), sensor_bias_sub_fd, &_sensors);
                const float acc_hor = sqrtf(_sensors.accel_x * _sensors.accel_x +
                                            _sensors.accel_y * _sensors.accel_y);
                _accel_horz_lp = _accel_horz_lp * 0.8f + acc_hor * 0.18f;
                if (_accel_horz_lp > max_horz)
                    max_horz = _accel_horz_lp;
//                PX4_INFO("_accel_horz_lp:%.3f",(double)_accel_horz_lp);
            }
            if (fds[2].revents & POLLIN) {

                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(vehicle_local_position), local_sub_fd, &_local_pos);

                // horizontal velocity
                float val = 0.97f * _velocity_xy_filtered + 0.03f * sqrtf(_local_pos.vx * _local_pos.vx + _local_pos.vy *
                                _local_pos.vy);

                if (fabsf(val)<20) {
                        _velocity_xy_filtered = val;
                        if (_velocity_xy_filtered > max_vxy)
                            max_vxy = _velocity_xy_filtered;
//                        PX4_INFO("_velocity_xy_filtered:%.3f",(double)_velocity_xy_filtered);
                }
                // vertical velocity
                val = 0.99f * _velocity_z_filtered + 0.01f * fabsf(_local_pos.vz);

                if (fabsf(val)<20) {
                        _velocity_z_filtered = val;
                        if (_velocity_z_filtered > max_vz)
                            max_vz = _velocity_z_filtered;
//                        PX4_INFO("_velocity_z_filtered:%.3f",(double)_velocity_z_filtered);
                }
            }
            if(fds[3].events & POLLIN)
            {

                orb_copy(ORB_ID(airspeed), airspeed_sub_fd, &_airspeed);
                if(fabs(_airspeed.true_airspeed_m_s)<100)
                    _airspeed_filtered = 0.95f * _airspeed_filtered + 0.05f * _airspeed.true_airspeed_m_s;
//                PX4_INFO("_airspeed:%.3f",(double)_airspeed.timestamp);
            }
            if (i % 20 == 0)
            {
                bool time = hrt_elapsed_time(&_local_pos.timestamp) < 500 * 1000;
                if (time)
                    PX4_INFO("Time Yes");
                else
                    PX4_INFO("Time No");

                PX4_INFO("throle:%.2f",(double)_manual_throttle);
                PX4_INFO("vxy:%.3f",(double)_velocity_xy_filtered);
                PX4_INFO("vz:%.3f",(double)_velocity_z_filtered);
                PX4_INFO("horz:%.3f",(double)_accel_horz_lp);
                PX4_INFO("airspeed:%.3f",(double)_airspeed_filtered);
                flyDetected = (_velocity_xy_filtered > 1.3f
                               || _velocity_z_filtered > 0.6f
                               || _airspeed_filtered > 4.0f
                               || _accel_horz_lp > 10.0f)
                               && _manual_throttle > 0.15f;
                landDetected = !flyDetected;
                if (landDetected)
                    PX4_INFO("Landed");
                else
                    PX4_INFO("Flying");
            }
			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	PX4_INFO("exiting");

	return 0;
}
