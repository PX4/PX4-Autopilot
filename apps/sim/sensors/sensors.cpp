/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file sensors.cpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 *
 * Sensor readout process.
 */

#include <nuttx/config.h>

#include <fcntl.h>
#include <poll.h>
#include <nuttx/analog/adc.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>

#include <drivers/drv_hrt.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>

// Constants
#define BAT_VOL_INITIAL 12.f

// Forward Declarations
class Sensors;

// Globals
namespace sensors {
    Sensors * g_sensors;
};

class Sensors
{
private:
    int         _sensors_task;              /**< task handle for sensor task */
    bool        _task_should_exit;          /**< if true, sensor task should exit */
	bool		_publishing;			    /**< if true, we are publishing sensor data */
    orb_advert_t    _sensor_pub;            /**< combined sensor data topic */
    orb_advert_t    _manual_control_pub;    /**< manual control signal topic */
    orb_advert_t    _rc_pub;                /**< raw r/c control topic */
public:
    Sensors() :
        _sensors_task(-1),
        _task_should_exit(false),
        _publishing(true),
        _sensor_pub(-1),
        _manual_control_pub(-1),
        _rc_pub(-1)
    {
    }
    ~Sensors()
    {
        if (_sensors_task != -1) {

            /* task wakes up every 100ms or so at the longest */
            _task_should_exit = true;

            /* wait for a second for the task to quit at our request */
            unsigned i = 0;
            do {
                /* wait 20ms */
                usleep(20000);

                /* if we have given up, kill it */
                if (++i > 50) {
                    task_delete(_sensors_task);
                    break;
                }
            } while (_sensors_task != -1);
        }

        sensors::g_sensors = nullptr;
    }
    static void task_main_trampoline(int argc, char *argv[])
    {
        sensors::g_sensors->task_main();
    }
    int start()
    {
        ASSERT(_sensors_task == -1);

        /* start the task */
        _sensors_task = task_spawn("sensors_task",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_MAX - 5,
                       6000,    /* XXX may be excesssive */
                       (main_t)&Sensors::task_main_trampoline,
                       nullptr);

        if (_sensors_task < 0) {
            warn("task start failed");
            return -errno;
        }
        return OK;
    }
    void task_main()
    {

        /* inform about start */
        printf("[sensors] Initializing..\n");
        fflush(stdout);

        /* start individual sensors */
        //accel_init();
        //gyro_init();
        //mag_init();
        //baro_init();
        //adc_init();

        /*
         * do subscriptions
         */
        //_gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));
        //_accel_sub = orb_subscribe(ORB_ID(sensor_accel));
        //_mag_sub = orb_subscribe(ORB_ID(sensor_mag));
        //_rc_sub = orb_subscribe(ORB_ID(input_rc));
        //_baro_sub = orb_subscribe(ORB_ID(sensor_baro));
        //_vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
        //_params_sub = orb_subscribe(ORB_ID(parameter_update));
        //_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

        /* rate limit vehicle status updates to 5Hz */
        //orb_set_interval(_vstatus_sub, 200);

        /*
         * do advertisements
         */
        struct sensor_combined_s raw;
        memset(&raw, 0, sizeof(raw));
        raw.timestamp = 0; hrt_absolute_time();
        raw.battery_voltage_v = BAT_VOL_INITIAL;
        raw.adc_voltage_v[0] = 0.9f;
        raw.adc_voltage_v[1] = 0.0f;
        raw.adc_voltage_v[2] = 0.0f;
        raw.battery_voltage_counter = 0;
        raw.battery_voltage_valid = false;

        /* get a set of initial values */
        //accel_poll(raw);
        //gyro_poll(raw);
        //mag_poll(raw);
        //baro_poll(raw);

        //parameter_update_poll(true [> forced <]);

        /* advertise the sensor_combined topic and make the initial publication */
        _sensor_pub = orb_advertise(ORB_ID(sensor_combined), &raw);

        /* advertise the manual_control topic */
        struct manual_control_setpoint_s manual_control;
        manual_control.mode = MANUAL_CONTROL_MODE_ATT_YAW_POS;
        manual_control.roll = 0.0f;
        manual_control.pitch = 0.0f;
        manual_control.yaw = 0.0f;
        manual_control.throttle = 0.0f;
        manual_control.aux1_cam_pan_flaps = 0.0f;
        manual_control.aux2_cam_tilt = 0.0f;
        manual_control.aux3_cam_zoom = 0.0f;
        manual_control.aux4_cam_roll = 0.0f;

        _manual_control_pub = orb_advertise(ORB_ID(manual_control_setpoint), &manual_control);

        /* advertise the rc topic */
        {
            struct rc_channels_s rc;
            memset(&rc, 0, sizeof(rc));
            _rc_pub = orb_advertise(ORB_ID(rc_channels), &rc);
        }

        /* wakeup source(s) */
        //struct pollfd fds[1];

        /* use the gyro to pace output - XXX BROKEN if we are using the L3GD20 */
        //fds[0].fd = _gyro_sub;
        //fds[0].events = POLLIN;

        while (!_task_should_exit) {

            
            /* wait for up to 500ms for data */
            //int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

            /* timed out - periodic check for _task_should_exit, etc. */
            //if (pret == 0)
                //continue;

            /* this is undesirable but not much we can do - might want to flag unhappy status */
            //if (pret < 0) {
                //warn("poll error %d, %d", pret, errno);
                //continue;
            //}

            //perf_begin(_loop_perf);

            /* check vehicle status for changes to publication state */
            //vehicle_status_poll();

            /* check parameters for updates */
            //parameter_update_poll();

            /* store the time closest to all measurements (this is bogus, sensor timestamps should be propagated...) */
            raw.timestamp = hrt_absolute_time();
            //printf("\nhrt time: %d\n", raw.timestamp);

            /* copy most recent sensor data */
            //gyro_poll(raw);
            //accel_poll(raw);
            //mag_poll(raw);
            //baro_poll(raw);

            /* check battery voltage */
            //adc_poll(raw);

            /* Inform other processes that new data is available to copy */
            if (_publishing)
                orb_publish(ORB_ID(sensor_combined), _sensor_pub, &raw);

#ifdef CONFIG_HRT_PPM
            /* Look for new r/c input data */
            //ppm_poll();
#endif

            //perf_end(_loop_perf);
            
            // sleep to allow simulation to switch tasks
            usleep(1);
        }

        printf("[sensors] exiting.\n");

        _sensors_task = -1;
        _exit(0);
    }

};

/**
 * Sensor app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int sensors_main(int argc, char *argv[]);

int sensors_main(int argc, char *argv[])
{
    if (argc != 2 || argv[1]==NULL)
        errx(1, "usage: sensors {start|stop|status}");

    if (!strcmp(argv[1], "start")) {

        if (sensors::g_sensors != nullptr)
            errx(1, "sensors task already running");

        sensors::g_sensors = new Sensors;
        if (sensors::g_sensors == nullptr)
            errx(1, "sensors task alloc failed");

        if (OK != sensors::g_sensors->start()) {
            delete sensors::g_sensors;
            sensors::g_sensors = nullptr;
            err(1, "sensors task start failed");
        }
        exit(0);
    }

    if (!strcmp(argv[1], "stop")) {
        if (sensors::g_sensors == nullptr)
            errx(1, "sensors task not running");
        delete sensors::g_sensors;
        sensors::g_sensors = nullptr;
        exit(0);
    }

    if (!strcmp(argv[1], "status")) {
        if (sensors::g_sensors) {
            errx(0, "task is running");
        } else {
            errx(1, "task is not running");
        }
    }

    errx(1, "unrecognized command");
}
