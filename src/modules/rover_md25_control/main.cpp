/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file main.cpp
 *
 * Example implementation of a rover steering controller.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <px4_config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/actuator_controls_1.h>
#include <uORB/topics/actuator_controls_2.h>
#include <uORB/topics/actuator_controls_3.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

/* process-specific header files */
#include "rover_params.h"
#include "BlockEncoderPositionEstimator.hpp"
/* Prototypes */

/**
 * Daemon management function.
 *
 * This function allows to start / stop the background task (daemon).
 * The purpose of it is to be able to start the controller on the
 * command line, query its status and stop it, without giving up
 * the command line to one particular process or the need for bg/fg
 * ^Z support by the shell.
 */
extern "C" __EXPORT int rover_md25_control_main(int argc, char *argv[]);

struct params {
    float yaw_p;
    float yaw_t;
    float thr_p;
};

struct param_handles {
    param_t yaw_p;
    param_t yaw_t;
    param_t thr_p;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct param_handles *h, struct params *p);

/**
 * Mainloop of daemon.
 */
int rover_md25_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/**
 * Control roll and pitch angle.
 *
 * This very simple roll and pitch controller takes the current roll angle
 * of the system and compares it to a reference. Pitch is controlled to zero and yaw remains
 * uncontrolled (tutorial code, not intended for flight).
 *
 * @param att_sp The current attitude setpoint - the values the system would like to reach.
 * @param att The current attitude. The controller should make the attitude match the setpoint
 */
void control_attitude(const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_attitude_s *att,
              struct actuator_controls_s *actuators);

/* Variables */
static bool thread_should_exit = false;     /**< Daemon exit flag */
static bool thread_running = false;     /**< Daemon status flag */
static int deamon_task;             /**< Handle of deamon task / thread */
static struct params pp;
static struct param_handles ph;
static BlockEncoderPositionEstimator estimator;
int parameters_init(struct param_handles *h)
{
    /* PID parameters */
    h->yaw_p    =   param_find("RV_YAW_P");
    h->yaw_t    =   param_find("RV_YAW_TRESH");
    h->thr_p    =   param_find("RV_THR_CRUISE");
    return OK;
}

int parameters_update(const struct param_handles *h, struct params *p)
{
    param_get(h->yaw_p, &(p->yaw_p));
    param_get(h->yaw_t, &(p->yaw_t));
    param_get(h->thr_p, &(p->thr_p));
    return OK;
}



void control_attitude(const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_attitude_s *att,
              struct actuator_controls_s *actuators)
{
    /*
     * The PX4 architecture provides a mixer outside of the controller.
     * The mixer is fed with a default vector of actuator controls, representing
     * moments applied to the vehicle frame. This vector
     * is structured as:
     *
     * Control Group 0 (attitude):
     *
     *    0  -  roll   (-1..+1)
     *    1  -  pitch  (-1..+1)
     *    2  -  yaw    (-1..+1)
     *    3  -  thrust ( 0..+1)
     *    4  -  flaps  (-1..+1)
     *    ...
     *
     * Control Group 1 (payloads / special):
     *
     *    ...
     */

    /*
     * Calculate yaw error and apply P gain
     */

    float yaw_err =  att_sp->yaw_body - att->yaw ;
    //normalisation de l'orientation du robot F.BERNAT
    yaw_err = (float)fmod((float)fmod((yaw_err + M_PI_F), M_TWOPI_F) + M_TWOPI_F, M_TWOPI_F) - M_PI_F;
   // fprintf(stderr, "att_sp->yaw_body=%0.2f,att->yaw=%0.2f \n",(double)att_sp->yaw_body,(double)att->yaw);
   // fprintf(stderr, "yaw_err=%0.2f \n",(double)yaw_err);
    float theta = fabs(yaw_err * pp.yaw_p);
    float thr_p;
    if (att_sp->thrust >0.0f)
      thr_p=att_sp->thrust + pp.thr_p;
    else
      thr_p=att_sp->thrust;

    if (fabs(yaw_err)<=(double)pp.yaw_t){
        actuators->control[0] = thr_p ;
        actuators->control[1] = thr_p ;
        }else
         if (yaw_err>0){
           actuators->control[0] = thr_p * cosf(theta);
           actuators->control[1] = thr_p * sinf(theta);
        }else{
           actuators->control[0] = thr_p * sinf(theta);
           actuators->control[1] = thr_p * cosf(theta);
    }

    actuators->timestamp = hrt_absolute_time();
}

/* Main Thread */
int rover_md25_control_thread_main(int argc, char *argv[])
{
    /* read arguments */
    bool verbose = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
            verbose = true;
        }
    }

    /* initialize parameters, first the handles, then the values */
    parameters_init(&ph);
    parameters_update(&ph, &pp);


    /*
     * Declare and safely initialize all structs to zero.
     *
     * These structs contain the system state and things
     * like attitude, position, the current waypoint, etc.
     */
    struct vehicle_attitude_s att;
    memset(&att, 0, sizeof(att));
    struct vehicle_attitude_setpoint_s att_sp;
    memset(&att_sp, 0, sizeof(att_sp));
    struct vehicle_global_position_s global_pos;
    memset(&global_pos, 0, sizeof(global_pos));
    struct manual_control_setpoint_s manual_sp;
    memset(&manual_sp, 0, sizeof(manual_sp));
    struct vehicle_status_s vstatus;
    memset(&vstatus, 0, sizeof(vstatus));
    struct position_setpoint_s global_sp;
    memset(&global_sp, 0, sizeof(global_sp));

    /* output structs - this is what is sent to the mixer */
    struct actuator_controls_s actuators;
    memset(&actuators, 0, sizeof(actuators));


    /* publish actuator controls with zero values */
    for (unsigned i = 0; i < (sizeof(actuators.control) / sizeof(actuators.control[0])); i++) {
        actuators.control[i] = 0.0f;
    }



    /*
     * Advertise that this controller will publish actuator
     * control values and the rate setpoint
     */
    orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);

    /* subscribe to topics. */

    int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));

    int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));

    int manual_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

    int vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));

    int att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));

    int param_sub = orb_subscribe(ORB_ID(parameter_update));



    /* Setup of loop */

    struct pollfd fds[3];

    fds[0].fd = param_sub;

    fds[0].events = POLLIN;

    fds[1].fd = att_sub;

    fds[1].events = POLLIN;

    fds[2].fd = global_pos_sub;

    fds[2].events = POLLIN;


    while (!thread_should_exit) {
       // estimator.update();
        /*
         * Wait for a sensor or param update, check for exit condition every 500 ms.
         * This means that the execution will block here without consuming any resources,
         * but will continue to execute the very moment a new attitude measurement or
         * a param update is published. So no latency in contrast to the polling
         * design pattern (do not confuse the poll() system call with polling).
         *
         * This design pattern makes the controller also agnostic of the attitude
         * update speed - it runs as fast as the attitude updates with minimal latency.
         */
        int ret = poll(fds, 3, 500);

        if (ret < 0) {
            /*
             * Poll error, this will not really happen in practice,
             * but its good design practice to make output an error message.
             */
            warnx("poll error");

        } else if (ret == 0) {
            /* no return value = nothing changed for 500 ms, ignore */
        } else {

            /* only update parameters if they changed */
            if (fds[0].revents & POLLIN) {
                /* read from param to clear updated flag (uORB API requirement) */
                struct parameter_update_s update;
                orb_copy(ORB_ID(parameter_update), param_sub, &update);

                /* if a param update occured, re-read our parameters */
                parameters_update(&ph, &pp);
            }

            /* only run controller if attitude changed */
            if (fds[2].revents & POLLIN) {

            /* Check if there is a new position measurement or position setpoint */
                bool global_pos_updated;
                orb_check(global_pos_sub, &global_pos_updated);
               /* get a local copy of pos */
                if (global_pos_updated) {

                  orb_copy(ORB_ID(vehicle_global_position), global_pos_sub, &global_pos);
                 }

            }
            /* only run controller if attitude changed */
            if (fds[1].revents & POLLIN) {


                /* Check if there is a new position measurement or position setpoint */

                bool att_sp_updated;
                orb_check(att_sp_sub, &att_sp_updated);
                bool manual_sp_updated;
                orb_check(manual_sp_sub, &manual_sp_updated);

                /* get a local copy of attitude */
                orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);



                if (att_sp_updated) {
                    orb_copy(ORB_ID(vehicle_attitude_setpoint), att_sp_sub, &att_sp);
                    /* control attitude / heading */
                    control_attitude(&att_sp, &att, &actuators);
                }


                if (manual_sp_updated)
                    /* get the RC (or otherwise user based) input */
                {
                    orb_copy(ORB_ID(manual_control_setpoint), manual_sp_sub, &manual_sp);

                    actuators.control[0]=manual_sp.x + manual_sp.y;
                    actuators.control[1]=manual_sp.x - manual_sp.y;
                  //  fprintf(stderr, "manual \n");
                }

                // XXX copy from manual depending on flight / usage mode to override

                /* get the system status and the flight mode we're in */
                orb_copy(ORB_ID(vehicle_status), vstatus_sub, &vstatus);

                /* sanity check and publish actuator outputs */
                if (isfinite(actuators.control[0]) &&
                    isfinite(actuators.control[1]) ) {

                    orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

                    if (verbose) {
                        warnx("published");
                    }
                }
            }
        }
    }

    warnx("exiting, stopping all motors.");
    thread_running = false;

    /* kill all outputs */
    for (unsigned i = 0; i < (sizeof(actuators.control) / sizeof(actuators.control[0])); i++) {
        actuators.control[i] = 0.0f;
    }

    actuators.timestamp = hrt_absolute_time();

    orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

    fflush(stdout);

    return 0;
}

/* Startup Functions */

static void
usage(const char *reason)
{
    if (reason) {
        fprintf(stderr, "%s\n", reason);
    }

    fprintf(stderr, "usage: rover_md25_control {start|stop|status}\n\n");
    exit(1);
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */
int rover_md25_control_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("missing command");
    }

    if (!strcmp(argv[1], "start")) {

        if (thread_running) {
            warnx("running");
            /* this is not an error */
            exit(0);
        }

        thread_should_exit = false;
        deamon_task = px4_task_spawn_cmd("rover_md25_control",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_MAX - 20,
                         2048,
                         rover_md25_control_thread_main,
                         (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
        thread_running = true;
        exit(0);
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        exit(0);
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("running");

        } else {
            warnx("not started");
        }

        exit(0);
    }

    usage("unrecognized command");
    exit(1);
}



