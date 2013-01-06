/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: Tobias Naegeli <naegelit@student.ethz.ch>
 *           Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
 *           Lorenz Meier <lm@inf.ethz.ch>
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
 * @file position_estimator_main.c
 * Model-identification based position estimator for multirotors
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <poll.h>
#include <systemlib/geo/geo.h>

#define N_STATES 6
#define ERROR_COVARIANCE_INIT 3

#define PROJECTION_INITIALIZE_COUNTER_LIMIT 5000
#define REPROJECTION_COUNTER_LIMIT 125

__EXPORT int position_estimator_main(int argc, char *argv[]);

static uint16_t position_estimator_counter_position_information;



/****************************************************************************
 * main
 ****************************************************************************/

int position_estimator_main(int argc, char *argv[])
{

    /* welcome user */
    printf("[position_estimator] started\n");

    /* initialize values */
    /*static float u[2] = {0, 0};*/
    static float z[3] = {0, 0, 0};
    /*static float xapo[N_STATES] = {0, 0, 0, 0, 0, 0};
    static float Papo[N_STATES * N_STATES] = {ERROR_COVARIANCE_INIT, 0, 0, 0, 0, 0,
            ERROR_COVARIANCE_INIT, 0, 0, 0, 0, 0,
            ERROR_COVARIANCE_INIT, 0, 0, 0, 0, 0,
            ERROR_COVARIANCE_INIT, 0, 0, 0, 0, 0,
            ERROR_COVARIANCE_INIT, 0, 0, 0, 0, 0,
            ERROR_COVARIANCE_INIT, 0, 0, 0, 0, 0
                         };*/

    //static float xapo1[N_STATES];
    //static float Papo1[36];

    /*static float gps_covariance[3] = {0.0f, 0.0f, 0.0f};*/

    static uint16_t counter = 0;
    position_estimator_counter_position_information = 0;

    /*uint8_t predict_only = 1;*/
    bool gps_valid = false;
    /*bool new_initialization = true;*/

    static double lat_current = 0.0d;//[°]] --> 47.0
    static double lon_current = 0.0d; //[°]] -->8.5
    float alt_current = 0.0f;


    //TODO: handle flight without gps but with estimator

    /* subscribe to vehicle status, attitude, gps */
    struct vehicle_gps_position_s gps;
    gps.fix_type = 0;
    struct vehicle_status_s vstatus;
    struct vehicle_attitude_s att;

    int vehicle_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
    int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

    /* subscribe to attitude at 100 Hz */
    int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

    /* wait until gps signal turns valid, only then can we initialize the projection */
    while (!gps_valid) {
        struct pollfd fds[1] = { {.fd = vehicle_gps_sub, .events = POLLIN} };

        /* wait for GPS updates, BUT READ VEHICLE STATUS (!)
         * this choice is critical, since the vehicle status might not
         * actually change, if this app is started after GPS lock was
         * aquired.
         */
        if (poll(fds, 1, 5000)) {
            /* Wait for the GPS update to propagate (we have some time) */
            usleep(5000);
            /* Read wether the vehicle status changed */
            orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);
            gps_valid = (gps.fix_type > 2);
        }
    }

    /* get gps value for first initialization */
    orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);
    lat_current = ((double)(gps.lat)) * 1.0e-7d;
    lon_current = ((double)(gps.lon)) * 1.0e-7d;
    alt_current = gps.alt * 1.0e-3d;

    /* initialize coordinates */
    map_projection_init(lat_current, lon_current);

    /* publish global position messages only after first GPS message */
    struct vehicle_local_position_s local_pos = {
        .x = 0,
        .y = 0,
        .z = 0
    };
    orb_advert_t local_pos_pub = orb_advertise(ORB_ID(vehicle_local_position), &local_pos);
    struct vehicle_global_position_s global_pos = {
        .lat = gps.lat,
        .lon = gps.lon,
        .alt = alt_current,
        .relative_alt = 0,
        .vx = gps.vel_n,
        .vy = gps.vel_e,
        .vz = gps.vel_d,
        .hdg = 0
    };
    orb_advert_t global_pos_pub = orb_advertise(ORB_ID(vehicle_global_position), &global_pos);

    printf("[position estimator] initialized projection with: lat: %.10f,  lon:%.10f\n", lat_current, lon_current);

    while (1) {

        /*This runs at the rate of the sensors, if we have also a new gps update this is used in the position_estimator function */
        struct pollfd fds[1] = { {.fd = vehicle_attitude_sub, .events = POLLIN} };

        if (poll(fds, 1, 5000) <= 0) {
            /* error / timeout */
        } else {

            orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
            /* got attitude, updating pos as well */
            orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);
            orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vstatus);
            
            gps_valid = (gps.fix_type > 2);

            /*copy attitude */
            /*u[0] = att.roll;*/
            /*u[1] = att.pitch;*/

            /* initialize map projection with the last estimate (not at full rate) */
            if (gps_valid) {
                /* Project gps lat lon (Geographic coordinate system) to plane*/
                map_projection_project(((double)(gps.lat)) * 1.0e-7d, ((double)(gps.lon)) * 1.0e-7d, &(z[0]), &(z[1]));

                local_pos.x = z[0];
                local_pos.y = z[1];
                /* negative offset from initialization altitude */
                local_pos.z = alt_current - (gps.alt) * 1.0f-3;

                orb_publish(ORB_ID(vehicle_local_position), local_pos_pub, &local_pos);

                /* global position */
                global_pos.lat = gps.lat;
                global_pos.lon = gps.lon;
                global_pos.alt = gps.alt*1.0e-3d;
                global_pos.vx = gps.vel_n;
                global_pos.vy = gps.vel_e;
                global_pos.vz = gps.vel_d;

                orb_publish(ORB_ID(vehicle_global_position), global_pos_pub, &global_pos);
            }


            //  gps_covariance[0] = gps.eph; //TODO: needs scaling
            //  gps_covariance[1] = gps.eph;
            //  gps_covariance[2] = gps.epv;

            // } else {
            //  /* we can not use the gps signal (it is of low quality) */
            //  predict_only = 1;
            // }

            // //       predict_only = 0;                                                               //TODO: only for testing, removeme, XXX
            // //       z[0] = sinf(((float)counter)/180.0f*3.14159265f);                               //TODO: only for testing, removeme, XXX
            // //       usleep(100000);                                                                 //TODO: only for testing, removeme, XXX


            // /*Get new estimation (this is calculated in the plane) */
            // //TODO: if new_initialization == true: use 0,0,0, else use xapo
            // if (true == new_initialization) {                                                                                                //TODO,XXX: uncomment!
            //  xapo[0] = 0; //we have a new plane initialization. the current estimate is in the center of the plane
            //  xapo[2] = 0;
            //  xapo[4] = 0;
            //  position_estimator(u, z, xapo, Papo, gps_covariance, predict_only, xapo1, Papo1);

            // } else {
            //  position_estimator(u, z, xapo, Papo, gps_covariance, predict_only, xapo1, Papo1);
            // }



            // /* Copy values from xapo1 to xapo */
            // int i;

            // for (i = 0; i < N_STATES; i++) {
            //  xapo[i] = xapo1[i];
            // }

            // if ((counter % REPROJECTION_COUNTER_LIMIT == 0) || (counter % (PROJECTION_INITIALIZE_COUNTER_LIMIT - 1) == 0)) {
            //  /* Reproject from plane to geographic coordinate system */
            //  //      map_projection_reproject(xapo1[0], xapo1[2], map_scale, phi_1, lambda_0, &lat_current, &lon_current) //TODO,XXX: uncomment!
            //  map_projection_reproject(z[0], z[1], &lat_current, &lon_current); //do not use estimator for projection testing, removeme
            //  //      //DEBUG
            //  //          if(counter%500 == 0)
            //  //          {
            //  //              printf("phi_1: %.10f\n", phi_1);
            //  //              printf("lambda_0: %.10f\n", lambda_0);
            //  //              printf("lat_estimated: %.10f\n", lat_current);
            //  //              printf("lon_estimated: %.10f\n", lon_current);
            //  //              printf("z[0]=%.10f, z[1]=%.10f, z[2]=%f\n", z[0], z[1], z[2]);
            //  //              fflush(stdout);
            //  //
            //  //          }

            //  //          if(!isnan(lat_current) && !isnan(lon_current))// && !isnan(xapo1[4]) && !isnan(xapo1[1]) && !isnan(xapo1[3]) && !isnan(xapo1[5]))
            //  //          {
            //  /* send out */

            //  global_pos.lat = lat_current;
            //  global_pos.lon = lon_current;
            //  global_pos.alt = xapo1[4];
            //  global_pos.vx = xapo1[1];
            //  global_pos.vy = xapo1[3];
            //  global_pos.vz = xapo1[5];

                /* publish current estimate */
                // orb_publish(ORB_ID(vehicle_global_position), global_pos_pub, &global_pos);
                //          }
                //          else
                //          {
                //              printf("[position estimator] ERROR: nan values, lat_current=%.4f, lon_current=%.4f, z[0]=%.4f z[1]=%.4f\n", lat_current, lon_current, z[0], z[1]);
                //              fflush(stdout);
                //          }

            // }

            counter++;
        }

    }

    return 0;
}


