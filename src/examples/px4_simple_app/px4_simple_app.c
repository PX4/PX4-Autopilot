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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

#include <uORB/topics/transponder_report.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_local_position.h>

#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_command.h>

#include <drivers/drv_hrt.h>


__EXPORT int px4_simple_app_main(int argc, char *argv[]);

/*void fake_traffic(const char *callsign, float distance, float direction, float traffic_heading,
                 float altitude_diff, float hor_velocity, float ver_velocity)
{
    double lat, lon;
    waypoint_from_heading_and_distance(get_global_position()->lat, get_global_position()->lon, direction, distance, &lat,
                       &lon);
    float alt = get_global_position()->alt + altitude_diff;

    // float vel_n = get_global_position()->vel_n;
    // float vel_e = get_global_position()->vel_e;
    // float vel_d = get_global_position()->vel_d;

    transponder_report_s tr = {};
    tr.timestamp = hrt_absolute_time();
    tr.icao_address = 1234;
    tr.lat = lat; // Latitude, expressed as degrees
    tr.lon = lon; // Longitude, expressed as degrees
    tr.altitude_type = 0;
    tr.altitude = alt;
    tr.heading = traffic_heading; //-atan2(vel_e, vel_n); // Course over ground in radians
    tr.hor_velocity	= hor_velocity; //sqrtf(vel_e * vel_e + vel_n * vel_n); // The horizontal velocity in m/s
    tr.ver_velocity = ver_velocity; //-vel_d; // The vertical velocity in m/s, positive is up
    strncpy(&tr.callsign[0], callsign, sizeof(tr.callsign) - 1);
    tr.callsign[sizeof(tr.callsign) - 1] = 0;
    tr.emitter_type = 0; // Type from ADSB_EMITTER_TYPE enum
    tr.tslc = 2; // Time since last communication in seconds
    tr.flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS | transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
           transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY |
           transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE |
           transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN; // Flags to indicate various statuses including valid data fields
    tr.squawk = 6667;

    orb_advert_t h = orb_advertise_queue(ORB_ID(transponder_report), &tr, transponder_report_s::ORB_QUEUE_LENGTH);
    (void)orb_unadvertise(h);
}*/

int px4_simple_app_main(int argc, char *argv[])
{
   	 PX4_INFO("Hello Sky!");
     PX4_WARN("This is the px4_simple_app!");

    /* subscribe to topic */
    int _traffic_sub = orb_subscribe(ORB_ID(transponder_report));
   	struct transponder_report_s tr;
	orb_copy(ORB_ID(transponder_report), _traffic_sub, &tr);

    PX4_INFO("Transponder report:");
    PX4_WARN("Lat, Lon,ICAO Add: %f5 %f5 %u", tr.lat, tr.lon, tr.icao_address);
    PX4_WARN("Alt: %f %f %f %f", (double)tr.altitude, (double)tr.heading, (double)tr.hor_velocity, (double)tr.ver_velocity);
	PX4_WARN("flags: %d %d %u %u", tr.flags, tr.squawk, tr.altitude_type, tr.emitter_type);
	PX4_WARN("tslc:  %u", tr.tslc);

    /* subscribe to topic */
    int _global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
    struct vehicle_global_position_s gl;
    orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &gl);

    PX4_INFO("vehicl_global_position:");
    PX4_WARN("pos:  %f5 %f5", gl.lat, gl.lon);

    /* subscribe to topic */
    int _gps_pos_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
    struct vehicle_gps_position_s gps;
    orb_copy(ORB_ID(vehicle_gps_position), _gps_pos_sub, &gps);

    PX4_INFO("vehicl_gps_position:");
    PX4_WARN("pos:  %i %i", gps.lat, gps.lon);

    //_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));






    /*
    //Navigator::fake_traffic("LX007", 100, 1.0f, -1.0f, 100.0f, 90.0f, 0.001f);//500
    PX4_INFO("own new Faketraffic:");
    struct transponder_report_s trnew = {};
    trnew.timestamp = hrt_absolute_time();
    trnew.icao_address = 1234;
    trnew.lat = 47.0; // Latitude, expressed as degrees
    trnew.lon = 8.0; // Longitude, expressed as degrees
    trnew.altitude_type = 0;
    trnew.altitude = 51.0f;
    trnew.heading = 0.2f; //-atan2(vel_e, vel_n); // Course over ground in radians
    trnew.hor_velocity	= 10.0f; //sqrtf(vel_e * vel_e + vel_n * vel_n); // The horizontal velocity in m/s
    trnew.ver_velocity = 0.2f; //-vel_d; // The vertical velocity in m/s, positive is up
    //strncpy(&trnew.callsign[0], callsign, sizeof(trnew.callsign) - 1);
    //trnew.callsign[sizeof(trnew.callsign) - 1] = 0;
    trnew.emitter_type = 0; // Type from ADSB_EMITTER_TYPE enum
    trnew.tslc = 2; // Time since last communication in seconds
    trnew.flags = 271;
    trnew.squawk = 6667;

    //advertise attitude topic
    memset(&trnew, 0, sizeof(trnew));
    //orb_advert_t h = orb_advertise_queue(ORB_ID(transponder_report), &trnew, transponder_report_s::ORB_QUEUE_LENGTH);
    //(void)orb_unadvertise(h);

    orb_advert_t h = orb_advertise(ORB_ID(transponder_report), &trnew);
    orb_publish(ORB_ID(transponder_report), h, &trnew);*/









	    /* advertise attitude topic */
    struct vehicle_attitude_s att;
    memset(&att, 0, sizeof(att));
    orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);  

	orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);

  //  PX4_WARN("U: %f %f, %f %f",U(0),U(1),(double)Vu(0),(double)Vu(1));
   // PX4_WARN("I: %f %f, %f %f",U(0),I(1),(double)Vi(0),(double)Vi(1));

    PX4_INFO("exiting");

    return 0;
}
