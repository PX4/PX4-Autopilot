/****************************************************************************
*
*   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file mount_rc.c
 * @author Leon Müller (thedevleon)
 *
 */

 #include "mount_rc.h"

 #include <stdlib.h>
 #include <stdio.h>
 #include <string.h>
 #include <arch/math.h>
 #include <geo/geo.h>

 #include <uORB/uORB.h>
 #include <uORB/topics/vehicle_roi.h>

bool mount_rc_init()
{
        //TODO
        return false;
}

void mount_rc_deinit()
{
        //TODO
}

void mount_rc_configure(int roi_mode, bool man_control)
{
        switch (roi_mode) {
        case vehicle_roi_s::VEHICLE_ROI_NONE:
                if(!man_control) {mount_rc_point_manual(0.0f, 0.0f, 0.0f); }
                break;
        case vehicle_roi_s::VEHICLE_ROI_WPNEXT:
                break;
        case vehicle_roi_s::VEHICLE_ROI_WPINDEX:
                break;
        case vehicle_roi_s::VEHICLE_ROI_LOCATION:
                break;
        case vehicle_roi_s::VEHICLE_ROI_TARGET:
                break;
        default:
                mount_rc_point_manual(0.0f, 0.0f, 0.0f);
                break;
        }
}

void mount_rc_point_location(double global_lat, double global_lon, float global_alt, double lat, double lon, float alt)
{
    float yaw = get_bearing_to_next_waypoint(global_lat, global_lon, lat, lon);
    float pitch = 0.0f;
    float roll = 0.0f;

    mount_rc_point_manual(pitch, roll, yaw);
}

void mount_rc_point_manual(float pitch, float roll, float yaw)
{
        //TODO
}
