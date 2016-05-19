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
 * @file vmount_onboard
 * @author Leon Müller (thedevleon)
 *
 */

#ifndef _VMOUNT_ONBOARD_H
#define _VMOUNT_ONBOARD_H

#include <sys/types.h>
#include <stdbool.h>
#include <uORB/topics/vehicle_attitude.h>

// Public functions
bool vmount_onboard_init(void);
void vmount_onboard_deinit(void);
void vmount_onboard_configure(int new_mount_mode, bool new_stab_roll, bool new_stab_pitch, bool new_stab_yaw);
void vmount_onboard_set_location(double lat, double lon, float alt);
void vmount_onboard_set_manual(double pitch_new, double roll_new, float yaw_new);
void vmount_onboard_set_mode(int new_mount_mode);
void vmount_onboard_point(double global_lat, double global_lon, float global_alt);
void vmount_onboard_point_manual(float pitch_target, float roll_target, float yaw_target);
void vmount_onboard_update_attitude(vehicle_attitude_s vehicle_attitude_new);
float vmount_onboard_calculate_pitch(double global_lat, double global_lon, float global_alt);

#endif /* _VMOUNT_ONBOARD_H */
