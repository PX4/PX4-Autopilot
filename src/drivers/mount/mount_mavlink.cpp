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
 * @file mount_mavlink.c
 * @author Leon Müllåer (thedevleon)
 *
 */

 #include "mount_mavlink.h"

 #include <stdlib.h>
 #include <stdio.h>
 #include <string.h>
 #include <arch/math.h>
 #include <geo/geo.h>

 #include <uORB/uORB.h>
 #include <uORB/topics/vehicle_command.h>
 #include <uORB/topics/vehicle_roi.h>

 /* uORB advertising */
 static struct vehicle_command_s *vehicle_command;
 static orb_advert_t vehicle_command_pub;

 static int sys_id;
 static int comp_id;

 bool mount_mavlink_init(int sysid, int compid)
 {
    sys_id = sysid;
    comp_id = compid;
    memset(&vehicle_command, 0, sizeof(vehicle_command));
    vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &vehicle_command);
    if(!vehicle_command_pub) return false;
    return true;
 }


 void mount_mavlink_deinit()
 {
    free(vehicle_command);
 }

 /*
  * MAV_CMD_DO_MOUNT_CONFIGURE (#204)
  *   param1 = mount_mode (1 = MAV_MOUNT_MODE_NEUTRAL recenters camera)
  */

 void mount_mavlink_configure(int roi_mode, bool man_control)
 {
    vehicle_command->command = vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE;
    vehicle_command->target_system = sys_id;
    vehicle_command->target_component = comp_id;

    switch (roi_mode) {
        case vehicle_roi_s::VEHICLE_ROI_NONE:
            if(man_control) vehicle_command->param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_RC_TARGETING;
            else vehicle_command->param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL;
            break;
        case vehicle_roi_s::VEHICLE_ROI_WPNEXT:
            vehicle_command->param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING;
            break;
        case vehicle_roi_s::VEHICLE_ROI_WPINDEX:
            vehicle_command->param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING;
            break;
        case vehicle_roi_s::VEHICLE_ROI_LOCATION:
            vehicle_command->param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING;
            break;
        case vehicle_roi_s::VEHICLE_ROI_TARGET:
            vehicle_command->param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING;
            break;
        default:
            vehicle_command->param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL;
            break;
    }

    orb_publish(ORB_ID(vehicle_command), vehicle_command_pub, vehicle_command);
 }


 /* MAV_CMD_DO_MOUNT_CONTROL (#205)
  *  param1 = pitch, angle in degree or pwm input
  *  param2 = roll, angle in degree or pwm input
  *  param3 = yaw, angle in degree or pwm input
  *  param6 = typeflags (uint16_t)
  *   0x0001: pitch, 0: unlimited, 1: limited, see CMD_SETANGLE
  *   0x0002: roll, 0: unlimited, 1: limited, see CMD_SETANGLE
  *   0x0004: yaw, 0: unlimited, 1: limited, see CMD_SETANGLE
  *   0x0100: input type, 0: angle input (see CMD_SETANGLE), 1: pwm input (see CMD_SETPITCHROLLYAW)
  */


 void mount_mavlink_point_location(float x, float y, float z)
 {
    vehicle_command->command = vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL;
    vehicle_command->target_system = sys_id;
    vehicle_command->target_component = comp_id;

    orb_publish(ORB_ID(vehicle_command), vehicle_command_pub, vehicle_command);
 }

 void mount_mavlink_point_manual(float roll, float pitch, float yaw)
 {
    vehicle_command->command = vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL;
    vehicle_command->target_system = sys_id;
    vehicle_command->target_component = comp_id;

    orb_publish(ORB_ID(vehicle_command), vehicle_command_pub, vehicle_command);
 }
