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
* @file vmount_params.c
* @author Leon Müller (thedevleon)
*
*/

#include <px4_config.h>
#include <systemlib/param/param.h>

/**
* Mount operation mode
* MAVLINK and RC use the ROI (or RC input if enabled) to control a gimbal.
* ONBOARD uses the vehicle_mount topic to control a rc gimbal.
* @value 0 DISABLE
* @value 1 MAVLINK
* @value 2 RC
* @value 3 ONBOARD
* @min 0
* @max 3
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_MODE, 0);

/**
* Auxiliary channel to override current mount mode (only in ONBOARD mode)
* if <0.0f then MODE_RETRACT
* if =0.0f then don't override
* if >0.0f then MODE_RC_TARGETING
* (if for example )
* @value 0 Disable
* @value 1 AUX1
* @value 2 AUX2
* @value 3 AUX3
* @value 4 AUX4
* @value 5 AUX5
* @min 0
* @max 5
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_MODE_OVR, 0);

/**
* Mavlink System ID
*
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_MAV_SYSID, 71);

/**
* Mavlink Component ID
*
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_MAV_COMPID, 67);

/**
* Do we want to allow the mount to be controlled when no ROI is set?
*
* If set to 1, the mount will be controlled by the rc channels below.
*
* @boolean
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_MAN_CONTROL, 0);

/**
* Auxiliary channel to control roll.
*
* @value 0 Disable
* @value 1 AUX1
* @value 2 AUX2
* @value 3 AUX3
* @min 0
* @max 3
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_MAN_ROLL, 0);

/**
* Auxiliary channel to control pitch.
*
* @value 0 Disable
* @value 1 AUX1
* @value 2 AUX2
* @value 3 AUX3
* @value 4 AUX4
* @value 5 AUX5
* @min 0
* @max 5
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_MAN_PITCH, 0);

/**
* Auxiliary channel to control yaw.
*
* @value 0 Disable
* @value 1 AUX1
* @value 2 AUX2
* @value 3 AUX3
* @value 4 AUX4
* @value 5 AUX5
* @min 0
* @max 5
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_MAN_YAW, 0);
