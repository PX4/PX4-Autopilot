/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file px4_includes.h
 *
 * Includes headers depending on the build target
 */

#pragma once

#include <stdbool.h>

#if defined(__PX4_ROS)
/*
 * Building for running within the ROS environment
 */

#ifdef __cplusplus
#include "ros/ros.h"
#include <px4_rc_channels.h>
#include <px4_vehicle_attitude.h>
#include <px4_vehicle_attitude_setpoint.h>
#include <px4_manual_control_setpoint.h>
#include <px4_actuator_controls.h>
#include <px4_vehicle_rates_setpoint.h>
#include <px4_mc_virtual_rates_setpoint.h>
#include <px4_vehicle_attitude.h>
#include <px4_vehicle_control_mode.h>
#include <px4_actuator_armed.h>
#include <px4_parameter_update.h>
#include <px4_vehicle_status.h>
#include <px4_vehicle_local_position_setpoint.h>
#include <px4_vehicle_local_position.h>
#include <px4_position_setpoint_triplet.h>
#include <px4_offboard_control_mode.h>
#include <px4_vehicle_force_setpoint.h>
#endif

#elif defined(__PX4_NUTTX)
/*
 * Building for NuttX
 */
#include <nuttx/config.h>
#include <uORB/uORB.h>
#ifdef __cplusplus
#include <platforms/nuttx/px4_messages/px4_rc_channels.h>
#include <platforms/nuttx/px4_messages/px4_vehicle_attitude_setpoint.h>
#include <platforms/nuttx/px4_messages/px4_manual_control_setpoint.h>
#include <platforms/nuttx/px4_messages/px4_actuator_controls.h>
#include <platforms/nuttx/px4_messages/px4_vehicle_rates_setpoint.h>
#include <platforms/nuttx/px4_messages/px4_vehicle_attitude.h>
#include <platforms/nuttx/px4_messages/px4_vehicle_control_mode.h>
#include <platforms/nuttx/px4_messages/px4_actuator_armed.h>
#include <platforms/nuttx/px4_messages/px4_parameter_update.h>
#include <platforms/nuttx/px4_messages/px4_vehicle_status.h>
#include <platforms/nuttx/px4_messages/px4_vehicle_local_position_setpoint.h>
#include <platforms/nuttx/px4_messages/px4_vehicle_local_position.h>
#include <platforms/nuttx/px4_messages/px4_position_setpoint_triplet.h>
#include <platforms/nuttx/px4_messages/px4_offboard_control_mode.h>
#include <platforms/nuttx/px4_messages/px4_vehicle_force_setpoint.h>
#include <platforms/nuttx/px4_messages/px4_camera_trigger.h>
#endif
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>

#elif defined(__PX4_POSIX) && !defined(__PX4_QURT)
#include <string.h>
#include <assert.h>
#include <uORB/uORB.h>

#define ASSERT(x) assert(x)

#ifdef __cplusplus
#include <platforms/posix/px4_messages/px4_rc_channels.h>
#include <platforms/posix/px4_messages/px4_vehicle_attitude_setpoint.h>
#include <platforms/posix/px4_messages/px4_manual_control_setpoint.h>
#include <platforms/posix/px4_messages/px4_actuator_controls.h>
#include <platforms/posix/px4_messages/px4_vehicle_rates_setpoint.h>
#include <platforms/posix/px4_messages/px4_vehicle_attitude.h>
#include <platforms/posix/px4_messages/px4_vehicle_control_mode.h>
#include <platforms/posix/px4_messages/px4_actuator_armed.h>
#include <platforms/posix/px4_messages/px4_parameter_update.h>
#include <platforms/posix/px4_messages/px4_vehicle_status.h>
#include <platforms/posix/px4_messages/px4_vehicle_local_position_setpoint.h>
#include <platforms/posix/px4_messages/px4_vehicle_local_position.h>
#include <platforms/posix/px4_messages/px4_position_setpoint_triplet.h>
#endif
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#elif defined(__PX4_QURT)
#include <string.h>
#include <assert.h>
#include <uORB/uORB.h>

#define ASSERT(x) assert(x)

#ifdef __cplusplus
#include <platforms/qurt/px4_messages/px4_rc_channels.h>
#include <platforms/qurt/px4_messages/px4_vehicle_attitude_setpoint.h>
#include <platforms/qurt/px4_messages/px4_manual_control_setpoint.h>
#include <platforms/qurt/px4_messages/px4_actuator_controls.h>
#include <platforms/qurt/px4_messages/px4_vehicle_rates_setpoint.h>
#include <platforms/qurt/px4_messages/px4_vehicle_attitude.h>
#include <platforms/qurt/px4_messages/px4_vehicle_control_mode.h>
#include <platforms/qurt/px4_messages/px4_actuator_armed.h>
#include <platforms/qurt/px4_messages/px4_parameter_update.h>
#include <platforms/qurt/px4_messages/px4_vehicle_status.h>
#include <platforms/qurt/px4_messages/px4_vehicle_local_position_setpoint.h>
#include <platforms/qurt/px4_messages/px4_vehicle_local_position.h>
#include <platforms/qurt/px4_messages/px4_position_setpoint_triplet.h>
#endif
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#else
#error "No target platform defined"
#endif
