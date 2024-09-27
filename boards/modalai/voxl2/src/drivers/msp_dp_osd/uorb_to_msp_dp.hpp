/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

/* uorb_to_msp_dp.hpp
 *
 * Declaration of functions which translate UORB messages into MSP DisplayPort specific structures.
 */

#pragma once

// basic types
#include <cmath>

// UORB topic structs
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/power_monitor.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/log_message.h>
#include <uORB/topics/parameter_selector.h>
#include <uORB/topics/manual_control_setpoint.h>

// PX4 events interface
#include <px4_platform_common/events.h>

// MSP structs
#include "msp_dp_defines.h"
#include "msp_osd_symbols.h"
#include <drivers/osd/msp_osd/msp_defines.h>
#include <drivers/osd/msp_osd/MessageDisplay/MessageDisplay.hpp>

namespace msp_dp_osd
{

// Construct a vtx config struct
msp_dp_vtx_config_t construct_vtx_config(uint8_t band, uint8_t channel);

// Construct a status struct
msp_dp_status_t construct_status(const vehicle_status_s &vehicle_status);

// Construct a RC struct
msp_rc_t construct_RC(const input_rc_s &input_rc, const msp_dp_rc_sticks_t &sticks);

// Construct a canvas struct
msp_dp_canvas_t construct_OSD_canvas(uint8_t row, uint8_t col);

// Construct a heartbeat command
displayportMspCommand_e construct_OSD_heartbeat();

// Construct a release command
displayportMspCommand_e construct_OSD_release();

// Construct a clear command
displayportMspCommand_e construct_OSD_clear();

// Construct a write struct given a string
uint8_t construct_OSD_write(uint8_t col, uint8_t row, bool blink, const char *string, uint8_t *output, uint8_t len);

// Construct a draw command
displayportMspCommand_e construct_OSD_draw();

// Construct a config command
msp_dp_config_t construct_OSD_config(resolutionType_e resolution, uint8_t fontType);

// Construct Flight mode Message
const char* construct_flight_mode(const vehicle_status_s &vehicle_status);

// Generate bearing symbol from direction
uint8_t get_symbol_from_bearing(float bearing);

// Convert warning message to upper case so OSD interprets the message as letters instead of symbols
void log_msg_to_upper(char* string);

} // namespace msp_dp_osd
