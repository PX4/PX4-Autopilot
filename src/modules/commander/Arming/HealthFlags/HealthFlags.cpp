/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file HealthFlags.cpp
 *
 * Contains helper functions to efficiently set the system health flags from commander and preflight check.
 *
 * @author Philipp Oettershagen (philipp.oettershagen@mavt.ethz.ch)
 */

#include "HealthFlags.h"
#include <uORB/topics/subsystem_info.h>

void set_health_flags(uint64_t subsystem_type, bool present, bool enabled, bool ok, vehicle_status_s &status)
{
	PX4_DEBUG("set_health_flags: Type %llu pres=%u enabl=%u ok=%u", subsystem_type, present, enabled, ok);

	if (present) {
		status.onboard_control_sensors_present |= (uint32_t)subsystem_type;

	} else {
		status.onboard_control_sensors_present &= ~(uint32_t)subsystem_type;
	}

	if (enabled) {
		status.onboard_control_sensors_enabled |= (uint32_t)subsystem_type;

	} else {
		status.onboard_control_sensors_enabled &= ~(uint32_t)subsystem_type;
	}

	if (ok) {
		status.onboard_control_sensors_health |= (uint32_t)subsystem_type;

	} else {
		status.onboard_control_sensors_health &= ~(uint32_t)subsystem_type;
	}
}

void set_health_flags_present_healthy(uint64_t subsystem_type, bool present, bool healthy, vehicle_status_s &status)
{
	set_health_flags(subsystem_type, present, status.onboard_control_sensors_enabled & (uint32_t)subsystem_type, healthy,
			 status);
}

void set_health_flags_healthy(uint64_t subsystem_type, bool healthy, vehicle_status_s &status)
{
	set_health_flags(subsystem_type, status.onboard_control_sensors_present & (uint32_t)subsystem_type,
			 status.onboard_control_sensors_enabled & (uint32_t)subsystem_type, healthy, status);
}

#define _print_sub(name, bit) PX4_INFO(name":\t\t%s\t%s", \
				       (status.onboard_control_sensors_enabled & bit) ? "EN" : " ", \
				       (status.onboard_control_sensors_present & bit) ? ((status.onboard_control_sensors_health & bit) ? "OK" : "ERR") : (status.onboard_control_sensors_enabled & bit) ? "OFF" : "")

void print_health_flags(vehicle_status_s &status)
{
	PX4_INFO("DEVICE\t\tSTATUS");
	PX4_INFO("----------------------------------");
	_print_sub("GYRO", subsystem_info_s::SUBSYSTEM_TYPE_GYRO);
	_print_sub("ACC", subsystem_info_s::SUBSYSTEM_TYPE_ACC);
	_print_sub("MAG", subsystem_info_s::SUBSYSTEM_TYPE_MAG);
	_print_sub("PRESS", subsystem_info_s::SUBSYSTEM_TYPE_ABSPRESSURE);
	_print_sub("AIRSP", subsystem_info_s::SUBSYSTEM_TYPE_DIFFPRESSURE);
	_print_sub("GPS", subsystem_info_s::SUBSYSTEM_TYPE_GPS);
	_print_sub("OPT", subsystem_info_s::SUBSYSTEM_TYPE_OPTICALFLOW);
	_print_sub("VIO", subsystem_info_s::SUBSYSTEM_TYPE_CVPOSITION);
	_print_sub("LASER", subsystem_info_s::SUBSYSTEM_TYPE_LASERPOSITION);
	_print_sub("GTRUTH", subsystem_info_s::SUBSYSTEM_TYPE_EXTERNALGROUNDTRUTH);
	_print_sub("RATES", subsystem_info_s::SUBSYSTEM_TYPE_ANGULARRATECONTROL);
	_print_sub("ATT", subsystem_info_s::SUBSYSTEM_TYPE_ATTITUDESTABILIZATION);
	_print_sub("YAW", subsystem_info_s::SUBSYSTEM_TYPE_YAWPOSITION);
	_print_sub("ALTCTL", subsystem_info_s::SUBSYSTEM_TYPE_ALTITUDECONTROL);
	_print_sub("POS", subsystem_info_s::SUBSYSTEM_TYPE_POSITIONCONTROL);
	_print_sub("MOT", subsystem_info_s::SUBSYSTEM_TYPE_MOTORCONTROL);
	_print_sub("RC  ", subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER);
	_print_sub("GYRO2", subsystem_info_s::SUBSYSTEM_TYPE_GYRO2);
	_print_sub("ACC2", subsystem_info_s::SUBSYSTEM_TYPE_ACC2);
	_print_sub("MAG2", subsystem_info_s::SUBSYSTEM_TYPE_MAG2);
	_print_sub("GEOFENCE", subsystem_info_s::SUBSYSTEM_TYPE_GEOFENCE);
	_print_sub("AHRS", subsystem_info_s::SUBSYSTEM_TYPE_AHRS);
	_print_sub("TERRAIN", subsystem_info_s::SUBSYSTEM_TYPE_TERRAIN);
	_print_sub("REVMOT", subsystem_info_s::SUBSYSTEM_TYPE_REVERSEMOTOR);
	_print_sub("LOGGIN", subsystem_info_s::SUBSYSTEM_TYPE_LOGGING);
	_print_sub("BATT", subsystem_info_s::SUBSYSTEM_TYPE_SENSORBATTERY);
	_print_sub("PROX", subsystem_info_s::SUBSYSTEM_TYPE_SENSORPROXIMITY);
	_print_sub("SATCOM", subsystem_info_s::SUBSYSTEM_TYPE_SATCOM);
	_print_sub("PREARM", subsystem_info_s::SUBSYSTEM_TYPE_PREARM_CHECK);
	_print_sub("OBSAVD", subsystem_info_s::SUBSYSTEM_TYPE_OBSTACLE_AVOIDANCE);
}
