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
 * @file HealthFlags.h
 *
 * Contains helper functions to efficiently set the system health flags from commander and preflight check.
 *
 * @author Philipp Oettershagen (philipp.oettershagen@mavt.ethz.ch)
 */

#pragma once

#include <px4_platform_common/log.h>
#include <uORB/topics/vehicle_status.h>

struct subsystem_info_s {
	// keep in sync with mavlink MAV_SYS_STATUS_SENSOR
	static constexpr uint64_t SUBSYSTEM_TYPE_GYRO = 1;
	static constexpr uint64_t SUBSYSTEM_TYPE_ACC = 2;
	static constexpr uint64_t SUBSYSTEM_TYPE_MAG = 4;
	static constexpr uint64_t SUBSYSTEM_TYPE_ABSPRESSURE = 8;
	static constexpr uint64_t SUBSYSTEM_TYPE_DIFFPRESSURE = 16;
	static constexpr uint64_t SUBSYSTEM_TYPE_GPS = 32;
	static constexpr uint64_t SUBSYSTEM_TYPE_OPTICALFLOW = 64;
	static constexpr uint64_t SUBSYSTEM_TYPE_CVPOSITION = 128;
	static constexpr uint64_t SUBSYSTEM_TYPE_LASERPOSITION = 256;
	static constexpr uint64_t SUBSYSTEM_TYPE_EXTERNALGROUNDTRUTH = 512;
	static constexpr uint64_t SUBSYSTEM_TYPE_ANGULARRATECONTROL = 1024;
	static constexpr uint64_t SUBSYSTEM_TYPE_ATTITUDESTABILIZATION = 2048;
	static constexpr uint64_t SUBSYSTEM_TYPE_YAWPOSITION = 4096;
	static constexpr uint64_t SUBSYSTEM_TYPE_ALTITUDECONTROL = 8192;
	static constexpr uint64_t SUBSYSTEM_TYPE_POSITIONCONTROL = 16384;
	static constexpr uint64_t SUBSYSTEM_TYPE_MOTORCONTROL = 32768;
	static constexpr uint64_t SUBSYSTEM_TYPE_RCRECEIVER = 65536;
	static constexpr uint64_t SUBSYSTEM_TYPE_GYRO2 = 131072;
	static constexpr uint64_t SUBSYSTEM_TYPE_ACC2 = 262144;
	static constexpr uint64_t SUBSYSTEM_TYPE_MAG2 = 524288;
	static constexpr uint64_t SUBSYSTEM_TYPE_GEOFENCE = 1048576;
	static constexpr uint64_t SUBSYSTEM_TYPE_AHRS = 2097152;
	static constexpr uint64_t SUBSYSTEM_TYPE_TERRAIN = 4194304;
	static constexpr uint64_t SUBSYSTEM_TYPE_REVERSEMOTOR = 8388608;
	static constexpr uint64_t SUBSYSTEM_TYPE_LOGGING = 16777216;
	static constexpr uint64_t SUBSYSTEM_TYPE_SENSORBATTERY = 33554432;
	static constexpr uint64_t SUBSYSTEM_TYPE_SENSORPROXIMITY = 67108864;
	static constexpr uint64_t SUBSYSTEM_TYPE_SATCOM = 134217728;
	static constexpr uint64_t SUBSYSTEM_TYPE_PREARM_CHECK = 268435456;
	static constexpr uint64_t SUBSYSTEM_TYPE_OBSTACLE_AVOIDANCE = 536870912;
};

void set_health_flags(uint64_t subsystem_type, bool present, bool enabled, bool ok, vehicle_status_s &status);
void set_health_flags_present_healthy(uint64_t subsystem_type, bool present, bool healthy, vehicle_status_s &status);
void set_health_flags_healthy(uint64_t subsystem_type, bool healthy, vehicle_status_s &status);
void print_health_flags(const vehicle_status_s &status);
