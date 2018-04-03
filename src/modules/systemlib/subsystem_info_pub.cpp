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
 * @file subsystem_info_pub.cpp
 *
 * Contains helper functions to efficiently publish the subsystem_info messages from various locations inside the code. It is basically a
 * helper function for commander. Approach:
 * 	- Before commander starts (which happens after some of the drivers have already published the respective subsystem_info), this helper
 * 	  code stores all requests for a publish_subsystem_info in the internal_status variable
 * 	- When commander starts up, it calls the publish_subsystem_info_init function. This 1) copies the internal_status into commander's
 * 	  vehicle status variable and 2) assigns the status pointer to commanders vehicle status
 * 	- After that, all requests to publish_subsystem_info are directly written to commander's vehicle status such that it is always up
 * 	  to date. Commander then publishes the vehicle_status uORB (and is in fact the only app that does that, which is why this approach works)
 *
 * @author Philipp Oettershagen (philipp.oettershagen@mavt.ethz.ch)
 */

#include "subsystem_info_pub.h"

vehicle_status_s internal_status = {};
vehicle_status_s *status = &internal_status;
bool *status_changed = nullptr;

/* initialize pointer to commander's vehicle status variable */
void publish_subsystem_info_init(vehicle_status_s *commander_vehicle_status_ptr, bool *commander_status_changed_ptr)
{
	status = commander_vehicle_status_ptr;
	status_changed = commander_status_changed_ptr;

	status->onboard_control_sensors_present = internal_status.onboard_control_sensors_present;
	status->onboard_control_sensors_enabled = internal_status.onboard_control_sensors_enabled;
	status->onboard_control_sensors_health = internal_status.onboard_control_sensors_health;
	*status_changed = true;
}

/* Writes the full state information for a specific subsystem type, either directly into commander's vehicle
 * status variable or into an internal variable that is later copied to commander's vehicle status variable*/
void publish_subsystem_info(uint64_t subsystem_type, bool present, bool enabled, bool ok)
{
	PX4_DEBUG("publish_subsystem_info (ext:%u): Type %llu pres=%u enabl=%u ok=%u", status != &internal_status,
		  subsystem_type, present, enabled, ok);

	if (present) {
		status->onboard_control_sensors_present |= (uint32_t)subsystem_type;

	} else {
		status->onboard_control_sensors_present &= ~(uint32_t)subsystem_type;
	}

	if (enabled) {
		status->onboard_control_sensors_enabled |= (uint32_t)subsystem_type;

	} else {
		status->onboard_control_sensors_enabled &= ~(uint32_t)subsystem_type;
	}

	if (ok) {
		status->onboard_control_sensors_health |= (uint32_t)subsystem_type;

	} else {
		status->onboard_control_sensors_health &= ~(uint32_t)subsystem_type;
	}

	if (status != &internal_status) { *status_changed = true; }
}

void publish_subsystem_info_present_healthy(uint64_t subsystem_type, bool present, bool healthy)
{
	publish_subsystem_info(subsystem_type, present, getEnabled(subsystem_type), healthy);
}

void publish_subsystem_info_present_enabled(uint64_t subsystem_type, bool present, bool enabled)
{
	publish_subsystem_info(subsystem_type, present, enabled, getHealthy(subsystem_type));
}

void publish_subsystem_info_enabled_healthy(uint64_t subsystem_type, bool enabled, bool ok)
{
	publish_subsystem_info(subsystem_type, getPresent(subsystem_type), enabled, ok);
}

void publish_subsystem_info_enabled(uint64_t subsystem_type, bool enabled)
{
	publish_subsystem_info(subsystem_type, getPresent(subsystem_type), enabled, getHealthy(subsystem_type));
}

void publish_subsystem_info_healthy(uint64_t subsystem_type, bool ok)
{
	publish_subsystem_info(subsystem_type, getPresent(subsystem_type), getEnabled(subsystem_type), ok);
}

void publish_subsystem_info_print()
{
	uint64_t type = 1;

	for (int i = 1; i < 31; i++) {
		PX4_DEBUG("subsystem_info: Type %llu pres=%u enabl=%u ok=%u", type,
			  (status->onboard_control_sensors_present & (uint32_t)type) > 0,
			  (status->onboard_control_sensors_enabled & (uint32_t)type) > 0,
			  (status->onboard_control_sensors_health & (uint32_t)type) > 0);
		type = type * 2;
	}
}

// Local helper functions
bool getPresent(uint64_t subsystem_type)
{
	return status->onboard_control_sensors_present & (uint32_t)subsystem_type;
}
bool getEnabled(uint64_t subsystem_type)
{
	return status->onboard_control_sensors_enabled & (uint32_t)subsystem_type;
}
bool getHealthy(uint64_t subsystem_type)
{
	return status->onboard_control_sensors_health & (uint32_t)subsystem_type;
}
