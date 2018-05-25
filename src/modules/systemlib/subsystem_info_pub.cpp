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
 * Contains helper functions to efficiently publish the subsystem_info messages from various locations inside the code.
 *
 * @author Philipp Oettershagen (philipp.oettershagen@mavt.ethz.ch)
 */

#include "subsystem_info_pub.h"
#include <uORB/topics/subsystem_info.h>

static orb_advert_t pub = nullptr;
struct subsystem_info_s info = {};
struct vehicle_status_s status;


void publish_subsystem_info(uint64_t subsystem_type, bool present, bool enabled, bool ok)
{
	PX4_INFO("publish_subsystem_info: Type %llu pres=%u enabl=%u ok=%u", subsystem_type, present, enabled, ok);

	// Keep a local copy of the status flags. Because we use queuing, it could be that the flags in the vehicle_status topics are
	// not up to date. When using those publish_subsystem_info_xxx functions that only write a subset of health flags but leave others
	// unchanged, we'd write outdated health flags to vehicle_status. Having an up to date local copy resolves that issue.
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

	/* Publish the subsystem_info message */
	info.present = present;
	info.enabled = enabled;
	info.ok = ok;
	info.subsystem_type = subsystem_type;

	if (pub != nullptr) {
		orb_publish(ORB_ID(subsystem_info), pub, &info);

	} else {
		pub = orb_advertise_queue(ORB_ID(subsystem_info), &info, subsystem_info_s::ORB_QUEUE_LENGTH);
	}
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

// Local helper functions
bool getPresent(uint64_t subsystem_type)
{
	return status.onboard_control_sensors_present & (uint32_t)subsystem_type;
}
bool getEnabled(uint64_t subsystem_type)
{
	return status.onboard_control_sensors_enabled & (uint32_t)subsystem_type;
}
bool getHealthy(uint64_t subsystem_type)
{
	return status.onboard_control_sensors_health & (uint32_t)subsystem_type;
}
