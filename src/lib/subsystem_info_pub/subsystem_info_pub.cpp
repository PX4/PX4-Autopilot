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

orb_advert_t subsys_pub = nullptr;
int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

void publish_subsystem_info(uint64_t subsystem_type, bool present, bool enabled, bool ok, vehicle_status_s *status_ptr)
{
	PX4_DEBUG("publish_subsystem_info (method:%s): Type %llu pres=%u enabl=%u ok=%u",
		  status_ptr == nullptr ? "publish" : "local", subsystem_type, present, enabled, ok);

	if (status_ptr == nullptr) {
		/* Publish the subsystem_info message via uORB. This is the case when this
		 * function was called from a module outside of commander */
		struct subsystem_info_s subsys_info = {};
		subsys_info.present = present;
		subsys_info.enabled = enabled;
		subsys_info.ok = ok;
		subsys_info.subsystem_type = subsystem_type;

		if (subsys_pub != nullptr) {
			orb_publish(ORB_ID(subsystem_info), subsys_pub, &subsys_info);

		} else {
			subsys_pub = orb_advertise_queue(ORB_ID(subsystem_info), &subsys_info, subsystem_info_s::ORB_QUEUE_LENGTH);
		}

	} else {
		/* Update locally, i.e. directly using the supplied status_ptr. This happens
		 * when this function was called from inside commander*/
		if (present) {
			status_ptr->onboard_control_sensors_present |= (uint32_t)subsystem_type;

		} else {
			status_ptr->onboard_control_sensors_present &= ~(uint32_t)subsystem_type;
		}

		if (enabled) {
			status_ptr->onboard_control_sensors_enabled |= (uint32_t)subsystem_type;

		} else {
			status_ptr->onboard_control_sensors_enabled &= ~(uint32_t)subsystem_type;
		}

		if (ok) {
			status_ptr->onboard_control_sensors_health |= (uint32_t)subsystem_type;

		} else {
			status_ptr->onboard_control_sensors_health &= ~(uint32_t)subsystem_type;
		}
	}
}

void publish_subsystem_info_present_healthy(uint64_t subsystem_type, bool present, bool healthy,
		vehicle_status_s *status_ptr)
{
	struct vehicle_status_s status;
	orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &status);
	publish_subsystem_info(subsystem_type, present, status.onboard_control_sensors_enabled & (uint32_t)subsystem_type,
			       healthy, status_ptr);
}

void publish_subsystem_info_healthy(uint64_t subsystem_type, bool ok, vehicle_status_s *status_ptr)
{
	struct vehicle_status_s status;
	orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &status);
	publish_subsystem_info(subsystem_type, status.onboard_control_sensors_present & (uint32_t)subsystem_type,
			       status.onboard_control_sensors_enabled & (uint32_t)subsystem_type, ok, status_ptr);
}
