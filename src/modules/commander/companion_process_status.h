/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file companion_process_status.h
 * Routine to determine health of companion processes
 */

#pragma once

#include <uORB/topics/companion_process_status.h>
#include <uORB/Subscription.hpp>
#include <systemlib/mavlink_log.h>

using namespace time_literals;

enum class ProcessType {	//used as indexes for _companion_process_status arrays
	Avoidance,
	VIO,
	Count
};

enum class MAV_COMPONENT {	//see mavlink::common::MAV_COMPONENT
	MAV_COMP_ID_OBSTACLE_AVOIDANCE = 196,
	MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY = 197
};

enum class MAV_STATE {	//strings used for user notifications
	UNINITIALIZED,
	STARTING,
	CALIBRATING,
	STANDBY,
	ACTIVE,
	TIMEOUT,
	EMERGENCY,
	POWEROFF,
	ABORTING
};


class CompanionProcessStatus
{

public:
	CompanionProcessStatus();
	void check_companion_process_status(orb_advert_t *mav_log_pub);

private:
	uORB::Subscription<companion_process_status_s> _companion_process_status_sub{ORB_ID(companion_process_status)};
	companion_process_status_s _companion_process_status_history[(int)ProcessType::Count]{};	/**< last status message of each process*/
	int _first_registration_time[(int)ProcessType::Count]{};	/**< time at which a process reported for the first time */

	bool _avoidance_required = false;
	bool _vio_required = false;
	ProcessType _process_type;


	hrt_abstime _time_zero;	/**< time of object construction*/
	hrt_abstime _time_message;	/**< time when last message was printed*/
	bool _new_status_received = false;

	static constexpr hrt_abstime STARTUP_TIMEOUT = 15_s;	/**< timeout for starting the companion process. counter starts when first message of starting is received */
	static constexpr hrt_abstime NO_SIGNAL_TIMEOUT = 15_s;	/**< timeout if no signal is received. counter starts when class object is created */
	static constexpr hrt_abstime THROTTLE_MESSAGES = 5_s;	/**< time interval on which messages are published */

	void poll_subscriptions();
	void determine_required_processes();
	void determine_action();
	const char* toString(ProcessType type);
	const char* toString(MAV_STATE state);
};
