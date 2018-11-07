/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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

#ifndef COMPANION_PROCESS_STATUS_H_
#define COMPANION_PROCESS_STATUS_H_

#include <stdint.h>
#include <uORB/uORB.h>
#include <uORB/topics/companion_process_status.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_time.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cmath>
#include <uORB/Subscription.hpp>
#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>
#include <cstring>

using uORB::Subscription;
using namespace time_literals;


class Companion_Process_Status
{

private:
	Subscription<companion_process_status_s>	_companion_process_status_sub{ORB_ID(companion_process_status)};
	companion_process_status_s					_companion_process_status{};								/**< current status message */
	companion_process_status_s					_companion_process_status_history[10]{};					/**< last status message of each process*/
	companion_process_status_s					_companion_process_status_first_registration[10]{};			/**< first status message of each process */
	int									_required_processes[3]{0};							/**< index corresponds to process type (see _companion_process_types).
																							value corresponds to number of required processes of that type*/
	hrt_abstime 						_time_zero;											/**< time of object construction*/
	hrt_abstime 						_time_message;										/**< time when last message was printed*/
	bool 								_new_status_received = false;

	const hrt_abstime 					STARTUP_TIMEOUT = 15_s;		/**< timeout for starting the companion process. counter starts when first message of starting is received */
	const hrt_abstime					NO_SIGNAL_TIMEOUT = 15_s;	/**< timeout if no signal is received. counter starts when class object is created */
	const hrt_abstime 					THROTTLE_MESSAGES = 5_s;	/**< time interval on which messages are published */

	const char *_companion_process_types[3] = {"GENERIC", "AVOIDANCE", "VIO"};
	const char *_companion_process_states[5] = {"HEALTHY", "TIMEOUT", "ABORT", "STARTING", "COMPONENT_FAIL"};


	void poll_subscriptions();
	void determine_required_processes(int32_t use_obs_avoid);
	void determine_action();

public:

	Companion_Process_Status();
	void check_companion_process_status(orb_advert_t *mav_log_pub, int32_t use_obs_avoid);
};

#endif /* COMPANION_PROCESS_STATUS_H_ */
