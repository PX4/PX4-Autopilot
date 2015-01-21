/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * @file subscriber_example.cpp
 * Example subscriber for ros and px4
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include "subscriber_params.h"
#include "subscriber_example.h"

using namespace px4;

// void rc_channels_callback_function(const PX4_TOPIC_T(rc_channels) &msg) {
void rc_channels_callback_function(const px4_rc_channels &msg);
void rc_channels_callback_function(const px4_rc_channels &msg) {
	PX4_INFO("I heard: [%llu]", msg.data().timestamp_last_valid);
}

SubscriberExample::SubscriberExample() :
	_n(),
	_p_sub_interv(PX4_PARAM_INIT(SUB_INTERV)),
	_interval(0),
	_p_test_float(PX4_PARAM_INIT(SUB_TESTF)),
	_test_float(0.0f)
{
	/* Read the parameter back as example */
	PX4_PARAM_GET(_p_sub_interv, &_interval);
	PX4_INFO("Param SUB_INTERV = %d", _interval);
	PX4_PARAM_GET(_p_test_float, &_test_float);
	PX4_INFO("Param SUB_TESTF = %.3f", (double)_test_float);

	/* Do some subscriptions */
	/* Function */
	// PX4_SUBSCRIBE(_n, rc_channels, rc_channels_callback_function, _interval);
	_n.subscribe<px4_rc_channels>(rc_channels_callback_function); //ROS version
	
	// [> Class Method <]
	// PX4_SUBSCRIBE(_n, rc_channels, SubscriberExample::rc_channels_callback, this, 1000);
	// [> No callback <]
	// _sub_rc_chan = PX4_SUBSCRIBE(_n, rc_channels, 500);

	PX4_INFO("subscribed");
}

/**
 * This tutorial demonstrates simple receipt of messages over the PX4 middleware system.
 * Also the current value of the _sub_rc_chan subscription is printed
 */
// void SubscriberExample::rc_channels_callback(const PX4_TOPIC_T(rc_channels) &msg) {
	// PX4_INFO("Subscriber callback: [%llu], value of _sub_rc_chan: [%llu]",
			// msg.timestamp_last_valid,
			// _sub_rc_chan->get().data().timestamp_last_valid);
// }
