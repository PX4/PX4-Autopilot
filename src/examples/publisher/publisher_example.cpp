
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
 * @file publisher_example.cpp
 * Example subscriber for ros and px4
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include "publisher_example.h"

using namespace px4;

PublisherExample::PublisherExample() :
	_n(appState),
	_rc_channels_pub(_n.advertise<px4_rc_channels>()),
	_v_att_pub(_n.advertise<px4_vehicle_attitude>()),
	_parameter_update_pub(_n.advertise<px4_parameter_update>())
{
}

px4::AppState PublisherExample::appState;

int PublisherExample::main()
{
	px4::Rate loop_rate(10);

	while (!appState.exitRequested()) {
		loop_rate.sleep();
		_n.spinOnce();

		/* Publish example message */
		px4_rc_channels rc_channels_msg;
		rc_channels_msg.data().timestamp_last_valid = px4::get_time_micros();
		PX4_INFO("rc: %" PRIu64, rc_channels_msg.data().timestamp_last_valid);
		_rc_channels_pub->publish(rc_channels_msg);

		/* Publish example message */
		px4_vehicle_attitude v_att_msg;
		v_att_msg.data().timestamp = px4::get_time_micros();
		PX4_INFO("att: %" PRIu64, v_att_msg.data().timestamp);
		_v_att_pub->publish(v_att_msg);

		/* Publish example message */
		px4_parameter_update parameter_update_msg;
		parameter_update_msg.data().timestamp = px4::get_time_micros();
		PX4_INFO("param update: %" PRIu64, parameter_update_msg.data().timestamp);
		_parameter_update_pub->publish(parameter_update_msg);

	}

	return 0;
}
