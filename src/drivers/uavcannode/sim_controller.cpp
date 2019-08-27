/****************************************************************************
 *
 *   Copyright (C) 2015 PX4 Development Team. All rights reserved.
 *   Author: Pavel Kirienko <pavel.kirienko@gmail.com>
 *           David Sidrane<david_s5@nscdg.com>
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
#include <px4_platform_common/config.h>

#include <syslog.h>

#include "sim_controller.hpp"
#include <uavcan/equipment/esc/RawCommand.hpp>
#include <uavcan/equipment/esc/RPMCommand.hpp>
#include <uavcan/equipment/esc/Status.hpp>
#include "led.hpp"


uavcan::Publisher<uavcan::equipment::esc::Status> *pub_status;
namespace
{
unsigned self_index = 0;
int rpm = 0;

static void cb_raw_command(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::RawCommand> &msg)
{
	if (msg.cmd.size() <= self_index) {
		rgb_led(0, 0, 0, 0);
		return;
	}

	const float scaled = msg.cmd[self_index] / float(
				     uavcan::equipment::esc::RawCommand::FieldTypes::cmd::RawValueType::max());

	static int c = 0;

	if (c++ % 100 == 0) {
		::syslog(LOG_INFO, "scaled:%d\n", (int)scaled);
	}

	if (scaled > 0) {
	} else {
	}
}

static void cb_rpm_command(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::RPMCommand> &msg)
{
	if (msg.rpm.size() <= self_index) {
		return;
	}

	rpm = msg.rpm[self_index];
	static int c = 0;

	if (c++ % 100 == 0) {
		::syslog(LOG_INFO, "rpm:%d\n", rpm);
	}

	if (rpm > 0) {
		rgb_led(255, 0, 0, rpm);

	} else {
		rgb_led(0, 0, 0, 0);
	}
}

void cb_10Hz(const uavcan::TimerEvent &event)
{
	uavcan::equipment::esc::Status msg;

	msg.esc_index = self_index;
	msg.rpm = rpm;
	msg.voltage = 3.3F;
	msg.current = 0.001F;
	msg.temperature = 24.0F;
	msg.power_rating_pct = static_cast<unsigned>(.5F * 100 + 0.5F);
	msg.error_count = 0;

	if (rpm != 0) {
		// Lower the publish rate to 1Hz if the motor is not running
		static uavcan::MonotonicTime prev_pub_ts;

		if ((event.scheduled_time - prev_pub_ts).toMSec() >= 990) {
			prev_pub_ts = event.scheduled_time;
			pub_status->broadcast(msg);
		}

	} else {
		pub_status->broadcast(msg);
	}
}

}
int init_sim_controller(uavcan::INode &node)
{

	typedef void (*cb)(const uavcan::TimerEvent &);
	static uavcan::Subscriber<uavcan::equipment::esc::RawCommand> sub_raw_command(node);
	static uavcan::Subscriber<uavcan::equipment::esc::RPMCommand> sub_rpm_command(node);
	static uavcan::TimerEventForwarder<cb>  timer_10hz(node);

	self_index = 0;

	int res = 0;

	res = sub_raw_command.start(cb_raw_command);

	if (res != 0) {
		return res;
	}

	res = sub_rpm_command.start(cb_rpm_command);

	if (res != 0) {
		return res;
	}

	pub_status = new uavcan::Publisher<uavcan::equipment::esc::Status>(node);
	res = pub_status->init();

	if (res != 0) {
		return res;
	}

	timer_10hz.setCallback(&cb_10Hz);
	timer_10hz.startPeriodic(uavcan::MonotonicDuration::fromMSec(100));

	return 0;
}
