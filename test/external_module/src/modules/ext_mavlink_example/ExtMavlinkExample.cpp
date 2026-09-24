/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "ExtMavlinkExample.hpp"

#include <modules/mavlink/mavlink_bridge_header.h>
#include <modules/mavlink/mavlink_ext_stream.h>

#include <string.h>

using namespace time_literals;

ModuleBase::Descriptor ExtMavlinkExample::desc{task_spawn, custom_command, print_usage};

ExtMavlinkExample::ExtMavlinkExample() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

ExtMavlinkExample::~ExtMavlinkExample()
{
	// Both return only once no callback is running, so `this` is safe to destroy afterwards.
	mavlink_ext_handler_unregister(MAVLINK_MSG_ID_EXT_EXAMPLE_PING);
	mavlink_ext_stream_unregister(MAVLINK_MSG_ID_EXT_EXAMPLE_STATUS);
}

bool ExtMavlinkExample::init()
{
	if (mavlink_ext_handler_register(MAVLINK_MSG_ID_EXT_EXAMPLE_PING, handle_ping, this) != 0) {
		PX4_ERR("EXT_EXAMPLE_PING handler registration failed");
		return false;
	}

	if (mavlink_ext_stream_register(MAVLINK_MSG_ID_EXT_EXAMPLE_STATUS, send_status, this, 500000) != 0) {
		PX4_ERR("EXT_EXAMPLE_STATUS stream registration failed");
		return false;
	}

	// The work queue only serves `stop`; all traffic runs on the mavlink threads.
	ScheduleOnInterval(1_s);
	return true;
}

void ExtMavlinkExample::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
	}
}

bool ExtMavlinkExample::handle_ping(const mavlink_message_t *msg, void *user_data)
{
	ExtMavlinkExample *self = static_cast<ExtMavlinkExample *>(user_data);

	mavlink_ext_example_ping_t ping{};
	mavlink_msg_ext_example_ping_decode(msg, &ping);
	self->_pings_received.fetch_add(1);

	mavlink_ext_example_pong_t pong{};
	pong.seq = ping.seq;
	memcpy(pong.payload, ping.payload, sizeof(pong.payload));

	for (uint8_t byte : ping.payload) {
		pong.payload_sum += byte;
	}

	const int sent = mavlink_ext_send(send_pong, &pong);

	if (sent > 0) {
		self->_pongs_sent.fetch_add((uint32_t)sent);
	}

	return true;
}

bool ExtMavlinkExample::send_pong(uint8_t channel, void *user_data)
{
	mavlink_msg_ext_example_pong_send_struct(static_cast<mavlink_channel_t>(channel),
			static_cast<const mavlink_ext_example_pong_t *>(user_data));
	return true;
}

bool ExtMavlinkExample::send_status(uint8_t channel, void *user_data)
{
	ExtMavlinkExample *self = static_cast<ExtMavlinkExample *>(user_data);

	mavlink_ext_example_status_t status{};
	status.pings_received = self->_pings_received.load();
	status.pongs_sent = self->_pongs_sent.load();
	status.status_seq = self->_status_seq.fetch_add(1) + 1;

	mavlink_msg_ext_example_status_send_struct(static_cast<mavlink_channel_t>(channel), &status);
	return true;
}

int ExtMavlinkExample::task_spawn(int argc, char *argv[])
{
	ExtMavlinkExample *instance = new ExtMavlinkExample();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int ExtMavlinkExample::print_status()
{
	PX4_INFO("pings received: %" PRIu32 ", pongs sent: %" PRIu32 ", status sent: %" PRIu32,
		 _pings_received.load(), _pongs_sent.load(), _status_seq.load());
	return 0;
}

int ExtMavlinkExample::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ExtMavlinkExample::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Out-of-tree example answering EXT_EXAMPLE_PING with EXT_EXAMPLE_PONG and streaming EXT_EXAMPLE_STATUS.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ext_mavlink_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ext_mavlink_example_main(int argc, char *argv[])
{
	return ModuleBase::main(ExtMavlinkExample::desc, argc, argv);
}
