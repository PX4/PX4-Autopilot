/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/log.h>

#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>

class UavcanLogMessage
{
public:
	UavcanLogMessage(uavcan::INode &node) : _sub_logmessage(node), _verbosity(1) {}
	~UavcanLogMessage() = default;

	int init()
	{
		int res = _sub_logmessage.start(LogMessageCbBinder(this, &UavcanLogMessage::logmessage_sub_cb));

		if (res < 0) {
			PX4_ERR("LogMessage sub failed %i", res);
			return res;
		}

		param_get(param_find("UAVCAN_LOG_LEVEL"), &_verbosity);

		return 0;
	}

private:
	typedef uavcan::MethodBinder < UavcanLogMessage *,
		void (UavcanLogMessage::*)(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::LogMessage> &) >
		LogMessageCbBinder;

	void logmessage_sub_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::LogMessage> &msg)
	{
		int px4_level = _PX4_LOG_LEVEL_INFO;

		switch (msg.level.value) {
		case uavcan::protocol::debug::LogLevel::DEBUG:
			px4_level = _PX4_LOG_LEVEL_DEBUG;

			if (_verbosity < 3) { return; }

			break;

		case uavcan::protocol::debug::LogLevel::INFO:
			px4_level = _PX4_LOG_LEVEL_INFO;

			if (_verbosity < 2) { return; }

			break;

		case uavcan::protocol::debug::LogLevel::WARNING:
			px4_level = _PX4_LOG_LEVEL_WARN;

			if (_verbosity < 1) { return; }

			break;

		case uavcan::protocol::debug::LogLevel::ERROR:
			px4_level = _PX4_LOG_LEVEL_ERROR;

			if (_verbosity < 0) { return; }

			break;

		default: // default info
			px4_level = _PX4_LOG_LEVEL_INFO;

			if (_verbosity < 2) { return; }

			break;
		}

		char module_name_buffer[80];
		snprintf(module_name_buffer, sizeof(module_name_buffer), "uavcan:%d:%s", msg.getSrcNodeID().get(), msg.source.c_str());
		px4_log_modulename(px4_level, module_name_buffer, msg.text.c_str());
	}

	uavcan::Subscriber<uavcan::protocol::debug::LogMessage, LogMessageCbBinder> _sub_logmessage;
	int32_t _verbosity;
};
