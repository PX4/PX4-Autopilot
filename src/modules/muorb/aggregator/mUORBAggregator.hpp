/****************************************************************************
 *
 * Copyright (C) 2022 ModalAI, Inc. All rights reserved.
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

#include <string>
#include <string.h>
#include "uORB/uORBCommunicator.hpp"

namespace mUORB
{

class Aggregator
{
public:
	typedef int (*sendFuncPtr)(const char *, const uint8_t *, int);

	void RegisterSendHandler(sendFuncPtr func) { sendFunc = func; }

	void RegisterHandler(uORBCommunicator::IChannelRxHandler *handler) { _RxHandler = handler; }

	int16_t ProcessTransmitTopic(const char *topic, const uint8_t *data, uint32_t length_in_bytes);

	void ProcessReceivedTopic(const char *topic, const uint8_t *data, uint32_t length_in_bytes);

	int16_t SendData();

private:
	static const bool debugFlag;

	const std::string topicName = "aggregation";

	// Master flag to enable aggregation
	const bool aggregationEnabled = true;

	const uint32_t syncFlag = 0x5A01FF00;
	const uint32_t syncFlagSize = 4;
	const uint32_t topicNameLengthSize = 4;
	const uint32_t dataLengthSize = 4;
	const uint32_t headerSize = syncFlagSize + topicNameLengthSize + dataLengthSize;
	static const uint32_t numBuffers = 2;
	static const uint32_t bufferSize = 2048;

	uint32_t bufferId;
	uint32_t bufferWriteIndex;
	uint8_t  buffer[numBuffers][bufferSize];

	uORBCommunicator::IChannelRxHandler *_RxHandler;

	sendFuncPtr sendFunc;

	bool isAggregate(const char *name) { return (strcmp(name, topicName.c_str()) == 0); }

	bool NewRecordOverflows(const char *messageName, int32_t length);

	void MoveToNextBuffer();

	void AddRecordToBuffer(const char *messageName, int32_t length, const uint8_t *data);
};

}
