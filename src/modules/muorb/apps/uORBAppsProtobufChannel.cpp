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

#include "uORBAppsProtobufChannel.hpp"
#include <string.h>

#include "fc_sensor.h"

// Initialize the static members
uORB::AppsProtobufChannel *uORB::AppsProtobufChannel::_InstancePtr = nullptr;

void uORB::AppsProtobufChannel::ReceiveCallback(const char *topic,
		const uint8_t *data,
		uint32_t length_in_bytes)
{
	if (strcmp(topic, "slpi_debug") == 0) {
		PX4_INFO("SLPI: %s", (const char *) data);

	} else if (strcmp(topic, "slpi_error") == 0) {
		PX4_ERR("SLPI: %s", (const char *) data);

	} else {
		PX4_INFO("Got received data callback for topic %s", topic);
	}
}

void uORB::AppsProtobufChannel::AdvertiseCallback(const char *topic)
{
	PX4_INFO("Got advertisement callback for topic %s", topic);
}

void uORB::AppsProtobufChannel::SubscribeCallback(const char *topic)
{
	PX4_INFO("Got subscription callback for topic %s", topic);
}

void uORB::AppsProtobufChannel::UnsubscribeCallback(const char *topic)
{
	PX4_INFO("Got remove subscription callback for topic %s", topic);
}

bool uORB::AppsProtobufChannel::Initialize(bool enable_debug)
{
	fc_callbacks cb = {&ReceiveCallback, &AdvertiseCallback,
			   &SubscribeCallback, &UnsubscribeCallback
			  };

	if (fc_sensor_initialize(enable_debug, &cb) != 0) {
		PX4_ERR("Error calling the muorb protobuf initalize method");

	} else {
		PX4_INFO("muorb protobuf initalize method succeeded");
	}

	return true;
}
