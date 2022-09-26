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

bool uORB::AppsProtobufChannel::test_flag = false;

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

	} else if (IS_MUORB_TEST(topic)) {
		// Validate the test data received
		bool test_passed = true;

		if (length_in_bytes != MUORB_TEST_DATA_LEN) {
			test_passed = false;

		} else {
			for (uint32_t i = 0; i < length_in_bytes; i++) {
				if ((uint8_t) i != data[i]) {
					test_passed = false;
					break;
				}
			}
		}

		if (test_passed) { test_flag = true; }

	} else {
		PX4_INFO("Got received data callback for topic %s", topic);
	}
}

void uORB::AppsProtobufChannel::AdvertiseCallback(const char *topic)
{
	PX4_INFO("Got advertisement callback for topic %s", topic);

	if (IS_MUORB_TEST(topic)) { test_flag = true; }
}

void uORB::AppsProtobufChannel::SubscribeCallback(const char *topic)
{
	PX4_INFO("Got subscription callback for topic %s", topic);

	if (IS_MUORB_TEST(topic)) { test_flag = true; }
}

void uORB::AppsProtobufChannel::UnsubscribeCallback(const char *topic)
{
	PX4_INFO("Got remove subscription callback for topic %s", topic);

	if (IS_MUORB_TEST(topic)) { test_flag = true; }
}

bool uORB::AppsProtobufChannel::Test(MUORBTestType test_type)
{
	int rc = -1;
	int timeout = 10;

	uint8_t test_data[MUORB_TEST_DATA_LEN];

	for (uint8_t i = 0; i < MUORB_TEST_DATA_LEN; i++) {
		test_data[i] = i;
	};

	test_flag = false;

	switch (test_type) {
	case ADVERTISE_TEST_TYPE:
		rc = fc_sensor_advertise(muorb_test_topic_name);
		break;

	case SUBSCRIBE_TEST_TYPE:
		rc = fc_sensor_subscribe(muorb_test_topic_name);
		break;

	case TOPIC_TEST_TYPE:
		rc = fc_sensor_send_data(muorb_test_topic_name, test_data, MUORB_TEST_DATA_LEN);
		break;

	case UNSUBSCRIBE_TEST_TYPE:
		rc = fc_sensor_unsubscribe(muorb_test_topic_name);
		break;

	default:
		break;
	}

	// non zero return code means test failed
	if (rc) { return false; }

	// Wait for test acknowledgement from DSP
	while ((! test_flag) && (timeout--)) {
		usleep(10000);
	}

	if (timeout == -1) {
		PX4_ERR("Test timed out waiting for response");
		return false;
	}

	return true;
}

bool uORB::AppsProtobufChannel::Test()
{
	if (! Test(ADVERTISE_TEST_TYPE)) { return false; }

	if (! Test(SUBSCRIBE_TEST_TYPE)) { return false; }

	if (! Test(TOPIC_TEST_TYPE)) { return false; }

	if (! Test(UNSUBSCRIBE_TEST_TYPE)) { return false; }

	PX4_INFO("muorb test passed");
	return true;
}

bool uORB::AppsProtobufChannel::Initialize(bool enable_debug)
{
	fc_callbacks cb = {&ReceiveCallback, &AdvertiseCallback,
			   &SubscribeCallback, &UnsubscribeCallback
			  };

	if (fc_sensor_initialize(enable_debug, &cb) != 0) {
		if (enable_debug) { PX4_INFO("Warning: muorb protobuf initalize method failed"); }

	} else {
		PX4_INFO("muorb protobuf initalize method succeeded");
	}

	return true;
}
