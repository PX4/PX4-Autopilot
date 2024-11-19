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
uORBCommunicator::IChannelRxHandler *uORB::AppsProtobufChannel::_RxHandler = nullptr;
mUORB::Aggregator uORB::AppsProtobufChannel::_Aggregator;
std::map<std::string, int> uORB::AppsProtobufChannel::_SlpiSubscriberCache;
pthread_mutex_t uORB::AppsProtobufChannel::_tx_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t uORB::AppsProtobufChannel::_rx_mutex = PTHREAD_MUTEX_INITIALIZER;
bool uORB::AppsProtobufChannel::_Debug = false;


void uORB::AppsProtobufChannel::ReceiveCallback(const char *topic,
		const uint8_t *data,
		uint32_t length_in_bytes)
{

	if (_Debug) { PX4_INFO("Got Receive callback for topic %s", topic); }

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

		return;

	} else if (_RxHandler) {
		_Aggregator.ProcessReceivedTopic(topic, data, length_in_bytes);

	} else {
		PX4_ERR("Couldn't handle topic %s in receive callback", topic);
	}
}

void uORB::AppsProtobufChannel::AdvertiseCallback(const char *topic)
{
	if (_Debug) { PX4_INFO("Got advertisement callback for topic %s", topic); }

	if (IS_MUORB_TEST(topic)) {
		test_flag = true;
		return;

	} else if (_RxHandler) {
		_RxHandler->process_remote_topic(topic);

	} else {
		PX4_ERR("Couldn't handle topic %s in advertise callback", topic);
	}
}

void uORB::AppsProtobufChannel::SubscribeCallback(const char *topic)
{
	if (_Debug) { PX4_INFO("Got subscription callback for topic %s", topic); }

	if (IS_MUORB_TEST(topic)) {
		test_flag = true;
		return;

	} else if (strcmp(topic, "CPULOAD") == 0) {
		// PX4_ERR("Got CPULOAD subscription");
		// This will happen when a newer PX4 version is talking to a
		// SLPI image that doesn't support the CPULOAD request. If the
		// SLPI image does support it then we wouldn't get this.
	} else if (_RxHandler) {
		pthread_mutex_lock(&_rx_mutex);
		_SlpiSubscriberCache[topic]++;
		pthread_mutex_unlock(&_rx_mutex);

		_RxHandler->process_add_subscription(topic);

	} else {
		PX4_ERR("Couldn't handle topic %s in subscribe callback", topic);
	}
}

void uORB::AppsProtobufChannel::UnsubscribeCallback(const char *topic)
{
	if (_Debug) { PX4_INFO("Got remove subscription callback for topic %s", topic); }

	if (IS_MUORB_TEST(topic)) {
		test_flag = true;
		return;

	} else if (_RxHandler) {
		pthread_mutex_lock(&_rx_mutex);

		if (_SlpiSubscriberCache[topic]) { _SlpiSubscriberCache[topic]--; }

		pthread_mutex_unlock(&_rx_mutex);

		_RxHandler->process_remove_subscription(topic);

	} else {
		PX4_ERR("Couldn't handle topic %s in unsubscribe callback", topic);
	}
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
		rc = topic_advertised(muorb_test_topic_name);
		PX4_INFO("succesfully did ADVERTISE_TEST_TYPE");
		break;

	case SUBSCRIBE_TEST_TYPE:
		rc = add_subscription(muorb_test_topic_name, 1000);
		PX4_INFO("succesfully did SUBSCRIBE_TEST_TYPE");
		break;

	case TOPIC_TEST_TYPE:
		rc = fc_sensor_send_data(muorb_test_topic_name, test_data, MUORB_TEST_DATA_LEN);
		PX4_INFO("succesfully did TOPIC_TEST_TYPE");
		break;

	case UNSUBSCRIBE_TEST_TYPE:
		rc = remove_subscription(muorb_test_topic_name);
		PX4_INFO("succesfully did UNSUBSCRIBE_TEST_TYPE");
		break;

	default:
		PX4_ERR("Unknown test type %d", test_type);
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
	if (! _Initialized) {
		fc_callbacks cb = { &ReceiveCallback, &AdvertiseCallback,
				    &SubscribeCallback, &UnsubscribeCallback
				  };

		if (fc_sensor_initialize(enable_debug, &cb) != 0) {
			if (enable_debug) { PX4_INFO("Warning: muorb protobuf initalize method failed"); }

		} else {
			PX4_INFO("muorb protobuf initalize method succeeded");
			_Initialized = true;
		}

	} else {
		PX4_INFO("AppsProtobufChannel already initialized");
	}

	return true;
}

int16_t uORB::AppsProtobufChannel::topic_advertised(const char *messageName)
{
	if (_Initialized) {
		if (_Debug) { PX4_INFO("Advertising topic %s to remote side", messageName); }

		pthread_mutex_lock(&_tx_mutex);
		int16_t rc = fc_sensor_advertise(messageName);
		pthread_mutex_unlock(&_tx_mutex);
		return rc;
	}

	return -1;
}

int16_t uORB::AppsProtobufChannel::add_subscription(const char *messageName, int msgRateInHz)
{
	// This parameter is unused.
	(void)(msgRateInHz);

	if (_Initialized) {
		pthread_mutex_lock(&_tx_mutex);
		int16_t rc = fc_sensor_subscribe(messageName);
		pthread_mutex_unlock(&_tx_mutex);
		return rc;
	}

	return -1;
}

int16_t uORB::AppsProtobufChannel::remove_subscription(const char *messageName)
{
	if (_Initialized) {
		pthread_mutex_lock(&_tx_mutex);
		int16_t rc = fc_sensor_unsubscribe(messageName);
		pthread_mutex_unlock(&_tx_mutex);
		return rc;
	}

	return -1;
}

int16_t uORB::AppsProtobufChannel::register_handler(uORBCommunicator::IChannelRxHandler *handler)
{
	_RxHandler = handler;
	_Aggregator.RegisterHandler(handler);
	return 0;
}

int16_t uORB::AppsProtobufChannel::send_message(const char *messageName, int length, uint8_t *data)
{
	// This is done to slow down high rate debug messages.
	bool enable_debug = false;

	if ((_MessageCounter++ % 100) == 0) {
		enable_debug = true;
	}

	if (_Initialized) {
		pthread_mutex_lock(&_rx_mutex);
		int has_subscribers = _SlpiSubscriberCache[messageName];
		pthread_mutex_unlock(&_rx_mutex);

		if (has_subscribers) {
			if (_Debug && enable_debug) {
				PX4_INFO("Sending data for topic %s", messageName);
			}

			pthread_mutex_lock(&_tx_mutex);
			int16_t rc = fc_sensor_send_data(messageName, data, length);
			pthread_mutex_unlock(&_tx_mutex);
			return rc;

		} else {
			if (_Debug && enable_debug) {
				PX4_INFO("No subscribers (yet) in %s for topic %s", __FUNCTION__, messageName);
			}

			return 0;
		}
	}

	return -1;
}
