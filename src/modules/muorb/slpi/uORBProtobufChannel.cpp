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

#include "uORBProtobufChannel.hpp"
#include "uORB/uORBManager.hpp"
#include "MUORBTest.hpp"
#include <string>

#include <drivers/drv_hrt.h>
#include <pthread.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/log.h>

// Definition of test to run when in muorb test mode
static MUORBTestType test_to_run;

fc_func_ptrs muorb_func_ptrs;

// static initialization.
uORB::ProtobufChannel uORB::ProtobufChannel::_Instance;
uORBCommunicator::IChannelRxHandler *uORB::ProtobufChannel::_RxHandler;
std::map<std::string, int> uORB::ProtobufChannel::_AppsSubscriberCache;
pthread_mutex_t uORB::ProtobufChannel::_rx_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t uORB::ProtobufChannel::_tx_mutex = PTHREAD_MUTEX_INITIALIZER;

bool uORB::ProtobufChannel::_debug = false;
bool _px4_muorb_debug = false;
static bool px4muorb_orb_initialized = false;

int16_t uORB::ProtobufChannel::topic_advertised(const char *messageName)
{
	if (_debug) { PX4_INFO("Advertising %s on remote side", messageName); }

	if (muorb_func_ptrs.advertise_func_ptr) {
		pthread_mutex_lock(&_tx_mutex);
		int16_t rc = muorb_func_ptrs.advertise_func_ptr(messageName);
		pthread_mutex_unlock(&_tx_mutex);
		return rc;
	}

	PX4_ERR("advertise_func_ptr is null in %s", __FUNCTION__);
	return -1;
}

int16_t uORB::ProtobufChannel::add_subscription(const char *messageName, int32_t msgRateInHz)
{
	// MsgRateInHz is unused in this function.
	if (_debug) { PX4_INFO("Subscribing to %s on remote side", messageName); }

	if (muorb_func_ptrs.subscribe_func_ptr) {
		pthread_mutex_lock(&_tx_mutex);
		int16_t rc = muorb_func_ptrs.subscribe_func_ptr(messageName);
		pthread_mutex_unlock(&_tx_mutex);
		return rc;
	}

	PX4_ERR("subscribe_func_ptr is null in %s", __FUNCTION__);
	return -1;
}

int16_t uORB::ProtobufChannel::remove_subscription(const char *messageName)
{
	if (_debug) { PX4_INFO("Unsubscribing from %s on remote side", messageName); }

	if (muorb_func_ptrs.unsubscribe_func_ptr) {
		pthread_mutex_lock(&_tx_mutex);
		int16_t rc = muorb_func_ptrs.unsubscribe_func_ptr(messageName);
		pthread_mutex_unlock(&_tx_mutex);
		return rc;
	}

	PX4_ERR("unsubscribe_func_ptr is null in %s", __FUNCTION__);
	return -1;
}

int16_t uORB::ProtobufChannel::register_handler(uORBCommunicator::IChannelRxHandler *handler)
{
	_RxHandler = handler;
	return 0;
}

int16_t uORB::ProtobufChannel::send_message(const char *messageName, int32_t length, uint8_t *data)
{
	// This function can be called from the PX4 log function so we have to make
	// sure that we do not call PX4_INFO, PX4_ERR, etc. That would cause an
	// infinite loop!
	bool is_not_slpi_log = true;

	if ((strcmp(messageName, "slpi_debug") == 0) || (strcmp(messageName, "slpi_error") == 0)) {
		is_not_slpi_log = false;
	}

	if (muorb_func_ptrs.topic_data_func_ptr) {
		if ((_debug) && (is_not_slpi_log)) {
			PX4_INFO("Got message for topic %s", messageName);
		}

		std::string temp(messageName);
		int has_subscribers = 0;
		pthread_mutex_lock(&_rx_mutex);
		has_subscribers = _AppsSubscriberCache[temp];
		pthread_mutex_unlock(&_rx_mutex);

		if ((has_subscribers) || (is_not_slpi_log == false)) {
			if ((_debug) && (is_not_slpi_log)) {
				PX4_INFO("Sending message for topic %s", messageName);
			}

			pthread_mutex_lock(&_tx_mutex);
			int16_t rc = muorb_func_ptrs.topic_data_func_ptr(messageName, data, length);
			pthread_mutex_unlock(&_tx_mutex);
			return rc;
		}

		if ((_debug) && (is_not_slpi_log)) {
			PX4_INFO("Skipping message for topic %s", messageName);
		}

		return 0;
	}

	if (is_not_slpi_log) {
		PX4_ERR("topic_data_func_ptr is null in %s", __FUNCTION__);
	}

	return -1;
}

static void *test_runner(void *)
{
	if (_px4_muorb_debug) { PX4_INFO("test_runner called"); }

	uORB::ProtobufChannel *channel = uORB::ProtobufChannel::GetInstance();

	switch (test_to_run) {
	case ADVERTISE_TEST_TYPE:
		(void) channel->topic_advertised(muorb_test_topic_name);
		break;

	case SUBSCRIBE_TEST_TYPE:
		(void) channel->add_subscription(muorb_test_topic_name, 1);
		break;

	case UNSUBSCRIBE_TEST_TYPE:
		(void) channel->remove_subscription(muorb_test_topic_name);
		break;

	case TOPIC_TEST_TYPE: {
			uint8_t data[MUORB_TEST_DATA_LEN];

			for (uint8_t i = 0; i < MUORB_TEST_DATA_LEN; i++) {
				data[i] = i;
			}

			(void) muorb_func_ptrs.topic_data_func_ptr(muorb_test_topic_name, data, MUORB_TEST_DATA_LEN);
		}

	default:
		break;
	}

	return nullptr;
}

int px4muorb_orb_initialize(fc_func_ptrs *func_ptrs, int32_t clock_offset_us)
{
	hrt_set_absolute_time_offset(clock_offset_us);

	if (! px4muorb_orb_initialized) {
		if (func_ptrs != nullptr) {
			muorb_func_ptrs = *func_ptrs;

		} else {
			PX4_ERR("NULL top level function pointer in %s", __FUNCTION__);
			return -1;
		}

		if ((muorb_func_ptrs.advertise_func_ptr == NULL) ||
		    (muorb_func_ptrs.subscribe_func_ptr == NULL) ||
		    (muorb_func_ptrs.unsubscribe_func_ptr == NULL) ||
		    (muorb_func_ptrs.topic_data_func_ptr == NULL) ||
		    (muorb_func_ptrs._config_spi_bus_func_t == NULL) ||
		    (muorb_func_ptrs._spi_transfer_func_t == NULL) ||
		    (muorb_func_ptrs._config_i2c_bus_func_t == NULL) ||
		    (muorb_func_ptrs._set_i2c_address_func_t == NULL) ||
		    (muorb_func_ptrs._i2c_transfer_func_t == NULL) ||
		    (muorb_func_ptrs.open_uart_func_t == NULL) ||
		    (muorb_func_ptrs.write_uart_func_t == NULL) ||
		    (muorb_func_ptrs.read_uart_func_t == NULL) ||
		    (muorb_func_ptrs.register_interrupt_callback == NULL)) {
			PX4_ERR("NULL function pointers in %s", __FUNCTION__);
			return -1;
		}

		uORB::Manager::initialize();
		uORB::Manager::get_instance()->set_uorb_communicator(
			uORB::ProtobufChannel::GetInstance());

		px4muorb_orb_initialized = true;

		if (_px4_muorb_debug) { PX4_INFO("px4muorb_orb_initialize called"); }
	}

	return 0;
}

#define TEST_STACK_SIZE 8192
char stack[TEST_STACK_SIZE];

void run_test(MUORBTestType test)
{
	test_to_run = test;
	(void) px4_task_spawn_cmd("test_MUORB",
				  SCHED_DEFAULT,
				  SCHED_PRIORITY_MAX - 2,
				  2000,
				  (px4_main_t)&test_runner,
				  nullptr);
}

int px4muorb_topic_advertised(const char *topic_name)
{
	if (IS_MUORB_TEST(topic_name)) {
		run_test(ADVERTISE_TEST_TYPE);

		if (_px4_muorb_debug) { PX4_INFO("px4muorb_topic_advertised for muorb test called"); }

		return 0;
	}

	uORB::ProtobufChannel *channel = uORB::ProtobufChannel::GetInstance();

	if (channel) {
		uORBCommunicator::IChannelRxHandler *rxHandler = channel->GetRxHandler();

		if (rxHandler) {
			return rxHandler->process_remote_topic(topic_name);

		} else {
			PX4_ERR("Null rx handler in %s", __FUNCTION__);
		}

	} else {
		PX4_ERR("Null channel pointer in %s",  __FUNCTION__);
	}

	return -1;
}

int px4muorb_add_subscriber(const char *topic_name)
{
	if (IS_MUORB_TEST(topic_name)) {
		run_test(SUBSCRIBE_TEST_TYPE);

		if (_px4_muorb_debug) { PX4_INFO("px4muorb_add_subscriber for muorb test called"); }

		return 0;
	}

	uORB::ProtobufChannel *channel = uORB::ProtobufChannel::GetInstance();

	if (channel) {
		uORBCommunicator::IChannelRxHandler *rxHandler = channel->GetRxHandler();

		if (rxHandler) {
			channel->AddRemoteSubscriber(topic_name);
			// Pick a high message rate of 1000 Hz
			return rxHandler->process_add_subscription(topic_name);

		} else {
			PX4_ERR("Null rx handler in %s", __FUNCTION__);
		}

	} else {
		PX4_ERR("Null channel pointer in %s",  __FUNCTION__);
	}

	return -1;
}

int px4muorb_remove_subscriber(const char *topic_name)
{
	if (IS_MUORB_TEST(topic_name)) {
		run_test(UNSUBSCRIBE_TEST_TYPE);

		if (_px4_muorb_debug) { PX4_INFO("px4muorb_remove_subscriber for muorb test called"); }

		return 0;
	}

	uORB::ProtobufChannel *channel = uORB::ProtobufChannel::GetInstance();

	if (channel) {
		uORBCommunicator::IChannelRxHandler *rxHandler = channel->GetRxHandler();

		if (rxHandler) {
			channel->RemoveRemoteSubscriber(topic_name);
			return rxHandler->process_remove_subscription(topic_name);

		} else {
			PX4_ERR("Null rx handler in %s", __FUNCTION__);
		}

	} else {
		PX4_ERR("Null channel pointer in %s",  __FUNCTION__);
	}

	return -1;
}

int px4muorb_send_topic_data(const char *topic_name, const uint8_t *data,
			     int data_len_in_bytes)
{
	if (IS_MUORB_TEST(topic_name)) {
		// Validate the test data received
		bool test_passed = true;

		if (data_len_in_bytes != MUORB_TEST_DATA_LEN) {
			test_passed = false;

		} else {
			for (int i = 0; i < data_len_in_bytes; i++) {
				if ((uint8_t) i != data[i]) {
					test_passed = false;
					break;
				}
			}
		}

		if (test_passed) { run_test(TOPIC_TEST_TYPE); }

		if (_px4_muorb_debug) { PX4_INFO("px4muorb_send_topic_data called"); }

		return 0;
	}

	uORB::ProtobufChannel *channel = uORB::ProtobufChannel::GetInstance();

	if (channel) {
		uORBCommunicator::IChannelRxHandler *rxHandler = channel->GetRxHandler();

		if (rxHandler) {
			return rxHandler->process_received_message(topic_name,
					data_len_in_bytes,
					(uint8_t *) data);

		} else {
			PX4_ERR("Null rx handler in %s", __FUNCTION__);
		}

	} else {
		PX4_ERR("Null channel pointer in %s",  __FUNCTION__);
	}

	return -1;
}
