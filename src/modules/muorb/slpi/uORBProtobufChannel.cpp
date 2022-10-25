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
#include "MUORBTest.hpp"
#include <string>

#include <qurt.h>
#include <qurt_thread.h>
#include <pthread.h>

// TODO: Move this out of here once we have px4-log functionality
extern "C" void HAP_debug(const char *msg, int level, const char *filename, int line);

// Definition of test to run when in muorb test mode
static MUORBTestType test_to_run;

fc_func_ptrs muorb_func_ptrs;

static void *test_runner(void *test)
{
	HAP_debug("test_runner called", 1, muorb_test_topic_name, 0);

	switch (*((MUORBTestType *) test)) {
	case ADVERTISE_TEST_TYPE:
		(void) muorb_func_ptrs.advertise_func_ptr(muorb_test_topic_name);
		break;

	case SUBSCRIBE_TEST_TYPE:
		(void) muorb_func_ptrs.subscribe_func_ptr(muorb_test_topic_name);
		break;

	case UNSUBSCRIBE_TEST_TYPE:
		(void) muorb_func_ptrs.unsubscribe_func_ptr(muorb_test_topic_name);
		break;

	case TOPIC_TEST_TYPE: {
			uint8_t data[MUORB_TEST_DATA_LEN];

			for (uint8_t i = 0; i < MUORB_TEST_DATA_LEN; i++) { data[i] = i; }

			(void) muorb_func_ptrs.topic_data_func_ptr(muorb_test_topic_name, data, MUORB_TEST_DATA_LEN);
		}

	default:
		break;
	}

	return nullptr;
}

int px4muorb_orb_initialize(fc_func_ptrs *func_ptrs, int32_t clock_offset_us)
{
	// These function pointers will only be non-null on the first call
	// so they must be saved off here
	if (func_ptrs != nullptr) { muorb_func_ptrs = *func_ptrs; }

	HAP_debug("px4muorb_orb_initialize called", 1, "init", 0);

	return 0;
}

#define TEST_STACK_SIZE 8192
char stack[TEST_STACK_SIZE];

void run_test(MUORBTestType test)
{
	pthread_t tid;
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setstacksize(&attr, TEST_STACK_SIZE);
	test_to_run = test;
	pthread_create(&tid, &attr, &test_runner, (void *) &test_to_run);
	pthread_attr_destroy(&attr);
}

int px4muorb_topic_advertised(const char *topic_name)
{
	if (IS_MUORB_TEST(topic_name)) { run_test(ADVERTISE_TEST_TYPE); }

	HAP_debug("px4muorb_topic_advertised called", 1, topic_name, 0);

	return 0;
}

int px4muorb_add_subscriber(const char *topic_name)
{
	if (IS_MUORB_TEST(topic_name)) { run_test(SUBSCRIBE_TEST_TYPE); }

	HAP_debug("px4muorb_add_subscriber called", 1, topic_name, 0);

	return 0;
}

int px4muorb_remove_subscriber(const char *topic_name)
{
	if (IS_MUORB_TEST(topic_name)) { run_test(UNSUBSCRIBE_TEST_TYPE); }

	HAP_debug("px4muorb_remove_subscriber called", 1, topic_name, 0);

	return 0;
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
	}

	HAP_debug("px4muorb_send_topic_data called", 1, topic_name, 0);

	return 0;
}
