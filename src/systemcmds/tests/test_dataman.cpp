/****************************************************************************
 *
 *  Copyright (C) 2023 PX4 Development Team. All rights reserved.
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
 * @file test_dataman.cpp
 * Tests for Dataman.
 */

#include <unit_test.h>

#include <errno.h>
#include <fcntl.h>
#include <px4_platform_common/px4_config.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>

#include "dataman_client/DatamanClient.hpp"

class DatamanTest : public UnitTest
{
public:
	virtual bool run_tests();

private:

	enum class State {
		Write,
		WriteWait,
		Read,
		ReadWait,
		Clear,
		ClearWait,
		Lock,
		LockWait,
		Unlock,
		UnlockWait,
		OperationCompleted,
		CompareBuffers,
		Exit
	};

	//Sync
	bool testSyncReadInvalidItem();
	bool testSyncWriteInvalidItem();
	bool testSyncReadInvalidIndex();
	bool testSyncWriteInvalidIndex();
	bool testSyncReadBufferOverflow();
	bool testSyncWriteBufferOverflow();
	bool testSyncMutipleClients();
	bool testSyncWriteReadAllItemsMaxSize();
	bool testSyncClearAll();

	//Async
	bool testAsyncReadInvalidItem();
	bool testAsyncWriteInvalidItem();
	bool testAsyncReadInvalidIndex();
	bool testAsyncWriteInvalidIndex();
	bool testAsyncReadBufferOverflow();
	bool testAsyncWriteBufferOverflow();
	bool testAsyncMutipleClientsNoLocks();
	bool testAsyncMutipleClientsWithLocks();
	bool testAsyncWriteReadAllItemsMaxSize();
	bool testAsyncClearAll();

	//Cache
	bool testCache();

	//The last test will reset the items so that FMU can boot without Dataman errors.
	bool testResetItems();

	static void *testLockThread(void *arg);

	DatamanClient _dataman_client1{};
	DatamanClient _dataman_client2{};
	DatamanClient _dataman_client3{};
	DatamanClient _dataman_client_thread1{};
	DatamanClient _dataman_client_thread2{};
	DatamanClient _dataman_client_thread3{};

	uint32_t _cache_size = 10;
	DatamanCache _dataman_cache{_cache_size};

	static void *testAsyncNoLocksThread(void *arg);
	static void *testAsyncWithLocksThread(void *arg);

	static constexpr uint32_t DM_MAX_DATA_SIZE{MISSION_ITEM_SIZE};
	static_assert(sizeof(dataman_response_s::data) == DM_MAX_DATA_SIZE, "data size != DM_MAX_DATA_SIZE");

	uint8_t _buffer_read[DM_MAX_DATA_SIZE];
	uint8_t _buffer_write[DM_MAX_DATA_SIZE];

	bool _response_success{false};

	uint32_t _thread_index;
	bool _thread_tests_success;

	static constexpr uint32_t OVERFLOW_LENGTH = sizeof(_buffer_write) + 1;
};

bool
DatamanTest::testSyncReadInvalidItem()
{

	bool success = _dataman_client1.readSync(DM_KEY_NUM_KEYS, 0, _buffer_read, sizeof(_buffer_read));
	return !success;
}

bool
DatamanTest::testSyncWriteInvalidItem()
{
	bool success = _dataman_client1.writeSync(DM_KEY_NUM_KEYS, 0, _buffer_write, sizeof(_buffer_write));
	return !success;
}


bool
DatamanTest::testSyncReadInvalidIndex()
{
	bool success = _dataman_client1.readSync(DM_KEY_SAFE_POINTS, DM_KEY_SAFE_POINTS_MAX, _buffer_read, 0);
	return !success;
}

bool
DatamanTest::testSyncWriteInvalidIndex()
{
	bool success = _dataman_client1.writeSync(DM_KEY_SAFE_POINTS, DM_KEY_SAFE_POINTS_MAX, _buffer_write, 0);
	return !success;
}

bool
DatamanTest::testSyncReadBufferOverflow()
{
	bool success = _dataman_client1.readSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0, _buffer_read, OVERFLOW_LENGTH);
	return !success;
}

bool
DatamanTest::testSyncWriteBufferOverflow()
{
	bool success = _dataman_client1.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0, _buffer_write, OVERFLOW_LENGTH);
	return !success;
}

bool
DatamanTest::testSyncMutipleClients()
{
	bool success = _dataman_client1.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0x11, _buffer_write, sizeof(_buffer_write));

	if (!success) {
		return false;
	}

	success = _dataman_client1.readSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0x11, _buffer_read, sizeof(_buffer_read));

	if (!success) {
		return false;
	}

	success = _dataman_client2.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0x22, _buffer_write, sizeof(_buffer_write));

	if (!success) {
		return false;
	}

	success = _dataman_client3.readSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0x22, _buffer_read, sizeof(_buffer_read));

	if (!success) {
		return false;
	}

	success = _dataman_client1.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0x33, _buffer_write, sizeof(_buffer_write));

	if (!success) {
		return false;
	}

	success = _dataman_client1.readSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0x33, _buffer_read, sizeof(_buffer_read));

	if (!success) {
		return false;
	}

	//Test locking
	success = _dataman_client1.lockSync(DM_KEY_MISSION_STATE);

	if (!success) {
		return false;
	}

	//Check if already locked
	success = _dataman_client1.lockSync(DM_KEY_MISSION_STATE, 10_ms);

	if (success) {
		return false;
	}

	//Check if already locked from another client
	success = _dataman_client2.lockSync(DM_KEY_MISSION_STATE, 10_ms);

	if (success) {
		return false;
	}

	//Check if can write while locked
	success = _dataman_client1.writeSync(DM_KEY_MISSION_STATE, 0, _buffer_write, g_per_item_size[DM_KEY_MISSION_STATE]);

	if (!success) {
		return false;
	}

	//Check if can read  while locked
	success = _dataman_client1.readSync(DM_KEY_MISSION_STATE, 0, _buffer_read, g_per_item_size[DM_KEY_MISSION_STATE]);

	if (!success) {
		return false;
	}

	//Compare content from buffers
	for (uint32_t i = 0; i < g_per_item_size[DM_KEY_MISSION_STATE]; ++i) {
		if (_buffer_read[i] != _buffer_write[i]) {
			return false;
		}
	}

	//Check if can write while locked from another client
	success = _dataman_client2.writeSync(DM_KEY_MISSION_STATE, 0, _buffer_write, g_per_item_size[DM_KEY_MISSION_STATE]);

	if (success) {
		return false;
	}

	//Check if can read  while locked from another client
	success = _dataman_client2.readSync(DM_KEY_MISSION_STATE, 0, _buffer_read, g_per_item_size[DM_KEY_MISSION_STATE]);

	if (success) {
		return false;
	}

	//Check if can unlock while locked from another client
	success = _dataman_client2.unlockSync(DM_KEY_MISSION_STATE);

	if (success) {
		return false;
	}

	//Check if can unlock
	success = _dataman_client1.unlockSync(DM_KEY_MISSION_STATE);

	if (!success) {
		return false;
	}

	//Lock from another thread and to test retry lock mechanism
	pthread_t thread{};
	uint32_t ret = pthread_create(&thread, NULL, &testLockThread, this);

	if (ret != 0) {
		printf("pthread_create failed: %" PRIu32 "\n", ret);
		return false;
	}

	px4_usleep(50_ms);

	//Should fail since timeout is to short
	success = _dataman_client1.lockSync(DM_KEY_MISSION_STATE, 10_ms);

	if (success) {
		pthread_join(thread, nullptr);
		return false;
	}

	//Should be able to lock since the task in the thread should unlock the item in the meantime
	success = _dataman_client1.lockSync(DM_KEY_MISSION_STATE);

	if (!success) {
		pthread_join(thread, nullptr);
		return false;
	}

	success = _dataman_client1.unlockSync(DM_KEY_MISSION_STATE);

	if (!success) {
		pthread_join(thread, nullptr);
		return false;
	}

	pthread_join(thread, nullptr);

	return success;
}

void *DatamanTest::testLockThread(void *arg)
{
	DatamanTest *dataman_test = (DatamanTest *)arg;
	dataman_test->_dataman_client_thread1.lockSync(DM_KEY_MISSION_STATE);
	px4_usleep(200_ms);
	dataman_test->_dataman_client_thread1.unlockSync(DM_KEY_MISSION_STATE);
	px4_usleep(200_ms);

	return nullptr;
}

bool
DatamanTest::testSyncWriteReadAllItemsMaxSize()
{
	bool success = false;

	// Iterate all items
	for (uint32_t item = DM_KEY_SAFE_POINTS; item < DM_KEY_NUM_KEYS; ++item) {

		// writeSync
		for (uint32_t index = 0U; index < g_per_item_max_index[item]; ++index) {

			// Prepare write buffer
			for (uint32_t i = 0; i < g_per_item_size[item]; ++i) {
				_buffer_write[i] = (uint8_t)(index % UINT8_MAX);
			}

			success = _dataman_client1.writeSync((dm_item_t)item, index, _buffer_write, g_per_item_size[item]);

			if (!success) {
				PX4_ERR("writeSync failed at item = %" PRIu32 ", index = %" PRIu32, item, index);
				return false;
			}
		}

		// readSync
		for (uint32_t index = 0U; index < g_per_item_max_index[item]; ++index) {

			success = _dataman_client1.readSync((dm_item_t)item, index, _buffer_read, g_per_item_size[item]);

			if (!success) {
				PX4_ERR("readSync failed at item =  %" PRIu32 ", index = %" PRIu32, item, index);
				return false;
			}

			// Check read buffer
			for (uint32_t i = 0U; i < g_per_item_size[item]; ++i) {

				uint8_t expected_value = (index % UINT8_MAX);

				if (expected_value != _buffer_read[i]) {
					PX4_ERR("readSync failed at item = %" PRIu32 ", index =  %" PRIu32 ", element= %" PRIu32 ", expected:  %" PRIu8
						", received:  %" PRIu8, item, index, i, expected_value, _buffer_read[i]);
					return false;
				}
			}
		}
	}

	return true;
}

bool
DatamanTest::testSyncClearAll()
{
	bool success = false;

	// Iterate all items
	for (uint32_t item = DM_KEY_SAFE_POINTS; item < DM_KEY_NUM_KEYS; ++item) {

		success = _dataman_client1.clearSync((dm_item_t)item);

		if (!success) {
			PX4_ERR("clearSync failed at item =  %" PRIu32, item);
			return false;
		}
	}

	return success;
}

bool
DatamanTest::testAsyncReadInvalidItem()
{
	bool success = true;

	State state = State::Read;
	hrt_abstime start_time = hrt_absolute_time();

	//While loop represents a task
	while (state != State::Exit) {

		_dataman_client1.update();


		switch (state) {

		case State::Read:

			state = State::ReadWait;
			success = _dataman_client1.readAsync(DM_KEY_NUM_KEYS, 0, _buffer_read, sizeof(_buffer_read));

			if (!success) {
				return false;
			}

			break;

		case State::ReadWait:
			if (_dataman_client1.lastOperationCompleted(_response_success)) {
				state = State::OperationCompleted;

				if (!_response_success) {
					//Test ends here
					return true;
				}
			}

			break;

		default:
			break;

		}

		if (hrt_elapsed_time(&start_time) > 1_s) {
			PX4_ERR("Test timeout!");
			return false;
		}

		//Simulate rescheduling the task after a 1 ms delay to allow time for the dataman task to operate.
		px4_usleep(1_ms);
	}

	return false;
}

bool
DatamanTest::testAsyncWriteInvalidItem()
{
	bool success = true;

	State state = State::Write;
	hrt_abstime start_time = hrt_absolute_time();

	//While loop represents a task
	while (state != State::Exit) {

		_dataman_client1.update();


		switch (state) {

		case State::Write:

			state = State::WriteWait;
			success = _dataman_client1.writeAsync(DM_KEY_NUM_KEYS, 0, _buffer_write, sizeof(_buffer_write));

			if (!success) {
				return false;
			}

			break;

		case State::WriteWait:
			if (_dataman_client1.lastOperationCompleted(_response_success)) {
				state = State::OperationCompleted;

				if (!_response_success) {
					//Test ends here
					return true;
				}
			}

			break;

		default:
			break;

		}

		if (hrt_elapsed_time(&start_time) > 1_s) {
			PX4_ERR("Test timeout!");
			return false;
		}

		//Simulate rescheduling the task after a 1 ms delay to allow time for the dataman task to operate.
		px4_usleep(1_ms);
	}

	return false;
}

bool
DatamanTest::testAsyncReadInvalidIndex()
{
	bool success = true;

	State state = State::Read;
	hrt_abstime start_time = hrt_absolute_time();

	//While loop represents a task
	while (state != State::Exit) {

		_dataman_client1.update();


		switch (state) {

		case State::Read:

			state = State::ReadWait;
			success = _dataman_client1.readAsync(DM_KEY_SAFE_POINTS, DM_KEY_SAFE_POINTS_MAX, _buffer_read, 2);

			if (!success) {
				return false;
			}

			break;

		case State::ReadWait:
			if (_dataman_client1.lastOperationCompleted(_response_success)) {
				state = State::OperationCompleted;

				if (!_response_success) {
					//Test ends here
					return true;
				}
			}

			break;

		default:
			break;

		}

		if (hrt_elapsed_time(&start_time) > 1_s) {
			PX4_ERR("Test timeout!");
			return false;
		}

		//Simulate rescheduling the task after a 1 ms delay to allow time for the dataman task to operate.
		px4_usleep(1_ms);
	}

	return false;
}

bool
DatamanTest::testAsyncWriteInvalidIndex()
{
	bool success = true;

	State state = State::Write;
	hrt_abstime start_time = hrt_absolute_time();

	//While loop represents a task
	while (state != State::Exit) {

		_dataman_client1.update();


		switch (state) {

		case State::Write:

			state = State::WriteWait;
			success = _dataman_client1.writeAsync(DM_KEY_SAFE_POINTS, DM_KEY_SAFE_POINTS_MAX, _buffer_write, 2);

			if (!success) {
				return false;
			}

			break;

		case State::WriteWait:
			if (_dataman_client1.lastOperationCompleted(_response_success)) {
				state = State::OperationCompleted;

				if (!_response_success) {
					//Test ends here
					return true;
				}
			}

			break;

		default:
			break;

		}

		if (hrt_elapsed_time(&start_time) > 1_s) {
			PX4_ERR("Test timeout!");
			return false;
		}

		//Simulate rescheduling the task after a 1 ms delay to allow time for the dataman task to operate.
		px4_usleep(1_ms);
	}

	return false;
}

bool
DatamanTest::testAsyncReadBufferOverflow()
{
	bool success = _dataman_client1.readAsync(DM_KEY_SAFE_POINTS, DM_KEY_SAFE_POINTS_MAX, _buffer_read, OVERFLOW_LENGTH);

	if (success) {
		return false;
	}

	return !success;
}

bool
DatamanTest::testAsyncWriteBufferOverflow()
{
	bool success = _dataman_client1.writeAsync(DM_KEY_SAFE_POINTS, DM_KEY_SAFE_POINTS_MAX, _buffer_write, OVERFLOW_LENGTH);

	if (success) {
		return false;
	}

	return !success;
}

bool
DatamanTest::testAsyncMutipleClientsNoLocks()
{
	pthread_t thread1{};
	pthread_t thread2{};
	pthread_t thread3{};

	_thread_tests_success = true;
	_thread_index = 0x0;

	// Test multiple dataman clients without locks
	uint32_t ret = pthread_create(&thread1, NULL, &testAsyncNoLocksThread, this);

	if (ret != 0) {
		printf("pthread_create failed: %" PRIu32 "\n", ret);
		_thread_tests_success = false;
	}

	ret = pthread_create(&thread2, NULL, &testAsyncNoLocksThread, this);

	if (ret != 0) {
		printf("pthread_create failed: %" PRIu32 "\n", ret);
		_thread_tests_success = false;
	}

	ret = pthread_create(&thread3, NULL, &testAsyncNoLocksThread, this);

	if (ret != 0) {
		printf("pthread_create failed: %" PRIu32 "\n", ret);
		_thread_tests_success = false;
	}

	pthread_join(thread1, nullptr);
	pthread_join(thread2, nullptr);
	pthread_join(thread3, nullptr);

	return _thread_tests_success;
}

void *DatamanTest::testAsyncNoLocksThread(void *arg)
{
	DatamanTest *dataman_test = (DatamanTest *)arg;
	uint32_t index = (dataman_test->_thread_index)++;
	State state = State::Write;

	hrt_abstime start_time = hrt_absolute_time();

	uint8_t buffer_read[DM_MAX_DATA_SIZE] = {};
	uint8_t buffer_write[DM_MAX_DATA_SIZE] = {};

	bool success;
	bool response_success;

	// Prepare write buffer
	for (uint8_t i = 0; i < g_per_item_size[DM_KEY_WAYPOINTS_OFFBOARD_0]; ++i) {
		buffer_write[i] = i * index;
	}

	DatamanClient *dataman_client{nullptr};

	if (dataman_test->_thread_index == 1) {
		dataman_client = &(dataman_test->_dataman_client_thread1);

	} else if (dataman_test->_thread_index == 2) {
		dataman_client = &(dataman_test->_dataman_client_thread2);

	} else if (dataman_test->_thread_index == 3) {
		dataman_client = &(dataman_test->_dataman_client_thread3);

	} else {
		PX4_ERR("Unknown thread %" PRIu32 "!", dataman_test->_thread_index);
		return nullptr;
	}

	while (state != State::Exit) {

		dataman_client->update();

		switch (state) {

		case State::Write:

			state = State::WriteWait;
			success = dataman_client->writeAsync(DM_KEY_WAYPOINTS_OFFBOARD_0, index, buffer_write, sizeof(buffer_write));

			if (!success) {
				PX4_ERR("writeAsync failed for index %" PRIu32 "!", index);
				state = State::Exit;
				dataman_test->_thread_tests_success = false;
			}

			break;

		case State::WriteWait:

			if (dataman_client->lastOperationCompleted(response_success)) {
				state = State::Read;

				if (!response_success) {
					PX4_ERR("writeAsync failed to get success operation complete for the index %" PRIu32 "!", index);
					state = State::Exit;
					dataman_test->_thread_tests_success = false;
				}
			}

			break;

		case State::Read:

			state = State::ReadWait;
			success = dataman_client->readAsync(DM_KEY_WAYPOINTS_OFFBOARD_0, index, buffer_read, sizeof(buffer_read));

			if (!success) {
				PX4_ERR("readAsync failed for index %" PRIu32 "!", index);
				state = State::Exit;
				dataman_test->_thread_tests_success = false;
			}

			break;

		case State::ReadWait:
			if (dataman_client->lastOperationCompleted(response_success)) {
				state = State::CompareBuffers;

				if (!response_success) {
					PX4_ERR("readAsync failed to get success operation complete for the index %" PRIu32 "!", index);
					state = State::Exit;
					dataman_test->_thread_tests_success = false;
				}
			}

			break;

		case State::CompareBuffers:

			for (uint32_t i = 0; i < g_per_item_size[DM_KEY_WAYPOINTS_OFFBOARD_0]; ++i) {
				if (buffer_write[i] != buffer_read[i]) {
					PX4_ERR("buffer are not the same for index %" PRIu32 "!", index);
					dataman_test->_thread_tests_success = false;
					break;
				}
			}

			state = State::Exit;
			break;

		default:
			break;

		}

		if (hrt_elapsed_time(&start_time) > 2_s) {
			PX4_ERR("Test timeout! index=%" PRIu32, index);
			state = State::Exit;
			dataman_test->_thread_tests_success = false;
		}

		//Simulate rescheduling the task after a 1 ms delay to allow time for the dataman task to operate.
		px4_usleep(1_ms);
	}

	PX4_INFO("Thread %" PRIu32 " finished!", dataman_test->_thread_index);
	px4_usleep(200_ms);

	return nullptr;
}

bool
DatamanTest::testAsyncMutipleClientsWithLocks()
{
	pthread_t thread1{};

	_thread_tests_success = true;
	_thread_index = 0x0;

	// Lock the item in the thread and then do other tests from this scope
	uint32_t ret = pthread_create(&thread1, NULL, &testAsyncWithLocksThread, this);

	if (ret != 0) {
		printf("pthread_create failed: %" PRIu32 "\n", ret);
		_thread_tests_success = false;
	}

	px4_usleep(10_ms); //give thread1 time to lock the item

	hrt_abstime start_time = hrt_absolute_time();
	bool completed;
	bool response_success;
	bool test_success = false;

	// Try lockAsync for already locked item
	_dataman_client1.lockAsync(DM_KEY_WAYPOINTS_OFFBOARD_0);

	while (true) {

		_dataman_client1.update();
		completed = _dataman_client1.lastOperationCompleted(response_success);

		if (completed) {

			if (response_success) {
				PX4_ERR("lockAsync successful, expected to fail!");

			} else {
				test_success = true;
			}

			break;
		}

		if (hrt_elapsed_time(&start_time) > 2_s) {
			PX4_ERR("Test timeout!");
			break;
		}

		//Simulate rescheduling the task after a 1 ms delay to allow time for the dataman task to operate.
		px4_usleep(1_ms);
	}

	if (!test_success) {
		pthread_join(thread1, nullptr);
		return false;
	}

	// Try writeAsync for already locked item
	_dataman_client1.writeAsync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0, _buffer_write, sizeof(_buffer_write));

	while (true) {

		_dataman_client1.update();
		completed = _dataman_client1.lastOperationCompleted(response_success);

		if (completed) {

			if (response_success) {
				PX4_ERR("writeAsync successful, expected to fail!");

			} else {
				test_success = true;
			}

			break;
		}

		if (hrt_elapsed_time(&start_time) > 2_s) {
			PX4_ERR("Test timeout!");
			break;
		}

		//Simulate rescheduling the task after a 1 ms delay to allow time for the dataman task to operate.
		px4_usleep(1_ms);
	}

	if (!test_success) {
		pthread_join(thread1, nullptr);
		return false;
	}

	// Try readAsync for already locked item
	_dataman_client1.readAsync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0, _buffer_read, sizeof(_buffer_read));

	while (true) {

		_dataman_client1.update();
		completed = _dataman_client1.lastOperationCompleted(response_success);

		if (completed) {

			if (response_success) {
				PX4_ERR("readAsync successful, expected to fail!");

			} else {
				test_success = true;
			}

			break;
		}

		if (hrt_elapsed_time(&start_time) > 2_s) {
			PX4_ERR("Test timeout!");
			break;
		}

		//Simulate rescheduling the task after a 1 ms delay to allow time for the dataman task to operate.
		px4_usleep(1_ms);
	}

	if (!test_success) {
		pthread_join(thread1, nullptr);
		return false;
	}

	// Try unlockAsync for already locked item
	_dataman_client1.unlockAsync(DM_KEY_WAYPOINTS_OFFBOARD_0);

	while (true) {

		_dataman_client1.update();
		completed = _dataman_client1.lastOperationCompleted(response_success);

		if (completed) {

			if (response_success) {
				PX4_ERR("unlockAsync successful, expected to fail!");

			} else {
				test_success = true;
			}

			break;
		}

		if (hrt_elapsed_time(&start_time) > 2_s) {
			PX4_ERR("Test timeout!");
			break;
		}

		//Simulate rescheduling the task after a 1 ms delay to allow time for the dataman task to operate.
		px4_usleep(1_ms);
	}

	if (!test_success) {
		pthread_join(thread1, nullptr);
		return false;
	}

	pthread_join(thread1, nullptr);

	return _thread_tests_success;
}

void *DatamanTest::testAsyncWithLocksThread(void *arg)
{
	DatamanTest *dataman_test = (DatamanTest *)arg;
	uint32_t index = (dataman_test->_thread_index)++;
	State state = State::Lock;
	hrt_abstime start_time = hrt_absolute_time();

	uint8_t buffer_read[DM_MAX_DATA_SIZE] = {};
	uint8_t buffer_write[DM_MAX_DATA_SIZE] = {};

	bool success;
	bool response_success;

	DatamanClient *dataman_client{nullptr};

	if (dataman_test->_thread_index == 1) {
		dataman_client = &(dataman_test->_dataman_client_thread1);

	} else if (dataman_test->_thread_index == 2) {
		dataman_client = &(dataman_test->_dataman_client_thread2);

	} else if (dataman_test->_thread_index == 3) {
		dataman_client = &(dataman_test->_dataman_client_thread3);

	} else {
		PX4_ERR("Unknown thread %" PRIu32 "!", dataman_test->_thread_index);
		return nullptr;
	}

	// Prepare write buffer
	for (uint8_t i = 0; i < g_per_item_size[DM_KEY_WAYPOINTS_OFFBOARD_0]; ++i) {
		buffer_write[i] = i + 1;
	}

	while (state != State::Exit) {

		dataman_client->update();

		switch (state) {

		case State::Lock:

			state = State::LockWait;
			success = dataman_client->lockAsync(DM_KEY_WAYPOINTS_OFFBOARD_0);

			if (!success) {
				PX4_ERR("lockAsync failed!");
				state = State::Exit;
				dataman_test->_thread_tests_success = false;
			}

			break;

		case State::LockWait:

			if (dataman_client->lastOperationCompleted(response_success)) {
				state = State::Write;

				if (!response_success) {
					PX4_ERR("lockAsync failed to get success operation complete!");
					state = State::Exit;
					dataman_test->_thread_tests_success = false;
				}
			}

			break;

		case State::Write:

			state = State::WriteWait;
			success = dataman_client->writeAsync(DM_KEY_WAYPOINTS_OFFBOARD_0, index, buffer_write, sizeof(buffer_write));

			if (!success) {
				PX4_ERR("writeAsync failed for index %" PRIu32 "!", index);
				state = State::Exit;
				dataman_test->_thread_tests_success = false;
			}

			break;

		case State::WriteWait:

			if (dataman_client->lastOperationCompleted(response_success)) {
				state = State::Read;

				if (!response_success) {
					PX4_ERR("writeAsync failed to get success operation complete for the index %" PRIu32 "!", index);
					state = State::Exit;
					dataman_test->_thread_tests_success = false;
				}
			}

			break;

		case State::Read:

			state = State::ReadWait;
			success = dataman_client->readAsync(DM_KEY_WAYPOINTS_OFFBOARD_0, index, buffer_read, sizeof(buffer_read));

			if (!success) {
				PX4_ERR("readAsync failed for index %" PRIu32 "!", index);
				state = State::Exit;
				dataman_test->_thread_tests_success = false;
			}

			break;

		case State::ReadWait:
			if (dataman_client->lastOperationCompleted(response_success)) {
				state = State::CompareBuffers;

				if (!response_success) {
					PX4_ERR("readAsync failed to get success operation complete for the index %" PRIu32 "!", index);
					state = State::Exit;
					dataman_test->_thread_tests_success = false;
				}
			}

			break;

		case State::CompareBuffers:

			for (uint32_t i = 0; i < g_per_item_size[DM_KEY_WAYPOINTS_OFFBOARD_0]; ++i) {
				if (buffer_write[i] != buffer_read[i]) {
					PX4_ERR("buffer are not the same for index %" PRIu32 "!", index);
					dataman_test->_thread_tests_success = false;
					break;
				}
			}

			state = State::Unlock;
			break;

		case State::Unlock:
			// Wait before unlocking so that we can perform the tests from another scope
			px4_usleep(500_ms);
			state = State::UnlockWait;
			success = dataman_client->unlockAsync(DM_KEY_WAYPOINTS_OFFBOARD_0);

			if (!success) {
				PX4_ERR("unlockAsync failed!");
				state = State::Exit;
				dataman_test->_thread_tests_success = false;
			}

			break;

		case State::UnlockWait:

			// Please wait some time before unlocking so that we can perform the test from another scope
			if (dataman_client->lastOperationCompleted(response_success)) {
				state = State::Exit;

				if (!response_success) {
					PX4_ERR("unlockAsync failed to get success operation complete!");
					dataman_test->_thread_tests_success = false;
				}
			}

			break;

		default:
			break;

		}

		if (hrt_elapsed_time(&start_time) > 2_s) {
			PX4_ERR("Test timeout! index=%" PRIu32, index);
			state = State::Exit;
			dataman_test->_thread_tests_success = false;
		}

		//Simulate rescheduling the task after a 1 ms delay to allow time for the dataman task to operate.
		px4_usleep(1_ms);
	}

	PX4_INFO("Thread %" PRIu32 " finished!", dataman_test->_thread_index);
	px4_usleep(200_ms);

	return nullptr;
}

bool
DatamanTest::testAsyncWriteReadAllItemsMaxSize()
{
	bool success = false;
	State state = State::Write;

	volatile uint32_t item = DM_KEY_SAFE_POINTS;
	volatile uint32_t index = 0U;

	hrt_abstime start_time = hrt_absolute_time();

	//While loop represents a task
	while (state != State::Exit) {

		_dataman_client1.update();

		switch (state) {

		case State::Write:

			state = State::WriteWait;

			// Prepare write buffer
			for (uint32_t i = 0; i < g_per_item_size[item]; ++i) {
				_buffer_write[i] = (uint8_t)(index % UINT8_MAX);
			}

			success = _dataman_client1.writeAsync((dm_item_t)item, index, _buffer_write, g_per_item_size[item]);

			if (!success) {
				return false;
			}

			break;

		case State::Read:
			state = State::ReadWait;
			success = _dataman_client1.readAsync((dm_item_t)item, index, _buffer_read, g_per_item_size[item]);

			if (!success) {
				return false;
			}

			break;

		case State::ReadWait:
			if (_dataman_client1.lastOperationCompleted(_response_success)) {
				if (!_response_success) {
					return false;
				}

				state = State::CompareBuffers;
			}

			break;

		case State::WriteWait:
			if (_dataman_client1.lastOperationCompleted(_response_success)) {
				if (!_response_success) {
					return false;
				}

				state = State::Read;
			}

			break;

		case State::CompareBuffers:
			state = State::Write;

			for (uint32_t i = 0U; i < g_per_item_size[item]; ++i) {

				if (_buffer_write[i] != _buffer_read[i]) {
					PX4_ERR("readAsync failed at item = %" PRIu32 ", index =  %" PRIu32 ", element= %" PRIu32 ", expected:  %" PRIu8
						", received:  %" PRIu8, item, index, i, _buffer_write[i], _buffer_read[i]);
					return false;
				}
			}

			if (index < g_per_item_max_index[item] - 1) {
				++index;

			} else {

				if (item < DM_KEY_NUM_KEYS - 1) {
					index = 0U;
					++item;

				} else {
					state = State::Exit;
				}
			}

			break;

		default:
			break;
		}

		if (hrt_elapsed_time(&start_time) > 20_s) {
			PX4_ERR("Test timeout!");
			return false;
		}

		//Simulate rescheduling the task after a 1 ms delay to allow time for the dataman task to operate.
		px4_usleep(1_ms);

	}

	return success;
}

bool
DatamanTest::testAsyncClearAll()
{
	bool success = true;

	State state = State::Clear;
	hrt_abstime start_time = hrt_absolute_time();
	volatile uint32_t item = DM_KEY_SAFE_POINTS;

	//While loop represents a task
	while (state != State::Exit) {

		_dataman_client1.update();

		switch (state) {

		case State::Clear:

			state = State::ClearWait;
			success = _dataman_client1.clearAsync((dm_item_t)item);

			if (!success) {
				PX4_ERR("Failed at item %" PRIu32, item);
				return false;
			}

			break;

		case State::ClearWait:
			if (_dataman_client1.lastOperationCompleted(_response_success)) {
				state = State::OperationCompleted;

				if (!_response_success) {
					PX4_ERR("Failed at item %" PRIu32, item);
					return false;
				}
			}

			break;

		case State::OperationCompleted:
			if (item < DM_KEY_NUM_KEYS - 1) {
				state = State::Clear;
				++item;

			} else {
				state = State::Exit;
			}

			break;

		default:
			break;

		}

		if (hrt_elapsed_time(&start_time) > 5_s) {
			PX4_ERR("Test timeout!");
			return false;
		}

		//Simulate rescheduling the task after a 1 ms delay to allow time for the dataman task to operate.
		px4_usleep(1_ms);
	}

	return true;
}

//Cache
bool
DatamanTest::testCache()
{
	bool success = false;
	dm_item_t item = DM_KEY_WAYPOINTS_OFFBOARD_0;
	uint32_t uniq_number = 13; // Use this to make sure stored data is from this test

	for (uint32_t index = 0; index < 15; ++index) {
		uint8_t value = index + uniq_number;
		memset(_buffer_write, value, sizeof(_buffer_write));
		success = _dataman_cache.client().writeSync(item, index, _buffer_write, sizeof(_buffer_write));

		if (!success) {
			return false;
		}
	}

	// Write one extra for loadWait with timeout
	uint32_t extra_index = 100;
	_buffer_write[0] = 123;
	success = _dataman_cache.client().writeSync(item, extra_index, _buffer_write, sizeof(_buffer_write));

	if (!success) {
		return false;
	}

	// Load cache
	for (uint32_t index = 0; index < _cache_size; ++index) {
		_dataman_cache.load(item, index);
	}

	hrt_abstime start_time = hrt_absolute_time();

	// loop represents the task, we collect the data
	while (hrt_elapsed_time(&start_time) < 500_ms) {

		px4_usleep(1_ms);
		_dataman_cache.update();
	}

	// check cached data
	for (uint32_t index = 0; index < _cache_size; ++index) {
		uint8_t value = index + uniq_number;
		success = _dataman_cache.loadWait(item, index, _buffer_read, sizeof(_buffer_read));

		if (!success) {
			PX4_ERR("Failed loadWait at index %" PRIu32, index);
			return false;
		}

		for (uint32_t i = 0; i < sizeof(_buffer_read); ++i) {
			if (_buffer_read[i] != value) {
				PX4_ERR("Wrong data recived %" PRIu8" , expected %" PRIu8, _buffer_read[i], value);
				return false;
			}
		}
	}

	// expected to fail without timeout set
	success = _dataman_cache.loadWait(item, extra_index, _buffer_read, sizeof(_buffer_read));

	if (success) {
		PX4_ERR("loadWait unexpectedly succeeded");
		return false;
	}

	// expected to success with timeout set
	success = _dataman_cache.loadWait(item, extra_index, _buffer_read, sizeof(_buffer_read), 100_ms);

	if (!success) {
		PX4_ERR("loadWait failed");
		return false;
	}

	uint32_t old_cache_size = _cache_size;
	_cache_size = 5;
	_dataman_cache.resize(_cache_size);

	// check cached data after resize (reduced)
	for (uint32_t index = 0; index < _cache_size; ++index) {
		uint8_t value = index + uniq_number;
		success = _dataman_cache.loadWait(item, index, _buffer_read, sizeof(_buffer_read));

		if (!success) {
			PX4_ERR("Failed loadWait at index %" PRIu32, index);
			return false;
		}

		for (uint32_t i = 0; i < sizeof(_buffer_read); ++i) {
			if (_buffer_read[i] != value) {
				PX4_ERR("Wrong data recived %" PRIu8" , expected %" PRIu8, _buffer_read[i], value);
				return false;
			}
		}
	}

	for (uint32_t index = _cache_size; index < old_cache_size; ++index) {
		uint8_t value = index + uniq_number;
		success = _dataman_cache.loadWait(item, index, _buffer_read, sizeof(_buffer_read));

		if (success) {
			PX4_ERR("loadWait unexpectedly succeeded at index %" PRIu32, index);
			return false;
		}
	}

	_cache_size = 15;
	_dataman_cache.resize(_cache_size);

	// Load cache
	for (uint32_t index = 0; index < _cache_size; ++index) {
		_dataman_cache.load(item, index);
	}

	start_time = hrt_absolute_time();

	// loop represents the task, we collect the data
	while (hrt_elapsed_time(&start_time) < 500_ms) {

		px4_usleep(1_ms);
		_dataman_cache.update();
	}

	// check cached data
	for (uint32_t index = 0; index < _cache_size; ++index) {
		uint8_t value = index + uniq_number;
		success = _dataman_cache.loadWait(item, index, _buffer_read, sizeof(_buffer_read));

		if (!success) {
			PX4_ERR("Failed loadWait at index %" PRIu32, index);
			return false;
		}

		for (uint32_t i = 0; i < sizeof(_buffer_read); ++i) {
			if (_buffer_read[i] != value) {
				PX4_ERR("Wrong data recived %" PRIu8" , expected %" PRIu8, _buffer_read[i], value);
				return false;
			}
		}
	}

	// invalidate and check cached data
	_dataman_cache.invalidate();

	for (uint32_t index = 0; index < _cache_size; ++index) {
		uint8_t value = index + uniq_number;
		success = _dataman_cache.loadWait(item, index, _buffer_read, sizeof(_buffer_read));

		// expected to fail
		if (success) {
			PX4_ERR("loadWait unexpectedly succeeded at index %" PRIu32, index);
			return false;
		}

	}

	return true;
}

bool
DatamanTest::testResetItems()
{
	bool success = false;

	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	mission.count = 0;
	mission.current_seq = 0;

	success = _dataman_client1.writeSync(DM_KEY_MISSION_STATE, 0, reinterpret_cast<uint8_t *>(&mission), sizeof(mission_s));

	if (!success) {
		PX4_ERR("failed to reset DM_KEY_MISSION_STATE");
		return false;
	}

	success = _dataman_client1.readSync(DM_KEY_MISSION_STATE, 0, reinterpret_cast<uint8_t *>(&mission), sizeof(mission_s));

	if (!success) {
		PX4_ERR("failed to read DM_KEY_MISSION_STATE");
		return false;
	}

	mission_stats_entry_s stats;
	stats.num_items = 0;
	stats.update_counter = 1;

	success = _dataman_client1.writeSync(DM_KEY_FENCE_POINTS, 0, reinterpret_cast<uint8_t *>(&stats),
					     sizeof(mission_stats_entry_s));

	if (!success) {
		PX4_ERR("failed to reset DM_KEY_FENCE_POINTS");
		return false;
	}

	success = _dataman_client1.writeSync(DM_KEY_SAFE_POINTS, 0, reinterpret_cast<uint8_t *>(&stats),
					     sizeof(mission_stats_entry_s));

	if (!success) {
		PX4_ERR("failed to reset DM_KEY_SAFE_POINTS");
		return false;
	}

	return success;
}

bool DatamanTest::run_tests()
{
	ut_run_test(testSyncReadInvalidItem);
	ut_run_test(testSyncWriteInvalidItem);
	ut_run_test(testSyncReadInvalidIndex);
	ut_run_test(testSyncWriteInvalidIndex);
	ut_run_test(testSyncReadBufferOverflow);
	ut_run_test(testSyncWriteBufferOverflow);
	ut_run_test(testSyncMutipleClients);
	ut_run_test(testSyncWriteReadAllItemsMaxSize);
	ut_run_test(testSyncClearAll);

	ut_run_test(testAsyncReadInvalidItem);
	ut_run_test(testAsyncWriteInvalidItem);
	ut_run_test(testAsyncReadInvalidIndex);
	ut_run_test(testAsyncWriteInvalidIndex);
	ut_run_test(testAsyncReadBufferOverflow);
	ut_run_test(testAsyncWriteBufferOverflow);
	ut_run_test(testAsyncMutipleClientsNoLocks);
	ut_run_test(testAsyncMutipleClientsWithLocks);
	ut_run_test(testAsyncWriteReadAllItemsMaxSize);
	ut_run_test(testAsyncClearAll);

	ut_run_test(testCache);

	ut_run_test(testResetItems);

	return (_tests_failed == 0);
}

ut_declare_test_c(test_dataman, DatamanTest)
