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
	bool testSyncReadInvalidItem();
	bool testSyncWriteInvalidItem();

	bool testSyncReadInvalidIndex();
	bool testSyncWriteInvalidIndex();

	bool testSyncReadBufferOverflow();
	bool testSyncWriteBufferOverflow();

	bool testMutipleClients();

	bool testSyncWriteReadAllItemsMaxSize();

	bool testSyncClearAll();

	bool testResetItems();

	static void *testLockThread(void *arg);

	DatamanClient _dataman_client1{};
	DatamanClient _dataman_client2{};
	DatamanClient _dataman_client3{};
	DatamanClient _dataman_client_thread1{};

	static constexpr uint32_t DM_MAX_DATA_SIZE{MISSION_ITEM_SIZE};
	static_assert(sizeof(dataman_response_s::data) == DM_MAX_DATA_SIZE, "data size != DM_MAX_DATA_SIZE");

	uint8_t _buffer_read[DM_MAX_DATA_SIZE];
	uint8_t _buffer_write[DM_MAX_DATA_SIZE];

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
DatamanTest::testMutipleClients()
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
		return false;
	}

	//Should be able to lock since the task in the thread should unlock the item in the meantime
	success = _dataman_client1.lockSync(DM_KEY_MISSION_STATE);

	if (!success) {
		return false;
	}

	success = _dataman_client1.unlockSync(DM_KEY_MISSION_STATE);

	if (!success) {
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
		for (volatile uint32_t index = 0U; index < g_per_item_max_index[item]; ++index) {

			success = _dataman_client1.readSync((dm_item_t)item, index, _buffer_read, g_per_item_size[item]);

			if (!success) {
				PX4_ERR("readSync failed at item =  %" PRIu32 ", index = %" PRIu32, item, index);
				return false;
			}

			// Check read buffer
			for (volatile uint32_t i = 0U; i < g_per_item_size[item]; ++i) {

				uint8_t expected_value = (index % UINT8_MAX);

				if (expected_value != _buffer_read[i]) {
					PX4_ERR("readSync failed at item = %" PRIu32 ", index =  %" PRIu32 ", element= %" PRIu32 ", expected:  %" PRIu8
						", received:  %" PRIu8,
						item, index, i, expected_value, _buffer_read[i]);
					return false;
				}
			}
		}
	}

	return true;;
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

	ut_run_test(testMutipleClients);

	ut_run_test(testSyncWriteReadAllItemsMaxSize);

	ut_run_test(testSyncClearAll);

	ut_run_test(testResetItems);

	return (_tests_failed == 0);
}

ut_declare_test_c(test_dataman, DatamanTest)
