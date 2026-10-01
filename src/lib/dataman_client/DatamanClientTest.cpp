/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file DatamanClientTest.cpp
 *
 * dataman identifies its clients by an 8 bit ID. Each client gets one when it is created and hands
 * it back when it is destroyed, so a process can create clients indefinitely.
 */

#include <gtest/gtest.h>

#include <dataman_client/DatamanClient.hpp>
#include <dataman/dataman.h>
#include <parameters/param.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>

#include <memory>
#include <vector>

extern "C" int dataman_main(int argc, char *argv[]);

class DatamanClientTest : public ::testing::Test
{
protected:
	static void SetUpTestSuite()
	{
		param_control_autosave(false);
		px4::WorkQueueManagerStart();
		char name[] = "dataman";
		char start[] = "start";
		char ram[] = "-r";
		char *argv[] = {name, start, ram};
		dataman_main(3, argv);
	}

	static void TearDownTestSuite()
	{
		char name[] = "dataman";
		char stop[] = "stop";
		char *argv[] = {name, stop};
		dataman_main(2, argv);
		px4::WorkQueueManagerStop();
	}

	// a client without an ID cannot write, so a successful write shows it got one
	static bool canWrite(DatamanClient &client, uint8_t marker)
	{
		uint8_t payload[4] = {marker, 0, 0, 0};
		return client.writeSync(DM_KEY_MISSION_STATE, 0, payload, sizeof(payload));
	}
};

// WHY: IDs used to come from a counter that never went back, so the 255th client a process ever
// created got none and every request it made failed. A test binary that builds a navigator for each
// case reached that after about forty cases.
// WHAT: clients created and destroyed one after another keep getting IDs well past 255.
TEST_F(DatamanClientTest, IdsAreReusedAfterAClientIsDestroyed)
{
	for (int i = 0; i < 600; i++) {
		DatamanClient client;
		ASSERT_TRUE(canWrite(client, static_cast<uint8_t>(i))) << "client " << i << " did not get an ID";
	}
}

// WHAT: an ID is only handed back once its client is gone, so clients alive together all have one.
TEST_F(DatamanClientTest, ClientsAliveTogetherAllHaveIds)
{
	std::vector<std::unique_ptr<DatamanClient>> clients;

	for (int i = 0; i < 200; i++) {
		clients.push_back(std::make_unique<DatamanClient>());
		ASSERT_TRUE(canWrite(*clients.back(), static_cast<uint8_t>(i))) << "client " << i << " did not get an ID";
	}

	clients.clear();

	for (int i = 0; i < 200; i++) {
		clients.push_back(std::make_unique<DatamanClient>());
		ASSERT_TRUE(canWrite(*clients.back(), static_cast<uint8_t>(i))) << "second round client " << i << " did not get an ID";
	}
}
