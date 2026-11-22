/****************************************************************************
*
*   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file NnControlStartupTest.cpp
 *
 * The observation check runs on every cycle from the first one, before any of the
 * three observation topics has necessarily published. The caches it reads start
 * empty, so that first check reports a missing position instead of reading whatever
 * the memory held.
 */

#include <gtest/gtest.h>
#include <cstring>
#include <new>

#include "mc_nn_control.hpp"

#include <drivers/drv_hrt.h>
#include <hrt_work.h>
#include <parameters/param.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>

class NnControlStartupTestPeer
{
public:
	static nn_control::ObservationFault checkObservations(MulticopterNeuralNetworkControl &module)
	{
		module.CheckObservations();
		return module._observation_fault;
	}
};

class NnControlStartupTestEnvironment : public ::testing::Environment
{
public:
	void TearDown() override
	{
		px4::WorkQueueManagerStop();
	}
};

static const auto *global_env = ::testing::AddGlobalTestEnvironment(new NnControlStartupTestEnvironment());

class NnControlStartupTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		static bool work_queue_started = false;

		if (!work_queue_started) {
			// The module is a scheduled work item, so its construction needs the work
			// queue manager, and the manager's timer lock needs hrt_init() first
			hrt_init();
			ASSERT_EQ(px4::WorkQueueManagerStart(), 0);
			work_queue_started = true;
		}

		param_control_autosave(false);
	}
};

TEST_F(NnControlStartupTest, nothingPublishedIsReportedAsPositionInvalid)
{
	// The module as it is right after construction, before init() and before any of
	// the observation topics has published. Its storage is filled beforehand so the
	// outcome does not depend on what the stack happened to hold, which is how the
	// caches were read before they were initialised.
	alignas(MulticopterNeuralNetworkControl) static unsigned char storage[sizeof(MulticopterNeuralNetworkControl)];
	memset(storage, 0xff, sizeof(storage));
	MulticopterNeuralNetworkControl *module = new (storage) MulticopterNeuralNetworkControl();

	EXPECT_EQ(NnControlStartupTestPeer::checkObservations(*module), nn_control::ObservationFault::PositionInvalid);

	module->~MulticopterNeuralNetworkControl();
}
