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

#include <gtest/gtest.h>
#include <lib/sensor_slot_binder/SensorSlotBinder.hpp>

static constexpr const char *kIdParamFormat = "SENS_FLOW%u_ID";
static constexpr uint32_t kDeviceA = 0x0A0B01;
static constexpr uint32_t kDeviceB = 0x0A0B02;
static constexpr uint32_t kDeviceC = 0x0A0B03;

class SensorSlotBinderTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);

		for (uint8_t slot = 0; slot < 2; slot++) {
			param_reset(idParamHandle(slot));
		}
	}

	param_t idParamHandle(uint8_t slot)
	{
		char param_name[17] {};
		snprintf(param_name, sizeof(param_name), kIdParamFormat, static_cast<unsigned>(slot));
		return param_find(param_name);
	}

	int32_t idParam(uint8_t slot)
	{
		int32_t id = 0;
		param_get(idParamHandle(slot), &id);
		return id;
	}

	void setIdParam(uint8_t slot, int32_t id)
	{
		param_set_no_notification(idParamHandle(slot), &id);
	}
};

TEST_F(SensorSlotBinderTest, bindsUnknownSensorsInArrivalOrder)
{
	SensorSlotBinder binder;
	binder.init(kIdParamFormat, 2);

	// the first sensor reporting data takes the first free slot, whatever its uORB instance
	EXPECT_EQ(binder.slotForInstance(1, kDeviceA), 0);
	EXPECT_EQ(binder.slotForInstance(0, kDeviceB), 1);

	// and the assignment is persisted
	EXPECT_EQ(idParam(0), static_cast<int32_t>(kDeviceA));
	EXPECT_EQ(idParam(1), static_cast<int32_t>(kDeviceB));
	EXPECT_TRUE(binder.isSlotBound(0));
	EXPECT_TRUE(binder.isSlotBound(1));
}

TEST_F(SensorSlotBinderTest, keepsPersistedBinding)
{
	setIdParam(1, kDeviceA);

	SensorSlotBinder binder;
	binder.init(kIdParamFormat, 2);

	EXPECT_EQ(binder.slotForInstance(0, kDeviceA), 1);
	EXPECT_EQ(binder.slotForInstance(1, kDeviceB), 0);
}

TEST_F(SensorSlotBinderTest, rejectsSensorWithoutDeviceId)
{
	SensorSlotBinder binder;
	binder.init(kIdParamFormat, 2);

	EXPECT_EQ(binder.slotForInstance(0, 0), -1);
	EXPECT_FALSE(binder.isSlotBound(0));
	EXPECT_EQ(idParam(0), 0);
}

TEST_F(SensorSlotBinderTest, fallsBackToInstanceWithoutDeviceId)
{
	SensorSlotBinder binder;
	binder.init(kIdParamFormat, 2);

	// a sensor without a device ID keeps its uORB instance as long as that slot is free
	EXPECT_EQ(binder.slotForInstanceWithFallback(1, 0), 1);
	EXPECT_FALSE(binder.isSlotBound(1));

	// but never takes a slot bound to another sensor
	EXPECT_EQ(binder.slotForInstanceWithFallback(0, kDeviceA), 0);
	EXPECT_EQ(binder.slotForInstanceWithFallback(1, kDeviceB), 1);
	EXPECT_EQ(binder.slotForInstanceWithFallback(1, 0), -1);
}

TEST_F(SensorSlotBinderTest, followsDeviceChangeOnInstance)
{
	SensorSlotBinder binder;
	binder.init(kIdParamFormat, 2);

	EXPECT_EQ(binder.slotForInstance(0, kDeviceA), 0);
	EXPECT_EQ(binder.slotForInstance(0, kDeviceB), 1);
	EXPECT_EQ(binder.slotForInstance(0, kDeviceA), 0);
}

TEST_F(SensorSlotBinderTest, rejectsSensorWhenAllSlotsBound)
{
	SensorSlotBinder binder;
	binder.init(kIdParamFormat, 2);

	EXPECT_EQ(binder.slotForInstance(0, kDeviceA), 0);
	EXPECT_EQ(binder.slotForInstance(1, kDeviceB), 1);
	EXPECT_EQ(binder.slotForInstance(0, kDeviceC), -1);
	EXPECT_EQ(binder.slotForInstance(1, kDeviceB), 1);
}

TEST_F(SensorSlotBinderTest, claimsSlotWhenBindingCannotBePersisted)
{
	// no such parameter: the binding lives in RAM only
	SensorSlotBinder binder;
	binder.init("NOPARAM%u_ID", 2);

	// a second sensor must not share the slot of the first one
	EXPECT_EQ(binder.slotForInstance(0, kDeviceA), 0);
	EXPECT_EQ(binder.slotForInstance(1, kDeviceB), 1);
	EXPECT_TRUE(binder.isSlotBound(0));
}
