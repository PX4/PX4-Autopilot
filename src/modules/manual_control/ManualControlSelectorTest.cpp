/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "ManualControlSelector.hpp"
#include <gtest/gtest.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

static constexpr uint64_t some_time = 12345678;

TEST(ManualControlSelector, RcInputContinuous)
{
	ManualControlSelector selector;
	selector.setRcInMode(0);
	selector.setTimeout(500_ms);

	uint64_t timestamp = some_time;

	// Now provide input with the correct source.
	manual_control_setpoint_s input {};
	input.data_source = manual_control_setpoint_s::SOURCE_RC;
	input.timestamp_sample = timestamp;

	for (int i = 0; i < 5; i++) {
		selector.updateWithNewInputSample(timestamp, input, 1);
		EXPECT_TRUE(selector.setpoint().valid);
		EXPECT_EQ(selector.setpoint().timestamp_sample, timestamp);
		EXPECT_EQ(selector.instance(), 1);
		EXPECT_TRUE(selector.setpoint().data_source == manual_control_setpoint_s::SOURCE_RC);
		timestamp += 100_ms;
		input.timestamp_sample = timestamp;
	}
}

TEST(ManualControlSelector, RcInputOnly)
{
	ManualControlSelector selector;
	selector.setRcInMode(0);
	selector.setTimeout(500_ms);

	uint64_t timestamp = some_time;

	manual_control_setpoint_s input {};
	input.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0;
	input.timestamp_sample = timestamp;
	selector.updateWithNewInputSample(timestamp, input, 0);

	EXPECT_FALSE(selector.setpoint().valid);

	timestamp += 100_ms;

	// Now provide input with the correct source.
	input.data_source = manual_control_setpoint_s::SOURCE_RC;
	input.timestamp_sample = timestamp;
	selector.updateWithNewInputSample(timestamp, input, 1);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_TRUE(selector.setpoint().data_source == manual_control_setpoint_s::SOURCE_RC);
	EXPECT_EQ(selector.instance(), 1);
}

TEST(ManualControlSelector, MavlinkInputOnly)
{
	ManualControlSelector selector;
	selector.setRcInMode(1);
	selector.setTimeout(500_ms);

	uint64_t timestamp = some_time;

	manual_control_setpoint_s input {};
	input.data_source = manual_control_setpoint_s::SOURCE_RC;
	input.timestamp_sample = timestamp;
	selector.updateWithNewInputSample(timestamp, input, 0);

	EXPECT_FALSE(selector.setpoint().valid);

	timestamp += 100_ms;

	// Now provide input with the correct source.
	input.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_3;
	input.timestamp_sample = timestamp;
	selector.updateWithNewInputSample(timestamp, input, 1);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_TRUE(selector.setpoint().data_source == manual_control_setpoint_s::SOURCE_MAVLINK_3);
	EXPECT_EQ(selector.instance(), 1);

	timestamp += 100_ms;

	// But only the first MAVLink source wins, others are too late.
	input.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_4;
	input.timestamp_sample = timestamp;
	selector.updateWithNewInputSample(timestamp, input, 1);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_TRUE(selector.setpoint().data_source == manual_control_setpoint_s::SOURCE_MAVLINK_3);
	EXPECT_EQ(selector.instance(), 1);
}

TEST(ManualControlSelector, AutoInput)
{
	ManualControlSelector selector;
	selector.setRcInMode(2);
	selector.setTimeout(500_ms);

	uint64_t timestamp = some_time;

	manual_control_setpoint_s input {};
	input.data_source = manual_control_setpoint_s::SOURCE_RC;
	input.timestamp_sample = timestamp;
	selector.updateWithNewInputSample(timestamp, input, 0);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_TRUE(selector.setpoint().data_source == manual_control_setpoint_s::SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);

	timestamp += 100_ms;

	// Now provide input from MAVLink as well which should get ignored.
	input.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0;
	input.timestamp_sample = timestamp;
	selector.updateWithNewInputSample(timestamp, input, 1);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_TRUE(selector.setpoint().data_source == manual_control_setpoint_s::SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);

	timestamp += 500_ms;

	// Now we'll let RC time out, so it should switch to MAVLINK.
	input.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0;
	input.timestamp_sample = timestamp;
	selector.updateWithNewInputSample(timestamp, input, 1);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_TRUE(selector.setpoint().data_source == manual_control_setpoint_s::SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), 1);
}

TEST(ManualControlSelector, FirstInput)
{
	ManualControlSelector selector;
	selector.setRcInMode(3);
	selector.setTimeout(500_ms);

	uint64_t timestamp = some_time;

	manual_control_setpoint_s input {};
	input.data_source = manual_control_setpoint_s::SOURCE_RC;
	input.timestamp_sample = timestamp;
	selector.updateWithNewInputSample(timestamp, input, 0);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_TRUE(selector.setpoint().data_source == manual_control_setpoint_s::SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);

	timestamp += 100_ms;

	// Now provide input from MAVLink as well which should get ignored.
	input.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0;
	input.timestamp_sample = timestamp;
	selector.updateWithNewInputSample(timestamp, input, 1);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_TRUE(selector.setpoint().data_source == manual_control_setpoint_s::SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);

	timestamp += 500_ms;

	// Now we'll let RC time out, but it should NOT switch to MAVLINK because RC was first
	input.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0;
	input.timestamp_sample = timestamp;
	selector.updateWithNewInputSample(timestamp, input, 1);

	EXPECT_FALSE(selector.setpoint().valid);
	EXPECT_FALSE(selector.setpoint().data_source == manual_control_setpoint_s::SOURCE_MAVLINK_0);
	EXPECT_EQ(selector.instance(), -1);

	timestamp += 100_ms;

	// Provide input from RC again and it should get accepted because it was the first.
	input.data_source = manual_control_setpoint_s::SOURCE_RC;
	input.timestamp_sample = timestamp;
	selector.updateWithNewInputSample(timestamp, input, 0);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_TRUE(selector.setpoint().data_source == manual_control_setpoint_s::SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);
}

TEST(ManualControlSelector, DisabledInput)
{
	ManualControlSelector selector;
	selector.setRcInMode(4);
	selector.setTimeout(500_ms);

	uint64_t timestamp = some_time;

	manual_control_setpoint_s input {};
	// Reject MAVLink stick input
	input.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0;
	input.timestamp_sample = timestamp;
	selector.updateWithNewInputSample(timestamp, input, 0);

	EXPECT_FALSE(selector.setpoint().valid);
	EXPECT_EQ(selector.instance(), -1);

	timestamp += 100_ms;

	// Reject RC stick input
	input.data_source = manual_control_setpoint_s::SOURCE_RC;
	input.timestamp_sample = timestamp;
	selector.updateWithNewInputSample(timestamp, input, 1);

	EXPECT_FALSE(selector.setpoint().valid);
	EXPECT_EQ(selector.instance(), -1);
}

TEST(ManualControlSelector, RcTimeout)
{
	ManualControlSelector selector;
	selector.setRcInMode(0);
	selector.setTimeout(500_ms);

	uint64_t timestamp = some_time;

	manual_control_setpoint_s input {};
	input.data_source = manual_control_setpoint_s::SOURCE_RC;
	input.timestamp_sample = timestamp;
	selector.updateWithNewInputSample(timestamp, input, 0);

	EXPECT_TRUE(selector.setpoint().valid);
	EXPECT_TRUE(selector.setpoint().data_source == manual_control_setpoint_s::SOURCE_RC);
	EXPECT_EQ(selector.instance(), 0);

	timestamp += 600_ms;

	// Update, to make sure it notices the timeout
	selector.updateValidityOfChosenInput(timestamp);

	EXPECT_FALSE(selector.setpoint().valid);
	EXPECT_EQ(selector.instance(), -1);
}

TEST(ManualControlSelector, RcOutdated)
{
	ManualControlSelector selector;
	selector.setRcInMode(0);
	selector.setTimeout(500_ms);

	uint64_t timestamp = some_time;

	manual_control_setpoint_s input {};
	input.data_source = manual_control_setpoint_s::SOURCE_RC;
	input.timestamp_sample = timestamp - 600_ms; // First sample is already outdated
	selector.updateWithNewInputSample(timestamp, input, 0);

	EXPECT_FALSE(selector.setpoint().valid);
	EXPECT_EQ(selector.instance(), -1);

	// If we update again it should still not get accepted
	selector.updateWithNewInputSample(timestamp, input, 0);

	EXPECT_FALSE(selector.setpoint().valid);
	EXPECT_EQ(selector.instance(), -1);
}
