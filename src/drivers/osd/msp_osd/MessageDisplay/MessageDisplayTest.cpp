/****************************************************************************
 *
 *   Copyright (C) 2022 PX4 Development Team. All rights reserved.
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

#include "MessageDisplay.hpp"

// default parameters
#define PERIOD 125000
#define DWELL  500000

// use base namespace
using namespace msp_osd;

// core test setup class
class MessageDisplayTest : public ::testing::Test
{
public:
	// test copy of the message display object
	std::unique_ptr<MessageDisplay> md_;

	// setup a new test
	void SetUp() override
	{
		// construct new default display
		md_ = std::make_unique<MessageDisplay>(PERIOD, DWELL);
		return;
	}
};

// check basic construction
TEST_F(MessageDisplayTest, testMessageDisplayConstruction)
{
	// make sure we have an object!
	ASSERT_TRUE(static_cast<bool>(md_));

	// verify default message
	char message[FULL_MSG_LENGTH];
	md_->get(message, 0);
	EXPECT_STREQ(message, "INITIALIZING");
}

// check setting and getting various status messages
TEST_F(MessageDisplayTest, testMessageDisplayStatic)
{
	// make sure we have an object!
	ASSERT_TRUE(static_cast<bool>(md_));

	// initialize output string
	char message[FULL_MSG_BUFFER];

	// set a random flight mode that's too short
	md_->set(MessageDisplayType::FLIGHT_MODE, "S");
	md_->get(message, 0);
	EXPECT_STREQ(message, "S|????|??");

	// set a random flight mode that's too long
	md_->set(MessageDisplayType::FLIGHT_MODE, "SOSSOS");
	md_->get(message, 0);
	EXPECT_STREQ(message, "SOS|????|??");

	// set a random flight mode that's just right
	md_->set(MessageDisplayType::FLIGHT_MODE, "SOS");
	md_->get(message, 0);
	EXPECT_STREQ(message, "SOS|????|??");

	// set a random arming status that's too short
	md_->set(MessageDisplayType::ARMING, "DS");
	md_->get(message, 0);
	EXPECT_STREQ(message, "SOS|DS|??");

	// set a random arming status that's too long
	md_->set(MessageDisplayType::ARMING, "DISARM");
	md_->get(message, 0);
	EXPECT_STREQ(message, "SOS|DISA|??");

	// set a random arming status that's just right
	md_->set(MessageDisplayType::ARMING, "DSRM");
	md_->get(message, 0);
	EXPECT_STREQ(message, "SOS|DSRM|??");

	// set a random heading that's too short
	md_->set(MessageDisplayType::HEADING, "N");
	md_->get(message, 0);
	EXPECT_STREQ(message, "SOS|DSRM|N");

	// set a random heading that's too short
	md_->set(MessageDisplayType::HEADING, "NBNW");
	md_->get(message, 0);
	EXPECT_STREQ(message, "SOS|DSRM|NB");

	// set a random heading that's too short
	md_->set(MessageDisplayType::HEADING, "NW");
	md_->get(message, 0);
	EXPECT_STREQ(message, "SOS|DSRM|NW");
}

// check setting and getting for a warning message
TEST_F(MessageDisplayTest, testMessageDisplayWarning)
{
	// make sure we have an object!
	ASSERT_TRUE(static_cast<bool>(md_));

	// set just a warning message, but a very long one
	md_->set(MessageDisplayType::WARNING,
		 "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.");

	// the ground truth message is the following:
	const char ground_truth[MSG_BUFFER_SIZE] =
		"???|????|??   WARN: Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea comm            ";

	// walk through the message as it scrolls
	char message[FULL_MSG_BUFFER];  // output message
	char correct[FULL_MSG_BUFFER];  // correct value of output message
	uint32_t stamp = DWELL - 1;     // start at the end of the DWELL time

	for (size_t i = 0; i != MSG_BUFFER_SIZE - FULL_MSG_LENGTH; ++i) {
		// get updated message for this time
		md_->get(message, stamp);

		// get substring that we should be seeing
		strncpy(correct, &ground_truth[i], FULL_MSG_LENGTH);
		EXPECT_STREQ(message, correct);

		// update time
		stamp += PERIOD;
	}

	// verify that we wrap around as expected at the end
	md_->get(message, stamp);
	strncpy(correct, ground_truth, FULL_MSG_LENGTH);
	EXPECT_STREQ(message, correct);
}

