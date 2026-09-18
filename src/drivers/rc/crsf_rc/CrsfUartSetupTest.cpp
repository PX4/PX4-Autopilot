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
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
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
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include "CrsfUartSetup.hpp"

namespace
{

// Records setup effects so tests can verify which operations follow each outcome.
class FakeSerial
{
public:
	bool isOpen() const { return is_open; }
	bool setBaudrate(uint32_t value) { baudrate = value; ++set_baudrate_calls; return set_baudrate_result; }
	bool open() { ++open_calls; resource_acquired = true; is_open = open_result; return open_result; }
	bool close() { ++close_calls; resource_acquired = false; is_open = false; return true; }
	void setSwapRxTxMode() { ++swap_calls; }
	void setSingleWireMode() { ++singlewire_calls; }
	void flush() { ++flush_calls; }

	bool is_open{false};
	bool resource_acquired{false};
	bool set_baudrate_result{true};
	bool open_result{true};
	uint32_t baudrate{0};
	int set_baudrate_calls{0};
	int open_calls{0};
	int close_calls{0};
	int swap_calls{0};
	int singlewire_calls{0};
	int flush_calls{0};
};

TEST(CrsfUartSetupTest, LeavesAnOpenPortAlone)
{
	FakeSerial serial;
	serial.is_open = true;
	EXPECT_EQ(CrsfUartSetup(serial, 420000, true, true), CrsfUartSetupResult::AlreadyOpen);
	EXPECT_EQ(serial.set_baudrate_calls, 0);
	EXPECT_EQ(serial.open_calls, 0);
	EXPECT_EQ(serial.flush_calls, 0);
}

TEST(CrsfUartSetupTest, ReportsBaudrateFailureWithoutTryingToOpen)
{
	FakeSerial serial;
	serial.set_baudrate_result = false;
	EXPECT_EQ(CrsfUartSetup(serial, 420000, false, false), CrsfUartSetupResult::BaudrateError);
	EXPECT_EQ(serial.baudrate, 420000u);
	EXPECT_EQ(serial.open_calls, 0);
	EXPECT_EQ(serial.flush_calls, 0);
}

TEST(CrsfUartSetupTest, ReportsOpenFailureWithoutApplyingPortModes)
{
	FakeSerial serial;
	serial.open_result = false;
	EXPECT_EQ(CrsfUartSetup(serial, 420000, true, true), CrsfUartSetupResult::OpenError);
	EXPECT_EQ(serial.close_calls, 1);
	EXPECT_FALSE(serial.resource_acquired);
	EXPECT_FALSE(serial.is_open);
	EXPECT_EQ(serial.swap_calls, 0);
	EXPECT_EQ(serial.singlewire_calls, 0);
	EXPECT_EQ(serial.flush_calls, 0);
}

TEST(CrsfUartSetupTest, ConfiguresAndFlushesAnOpenedPort)
{
	FakeSerial serial;
	EXPECT_EQ(CrsfUartSetup(serial, 420000, true, true), CrsfUartSetupResult::Opened);
	EXPECT_TRUE(serial.is_open);
	EXPECT_EQ(serial.swap_calls, 1);
	EXPECT_EQ(serial.singlewire_calls, 1);
	EXPECT_EQ(serial.flush_calls, 1);
}

} // namespace
