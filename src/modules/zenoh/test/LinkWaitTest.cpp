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
 * @file LinkWaitTest.cpp
 *
 * Runs the zenoh module's link wait (zenoh_link_wait.cpp) against a fake interface, so
 * the loop, its bound and its stop handling are the code the module executes on NuttX.
 * The SIOCGIFFLAGS read itself is target-only and not covered here.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "zenoh_link_wait.hpp"

using namespace zenoh_link;

namespace
{

class FakeIo : public Io
{
public:
	// flags returned poll by poll, the last entry repeats
	std::vector<unsigned> flags_sequence{0};
	bool read_fails{false};
	bool stop_now{false};		///< shouldExit() is true from the start
	unsigned stop_at_poll{0};	///< shouldExit() turns true once this many reads happened, 0: never
	unsigned stop_at_sleep{0};	///< shouldExit() turns true once this many sleeps happened, 0: never

	unsigned reads{0};
	unsigned sleeps{0};
	uint64_t slept_us{0};
	std::string last_ifname;

	bool readFlags(const char *ifname, unsigned &flags) override
	{
		reads++;
		last_ifname = ifname;

		if (read_fails) {
			return false;
		}

		const size_t index = std::min<size_t>(reads - 1, flags_sequence.size() - 1);
		flags = flags_sequence[index];
		return true;
	}

	void sleep(uint32_t duration_us) override
	{
		sleeps++;
		slept_us += duration_us;
	}

	bool shouldExit() override
	{
		return stop_now || (stop_at_poll != 0 && reads >= stop_at_poll) || (stop_at_sleep != 0 && sleeps >= stop_at_sleep);
	}
};

constexpr unsigned kOtherFlags = IFF_BROADCAST | IFF_MULTICAST;

} // namespace

TEST(LinkWait, linkUpAtFirstPollDoesNotSleep)
{
	FakeIo io;
	io.flags_sequence = {IFF_UP | IFF_RUNNING | kOtherFlags};

	const WaitOutcome outcome = waitForLink("eth0", io);

	EXPECT_EQ(outcome.result, WaitResult::LinkUp);
	EXPECT_EQ(outcome.polls, 1u);
	EXPECT_EQ(outcome.flags, IFF_UP | IFF_RUNNING | kOtherFlags);
	EXPECT_EQ(io.sleeps, 0u);
	EXPECT_EQ(io.last_ifname, "eth0");
}

TEST(LinkWait, linkUpOnceCarrierIsReported)
{
	FakeIo io;
	io.flags_sequence = {IFF_UP, IFF_UP, IFF_UP | IFF_RUNNING};

	const WaitOutcome outcome = waitForLink("eth0", io);

	EXPECT_EQ(outcome.result, WaitResult::LinkUp);
	EXPECT_EQ(outcome.polls, 3u);
	EXPECT_EQ(io.sleeps, 2u);
	EXPECT_EQ(io.slept_us, 2u * kPollInterval);
}

TEST(LinkWait, upWithoutRunningTimesOutAfterTheBound)
{
	// what fmu-v6xrt reports without the netdev_carrier_on() backport: UP but never RUNNING
	FakeIo io;
	io.flags_sequence = {IFF_UP | kOtherFlags};

	const WaitOutcome outcome = waitForLink("eth0", io);

	EXPECT_EQ(outcome.result, WaitResult::TimedOut);
	EXPECT_EQ(outcome.polls, kMaxPolls);
	EXPECT_EQ(outcome.flags, IFF_UP | kOtherFlags);
	EXPECT_EQ(io.sleeps, kMaxPolls);
	EXPECT_EQ(io.slept_us, static_cast<uint64_t>(kMaxPolls) * kPollInterval);
	EXPECT_EQ(io.slept_us, 5000000u);
}

TEST(LinkWait, runningWithoutUpIsNotALink)
{
	FakeIo io;
	io.flags_sequence = {IFF_RUNNING};

	EXPECT_EQ(waitForLink("eth0", io).result, WaitResult::TimedOut);
}

TEST(LinkWait, unreadableFlagsCountAsNoLink)
{
	FakeIo io;
	io.read_fails = true;

	const WaitOutcome outcome = waitForLink("eth0", io);

	EXPECT_EQ(outcome.result, WaitResult::TimedOut);
	EXPECT_EQ(outcome.polls, kMaxPolls);
	EXPECT_EQ(outcome.flags, 0u);
}

TEST(LinkWait, stopRequestEndsTheWaitWithoutSleeping)
{
	FakeIo io;
	io.flags_sequence = {IFF_UP};
	io.stop_at_poll = 3;

	const WaitOutcome outcome = waitForLink("eth0", io);

	EXPECT_EQ(outcome.result, WaitResult::Stopped);
	EXPECT_EQ(outcome.polls, 3u);
	EXPECT_EQ(io.sleeps, 2u);
}

TEST(LinkWait, stopRequestBeforeTheFirstSleep)
{
	FakeIo io;
	io.stop_at_poll = 1;

	const WaitOutcome outcome = waitForLink("eth0", io);

	EXPECT_EQ(outcome.result, WaitResult::Stopped);
	EXPECT_EQ(outcome.polls, 1u);
	EXPECT_EQ(io.sleeps, 0u);
}

TEST(LinkWait, pendingStopWinsOverALinkThatIsUp)
{
	FakeIo io;
	io.flags_sequence = {IFF_UP | IFF_RUNNING};
	io.stop_now = true;

	const WaitOutcome outcome = waitForLink("eth0", io);

	EXPECT_EQ(outcome.result, WaitResult::Stopped);
	EXPECT_EQ(outcome.polls, 0u);
	EXPECT_EQ(io.sleeps, 0u);
}

TEST(LinkWait, stopDuringTheLastSleepDoesNotTimeOut)
{
	FakeIo io;
	io.flags_sequence = {IFF_UP};
	io.stop_at_sleep = kMaxPolls;

	const WaitOutcome outcome = waitForLink("eth0", io);

	EXPECT_EQ(outcome.result, WaitResult::Stopped);
	EXPECT_EQ(outcome.polls, kMaxPolls);
	EXPECT_EQ(io.sleeps, kMaxPolls);
}

TEST(LinkWait, stopDuringAnEarlySleepIsSeenBeforeTheNextPoll)
{
	FakeIo io;
	io.flags_sequence = {IFF_UP, IFF_UP, IFF_UP | IFF_RUNNING};
	io.stop_at_sleep = 2;

	const WaitOutcome outcome = waitForLink("eth0", io);

	EXPECT_EQ(outcome.result, WaitResult::Stopped);
	EXPECT_EQ(outcome.polls, 2u);
	EXPECT_EQ(io.sleeps, 2u);
}

TEST(LinkWait, pollsTheRequestedInterface)
{
	FakeIo io;
	io.flags_sequence = {IFF_UP | IFF_RUNNING};

	waitForLink("eth1", io);

	EXPECT_EQ(io.last_ifname, "eth1");
}

TEST(LinkWaitLocator, defaultNuttxLocatorNamesEth0)
{
	char ifname[IFNAMSIZ] {};

	ASSERT_TRUE(interfaceFromLocator("tcp/10.41.10.1:7447#iface=eth0", ifname, sizeof(ifname)));
	EXPECT_STREQ(ifname, "eth0");
}

TEST(LinkWaitLocator, locatorWithoutConfigUsesTheDefaultInterface)
{
	char ifname[IFNAMSIZ] {};

	ASSERT_TRUE(interfaceFromLocator("tcp/10.41.10.1:7447", ifname, sizeof(ifname)));
	EXPECT_STREQ(ifname, kDefaultInterface);
	EXPECT_STREQ(ifname, "eth0");
}

TEST(LinkWaitLocator, interfaceEntryAnywhereInTheConfigSection)
{
	char ifname[IFNAMSIZ] {};

	ASSERT_TRUE(interfaceFromLocator("tcp/10.41.10.1:7447#tout=1000;iface=eth1", ifname, sizeof(ifname)));
	EXPECT_STREQ(ifname, "eth1");

	ASSERT_TRUE(interfaceFromLocator("tcp/10.41.10.1:7447#iface=eth1;tout=1000", ifname, sizeof(ifname)));
	EXPECT_STREQ(ifname, "eth1");
}

TEST(LinkWaitLocator, metadataSectionDoesNotHideTheConfig)
{
	char ifname[IFNAMSIZ] {};

	ASSERT_TRUE(interfaceFromLocator("tcp/10.41.10.1:7447?meta=1#iface=eth1", ifname, sizeof(ifname)));
	EXPECT_STREQ(ifname, "eth1");
}

TEST(LinkWaitLocator, keyMustMatchExactly)
{
	char ifname[IFNAMSIZ] {};

	ASSERT_TRUE(interfaceFromLocator("tcp/10.41.10.1:7447#ifacex=eth1;xiface=eth2", ifname, sizeof(ifname)));
	EXPECT_STREQ(ifname, "eth0");
}

TEST(LinkWaitLocator, udpMulticastPeerLocatorWaitsToo)
{
	char ifname[IFNAMSIZ] {};

	ASSERT_TRUE(interfaceFromLocator("udp/224.0.0.224:7446#iface=eth0", ifname, sizeof(ifname)));
	EXPECT_STREQ(ifname, "eth0");
}

TEST(LinkWaitLocator, scoutingWaitsOnTheDefaultInterface)
{
	char ifname[IFNAMSIZ] {};

	ASSERT_TRUE(interfaceFromLocator("", ifname, sizeof(ifname)));
	EXPECT_STREQ(ifname, "eth0");
}

TEST(LinkWaitLocator, serialLocatorHasNoLinkToWaitFor)
{
	char ifname[IFNAMSIZ] {};

	EXPECT_FALSE(interfaceFromLocator("serial//dev/ttyS3#baudrate=115200", ifname, sizeof(ifname)));
	EXPECT_FALSE(interfaceFromLocator("tls/10.41.10.1:7447#iface=eth0", ifname, sizeof(ifname)));
}

TEST(LinkWaitLocator, unusableInterfaceNamesAreRejected)
{
	char ifname[IFNAMSIZ] {};

	EXPECT_FALSE(interfaceFromLocator("tcp/10.41.10.1:7447#iface=", ifname, sizeof(ifname)));
	EXPECT_FALSE(interfaceFromLocator("tcp/10.41.10.1:7447#iface=eth0123456789abcdef", ifname, sizeof(ifname)));
	EXPECT_FALSE(interfaceFromLocator("tcp/10.41.10.1:7447", ifname, 2));
}
