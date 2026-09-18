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
 * @file zenoh_link_wait.hpp
 *
 * Waits for the network interface behind a zenoh locator to report a link before the
 * session is opened. On NuttX the module polls the interface flags with SIOCGIFFLAGS
 * until IFF_UP and IFF_RUNNING are both set, at most kMaxPolls times kPollInterval, and
 * falls through to the session open on timeout.
 *
 * The flag reader, the sleep and the stop request are injected through Io so that the
 * loop and the locator parsing run unchanged in the host unit tests. The ioctl itself
 * is only exercised on the target.
 */

#pragma once

#include <net/if.h>
#include <stddef.h>
#include <stdint.h>

namespace zenoh_link
{

/** Platform hooks of the wait loop. */
class Io
{
public:
	virtual ~Io() = default;

	/**
	 * @brief Reads the interface flags (SIOCGIFFLAGS) of ifname.
	 * @return false if the flags cannot be read, which counts as "no link" for this poll
	 */
	virtual bool readFlags(const char *ifname, unsigned &flags) = 0;

	virtual void sleep(uint32_t duration_us) = 0;

	/** @return true if the module was asked to stop */
	virtual bool shouldExit() = 0;
};

enum class WaitResult { LinkUp, TimedOut, Stopped };

struct WaitOutcome {
	WaitResult result{WaitResult::TimedOut};
	unsigned polls{0};	///< flag reads carried out
	unsigned flags{0};	///< last flags read, 0 if none could be read
};

// Both flags are required: IFF_UP is the administrative state, IFF_RUNNING is set by the
// driver through netdev_carrier_on() (NuttX include/net/if.h)
static constexpr unsigned kLinkFlags = IFF_UP | IFF_RUNNING;
// 50 polls of 100 ms: a 5 s nominal sleep budget, the same length as the delay between
// session open attempts (ZENOH::kSessionRetryDelay)
static constexpr unsigned kMaxPolls = 50;
static constexpr unsigned kPollIntervalMs = 100;
static constexpr uint32_t kPollInterval = kPollIntervalMs * 1000;	///< [us]
// Interface used when the locator names none. It is what rcS brings up with
// "netman update -i eth0" and what the module's default locator names on NuttX.
static constexpr char kDefaultInterface[] = "eth0";

inline bool linkIsUp(unsigned flags) { return (flags & kLinkFlags) == kLinkFlags; }

/**
 * @brief Selects the interface whose link the locator depends on.
 *
 * The name comes from the "iface=<name>" entry of the locator's config section, e.g.
 * "tcp/10.41.10.1:7447#iface=eth0", and defaults to kDefaultInterface. The config
 * section syntax is the one zenoh-pico parses: '#' introduces the section, entries are
 * separated by ';' and written key=value (ENDPOINT_CONFIG_SEPARATOR,
 * INT_STR_MAP_LIST_SEPARATOR, INT_STR_MAP_KEYVALUE_SEPARATOR, UDP_CONFIG_IFACE_STR).
 * @param[in] locator zenoh locator, empty for scouting
 * @param[out] ifname interface name, at least IFNAMSIZ bytes
 * @param[in] ifname_size size of ifname
 * @return false if there is nothing to wait for: the locator is not an IP transport
 *         (tcp/, udp/ or scouting) or the configured name does not fit ifname
 */
bool interfaceFromLocator(const char *locator, char *ifname, size_t ifname_size);

/**
 * @brief Polls the interface flags until the link is up, the polls are exhausted or a stop
 *        is requested.
 *
 * A pending stop request is checked before every poll and before every sleep and takes
 * precedence over a link that is up. Each unsuccessful poll is followed by one
 * io.sleep(kPollInterval), so the sleep budget is kMaxPolls * kPollInterval (nominal:
 * the ioctl and scheduling add wall time on top).
 */
WaitOutcome waitForLink(const char *ifname, Io &io);

} // namespace zenoh_link
