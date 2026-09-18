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

#include "zenoh_link_wait.hpp"

#include <string.h>

namespace zenoh_link
{

namespace
{
// zenoh locator syntax, see zenoh-pico link/endpoint.h and collections/string.h
constexpr char kTcpPrefix[] = "tcp/";
constexpr char kUdpPrefix[] = "udp/";
constexpr char kConfigSeparator = '#';
constexpr char kConfigListSeparator = ';';
constexpr char kConfigKeyValueSeparator = '=';
constexpr char kInterfaceKey[] = "iface";

bool hasPrefix(const char *str, const char *prefix)
{
	return strncmp(str, prefix, strlen(prefix)) == 0;
}
} // namespace

bool interfaceFromLocator(const char *locator, char *ifname, size_t ifname_size)
{
	// Only the IP transports run over the network link; a serial locator has no link to wait for
	if (locator[0] != '\0' && !hasPrefix(locator, kTcpPrefix) && !hasPrefix(locator, kUdpPrefix)) {
		return false;
	}

	if (ifname_size < sizeof(kDefaultInterface)) {
		return false;
	}

	memcpy(ifname, kDefaultInterface, sizeof(kDefaultInterface));

	const char *entry = strchr(locator, kConfigSeparator);

	if (entry == nullptr) {
		return true;
	}

	// walk the "key=value;key=value" config section
	for (entry++; *entry != '\0';) {
		const char *entry_end = strchr(entry, kConfigListSeparator);
		const size_t entry_len = (entry_end != nullptr) ? static_cast<size_t>(entry_end - entry) : strlen(entry);
		const size_t key_len = strlen(kInterfaceKey);

		if (entry_len > key_len && strncmp(entry, kInterfaceKey, key_len) == 0 && entry[key_len] == kConfigKeyValueSeparator) {
			const char *name = entry + key_len + 1;
			const size_t name_len = entry_len - key_len - 1;

			// an empty or over-long name cannot be looked up, do not wait on it
			if (name_len == 0 || name_len >= ifname_size) {
				return false;
			}

			memcpy(ifname, name, name_len);
			ifname[name_len] = '\0';
			return true;
		}

		entry += entry_len;

		if (*entry == kConfigListSeparator) {
			entry++;
		}
	}

	return true;
}

WaitOutcome waitForLink(const char *ifname, Io &io)
{
	WaitOutcome outcome;

	while (true) {
		// a pending stop wins over everything else, also after the last sleep and when the link is up
		if (io.shouldExit()) {
			outcome.result = WaitResult::Stopped;
			return outcome;
		}

		if (outcome.polls >= kMaxPolls) {
			outcome.result = WaitResult::TimedOut;
			return outcome;
		}

		unsigned flags = 0;
		outcome.polls++;

		if (io.readFlags(ifname, flags)) {
			outcome.flags = flags;

			if (linkIsUp(flags)) {
				outcome.result = WaitResult::LinkUp;
				return outcome;
			}
		}

		// do not start a sleep that a stop request would have to wait for
		if (io.shouldExit()) {
			outcome.result = WaitResult::Stopped;
			return outcome;
		}

		io.sleep(kPollInterval);
	}
}

} // namespace zenoh_link
