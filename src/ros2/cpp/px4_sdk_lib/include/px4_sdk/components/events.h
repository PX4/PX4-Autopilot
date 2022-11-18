/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_msgs/msg/arming_check_reply.hpp>

namespace events
{

using EventType = px4_msgs::msg::Event;

enum class LogLevel : uint8_t {
	Emergency = 0,
	Alert = 1,
	Critical = 2,
	Error = 3,
	Warning = 4,
	Notice = 5,
	Info = 6,
	Debug = 7,
	Protocol = 8,
	Disabled = 9,

	Count
};

enum class LogLevelInternal : uint8_t {
	Emergency = 0,
	Alert = 1,
	Critical = 2,
	Error = 3,
	Warning = 4,
	Notice = 5,
	Info = 6,
	Debug = 7,
	Protocol = 8,
	Disabled = 9,

	Count
};

using Log = LogLevel;
using LogInternal = LogLevelInternal;

struct LogLevels {
	LogLevels() {}
	LogLevels(Log external_level) : external(external_level), internal((LogInternal)external_level) {}
	LogLevels(Log external_level, LogInternal internal_level)
		: external(external_level), internal(internal_level) {}

	Log external{Log::Info};
	LogInternal internal{LogInternal::Info};
};

namespace util
{

// source: https://gist.github.com/ruby0x1/81308642d0325fd386237cfa3b44785c
constexpr uint32_t val_32_const = 0x811c9dc5;
constexpr uint32_t prime_32_const = 0x1000193;
constexpr uint64_t val_64_const = 0xcbf29ce484222325;
constexpr uint64_t prime_64_const = 0x100000001b3;
inline constexpr uint32_t hash_32_fnv1a_const(const char *const str, const uint32_t value = val_32_const) noexcept
{
	return (str[0] == '\0') ? value : hash_32_fnv1a_const(&str[1], (value ^ uint32_t(str[0])) * prime_32_const);
}

template<typename T>
inline constexpr void fillEventArguments(uint8_t *buf, T arg)
{
	// This assumes we're on little-endian
	memcpy(buf, &arg, sizeof(T));
}

template<typename T, typename... Args>
inline constexpr void fillEventArguments(uint8_t *buf, T arg, Args... args)
{
	fillEventArguments(buf, arg);
	fillEventArguments(buf + sizeof(T), args...);
}

constexpr unsigned sizeofArguments() { return 0; }

template <typename T, typename... Args>
constexpr unsigned sizeofArguments(const T &t, const Args &... args)
{
	return sizeof(T) + sizeofArguments(args...);
}

} // namespace util

/**
 * Generate event ID from an event name
 */
template<size_t N>
constexpr uint32_t ID(const char (&name)[N])
{
	// Note: the generated ID must match with the python generator under Tools/px4events
	uint32_t component_id = 1u << 24; // autopilot component
	return (0xffffff & util::hash_32_fnv1a_const(name)) | component_id;
}


} // namespace events

