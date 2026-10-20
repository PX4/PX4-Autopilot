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

#include <cstddef>
#include <cstdint>
#include <cstring>

// Field-level bridge shared by MAVLink streams and DroneCAN. No enum reinterpretation.
namespace open_drone_id_bridge
{
template<typename Message>
bool is_local_configuration(const Message &message, uint8_t system_id, uint8_t component_id)
{
	// System/component zero is broadcast. Other-aircraft data is identified by id_or_mac.
	if ((message.target_system != 0 && message.target_system != system_id)
	    || (message.target_component != component_id && message.target_component != 0)) {
		return false;
	}

	for (const auto byte : message.id_or_mac) {
		if (byte != 0) {
			return false;
		}
	}

	return true;
}

inline bool send_on_channel(uint8_t input_channel, uint8_t output_channel)
{
	return input_channel != output_channel;
}

// Stop replaying obsolete dynamic operator information while retaining the
// classification and area configuration. Timestamps are local microseconds.
static constexpr uint64_t SYSTEM_TIMEOUT_US = 5'000'000;

inline bool system_fresh(uint64_t timestamp, uint64_t now)
{
	return timestamp != 0 && now >= timestamp && now - timestamp <= SYSTEM_TIMEOUT_US;
}

template<typename Destination, typename Source, size_t N, size_t M>
void copy_bytes(Destination(&destination)[N], const Source(&source)[M])
{
	static_assert(N == M && sizeof(Destination) == 1 && sizeof(Source) == 1, "RID array size mismatch");
	memcpy(destination, source, N);
}

template<typename Destination, typename Source, size_t N>
void copy_bytes(Destination &destination, const Source(&source)[N])
{
	static_assert(Destination::MaxSize == N, "RID DSDL array capacity mismatch");
	destination.clear();

	for (size_t i = 0; i < N; ++i) {
		destination.push_back(source[i]);
	}
}

template<typename Destination, typename Source>
void basic_id(Destination &destination, const Source &source)
{
	copy_bytes(destination.id_or_mac, source.id_or_mac);
	destination.id_type = source.id_type;
	destination.ua_type = source.ua_type;
	copy_bytes(destination.uas_id, source.uas_id);
}

template<typename Destination, typename Source>
void operator_id(Destination &destination, const Source &source)
{
	copy_bytes(destination.id_or_mac, source.id_or_mac);
	destination.operator_id_type = source.operator_id_type;
	copy_bytes(destination.operator_id, source.operator_id);
}

template<typename Destination, typename Source>
void system(Destination &destination, const Source &source)
{
	copy_bytes(destination.id_or_mac, source.id_or_mac);
	destination.timestamp = source.timestamp_system;
	destination.operator_location_type = source.operator_location_type;
	destination.classification_type = source.classification_type;
	destination.operator_latitude = source.operator_latitude;
	destination.operator_longitude = source.operator_longitude;
	destination.area_count = source.area_count;
	destination.area_radius = source.area_radius;
	destination.area_ceiling = source.area_ceiling;
	destination.area_floor = source.area_floor;
	destination.category_eu = source.category_eu;
	destination.class_eu = source.class_eu;
	destination.operator_altitude_geo = source.operator_altitude_geo;
}
template<typename Destination, typename Source>
void system(Destination &destination, const Source &source, uint64_t now)
{
	system(destination, source);

	if (!system_fresh(source.timestamp, now)) {
		destination.operator_latitude = 0;
		destination.operator_longitude = 0;
		destination.operator_altitude_geo = -1000.f;
		destination.timestamp = 0;
	}
}

} // namespace open_drone_id_bridge
