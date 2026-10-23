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

#include <gtest/gtest.h>
#include <mavlink.h>
#include <uORB/topics/open_drone_id_basic_id.h>
#include <uORB/topics/open_drone_id_operator_id.h>
#include <uORB/topics/open_drone_id_system.h>

#include "open_drone_id_bridge.hpp"

using namespace open_drone_id_bridge;

// Exercise the dynamic-array adapter without generated DSDL or CAN hardware.
// Actual DSDL capacities are additionally checked by static_assert in the driver.
struct ByteArray {
	static constexpr size_t MaxSize = 20;
	uint8_t data[MaxSize] {};
	size_t size{0};
	void clear() { size = 0; }
	void push_back(uint8_t value) { ASSERT_LT(size, MaxSize); data[size++] = value; }
};

struct CanBasicId {
	ByteArray id_or_mac;
	uint8_t id_type{};
	uint8_t ua_type{};
	ByteArray uas_id;
};

struct CanOperatorId {
	ByteArray id_or_mac;
	uint8_t operator_id_type{};
	ByteArray operator_id;
};

struct CanSystem {
	ByteArray id_or_mac;
	uint32_t timestamp{};
	uint8_t operator_location_type{};
	uint8_t classification_type{};
	int32_t operator_latitude{};
	int32_t operator_longitude{};
	uint16_t area_count{};
	uint16_t area_radius{};
	float area_ceiling{};
	float area_floor{};
	uint8_t category_eu{};
	uint8_t class_eu{};
	float operator_altitude_geo{};
};

TEST(OpenDroneIdBridge, FullBasicIdAndDynamicArray)
{
	open_drone_id_basic_id_s input{};
	memcpy(input.uas_id, "0123456789ABCDEFGHIJ", sizeof(input.uas_id));
	input.id_type = 1;
	input.ua_type = 2;
	mavlink_open_drone_id_basic_id_t output{};
	basic_id(output, input);
	mavlink_message_t wire{};
	mavlink_msg_open_drone_id_basic_id_encode(1, 1, &wire, &output);
	mavlink_open_drone_id_basic_id_t decoded{};
	mavlink_msg_open_drone_id_basic_id_decode(&wire, &decoded);
	EXPECT_EQ(memcmp(decoded.uas_id, input.uas_id, 20), 0);
	EXPECT_EQ(decoded.id_type, 1);
	EXPECT_EQ(decoded.ua_type, 2);
	EXPECT_EQ(decoded.target_system, 0);
	EXPECT_EQ(decoded.target_component, 0);
	CanBasicId can{};
	basic_id(can, input);
	EXPECT_EQ(can.uas_id.size, 20u);
	EXPECT_EQ(can.id_or_mac.size, 20u);
	EXPECT_EQ(memcmp(can.uas_id.data, input.uas_id, 20), 0);
	EXPECT_EQ(can.id_type, input.id_type);
	EXPECT_EQ(can.ua_type, input.ua_type);
	// Repeated updates replace arrays rather than appending or truncating them.
	input.uas_id[19] = 'Z';
	basic_id(can, input);
	EXPECT_EQ(can.uas_id.size, 20u);
	EXPECT_EQ(can.uas_id.data[19], 'Z');
}

TEST(OpenDroneIdBridge, CnProductIdPreservesProposedWireTypeAndAllBytes)
{
	open_drone_id_basic_id_s input{};
	memcpy(input.uas_id, "ABCD0001000000000001", sizeof(input.uas_id));
	input.id_type = 5; // Proposed CN product ID wire value; do not reinterpret as CTA type 1.
	input.ua_type = 2;
	mavlink_open_drone_id_basic_id_t output{};
	basic_id(output, input);
	mavlink_message_t wire{};
	mavlink_msg_open_drone_id_basic_id_encode(1, 1, &wire, &output);
	mavlink_open_drone_id_basic_id_t decoded{};
	mavlink_msg_open_drone_id_basic_id_decode(&wire, &decoded);
	EXPECT_EQ(decoded.id_type, 5);
	EXPECT_EQ(decoded.ua_type, input.ua_type);
	EXPECT_EQ(memcmp(decoded.uas_id, input.uas_id, sizeof(input.uas_id)), 0);
	EXPECT_EQ(memcmp(decoded.id_or_mac, input.id_or_mac, sizeof(input.id_or_mac)), 0);
	EXPECT_EQ(decoded.target_system, 0);
	EXPECT_EQ(decoded.target_component, 0);

	CanBasicId can{};
	basic_id(can, input);
	EXPECT_EQ(can.id_type, 5);
	EXPECT_EQ(can.ua_type, input.ua_type);
	EXPECT_EQ(can.uas_id.size, sizeof(input.uas_id));
	EXPECT_EQ(can.id_or_mac.size, sizeof(input.id_or_mac));
	EXPECT_EQ(memcmp(can.uas_id.data, input.uas_id, sizeof(input.uas_id)), 0);
	EXPECT_EQ(memcmp(can.id_or_mac.data, input.id_or_mac, sizeof(input.id_or_mac)), 0);
}

TEST(OpenDroneIdBridge, OperatorLeadingAndTrailingZeros)
{
	open_drone_id_operator_id_s input{};
	input.operator_id_type = 1; // QGC CN draft value, intentionally not a local enum.
	memcpy(input.operator_id, "00001234", 8);
	mavlink_open_drone_id_operator_id_t output{};
	operator_id(output, input);
	mavlink_message_t wire{};
	mavlink_msg_open_drone_id_operator_id_encode(1, 1, &wire, &output);
	EXPECT_EQ(wire.magic, MAVLINK_STX);
	EXPECT_LT(wire.len, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN);
	mavlink_open_drone_id_operator_id_t decoded{};
	memset(&decoded, 0xff, sizeof(decoded));
	mavlink_msg_open_drone_id_operator_id_decode(&wire, &decoded);
	EXPECT_EQ(decoded.operator_id_type, 1);
	EXPECT_EQ(memcmp(decoded.operator_id, input.operator_id, 20), 0);
	CanOperatorId can{};
	operator_id(can, input);
	EXPECT_EQ(can.operator_id.size, 20u);
	EXPECT_EQ(can.operator_id_type, 1);
	EXPECT_EQ(memcmp(can.operator_id.data, input.operator_id, 20), 0);
}

TEST(OpenDroneIdBridge, SystemIndependentOfIdentityAndLocalReceptionTime)
{
	open_drone_id_system_s input{};
	input.timestamp = 987654321000ULL;
	input.timestamp_system = 246813579;
	input.operator_location_type = 2;
	input.operator_latitude = 312345678;
	input.operator_longitude = 1212345678;
	input.area_count = 3;
	input.area_radius = 42;
	input.area_ceiling = 123.5f;
	input.area_floor = -1000.f;
	input.operator_altitude_geo = 88.25f;
	input.category_eu = 1;

	// CN light, CN micro, EU, and unknown numeric classifications are all opaque.
	for (uint8_t classification : {2, 1, 255}) {
		for (uint8_t ua_class : {1, 0}) {
			input.classification_type = classification;
			input.class_eu = ua_class;
			mavlink_open_drone_id_system_t output{};
			system(output, input);
			mavlink_message_t wire{};
			mavlink_msg_open_drone_id_system_encode(1, 1, &wire, &output);
			mavlink_open_drone_id_system_t decoded{};
			mavlink_msg_open_drone_id_system_decode(&wire, &decoded);
			EXPECT_EQ(decoded.operator_location_type, input.operator_location_type);
			EXPECT_EQ(decoded.classification_type, input.classification_type);
			EXPECT_EQ(decoded.operator_latitude, input.operator_latitude);
			EXPECT_EQ(decoded.operator_longitude, input.operator_longitude);
			EXPECT_EQ(decoded.area_count, input.area_count);
			EXPECT_EQ(decoded.area_radius, input.area_radius);
			EXPECT_EQ(decoded.area_ceiling, input.area_ceiling);
			EXPECT_EQ(decoded.area_floor, input.area_floor);
			EXPECT_EQ(decoded.category_eu, input.category_eu);
			EXPECT_EQ(decoded.class_eu, input.class_eu);
			EXPECT_EQ(decoded.operator_altitude_geo, input.operator_altitude_geo);
			CanSystem can{};
			system(can, input);
			EXPECT_EQ(can.id_or_mac.size, 20u);
			EXPECT_EQ(memcmp(can.id_or_mac.data, input.id_or_mac, 20), 0);
			EXPECT_EQ(can.timestamp, input.timestamp_system);
			EXPECT_EQ(can.operator_location_type, input.operator_location_type);
			EXPECT_EQ(can.classification_type, input.classification_type);
			EXPECT_EQ(can.operator_latitude, input.operator_latitude);
			EXPECT_EQ(can.operator_longitude, input.operator_longitude);
			EXPECT_EQ(can.area_count, input.area_count);
			EXPECT_EQ(can.area_radius, input.area_radius);
			EXPECT_EQ(can.area_ceiling, input.area_ceiling);
			EXPECT_EQ(can.area_floor, input.area_floor);
			EXPECT_EQ(can.category_eu, input.category_eu);
			EXPECT_EQ(can.class_eu, input.class_eu);
			EXPECT_EQ(can.operator_altitude_geo, input.operator_altitude_geo);
			EXPECT_EQ(decoded.timestamp, input.timestamp_system);
			EXPECT_EQ(decoded.target_system, 0);
			EXPECT_EQ(decoded.target_component, 0);
		}
	}
}

TEST(OpenDroneIdBridge, AddressAssociationAndNoEcho)
{
	mavlink_open_drone_id_system_t input{};
	input.target_system = 7;
	input.target_component = MAV_COMP_ID_AUTOPILOT1;
	EXPECT_TRUE(is_local_configuration(input, 7, MAV_COMP_ID_AUTOPILOT1));
	input.target_component = MAV_COMP_ID_ALL;
	EXPECT_TRUE(is_local_configuration(input, 7, MAV_COMP_ID_AUTOPILOT1));
	input.target_component = MAV_COMP_ID_ODID_TXRX_1;
	EXPECT_FALSE(is_local_configuration(input, 7, MAV_COMP_ID_AUTOPILOT1));
	input.target_component = MAV_COMP_ID_ALL;
	input.target_system = 8;
	EXPECT_FALSE(is_local_configuration(input, 7, MAV_COMP_ID_AUTOPILOT1));
	input.target_system = 0;
	EXPECT_TRUE(is_local_configuration(input, 7, MAV_COMP_ID_AUTOPILOT1));
	input.target_system = 7;

	for (size_t i = 0; i < sizeof(input.id_or_mac); ++i) {
		input.id_or_mac[i] = 1;
		EXPECT_FALSE(is_local_configuration(input, 7, MAV_COMP_ID_AUTOPILOT1));
		input.id_or_mac[i] = 0;
	}

	EXPECT_FALSE(send_on_channel(0, 0));
	EXPECT_FALSE(send_on_channel(1, 1));
	EXPECT_TRUE(send_on_channel(0, 1));
	EXPECT_TRUE(send_on_channel(1, 0));
}

TEST(OpenDroneIdBridge, SystemFreshnessWindow)
{
	constexpr uint64_t received = 10'000'000;
	EXPECT_FALSE(system_fresh(0, received));
	EXPECT_FALSE(system_fresh(received, received - 1));
	EXPECT_TRUE(system_fresh(received, received));
	EXPECT_TRUE(system_fresh(received, received + 5'000'000));
	EXPECT_FALSE(system_fresh(received, received + 5'000'001));
}

TEST(OpenDroneIdBridge, FreshSystemPreservesOperatorPositionAndProtocolTime)
{
	open_drone_id_system_s input{};
	input.timestamp = 10'000'000;
	input.timestamp_system = 246813579;
	input.operator_latitude = 312345678;
	input.operator_longitude = 1212345678;
	input.operator_altitude_geo = 88.25f;

	for (uint64_t age : {0ULL, 5'000'000ULL}) {
		mavlink_open_drone_id_system_t output{};
		CanSystem can{};
		system(output, input, input.timestamp + age);
		system(can, input, input.timestamp + age);
		EXPECT_EQ(output.timestamp, input.timestamp_system);
		EXPECT_EQ(output.operator_latitude, input.operator_latitude);
		EXPECT_EQ(output.operator_longitude, input.operator_longitude);
		EXPECT_EQ(output.operator_altitude_geo, input.operator_altitude_geo);
		EXPECT_EQ(can.timestamp, input.timestamp_system);
		EXPECT_EQ(can.operator_latitude, input.operator_latitude);
		EXPECT_EQ(can.operator_longitude, input.operator_longitude);
		EXPECT_EQ(can.operator_altitude_geo, input.operator_altitude_geo);
	}
}

TEST(OpenDroneIdBridge, ExpiredSystemKeepsClassificationWithUnknownOperatorPosition)
{
	constexpr uint64_t now = 20'000'000;
	open_drone_id_system_s input{};
	input.timestamp_system = 246813579;
	input.operator_location_type = 2;
	input.classification_type = 2;
	input.category_eu = 1;
	input.operator_latitude = 312345678;
	input.operator_longitude = 1212345678;
	input.operator_altitude_geo = 88.25f;
	input.area_count = 3;
	input.area_radius = 42;
	input.area_ceiling = 123.5f;
	input.area_floor = -1000.f;
	memset(input.id_or_mac, 0x12, sizeof(input.id_or_mac));

	for (uint64_t received : {0ULL, now - 5'000'001ULL, now + 1ULL}) {
		input.timestamp = received;

		for (uint8_t ua_class : {0, 1, 255}) {
			input.class_eu = ua_class;
			mavlink_open_drone_id_system_t output{};
			CanSystem can{};
			system(output, input, now);
			system(can, input, now);
			EXPECT_EQ(output.timestamp, 0u);
			EXPECT_EQ(output.operator_latitude, 0);
			EXPECT_EQ(output.operator_longitude, 0);
			EXPECT_EQ(output.operator_altitude_geo, -1000.f);
			EXPECT_EQ(output.operator_location_type, input.operator_location_type);
			EXPECT_EQ(output.classification_type, input.classification_type);
			EXPECT_EQ(output.category_eu, input.category_eu);
			EXPECT_EQ(output.class_eu, input.class_eu);
			EXPECT_EQ(output.area_count, input.area_count);
			EXPECT_EQ(output.area_radius, input.area_radius);
			EXPECT_EQ(output.area_ceiling, input.area_ceiling);
			EXPECT_EQ(output.area_floor, input.area_floor);
			EXPECT_EQ(memcmp(output.id_or_mac, input.id_or_mac, sizeof(input.id_or_mac)), 0);
			EXPECT_EQ(can.timestamp, 0u);
			EXPECT_EQ(can.operator_latitude, 0);
			EXPECT_EQ(can.operator_longitude, 0);
			EXPECT_EQ(can.operator_altitude_geo, -1000.f);
			EXPECT_EQ(can.operator_location_type, input.operator_location_type);
			EXPECT_EQ(can.classification_type, input.classification_type);
			EXPECT_EQ(can.category_eu, input.category_eu);
			EXPECT_EQ(can.class_eu, input.class_eu);
			EXPECT_EQ(can.area_count, input.area_count);
			EXPECT_EQ(can.area_radius, input.area_radius);
			EXPECT_EQ(can.area_ceiling, input.area_ceiling);
			EXPECT_EQ(can.area_floor, input.area_floor);
			EXPECT_EQ(can.id_or_mac.size, sizeof(input.id_or_mac));
			EXPECT_EQ(memcmp(can.id_or_mac.data, input.id_or_mac, sizeof(input.id_or_mac)), 0);
		}
	}
}
