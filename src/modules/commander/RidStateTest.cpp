/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include <gtest/gtest.h>
#include <lib/remote_id/RidState.hpp>

namespace
{
bool put(rid::IdentityReader &reader, const char *text)
{
	return reader.string(reinterpret_cast<const uint8_t *>(text), strlen(text));
}

bool snapshot(rid::IdentityReader &reader, bool active = true, uint32_t final_revision = 7, uint32_t final_boot = 100,
	      const char *registration = "registration", const char *uin = "stration")
{
	return reader.integer(100) && reader.integer(active ? 7 : 0) && reader.integer(1)
	       && reader.integer(active ? 1 : 0) && put(reader, active ? "12345678901234567890" : "")
	       && put(reader, registration) && put(reader, uin) && reader.integer(2)
	       && put(reader, "AABBCCDDEEFF")
	       && put(reader, active ? "0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF" : "")
	       && reader.integer(8) && reader.integer(0) && reader.integer(final_revision) && reader.integer(final_boot);
}

void readyPair(rid_identity_s &id, rid_uom_status_s &status)
{
	id.timestamp = status.timestamp = 10000000;
	id.result = rid_identity_s::RESULT_OK;
	id.activation_state = rid_identity_s::ACTIVATION_ACTIVE;
	id.boot_id = status.boot_id = 123;
	id.session_id = status.session_id = 456;
	id.identity_revision = status.identity_revision = 7;
	id.rid_boot_id = status.rid_boot_id = 100;
	id.rid_node_id = 42;
	id.target_system = status.source_system = 255;
	id.target_component = status.source_component = 190;
	memset(id.identity_hash, 0x12, sizeof(id.identity_hash));
	memcpy(status.identity_hash, id.identity_hash, sizeof(status.identity_hash));
	status.environment = rid_uom_status_s::ENVIRONMENT_OFFICIAL;
	status.state = rid_uom_status_s::STATE_READY;
	status.flags = 15;
}
}

TEST(RidState, ReadsCompleteSnapshotAndPreservesPending)
{
	rid::IdentityReader reader;
	ASSERT_TRUE(snapshot(reader));
	ASSERT_TRUE(reader.done());
	EXPECT_EQ(reader.identity().identity_revision, 7u);
	EXPECT_EQ(reader.identity().pending_revision, 8u);
	EXPECT_EQ(reader.identity().activation_state, rid_identity_s::ACTIVATION_ACTIVE);
	EXPECT_EQ(reader.identity().identity_hash[0], 0x01);
	EXPECT_EQ(reader.identity().identity_hash[31], 0xEF);
}

TEST(RidState, ExposesUnactivatedDeviceWithoutInventingIdentity)
{
	rid::IdentityReader reader;
	ASSERT_TRUE(snapshot(reader, false, 0));
	EXPECT_EQ(reader.identity().identity_revision, 0u);
	EXPECT_EQ(reader.identity().rid_boot_id, 100u);
	EXPECT_EQ(memcmp(reader.identity().rid_uid, "AABBCCDDEEFF", 12), 0);
	EXPECT_FALSE(rid::identityReady(reader.identity(), 10000000));
}

TEST(RidState, RejectsRevisionChangeAndRebootDuringRead)
{
	rid::IdentityReader reader;
	EXPECT_FALSE(snapshot(reader, true, 8));
	EXPECT_EQ(reader.result(), rid_identity_s::RESULT_CONFLICT);
	reader.reset();
	EXPECT_FALSE(snapshot(reader, true, 7, 101));
	EXPECT_EQ(reader.result(), rid_identity_s::RESULT_CONFLICT);
}

TEST(RidState, RejectsWrongTypeRangeVersionAndStringLength)
{
	rid::IdentityReader reader;
	EXPECT_FALSE(put(reader, "100"));
	reader.reset();
	EXPECT_FALSE(reader.integer(-1));
	reader.reset();
	EXPECT_FALSE(reader.integer(int64_t(UINT32_MAX) + 1));
	reader.reset();
	ASSERT_TRUE(reader.integer(100));
	ASSERT_TRUE(reader.integer(7));
	EXPECT_FALSE(reader.integer(2));
	EXPECT_EQ(reader.result(), rid_identity_s::RESULT_UNSUPPORTED);
	reader.reset();
	ASSERT_TRUE(reader.integer(100)); ASSERT_TRUE(reader.integer(7));
	ASSERT_TRUE(reader.integer(1)); ASSERT_TRUE(reader.integer(1));
	EXPECT_FALSE(put(reader, "nineteen_characters"));
}

TEST(RidState, RejectsReplayedSequenceAndAcceptsWrap)
{
	EXPECT_FALSE(rid::sequenceNewer(5, 5));
	EXPECT_FALSE(rid::sequenceNewer(4, 5));
	EXPECT_FALSE(rid::sequenceNewer(0x80000005u, 5));
	EXPECT_TRUE(rid::sequenceNewer(0, UINT32_MAX));
	EXPECT_TRUE(rid::sequenceNewer(6, 5));
}

TEST(RidState, BindsVehicleBootSessionDeviceBootRevisionHashAndSender)
{
	rid_identity_s id{}; rid_uom_status_s status{};
	readyPair(id, status);
	ASSERT_TRUE(rid::uomReady(id, status, 10000000, false));
	status.boot_id++; EXPECT_FALSE(rid::uomReady(id, status, 10000000, false)); status.boot_id--;
	status.session_id++; EXPECT_FALSE(rid::uomReady(id, status, 10000000, false)); status.session_id--;
	status.rid_boot_id++; EXPECT_FALSE(rid::uomReady(id, status, 10000000, false)); status.rid_boot_id--;
	status.identity_revision++; EXPECT_FALSE(rid::uomReady(id, status, 10000000, false)); status.identity_revision--;
	status.source_system++; EXPECT_FALSE(rid::uomReady(id, status, 10000000, false)); status.source_system--;
	status.source_component++; EXPECT_FALSE(rid::uomReady(id, status, 10000000, false)); status.source_component--;
	status.identity_hash[0] ^= 1; EXPECT_FALSE(rid::uomReady(id, status, 10000000, false));
}

TEST(RidState, AgesHeartbeatAckAndIdentityIndependently)
{
	rid_identity_s id{}; rid_uom_status_s status{};
	readyPair(id, status);
	EXPECT_TRUE(rid::uomReady(id, status, 13000000, false));
	EXPECT_FALSE(rid::uomReady(id, status, 13000001, false));
	EXPECT_FALSE(rid::uomReady(id, status, 9999999, false));
	status.last_ack_age_ms = 4500;
	EXPECT_TRUE(rid::uomReady(id, status, 10500000, false));
	EXPECT_FALSE(rid::uomReady(id, status, 10501000, false));
	status.last_ack_age_ms = 5001;
	EXPECT_FALSE(rid::uomReady(id, status, 10000000, false));
	status.last_ack_age_ms = 0; status.timestamp = 22000001;
	EXPECT_FALSE(rid::uomReady(id, status, 22000001, false));
}

TEST(RidState, ProductionRejectsMockAndTestBuildCanAcceptIt)
{
	rid_identity_s id{}; rid_uom_status_s status{};
	readyPair(id, status);
	status.environment = rid_uom_status_s::ENVIRONMENT_MOCK;
	EXPECT_FALSE(rid::uomReady(id, status, 10000000, false));
	EXPECT_TRUE(rid::uomReady(id, status, 10000000, true));
	status.flags = 7;
	EXPECT_FALSE(rid::uomReady(id, status, 10000000, true));
	status.flags = 15; status.state = rid_uom_status_s::STATE_FAILED;
	EXPECT_FALSE(rid::uomReady(id, status, 10000000, true));
	status.state = rid_uom_status_s::STATE_READY; status.error_code = 5;
	EXPECT_FALSE(rid::uomReady(id, status, 10000000, true));
}

TEST(RidState, OwnerLeasePreventsOtherGcsAndSessionTakeoverUntilTimeout)
{
	rid::Session session;
	ASSERT_TRUE(session.claim(1, 255, 190, 10000000, false));
	ASSERT_TRUE(session.accept(1, 255, 190, 10, 10000000));
	EXPECT_TRUE(session.claim(1, 255, 190, 11000000, true));
	EXPECT_FALSE(session.claim(2, 255, 190, 13000000, true));
	EXPECT_FALSE(session.claim(1, 254, 190, 13000000, true));
	EXPECT_FALSE(session.accept(1, 254, 190, 11, 12000000));
	EXPECT_FALSE(session.accept(1, 255, 190, 10, 12000000));
	EXPECT_FALSE(session.sourceAllowed(254, 190, 12000000));
	EXPECT_TRUE(session.claim(2, 254, 190, 13000001, true));
	EXPECT_FALSE(session.accept(1, 255, 190, 11, 13000002));
	EXPECT_TRUE(session.accept(2, 254, 190, 1, 13000002));
	EXPECT_TRUE(session.sourceAllowed(254, 190, 13000003));
	EXPECT_TRUE(session.sourceAllowed(255, 190, 16000003)); // Legacy System senders are free after the lease.
	EXPECT_TRUE(session.claim(3, 255, 190, 13000003, false)); // Identity invalidation releases lease immediately.
}

TEST(RidState, RejectsSpacesAndRegistrationUinMismatch)
{
	rid::IdentityReader reader;
	EXPECT_FALSE(snapshot(reader, true, 7, 100, "reg istration", "stration"));
	reader.reset();
	EXPECT_FALSE(snapshot(reader, true, 7, 100, "registration", "12345678"));
	reader.reset();
	EXPECT_FALSE(snapshot(reader, true, 7, 100, "short", ""));
	reader.reset();
	EXPECT_FALSE(snapshot(reader, true, 7, 100, "registration", ""));
	reader.reset();
	EXPECT_TRUE(snapshot(reader, true, 7, 100, "", "12345678"));
	reader.reset();
	EXPECT_TRUE(snapshot(reader, true, 7, 100, "", ""));
}

TEST(RidState, SystemWholeSecondUtcTolerates1200msProgressAt5Hz)
{
	constexpr uint64_t start = 10000000;
	open_drone_id_system_s system{};
	system.timestamp = system.timestamp_sample = start;
	system.timestamp_utc = 1000;
	unsigned forwarded = 0;

	for (unsigned tick = 1; tick <= 30; ++tick) {
		const uint64_t now = start + tick * 200000;

		// Worst phase: the CAN timer runs just before this tick's GCS packet.
		// A UTC advance delayed to 1.2s must not cause a periodic missing frame.
		if (rid::systemFresh(system, now)) { ++forwarded; }

		system.timestamp = now;

		if (tick % 6 == 0) {
			++system.timestamp_utc;
			system.timestamp_sample = now;
		}

		EXPECT_TRUE(rid::systemFresh(system, now));
	}

	EXPECT_EQ(forwarded, 30u);
}

TEST(RidState, SystemRepeatPacketsCannotHideFrozenUtcOrLostReception)
{
	constexpr uint64_t start = 10000000;
	open_drone_id_system_s system{};
	system.timestamp = system.timestamp_sample = start;
	system.timestamp_utc = 1000;

	for (unsigned tick = 1; tick <= 10; ++tick) {
		const uint64_t now = start + tick * 200000;
		system.timestamp = now; // New packet, but the source UTC has frozen.
		EXPECT_EQ(rid::systemFresh(system, now), tick < 10);
	}

	++system.timestamp_utc;
	system.timestamp_sample = system.timestamp;
	EXPECT_TRUE(rid::systemFresh(system, system.timestamp));
	EXPECT_TRUE(rid::systemFresh(system, system.timestamp + 999999));
	EXPECT_FALSE(rid::systemFresh(system, system.timestamp + 1000000));
	EXPECT_FALSE(rid::systemFresh(system, system.timestamp - 1));
	system.timestamp_utc = 0;
	EXPECT_FALSE(rid::systemFresh(system, system.timestamp));
}
