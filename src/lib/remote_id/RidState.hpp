/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <stdint.h>
#include <string.h>
#include <uORB/topics/rid_identity.h>
#include <uORB/topics/open_drone_id_system.h>
#include <uORB/topics/rid_uom_status.h>

namespace rid
{
static constexpr uint64_t HEARTBEAT_TIMEOUT_US = 3000000;
static constexpr uint64_t IDENTITY_TIMEOUT_US = 12000000;
static constexpr uint32_t ACK_TIMEOUT_MS = 5000;
static constexpr uint64_t SYSTEM_RECEIPT_TIMEOUT_US = 1000000;
static constexpr uint64_t SYSTEM_UTC_PROGRESS_TIMEOUT_US = 2000000;

inline bool recent(uint64_t timestamp, uint64_t now, uint64_t limit)
{
	return timestamp != 0 && timestamp <= now && now - timestamp <= limit;
}

inline bool systemFresh(const open_drone_id_system_s &system, uint64_t now)
{
	// System UTC has whole-second resolution. Permit quantization and transport
	// jitter between UTC advances, while requiring fresh packets independently.
	// Repeated packets cannot extend the last UTC-advance time indefinitely.
	return system.timestamp_utc != 0
	       && recent(system.timestamp, now, SYSTEM_RECEIPT_TIMEOUT_US - 1)
	       && recent(system.timestamp_sample, now, SYSTEM_UTC_PROGRESS_TIMEOUT_US - 1);
}

inline bool sequenceNewer(uint32_t value, uint32_t previous)
{
	const uint32_t delta = value - previous;
	return delta != 0 && delta < 0x80000000U;
}

inline bool identityReady(const rid_identity_s &id, uint64_t now)
{
	return recent(id.timestamp, now, IDENTITY_TIMEOUT_US) && id.result == rid_identity_s::RESULT_OK
	       && id.activation_state == rid_identity_s::ACTIVATION_ACTIVE && id.identity_revision != 0
	       && id.boot_id != 0 && id.session_id != 0 && id.rid_boot_id != 0 && id.rid_node_id != 0;
}

inline bool matches(const rid_identity_s &id, const rid_uom_status_s &status)
{
	return id.boot_id == status.boot_id && id.session_id == status.session_id
	       && id.rid_boot_id == status.rid_boot_id && id.identity_revision == status.identity_revision
	       && id.target_system == status.source_system && id.target_component == status.source_component
	       && memcmp(id.identity_hash, status.identity_hash, sizeof(id.identity_hash)) == 0;
}

inline bool uomReady(const rid_identity_s &id, const rid_uom_status_s &status, uint64_t now, bool allow_mock)
{
	return identityReady(id, now) && matches(id, status)
	       && recent(status.timestamp, now, HEARTBEAT_TIMEOUT_US)
	       && (status.environment == rid_uom_status_s::ENVIRONMENT_OFFICIAL
		   || (allow_mock && status.environment == rid_uom_status_s::ENVIRONMENT_MOCK))
	       && (status.state == rid_uom_status_s::STATE_READY || status.state == rid_uom_status_s::STATE_REPORTING)
	       && status.flags == 15 && status.error_code == 0 && status.last_ack_age_ms <= ACK_TIMEOUT_MS
	       && (now - status.timestamp) / 1000 <= ACK_TIMEOUT_MS - status.last_ack_age_ms;
}

// One reporting owner per vehicle; sequence and lease are protected by the caller's mutex.
class Session
{
public:
	bool isOwner(uint64_t session, uint8_t system, uint8_t component) const
	{
		return _session != 0 && _session == session && _system == system && _component == component;
	}
	bool claim(uint64_t session, uint8_t system, uint8_t component, uint64_t now, bool identity_valid)
	{
		if (session == 0) { return false; }

		if (isOwner(session, system, component)) { return true; }

		if (identity_valid && recent(_last_status, now, HEARTBEAT_TIMEOUT_US)) { return false; }

		_session = session; _system = system; _component = component;
		_last_status = 0; _sequence = 0; _have_sequence = false;
		return true;
	}
	bool accept(uint64_t session, uint8_t system, uint8_t component, uint32_t sequence, uint64_t now)
	{
		if (!isOwner(session, system, component) || (_have_sequence && !sequenceNewer(sequence, _sequence))) { return false; }

		_last_status = now; _sequence = sequence; _have_sequence = true;
		return true;
	}
	bool sourceAllowed(uint8_t system, uint8_t component, uint64_t now) const
	{
		return _session == 0 || !recent(_last_status, now, HEARTBEAT_TIMEOUT_US)
		       || (_system == system && _component == component);
	}
private:
	uint64_t _session{0};
	uint64_t _last_status{0};
	uint32_t _sequence{0};
	uint8_t _system{0};
	uint8_t _component{0};
	bool _have_sequence{false};
};

// Dedicated, bounded GetSet snapshot decoder; independent of the CAN transport.
class IdentityReader
{
public:
	static constexpr unsigned FIELD_COUNT = 14;
	const char *name() const
	{
		static constexpr const char *names[FIELD_COUNT] = {
			"RID_BOOT_ID", "RID_ID_REV", "RID_ID_SCHEMA", "RID_ACT_STATE", "RID_UAS_ID", "RID_REG_ID", "RID_UIN",
			"RID_CLASS", "RID_UID", "RID_ID_HASH", "RID_PENDING", "RID_ACT_ERR", "RID_ID_REV", "RID_BOOT_ID"
		};
		return names[_index < FIELD_COUNT ? _index : 0];
	}
	void reset() { _index = 0; _result = rid_identity_s::RESULT_OK; _id = {}; _uas_len = 0; _hash_len = 0; _reg_len = 0; _uin_len = 0; }
	bool done() const { return _index == FIELD_COUNT; }
	uint8_t result() const { return _result; }
	const rid_identity_s &identity() const { return _id; }
	bool integer(int64_t value)
	{
		if (value < 0 || uint64_t(value) > UINT32_MAX) { return fail(rid_identity_s::RESULT_INVALID); }

		switch (_index) {
		case 0: _id.rid_boot_id = value;

			if (!value) { return fail(rid_identity_s::RESULT_INVALID); } break;

		case 1: _id.identity_revision = value; break;

		case 2: if (value != 1) { return fail(rid_identity_s::RESULT_UNSUPPORTED); } break;

		case 3: if (value > 2) { return fail(rid_identity_s::RESULT_INVALID); } _id.activation_state = value; break;

		case 7: if (value > 4) { return fail(rid_identity_s::RESULT_INVALID); } _id.aircraft_class = value; break;

		case 10: _id.pending_revision = value; break;

		case 11: if (value > UINT8_MAX) { return fail(rid_identity_s::RESULT_INVALID); } _id.activation_error = value; break;

		case 12: if (value != _id.identity_revision) { return fail(rid_identity_s::RESULT_CONFLICT); } break;

		case 13: if (value != _id.rid_boot_id) { return fail(rid_identity_s::RESULT_CONFLICT); } break;

		default: return fail(rid_identity_s::RESULT_INVALID);
		}

		++_index;
		return validate();
	}
	bool string(const uint8_t *value, unsigned length)
	{
		for (unsigned i = 0; i < length; ++i) {
			if (value[i] < 0x21 || value[i] > 0x7e) { return fail(rid_identity_s::RESULT_INVALID); }
		}

		switch (_index) {
		case 4:
			if (length != 0 && length != sizeof(_id.uas_id)) { return fail(rid_identity_s::RESULT_INVALID); }

			memcpy(_id.uas_id, value, length); _uas_len = length; break;

		case 5:
			if (length > sizeof(_id.registration_id)) { return fail(rid_identity_s::RESULT_INVALID); }

			memcpy(_id.registration_id, value, length); _reg_len = length; break;

		case 6:
			if (length != 0 && length != sizeof(_id.uin)) { return fail(rid_identity_s::RESULT_INVALID); }

			memcpy(_id.uin, value, length); _uin_len = length; break;

		case 8:
			if (length != sizeof(_id.rid_uid)) { return fail(rid_identity_s::RESULT_INVALID); }

			for (unsigned i = 0; i < length; ++i) { if (hex(value[i]) < 0) { return fail(rid_identity_s::RESULT_INVALID); } }

			memcpy(_id.rid_uid, value, length); break;

		case 9:
			if (length != 0 && length != 64) { return fail(rid_identity_s::RESULT_INVALID); }

			for (unsigned i = 0; i < length / 2; ++i) {
				const int hi = hex(value[2 * i]); const int lo = hex(value[2 * i + 1]);

				if (hi < 0 || lo < 0) { return fail(rid_identity_s::RESULT_INVALID); }

				_id.identity_hash[i] = (hi << 4) | lo;
			}

			_hash_len = length; break;

		default: return fail(rid_identity_s::RESULT_INVALID);
		}

		++_index;
		return validate();
	}
	static int hex(uint8_t ch) { return ch >= '0' && ch <= '9' ? ch - '0' : ch >= 'A' && ch <= 'F' ? ch - 'A' + 10 : -1; }
private:
	bool fail(uint8_t result) { _result = result; return false; }
	bool validate()
	{
		if (done() && _id.activation_state == rid_identity_s::ACTIVATION_ACTIVE
		    && (_id.identity_revision == 0 || _uas_len != 20 || _hash_len != 64
			|| (_reg_len != 0 && (_reg_len < 8 || _uin_len != 8
					      || memcmp(_id.registration_id + _reg_len - 8, _id.uin, 8) != 0)))) {
			return fail(rid_identity_s::RESULT_INVALID);
		}

		return true;
	}
	rid_identity_s _id{};
	unsigned _index{0};
	unsigned _uas_len{0};
	unsigned _hash_len{0};
	unsigned _reg_len{0};
	unsigned _uin_len{0};
	uint8_t _result{rid_identity_s::RESULT_OK};
};
} // namespace rid
