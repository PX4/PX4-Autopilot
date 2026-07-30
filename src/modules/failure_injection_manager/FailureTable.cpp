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

#include "FailureTable.hpp"

#include <cstring>

namespace failure_injection
{

bool FailureTable::isSupported(uint8_t unit, uint8_t type)
{
	// Capability catalogue: the (unit, type) combinations implemented today.
	// FAILURE_TYPE_OK is always accepted for a known unit (it clears the unit).
	switch (unit) {
	case failure_injection_s::FAILURE_UNIT_SENSOR_GYRO:
	case failure_injection_s::FAILURE_UNIT_SENSOR_ACCEL:
	case failure_injection_s::FAILURE_UNIT_SENSOR_MAG:
	case failure_injection_s::FAILURE_UNIT_SENSOR_BARO:
	case failure_injection_s::FAILURE_UNIT_SENSOR_DISTANCE_SENSOR:
		return type == failure_injection_s::FAILURE_TYPE_OK
		       || type == failure_injection_s::FAILURE_TYPE_OFF
		       || type == failure_injection_s::FAILURE_TYPE_STUCK;

	case failure_injection_s::FAILURE_UNIT_SENSOR_GPS:
	case failure_injection_s::FAILURE_UNIT_SENSOR_AIRSPEED:
	case failure_injection_s::FAILURE_UNIT_SYSTEM_MOTOR:
		return type == failure_injection_s::FAILURE_TYPE_OK
		       || type == failure_injection_s::FAILURE_TYPE_OFF
		       || type == failure_injection_s::FAILURE_TYPE_STUCK
		       || type == failure_injection_s::FAILURE_TYPE_WRONG;

	case failure_injection_s::FAILURE_UNIT_SENSOR_VIO:
	case failure_injection_s::FAILURE_UNIT_SYSTEM_BATTERY:
		return type == failure_injection_s::FAILURE_TYPE_OK
		       || type == failure_injection_s::FAILURE_TYPE_OFF;

	default:
		return false;
	}
}

uint16_t FailureTable::instanceToMask(uint8_t instance)
{
	if (instance == 0) {
		return 0xFFFF; // all instances
	}

	if (instance > 16) {
		return 0; // out of range, addressable instances are 1..16
	}

	return static_cast<uint16_t>(1u << (instance - 1));
}

int FailureTable::findEntry(uint8_t unit, uint8_t type) const
{
	for (uint8_t i = 0; i < _count; i++) {
		if (_entries[i].unit == unit && _entries[i].type == type) {
			return i;
		}
	}

	return -1;
}

void FailureTable::clearInstances(uint8_t unit, uint16_t mask, uint8_t keep_type)
{
	for (uint8_t i = 0; i < _count; i++) {
		if (_entries[i].unit == unit && _entries[i].type != keep_type
		    && (_entries[i].instance_mask & mask) != 0) {
			_entries[i].instance_mask &= ~mask;
			_changed = true;
		}
	}

	compact();
}

void FailureTable::compact()
{
	uint8_t write = 0;

	for (uint8_t read = 0; read < _count; read++) {
		if (_entries[read].instance_mask != 0) {
			if (write != read) {
				_entries[write] = _entries[read];
			}

			write++;
		}
	}

	_count = write;

	// Zero the unused tail so dropped entries leave no stale state behind.
	for (uint8_t i = _count; i < failure_injection_s::MAX_FAILURES; i++) {
		_entries[i] = Entry{};
	}
}

FailureTable::AckResult FailureTable::inject(uint8_t unit, uint8_t type, uint8_t instance)
{
	return injectMask(unit, type, instanceToMask(instance));
}

FailureTable::AckResult FailureTable::injectMask(uint8_t unit, uint8_t type, uint16_t mask)
{
	// Validate the request against the capability catalogue.
	if (!isSupported(unit, type)) {
		return AckResult::Unsupported;
	}

	if (mask == 0) {
		// Out-of-range instance: nothing addressable to do.
		return AckResult::Accepted;
	}

	// Snapshot for rollback if the request would overflow the table. _changed is
	// set at the mutation sites below, so it has to be restored on rollback too.
	Entry   saved_entries[failure_injection_s::MAX_FAILURES];
	uint8_t saved_count = _count;
	bool    saved_changed = _changed;
	memcpy(saved_entries, _entries, sizeof(_entries));

	if (type == failure_injection_s::FAILURE_TYPE_OK) {
		// Clear the addressed instances from every failure mode of this unit.
		clearInstances(unit, mask, 0xFF);

	} else {
		// An instance is in at most one mode per unit: drop these instances
		// from the unit's other modes before applying the requested one.
		clearInstances(unit, mask, type);

		int idx = findEntry(unit, type);

		if (idx < 0) {
			if (_count >= failure_injection_s::MAX_FAILURES) {
				// Would exceed the cap: roll back and reject.
				memcpy(_entries, saved_entries, sizeof(_entries));
				_count = saved_count;
				_changed = saved_changed;
				return AckResult::Rejected;
			}

			idx = _count++;
			_entries[idx].unit = unit;
			_entries[idx].type = type;
			_entries[idx].instance_mask = 0;
			_changed = true;
		}

		// Only a mask that actually adds instances counts as a change (dedup).
		if ((_entries[idx].instance_mask & mask) != mask) {
			_entries[idx].instance_mask |= mask;
			_changed = true;
		}
	}

	return AckResult::Accepted;
}

void FailureTable::fill(failure_injection_s &msg) const
{
	msg.count = _count;

	for (uint8_t i = 0; i < failure_injection_s::MAX_FAILURES; i++) {
		if (i < _count) {
			msg.unit[i] = _entries[i].unit;
			msg.instance_mask[i] = _entries[i].instance_mask;
			msg.failure_type[i] = _entries[i].type;

		} else {
			msg.unit[i] = 0;
			msg.instance_mask[i] = 0;
			msg.failure_type[i] = 0;
		}
	}
}

} // namespace failure_injection
