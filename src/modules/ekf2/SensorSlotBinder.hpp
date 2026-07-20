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

#pragma once

#include <lib/parameters/param.h>
#include <px4_platform_common/defines.h>

#include <stdint.h>
#include <stdio.h>

/**
 * Maps uORB topic instances to fixed sensor slots by device/sensor ID.
 *
 * Each slot is associated with an ID parameter (e.g. EKF2_OF0_ID). An incoming
 * sensor is matched against the slot IDs; an unknown sensor is bound to the first free
 * slot (ID parameter 0) and the assignment is persisted so that it remains stable
 * across reboots.
 */
class SensorSlotBinder
{
public:
	static constexpr uint8_t kMaxSlots = 4;

	// id_param_format: printf-style format for the per-slot ID parameter name, e.g. "EKF2_OF%u_ID"
	void init(const char *id_param_format, uint8_t num_slots)
	{
		_num_slots = (num_slots <= kMaxSlots) ? num_slots : kMaxSlots;

		for (uint8_t slot = 0; slot < _num_slots; slot++) {
			char param_name[17] {};
			snprintf(param_name, sizeof(param_name), id_param_format, static_cast<unsigned>(slot));
			// use _no_notification so an unset ID stays hidden from the GCS until it is
			// explicitly assigned; auto-bind also persists silently (param_set_no_notification)
			_id_param_handles[slot] = param_find_no_notification(param_name);

			if (_id_param_handles[slot] != PARAM_INVALID) {
				param_get(_id_param_handles[slot], &_ids[slot]);
			}
		}
	}

	// map a uORB instance to a sensor slot given the instance's current device/sensor ID,
	// returns -1 if the ID is invalid (0) or no free slot is available
	int8_t slotForInstance(uint8_t instance, uint32_t device_id)
	{
		if (instance >= _num_slots) {
			return -1;
		}

		if ((_instance_slot_map[instance] < 0) || (_instance_device_id[instance] != device_id)) {
			_instance_slot_map[instance] = mapToSlot(device_id);
			_instance_device_id[instance] = device_id;
		}

		return _instance_slot_map[instance];
	}

private:
	int8_t mapToSlot(uint32_t device_id)
	{
		if (device_id == 0) {
			return -1;
		}

		// look for a slot already bound to this sensor
		for (uint8_t slot = 0; slot < _num_slots; slot++) {
			if (_ids[slot] == static_cast<int32_t>(device_id)) {
				return slot;
			}
		}

		// otherwise bind the first free (unbound) slot and persist the assignment
		for (uint8_t slot = 0; slot < _num_slots; slot++) {
			if (_ids[slot] == 0) {
				const int32_t id = static_cast<int32_t>(device_id);

				if (param_set_no_notification(_id_param_handles[slot], &id) == PX4_OK) {
					_ids[slot] = id;
				}

				return slot;
			}
		}

		return -1;
	}

	param_t _id_param_handles[kMaxSlots] {PARAM_INVALID, PARAM_INVALID, PARAM_INVALID, PARAM_INVALID};
	int32_t _ids[kMaxSlots] {};
	int8_t _instance_slot_map[kMaxSlots] {-1, -1, -1, -1};
	uint32_t _instance_device_id[kMaxSlots] {};
	uint8_t _num_slots{0};
};
