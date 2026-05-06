/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#ifndef BATTERY_STATUS_V2_HPP
#define BATTERY_STATUS_V2_HPP

#include <uORB/topics/battery_status.h>

class MavlinkStreamBatteryStatusV2 : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamBatteryStatusV2(mavlink); }

	static constexpr const char *get_name_static() { return "BATTERY_STATUS_V2"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_BATTERY_STATUS_V2; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		static constexpr unsigned size_per_battery = MAVLINK_MSG_ID_BATTERY_STATUS_V2_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		return size_per_battery * _battery_status_subs.advertised_count();
	}

private:
	explicit MavlinkStreamBatteryStatusV2(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::SubscriptionMultiArray<battery_status_s, battery_status_s::MAX_INSTANCES> _battery_status_subs{ORB_ID::battery_status};

	bool send() override
	{
		bool updated = false;

		for (auto &battery_sub : _battery_status_subs) {
			battery_status_s battery_status;

			if (battery_sub.update(&battery_status)) {
				mavlink_battery_status_v2_t bat_msg{};

				bat_msg.id = battery_status.id - 1; // uORB id is 1-indexed, MAVLink is 0-indexed (matches BATTERY_STATUS and BATTERY_INFO)

				if (battery_status.connected && PX4_ISFINITE(battery_status.temperature)) {
					bat_msg.temperature = static_cast<int16_t>(battery_status.temperature * 1e2f);

				} else {
					bat_msg.temperature = INT16_MAX;
				}

				if (battery_status.connected && PX4_ISFINITE(battery_status.voltage_v)
				    && !math::isZero(battery_status.voltage_v)) {
					bat_msg.voltage = battery_status.voltage_v;

				} else {
					bat_msg.voltage = float(NAN);
				}

				if (battery_status.connected && fabsf(battery_status.current_a + 1.f) > FLT_EPSILON) {
					bat_msg.current = battery_status.current_a;

				} else {
					bat_msg.current = float(NAN);
				}

				if (battery_status.connected && fabsf(battery_status.discharged_mah + 1.f) > FLT_EPSILON) {
					bat_msg.capacity_consumed = battery_status.discharged_mah * 1e-3f;

				} else {
					bat_msg.capacity_consumed = float(NAN);
				}

				if (battery_status.connected && PX4_ISFINITE(battery_status.remaining_capacity_wh)
				    && PX4_ISFINITE(battery_status.nominal_voltage)
				    && !math::isZero(battery_status.nominal_voltage)) {
					bat_msg.capacity_remaining = battery_status.remaining_capacity_wh / battery_status.nominal_voltage;

				} else {
					bat_msg.capacity_remaining = float(NAN);
				}

				if (battery_status.connected && fabsf(battery_status.remaining + 1.f) > FLT_EPSILON) {
					bat_msg.percent_remaining = static_cast<uint8_t>(roundf(battery_status.remaining * 1e2f));

				} else {
					bat_msg.percent_remaining = UINT8_MAX;
				}

				bat_msg.status_flags = get_status_flags(battery_status);

				mavlink_msg_battery_status_v2_send_struct(_mavlink->get_channel(), &bat_msg);
				updated = true;
			}
		}

		return updated;
	}

	uint32_t get_status_flags(const battery_status_s &battery_status) const
	{
		uint32_t status_flags = 0;
		uint16_t faults = battery_status.faults;

		if (battery_status.warning == battery_status_s::STATE_UNHEALTHY) {
			status_flags |= MAV_BATTERY_STATUS_FLAGS_NOT_READY_TO_USE;
		}

		if (battery_status.warning == battery_status_s::STATE_CHARGING) {
			status_flags |= MAV_BATTERY_STATUS_FLAGS_CHARGING;
		}

		if (faults & battery_status_s::FAULT_HARDWARE_FAILURE) {
			status_flags |= MAV_BATTERY_STATUS_FLAGS_REQUIRES_SERVICE | MAV_BATTERY_STATUS_FLAGS_BAD_BATTERY;
		}

		if (faults & battery_status_s::FAULT_CELL_FAIL) {
			status_flags |= MAV_BATTERY_STATUS_FLAGS_BAD_BATTERY;
		}

		if (faults & battery_status_s::FAULT_OVER_TEMPERATURE) {
			status_flags |= MAV_BATTERY_STATUS_FLAGS_FAULT_OVER_TEMPERATURE;
		}

		if (faults & battery_status_s::FAULT_UNDER_TEMPERATURE) {
			status_flags |= MAV_BATTERY_STATUS_FLAGS_FAULT_UNDER_TEMPERATURE;
		}

		if (faults & battery_status_s::FAULT_OVER_CURRENT) {
			status_flags |= MAV_BATTERY_STATUS_FLAGS_FAULT_OVER_CURRENT;
		}

		if (faults & battery_status_s::FAULT_INCOMPATIBLE_VOLTAGE) {
			status_flags |= MAV_BATTERY_STATUS_FLAGS_FAULT_INCOMPATIBLE_VOLTAGE;
		}

		if (faults & battery_status_s::FAULT_INCOMPATIBLE_FIRMWARE) {
			status_flags |= MAV_BATTERY_STATUS_FLAGS_FAULT_INCOMPATIBLE_FIRMWARE;
		}

		return status_flags;
	}
};

#endif // BATTERY_STATUS_V2_HPP
