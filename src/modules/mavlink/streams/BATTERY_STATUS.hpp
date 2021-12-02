/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#ifndef BATTERY_STATUS_HPP
#define BATTERY_STATUS_HPP

#include <uORB/topics/battery_status.h>

class MavlinkStreamBatteryStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamBatteryStatus(mavlink); }

	static constexpr const char *get_name_static() { return "BATTERY_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_BATTERY_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		static constexpr unsigned size_per_battery = MAVLINK_MSG_ID_BATTERY_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		return size_per_battery * _battery_status_subs.advertised_count();
	}

private:
	explicit MavlinkStreamBatteryStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::SubscriptionMultiArray<battery_status_s, battery_status_s::MAX_INSTANCES> _battery_status_subs{ORB_ID::battery_status};

	bool send() override
	{
		bool updated = false;

		for (auto &battery_sub : _battery_status_subs) {
			battery_status_s battery_status;

			if (battery_sub.update(&battery_status)) {
				/* battery status message with higher resolution */
				mavlink_battery_status_t bat_msg{};
				// TODO: Determine how to better map between battery ID within the firmware and in MAVLink
				bat_msg.id = battery_status.id - 1;
				bat_msg.battery_function = MAV_BATTERY_FUNCTION_ALL;
				bat_msg.type = MAV_BATTERY_TYPE_LIPO;
				bat_msg.current_consumed = (battery_status.connected) ? battery_status.discharged_mah : -1;
				bat_msg.energy_consumed = -1;
				bat_msg.current_battery = (battery_status.connected) ? battery_status.current_filtered_a * 100 : -1;
				bat_msg.battery_remaining = (battery_status.connected) ? ceilf(battery_status.remaining * 100.f) : -1;
				// MAVLink extension: 0 is unsupported, in uORB it's NAN
				bat_msg.time_remaining = (battery_status.connected && (PX4_ISFINITE(battery_status.time_remaining_s))) ?
							 math::max((int)battery_status.time_remaining_s, 1) : 0;

				switch (battery_status.warning) {
				case (battery_status_s::BATTERY_WARNING_NONE):
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_OK;
					break;

				case (battery_status_s::BATTERY_WARNING_LOW):
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_LOW;
					break;

				case (battery_status_s::BATTERY_WARNING_CRITICAL):
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_CRITICAL;
					break;

				case (battery_status_s::BATTERY_WARNING_EMERGENCY):
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_EMERGENCY;
					break;

				case (battery_status_s::BATTERY_WARNING_FAILED):
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_FAILED;
					break;

				default:
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_UNDEFINED;
					break;
				}

				// check if temperature valid
				if (battery_status.connected && PX4_ISFINITE(battery_status.temperature)) {
					bat_msg.temperature = battery_status.temperature * 100.f;

				} else {
					bat_msg.temperature = INT16_MAX;
				}

				// fill cell voltages
				static constexpr int mavlink_cell_slots = (sizeof(bat_msg.voltages) / sizeof(bat_msg.voltages[0]));
				static constexpr int mavlink_cell_slots_extension
					= (sizeof(bat_msg.voltages_ext) / sizeof(bat_msg.voltages_ext[0]));
				uint16_t cell_voltages[mavlink_cell_slots + mavlink_cell_slots_extension];

				for (auto &voltage : cell_voltages) {
					voltage = UINT16_MAX;
				}

				if (battery_status.connected) {
					// We don't know the cell count or we don't know the indpendent cell voltages so we report the total voltage in the first cell
					if (battery_status.cell_count == 0 || battery_status.voltage_cell_v[0] < 0.0001f) {
						cell_voltages[0] = battery_status.voltage_filtered_v * 1000.f;

					} else {
						static constexpr int uorb_cell_slots =
							(sizeof(battery_status.voltage_cell_v) / sizeof(battery_status.voltage_cell_v[0]));

						const int cell_slots = math::min(static_cast<int>(battery_status.cell_count),
										 uorb_cell_slots,
										 mavlink_cell_slots + mavlink_cell_slots_extension);

						for (int cell = 0; cell < cell_slots; cell++) {
							cell_voltages[cell] = battery_status.voltage_cell_v[cell] * 1000.f;
						}
					}
				}

				// voltage fields 1-10
				for (int cell = 0; cell < mavlink_cell_slots; cell++) {
					bat_msg.voltages[cell] = cell_voltages[cell];
				}

				// voltage fields 11-14 into the extension
				for (int cell = 0; cell < mavlink_cell_slots_extension; cell++) {
					bat_msg.voltages_ext[cell] = cell_voltages[mavlink_cell_slots + cell];
				}

				mavlink_msg_battery_status_send_struct(_mavlink->get_channel(), &bat_msg);
				updated = true;
			}
		}

		return updated;
	}
};

#endif // BATTERY_STATUS_HPP
