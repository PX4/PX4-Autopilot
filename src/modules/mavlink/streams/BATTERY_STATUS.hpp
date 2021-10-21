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

		for (int uorb_index = 0; uorb_index < _battery_status_subs.size(); uorb_index++) {
			battery_status_s battery_status;

			if (_battery_status_subs[uorb_index].update(&battery_status) && battery_status.connected) {
				mavlink_battery_status_t msg{};
				msg.id = uorb_index; // TODO: Determine how to better map between battery ID within the firmware and in MAVLink
				msg.battery_function = MAV_BATTERY_FUNCTION_ALL;
				msg.type = MAV_BATTERY_TYPE_LIPO;

				// temperature int16_t (cdegC)
				if (PX4_ISFINITE(battery_status.temperature)) {
					msg.temperature = math::constrain(roundf(battery_status.temperature * 100.f), (float)INT16_MIN, (float)(INT16_MAX - 1));

				} else {
					msg.temperature = INT16_MAX;
				}

				// voltages uint16_t[10] (mV)
				static constexpr int MAVLINK_CELLS_MAX = sizeof(msg.voltages) / sizeof(msg.voltages[0]);
				static constexpr int UORB_CELLS_MAX = sizeof(battery_status.voltage_cell_v) / sizeof(battery_status.voltage_cell_v[0]);

				for (int cell = 0; cell < MAVLINK_CELLS_MAX; cell++) {
					if ((cell < battery_status.cell_count) && (cell < UORB_CELLS_MAX)) {
						msg.voltages[cell] = math::constrain(roundf(battery_status.voltage_cell_v[cell] * 1000.f),
										     0.f, (float)(UINT16_MAX - 1));

					} else {
						msg.voltages[cell] = UINT16_MAX;
					}
				}

				// current_battery int16_t (cA)
				if (battery_status.current_filtered_a > FLT_EPSILON) {
					msg.current_battery = math::constrain(roundf(battery_status.current_filtered_a * 100.f), 0.f, (float)INT16_MAX);

				} else {
					msg.current_battery = -1;
				}

				// current_consumed int32_t (mAh)
				if (battery_status.discharged_mah >= 0.f) {
					msg.current_consumed = math::constrain(roundf(battery_status.discharged_mah), 0.f, (float)INT32_MAX);

				} else {
					msg.current_consumed = -1;
				}

				// energy_consumed int32_t (hJ)
				// TODO
				msg.energy_consumed = -1;

				// battery_remaining int8_t (%)
				if (battery_status.remaining >= 0.f) {
					msg.battery_remaining = math::constrain(ceilf(battery_status.remaining * 100.f), 0.f, 100.f);

				} else {
					msg.battery_remaining = -1;
				}

				// time_remaining int32_t (s)
				if (battery_status.run_time_to_empty > 0) {
					msg.time_remaining = math::constrain(battery_status.run_time_to_empty * 60.f, 1.f, (float)INT32_MAX);

				} else {
					msg.time_remaining = -1;
				}

				// charge_state uint8_t (MAV_BATTERY_CHARGE_STATE)
				switch (battery_status.warning) {
				case (battery_status_s::BATTERY_WARNING_NONE):
					msg.charge_state = MAV_BATTERY_CHARGE_STATE_OK;
					break;

				case (battery_status_s::BATTERY_WARNING_LOW):
					msg.charge_state = MAV_BATTERY_CHARGE_STATE_LOW;
					break;

				case (battery_status_s::BATTERY_WARNING_CRITICAL):
					msg.charge_state = MAV_BATTERY_CHARGE_STATE_CRITICAL;
					break;

				case (battery_status_s::BATTERY_WARNING_EMERGENCY):
					msg.charge_state = MAV_BATTERY_CHARGE_STATE_EMERGENCY;
					break;

				case (battery_status_s::BATTERY_WARNING_FAILED):
					msg.charge_state = MAV_BATTERY_CHARGE_STATE_FAILED;
					break;

				default:
					msg.charge_state = MAV_BATTERY_CHARGE_STATE_UNDEFINED;
					break;
				}

				// voltages_ext uint16_t[4] (mV)
				static constexpr int MAVLINK_CELLS_EXT_MAX = (sizeof(msg.voltages_ext) / sizeof(msg.voltages_ext[0]));

				for (int ext_cell = 0; ext_cell < MAVLINK_CELLS_EXT_MAX; ext_cell++) {
					int cell = MAVLINK_CELLS_MAX + ext_cell;

					if ((cell < battery_status.cell_count) && (cell < UORB_CELLS_MAX)) {
						// Battery voltages for cells 11 to 14.
						// Cells above the valid cell count for this battery should have a value of 0, where zero indicates not supported
						// (note, this is different than for the voltages field and allows empty byte truncation).
						// If the measured value is 0 then 1 should be sent instead.
						msg.voltages_ext[ext_cell] = math::constrain(roundf(battery_status.voltage_cell_v[cell] * 1000.f),
									     1.f, (float)(UINT16_MAX - 1));

					} else {
						// TODO: QGC still using UINT16_MAX for invalid voltages_ext cells
						msg.voltages_ext[ext_cell] = UINT16_MAX;
					}
				}

				// mode uint8_t (MAV_BATTERY_MODE)
				msg.mode = MAV_BATTERY_MODE_UNKNOWN; // TODO

				// fault_bitmask uint32_t (MAV_BATTERY_FAULT)
				msg.fault_bitmask = 0; // TODO

				mavlink_msg_battery_status_send_struct(_mavlink->get_channel(), &msg);
				updated = true;
			}
		}

		return updated;
	}
};

#endif // BATTERY_STATUS_HPP
