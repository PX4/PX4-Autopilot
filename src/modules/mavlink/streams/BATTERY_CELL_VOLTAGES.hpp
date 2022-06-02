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

#ifndef BATTERY_CELL_VOLTAGES_HPP
#define BATTERY_CELL_VOLTAGES_HPP

#include <uORB/topics/battery_status.h>

class MavlinkStreamBatteryCellVoltages : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamBatteryCellVoltages(mavlink); }

	static constexpr const char *get_name_static() { return "BATTERY_CELL_VOLTAGES"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_BATTERY_CELL_VOLTAGES; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		static constexpr unsigned size_per_battery = MAVLINK_MSG_ID_BATTERY_CELL_VOLTAGES_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		return size_per_battery * _battery_status_subs.advertised_count();
	}

private:
	explicit MavlinkStreamBatteryCellVoltages(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::SubscriptionMultiArray<battery_status_s, battery_status_s::MAX_INSTANCES> _battery_status_subs{ORB_ID::battery_status};

	bool send() override
	{
		bool updated = false;

		for (auto &battery_sub : _battery_status_subs) {
			battery_status_s battery_status;

			if (battery_sub.update(&battery_status)) {

				if (battery_status.connected) {
					// We don't know the cell count or we don't know the independent cell voltages we don't report anything - leave cells alone
					if (battery_status.cell_count == 0 || battery_status.voltage_cell_v[0] < 0.0001f) {
						// do nothing. We have nothing to send for this battery.
					} else {
  
                        /* Get number of cells in message (12) using temporary message */
						mavlink_battery_cell_voltages_t temp_bat_msg{};
						static constexpr int mavlink_cell_slots = (sizeof(temp_bat_msg.voltages) / sizeof(temp_bat_msg.voltages[0]));

						/* work out how many cells that we need to send and hence how many messages */
						static constexpr int uorb_cell_slots =
							(sizeof(battery_status.voltage_cell_v) / sizeof(battery_status.voltage_cell_v[0]));

						const int cell_slots = math::min(static_cast<int>(battery_status.cell_count), uorb_cell_slots);
						const int number_full_messages_of_cells = cell_slots / mavlink_cell_slots;
						const int number_partial_messages = cell_slots % mavlink_cell_slots;
						const int number_of_messages = number_full_messages_of_cells + (number_partial_messages==0) ? 0 : 1;

						/* Create message for each index of cells */
						for (int message_number = 0; message_number < number_of_messages; message_number++) {
							/* battery status message with higher resolution */
							mavlink_battery_cell_voltages_t bat_msg{};
							// TODO: Determine how to better map between battery ID within the firmware and in MAVLink
							bat_msg.id = battery_status.id - 1;
							bat_msg.index = message_number;
							/* populate each cell in current index */
							for (int cell = 0; cell < mavlink_cell_slots; cell++) {
								int current_cell = cell + (message_number*mavlink_cell_slots);
								bat_msg.voltages[cell] = 0; /* reset all fields to zero */
                                // populate cell from uorb (if it exists)
								if (current_cell <= cell_slots) {
									bat_msg.voltages[cell] = battery_status.voltage_cell_v[current_cell] * 1000.f;
								}
							}
							mavlink_msg_battery_cell_voltages_send_struct(_mavlink->get_channel(), &bat_msg);
							updated = true;
						}
					}
				}
			}
		}
		return updated;
	}
};

#endif // BATTERY_CELL_VOLTAGES_HPP
