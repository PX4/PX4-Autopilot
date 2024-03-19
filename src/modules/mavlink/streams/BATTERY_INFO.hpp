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

#ifndef BATTERY_INFO_HPP
#define BATTERY_INFO_HPP

#include <uORB/topics/battery_status.h>

class MavlinkStreamBatteryInfo : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamBatteryInfo(mavlink); }

	static constexpr const char *get_name_static() { return "BATTERY_INFO"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_BATTERY_INFO; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		static constexpr unsigned size_per_battery = MAVLINK_MSG_ID_BATTERY_INFO_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		return size_per_battery * _battery_status_subs.advertised_count();
	}

private:
	explicit MavlinkStreamBatteryInfo(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::SubscriptionMultiArray<battery_status_s, battery_status_s::MAX_INSTANCES> _battery_status_subs{ORB_ID::battery_status};

	bool send() override
	{
		bool updated = false;

		for (auto &battery_sub : _battery_status_subs) {
			battery_status_s battery_status;

			if (battery_sub.update(&battery_status)) {
				if (battery_status.serial_number == 0) {
					// Required to emit
					continue;
				}

				mavlink_battery_info_t msg{};

				msg.id = battery_status.id - 1;
				msg.design_capacity = (float)(battery_status.capacity * 1000);
				msg.full_charge_capacity = (float)(battery_status.state_of_health * battery_status.capacity * 1000.f) / 100.f;
				msg.cycle_count = battery_status.cycle_count;

				if (battery_status.manufacture_date) {
					uint16_t day = battery_status.manufacture_date % 32;
					uint16_t month = (battery_status.manufacture_date >> 5) % 16;
					uint16_t year = (80 + (battery_status.manufacture_date >> 9));
					uint16_t year2dig = year % 100;

					//Formatted as 'ddmmyyyy' (maxed 9 chars)
					snprintf(msg.manufacture_date, sizeof(msg.manufacture_date), "%d%d%d", day, month, year);
					//Formatted as 'dd/mm/yy-123456' (maxed 15 + 1 chars)
					snprintf(msg.serial_number, sizeof(msg.serial_number), "%d/%d/%d-%d", day, month, year2dig,
						 battery_status.serial_number);

				} else {

					snprintf(msg.serial_number, sizeof(msg.serial_number), "%d", battery_status.serial_number);
				}

				// Not supported by PX4 (not in battery_status uorb topic)
				/*
				msg.name = 0; // char[50]
				msg.weight = 0;
				msg.discharge_minimum_voltage = 0;
				msg.charging_minimum_voltage = 0;
				msg.resting_minimum_voltage = 0;
				msg.charging_maximum_voltage = 0;
				msg.charging_maximum_current = 0;
				msg.discharge_maximum_current = 0;
				msg.discharge_maximum_burst_current = 0;
				msg.cells_in_series = 0;
				msg.nominal_voltage = 0;
				*/

				mavlink_msg_battery_info_send_struct(_mavlink->get_channel(), &msg);
				updated = true;
			}
		}

		return updated;
	}
};

#endif // BATTERY_INFO_HPP
