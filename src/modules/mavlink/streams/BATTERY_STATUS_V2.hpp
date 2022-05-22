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
				/* battery status message with higher resolution */
				mavlink_battery_status_v2_t bat_msg{};

				// TODO: Determine how to better map between battery ID within the firmware and in MAVLink
				bat_msg.id = battery_status.id - 1;
				bat_msg.current_consumed = (battery_status.connected) ? battery_status.discharged_mah : UINT32_MAX;
                bat_msg.current = (battery_status.connected) ? battery_status.current_filtered_a * 1000 : UINT32_MAX; // mA
				bat_msg.percent_remaining = (battery_status.connected) ? roundf(battery_status.remaining * 100.f) : UINT32_MAX;
				bat_msg.status_flags = battery_status.status_flags;
				bat_msg.voltage = (battery_status.connected) ? battery_status.voltage_filtered_v * 1000.f : UINT32_MAX; 
				
				// get remaining charge in mAH
				if (battery_status.connected && PX4_ISFINITE(battery_status.remaining_capacity_wh) && (battery_status.voltage_filtered_v > 0.f) ) {
					bat_msg.current_remaining = (battery_status.remaining_capacity_wh / battery_status.voltage_filtered_v) * 1000.f; // mAH

				} else {
					bat_msg.current_remaining = UINT32_MAX;
				}

				// check if temperature valid
				if (battery_status.connected && PX4_ISFINITE(battery_status.temperature)) {
					bat_msg.temperature = battery_status.temperature * 100.f;

				} else {
					bat_msg.temperature = INT16_MAX;
				}

				mavlink_msg_battery_status_v2_send_struct(_mavlink->get_channel(), &bat_msg);
				updated = true;
			}
		}

		return updated;
	}
};

#endif // BATTERY_STATUS_V2_HPP
