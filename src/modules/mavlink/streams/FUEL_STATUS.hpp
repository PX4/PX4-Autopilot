/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#ifndef FUEL_STATUS_HPP
#define FUEL_STATUS_HPP

#include <uORB/topics/fuel_tank_status.h>

class MavlinkStreamFuelStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamFuelStatus(mavlink); }

	static constexpr const char *get_name_static() { return "FUEL_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_FUEL_STATUS; }

	const char *get_name() const override { return MavlinkStreamFuelStatus::get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _fuel_tank_status_sub.advertised() ? MAVLINK_MSG_ID_FUEL_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamFuelStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _fuel_tank_status_sub{ORB_ID(fuel_tank_status)};

	bool send() override
	{
		fuel_tank_status_s fuel_status;

		if (_fuel_tank_status_sub.update(&fuel_status)) {
			mavlink_fuel_status_t msg{};

			msg.id = fuel_status.fuel_tank_id;
			msg.maximum_fuel = fuel_status.maximum_fuel_capacity;
			msg.consumed_fuel = fuel_status.consumed_fuel;
			msg.remaining_fuel = fuel_status.remaining_fuel;
			msg.percent_remaining = fuel_status.percent_remaining;
			msg.flow_rate = fuel_status.fuel_consumption_rate;
			msg.temperature = fuel_status.temperature;
			msg.fuel_type = fuel_status.fuel_type;

			mavlink_msg_fuel_status_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}

};

#endif // FUEL_STATUS_HPP
