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

#ifndef OPEN_DRONE_ID_BASIC_ID_HPP
#define OPEN_DRONE_ID_BASIC_ID_HPP

#include <uORB/topics/open_drone_id_basic_id.h>
#include <uORB/topics/vehicle_status.h>
#include <modules/mavlink/open_drone_id_bridge.hpp>
#include <modules/mavlink/open_drone_id_translations.hpp>
#if defined(CONFIG_BOARD_HAS_RID_EEPROM)
#include <lib/open_drone_id/odid_bid_storage.hpp>
#endif

class MavlinkStreamOpenDroneIdBasicId : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamOpenDroneIdBasicId(mavlink); }

	static constexpr const char *get_name_static() { return "OPEN_DRONE_ID_BASIC_ID"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
#if defined(CONFIG_BOARD_HAS_RID_EEPROM)

		if (_odid_bid_src == open_drone_id::UAS_ID_MODE_LOCK_FIRST_BASIC_ID) {
			return MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

#endif
		return (_configuration_sub.advertised() || _vehicle_status_sub.advertised())
		       ? MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamOpenDroneIdBasicId(Mavlink *mavlink) : MavlinkStream(mavlink)
	{
#if defined(CONFIG_BOARD_HAS_RID_EEPROM)
		// ODID_BID_SRC requires a reboot, as on the DroneCAN path.
		param_get(param_handle(px4::params::ODID_BID_SRC), &_odid_bid_src);
#endif
	}

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _configuration_sub{ORB_ID(open_drone_id_basic_id)};
#if defined(CONFIG_BOARD_HAS_RID_EEPROM)
	int32_t _odid_bid_src {0};
	open_drone_id::BidLockInfo _odid_bid_lock_info{};
	bool _odid_bid_lock_loaded{false};
#endif

	bool send() override
	{
		open_drone_id_basic_id_s configuration{};
#if defined(CONFIG_BOARD_HAS_RID_EEPROM)
		const bool identity_updated = _configuration_sub.updated();
#endif
		const bool have_configuration = _configuration_sub.copy(&configuration);

		if (have_configuration
		    && !open_drone_id_bridge::send_on_channel(configuration.mavlink_channel, _mavlink->get_channel())) {
			return false;
		}

		mavlink_open_drone_id_basic_id_t msg{};
#if defined(CONFIG_BOARD_HAS_RID_EEPROM)

		if (_odid_bid_src == open_drone_id::UAS_ID_MODE_LOCK_FIRST_BASIC_ID) {
			if (!_odid_bid_lock_loaded || (identity_updated && !_odid_bid_lock_info.locked())) {
				_odid_bid_lock_info = open_drone_id::BidStorage::get();
				_odid_bid_lock_loaded = true;
			}

			if (!_odid_bid_lock_info.locked()) {
				return false;
			}

			msg.id_type = MAV_ODID_ID_TYPE_SERIAL_NUMBER;
			msg.ua_type = _odid_bid_lock_info.id.ua_type;
			open_drone_id_bridge::copy_bytes(msg.uas_id, _odid_bid_lock_info.id.uas_id);
			mavlink_msg_open_drone_id_basic_id_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

#endif

		if (have_configuration) {
			open_drone_id_bridge::basic_id(msg, configuration);
			mavlink_msg_open_drone_id_basic_id_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		vehicle_status_s vehicle_status{};

		if (_vehicle_status_sub.update(&vehicle_status)) {
			msg.id_type = MAV_ODID_ID_TYPE_SERIAL_NUMBER;
			msg.ua_type = open_drone_id_translations::odidTypeForMavType(vehicle_status.system_type);
			// Legacy fallback when persistent identity locking is disabled.
			board_get_px4_guid_formated(reinterpret_cast<char *>(msg.uas_id), sizeof(msg.uas_id));
			mavlink_msg_open_drone_id_basic_id_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // OPEN_DRONE_ID_BASIC_ID_HPP
