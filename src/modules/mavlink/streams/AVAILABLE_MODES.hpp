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

#ifndef AVAILABLE_MODES_HPP
#define AVAILABLE_MODES_HPP

#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <lib/modes/standard_modes.hpp>
#include <lib/modes/ui.hpp>

class MavlinkStreamAvailableModes : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamAvailableModes(mavlink); }

	~MavlinkStreamAvailableModes() { delete[] _external_mode_names; }

	static constexpr const char *get_name_static() { return "AVAILABLE_MODES"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_AVAILABLE_MODES; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _had_dynamic_update ? MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	static constexpr int MAX_NUM_EXTERNAL_MODES = vehicle_status_s::NAVIGATION_STATE_EXTERNAL8 -
			vehicle_status_s::NAVIGATION_STATE_EXTERNAL1 + 1;

	explicit MavlinkStreamAvailableModes(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	struct ExternalModeName {
		char name[sizeof(register_ext_component_reply_s::name)] {};
	};
	ExternalModeName *_external_mode_names{nullptr};

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _register_ext_component_reply_sub{ORB_ID(register_ext_component_reply)};

	bool _had_dynamic_update{false};
	uint8_t _dynamic_update_seq{0};
	uint32_t _last_valid_nav_states_mask{0};
	uint32_t _last_can_set_nav_states_mask{0};

	void send_single_mode(const vehicle_status_s &vehicle_status, int mode_index, int total_num_modes, uint8_t nav_state)
	{
		mavlink_available_modes_t available_modes{};
		available_modes.mode_index = mode_index;
		available_modes.number_modes = total_num_modes;
		px4_custom_mode custom_mode{get_px4_custom_mode(nav_state)};
		available_modes.custom_mode = custom_mode.data;
		const bool cannot_be_selected = (vehicle_status.can_set_nav_states_mask & (1u << nav_state)) == 0;

		// Set the mode name if not a standard mode
		available_modes.standard_mode = (uint8_t)mode_util::getStandardModeFromNavState(nav_state, vehicle_status.vehicle_type,
						vehicle_status.is_vtol);

		if (mode_util::isAdvanced(nav_state)) {
			available_modes.properties |= MAV_MODE_PROPERTY_ADVANCED;
		}

		if (available_modes.standard_mode == MAV_STANDARD_MODE_NON_STANDARD) {
			static_assert(sizeof(available_modes.mode_name) >= sizeof(ExternalModeName::name), "mode name too short");

			// Is it an external mode?
			unsigned external_mode_index = nav_state - vehicle_status_s::NAVIGATION_STATE_EXTERNAL1;

			if (nav_state >= vehicle_status_s::NAVIGATION_STATE_EXTERNAL1 && external_mode_index < MAX_NUM_EXTERNAL_MODES) {
				if (cannot_be_selected) {
					// If not selectable, it's not registered
					strcpy(available_modes.mode_name, "(Mode not available)");

				} else if (_external_mode_names) {
					strncpy(available_modes.mode_name, _external_mode_names[external_mode_index].name, sizeof(available_modes.mode_name));
					available_modes.mode_name[sizeof(available_modes.mode_name) - 1] = '\0';
				}

			} else { // Internal
				if (nav_state < sizeof(mode_util::nav_state_names) / sizeof(mode_util::nav_state_names[0])) {
					strncpy(available_modes.mode_name, mode_util::nav_state_names[nav_state], sizeof(available_modes.mode_name));
					available_modes.mode_name[sizeof(available_modes.mode_name) - 1] = '\0';
				}
			}
		}

		if (cannot_be_selected) {
			available_modes.properties |= MAV_MODE_PROPERTY_NOT_USER_SELECTABLE;
		}

		mavlink_msg_available_modes_send_struct(_mavlink->get_channel(), &available_modes);
	}

	bool request_message(float param2, float param3, float param4,
			     float param5, float param6, float param7) override
	{
		bool ret = false;
		int mode_index = roundf(param2);
		PX4_DEBUG("AVAILABLE_MODES request (%i)", mode_index);

		vehicle_status_s vehicle_status;

		if (!_vehicle_status_sub.copy(&vehicle_status)) {
			return false;
		}

		int total_num_modes = math::countSetBits(vehicle_status.valid_nav_states_mask);

		if (mode_index == 0) { // All
			int cur_mode_index = 1;

			for (uint8_t nav_state = 0; nav_state < vehicle_status_s::NAVIGATION_STATE_MAX; ++nav_state) {
				if ((1u << nav_state) & vehicle_status.valid_nav_states_mask) {
					send_single_mode(vehicle_status, cur_mode_index, total_num_modes, nav_state);
					++cur_mode_index;
				}
			}

			ret = true;

		} else if (mode_index <= total_num_modes) {
			// Find index
			int cur_index = 0;
			uint8_t nav_state = 0;

			for (; nav_state < vehicle_status_s::NAVIGATION_STATE_MAX; ++nav_state) {
				if ((1u << nav_state) & vehicle_status.valid_nav_states_mask) {
					if (++cur_index == mode_index) {
						break;
					}
				}
			}

			if (nav_state < vehicle_status_s::NAVIGATION_STATE_MAX) {
				send_single_mode(vehicle_status, mode_index, total_num_modes, nav_state);
			}

			ret = true;
		}

		return ret;
	}

	void update_data() override
	{
		// Keep track of externally registered modes
		register_ext_component_reply_s reply;
		bool dynamic_update = false;

		if (_register_ext_component_reply_sub.update(&reply)) {
			if (reply.success && reply.mode_id != -1) {
				if (!_external_mode_names) {
					_external_mode_names = new ExternalModeName[MAX_NUM_EXTERNAL_MODES];
				}

				unsigned mode_index = reply.mode_id - vehicle_status_s::NAVIGATION_STATE_EXTERNAL1;

				if (_external_mode_names && mode_index < MAX_NUM_EXTERNAL_MODES) {
					memcpy(_external_mode_names[mode_index].name, reply.name, sizeof(ExternalModeName::name));
				}

				dynamic_update = true;
			}
		}

		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			if (_last_valid_nav_states_mask == 0) {
				_last_valid_nav_states_mask = vehicle_status.valid_nav_states_mask;
			}

			if (_last_can_set_nav_states_mask == 0) {
				_last_can_set_nav_states_mask = vehicle_status.can_set_nav_states_mask;
			}

			if (vehicle_status.valid_nav_states_mask != _last_valid_nav_states_mask) {
				dynamic_update = true;
				_last_valid_nav_states_mask = vehicle_status.valid_nav_states_mask;
			}

			if (vehicle_status.can_set_nav_states_mask != _last_can_set_nav_states_mask) {
				dynamic_update = true;
				_last_can_set_nav_states_mask = vehicle_status.can_set_nav_states_mask;
			}
		}

		if (dynamic_update) {
			_had_dynamic_update = true;
			++_dynamic_update_seq;
		}
	}

	bool send() override
	{
		if (_had_dynamic_update) {
			mavlink_available_modes_monitor_t monitor{};
			monitor.seq = _dynamic_update_seq;
			mavlink_msg_available_modes_monitor_send_struct(_mavlink->get_channel(), &monitor);
			return true;
		}

		return false;
	}
};

#endif // AVAILABLE_MODES_HPP
