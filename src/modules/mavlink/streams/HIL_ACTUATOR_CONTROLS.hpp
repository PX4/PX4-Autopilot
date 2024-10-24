/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef HIL_ACTUATOR_CONTROLS_HPP
#define HIL_ACTUATOR_CONTROLS_HPP

#include <px4_platform_common/module_params.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/esc_status.h>

class MavlinkStreamHILActuatorControls : public MavlinkStream, ModuleParams
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamHILActuatorControls(mavlink); }

	static constexpr const char *get_name_static() { return "HIL_ACTUATOR_CONTROLS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _act_sub.advertised() ? MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamHILActuatorControls(Mavlink *mavlink) :
		MavlinkStream(mavlink),
		ModuleParams(nullptr)
	{
		_act_sub = uORB::Subscription{ORB_ID(actuator_outputs_sim)};
		mavlink->register_orb_poll(get_id_static(), _orbs, arraySize(_orbs));
		for (int i = 0; i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; ++i) {
			char param_name[17];
			snprintf(param_name, sizeof(param_name), "%s_%s%d", "PWM_MAIN", "FUNC", i + 1);
			param_get(param_find(param_name), &_output_functions[i]);
		}
	}

	~MavlinkStreamHILActuatorControls()
	{
		_mavlink->unregister_orb_poll(get_id_static());
	}

	ORB_ID _orbs[3] {
		ORB_ID::actuator_outputs,
		ORB_ID::vehicle_status,
		ORB_ID::vehicle_control_mode
	};

	uORB::Subscription _act_sub{ORB_ID(actuator_outputs)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};

	int32_t _output_functions[actuator_outputs_s::NUM_ACTUATOR_OUTPUTS] {};

	void send_esc_telemetry(mavlink_hil_actuator_controls_t &hil_act_control, vehicle_status_s &vehicle_status)
	{
		esc_status_s esc_status{};
		esc_status.timestamp = hrt_absolute_time();
		const int max_esc_count = math::min(actuator_outputs_s::NUM_ACTUATOR_OUTPUTS, esc_status_s::CONNECTED_ESC_MAX);

		const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		int max_esc_index = 0;

		for (int i = 0; i < max_esc_count; i++) {
			if (_output_functions[i] != 0) {
				max_esc_index = i;
			}

			esc_status.esc[i].actuator_function = _output_functions[i]; // TODO: this should be in pwm_sim...
			esc_status.esc[i].timestamp = esc_status.timestamp;
			esc_status.esc[i].esc_errorcount = 0; // TODO
			esc_status.esc[i].esc_voltage = 11.1f + math::abs_t(hil_act_control.controls[i]) * 3.0f; // TODO: magic number
			esc_status.esc[i].esc_current = armed ? 1.0f + math::abs_t(hil_act_control.controls[i]) * 15.0f :
							0.0f; // TODO: magic number
			esc_status.esc[i].esc_rpm = hil_act_control.controls[i] * 6000;  // TODO: magic number
			esc_status.esc[i].esc_temperature = 20.0f + math::abs_t(hil_act_control.controls[i]) * 40.0f;
		}

		esc_status.esc_count = max_esc_index + 1;
		esc_status.esc_armed_flags = (1u << esc_status.esc_count) - 1;
		esc_status.esc_online_flags = (1u << esc_status.esc_count) - 1;

		_esc_status_pub.publish(esc_status);
	}

	bool send() override
	{
		actuator_outputs_s act;

		if (_act_sub.update(&act)) {
			mavlink_hil_actuator_controls_t msg{};
			msg.time_usec = act.timestamp;

			for (unsigned i = 0; i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; i++) {
				msg.controls[i] = act.output[i];
			}

			// mode (MAV_MODE_FLAG)
			msg.mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

			vehicle_control_mode_s control_mode;

			if (_vehicle_control_mode_sub.copy(&control_mode)) {
				if (control_mode.flag_control_auto_enabled) {
					msg.mode |= MAV_MODE_FLAG_AUTO_ENABLED;
				}

				if (control_mode.flag_control_manual_enabled) {
					msg.mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
				}

				if (control_mode.flag_control_attitude_enabled) {
					msg.mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
				}
			}

			vehicle_status_s status;

			if (_vehicle_status_sub.copy(&status)) {
				if (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
					msg.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
				}

				if (status.hil_state == vehicle_status_s::HIL_STATE_ON) {
					msg.mode |= MAV_MODE_FLAG_HIL_ENABLED;
				}

				if (status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
					msg.mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
				}
			}

			msg.flags = 0;

			mavlink_msg_hil_actuator_controls_send_struct(_mavlink->get_channel(), &msg);

			send_esc_telemetry(msg, status);

			return true;
		}

		return false;
	}
};

#endif // HIL_ACTUATOR_CONTROLS_HPP
