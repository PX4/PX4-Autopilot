/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "DShot.h"
#include <cmath>
#include <px4_arch/io_timer.h>
#include <px4_platform_common/sem.hpp>

namespace
{
constexpr hrt_abstime TURTLE_PARAMETER_INTERVAL = 250_ms;
constexpr hrt_abstime TURTLE_STATE_TIMEOUT = 750_ms;
constexpr hrt_abstime TURTLE_ATTITUDE_TIMEOUT = 300_ms;
constexpr hrt_abstime TURTLE_BACKUP_INTERVAL = 10_ms;
constexpr float TURTLE_SWITCH_ON = 0.7f;
constexpr float TURTLE_SWITCH_OFF = -0.3f;
constexpr float TURTLE_THROTTLE_LOW = -0.85f;
constexpr float TURTLE_STICK_TOLERANCE = 1.05f;
// Temporary direction relative to the ESC's saved direction; never followed by SAVE_SETTINGS.
constexpr uint16_t TURTLE_DIRECTION_NORMAL = 20;
constexpr uint16_t TURTLE_DIRECTION_REVERSE = 21;
}

ModuleBase::Descriptor DShot::desc{task_spawn, custom_command, print_usage};

char DShot::_serial_port_path[20] {};
bool DShot::_telemetry_swap_rxtx{false};
px4::atomic_bool DShot::_request_telemetry_init{false};

DShot::DShot() :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_mixing_output.setAllDisarmedValues(DSHOT_DISARM_VALUE);
	_mixing_output.setAllMinValues(DSHOT_MIN_THROTTLE);
	_mixing_output.setAllMaxValues(DSHOT_MAX_THROTTLE);

	// Avoid using the PWM failsafe params
	_mixing_output.setAllFailsafeValues(UINT16_MAX);

	// Advertise early to ensure we beat uavcan
	_esc_status_pub.advertise();
}

DShot::~DShot()
{
	// make sure outputs are off
	up_dshot_arm(false);

	perf_free(_cycle_perf);
	perf_free(_bdshot_recv_perf);
	perf_free(_bdshot_error_perf);
	perf_free(_serial_telem_success_perf);
	perf_free(_serial_telem_error_perf);
	perf_free(_serial_telem_timeout_perf);
	perf_free(_serial_telem_allsampled_perf);
}

int DShot::init()
{
	update_params();

	if (initialize_dshot()) {
		ScheduleNow();
		return PX4_OK;
	}

	return PX4_ERROR;
}

void DShot::Run()
{
	// An owned transaction must finish restoration and OFF qualification before normal shutdown.
	// A latched fault survives destruction through the shared interlock until the FC reboots.
	if (should_exit() && (!_trtl_core.ownsOutputs() || _trtl_core.phase == dragonfly_trtl::Phase::FAULT)) {
		ScheduleClear();
		_mixing_output.unregister();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_cycle_perf);

	updateTurtleInputs();
	updateTurtleParameters();
	_trtl_output_seen = false;
	_mixing_output.update();

	if (!_trtl_output_seen && (_trtl_capable || _trtl_requested || _trtl_core.ownsOutputs()
				   || dragonfly_trtl::interlock().recoveryOwned()) && _hardware_initialized) {
		// Only recovery frames may be synthesized when the mixer had no update.
		_trtl_fallback = true;
		updateOutputs(nullptr, 0, 0);
		_trtl_fallback = false;
	}

	bool serial_updated = process_serial_telemetry();
	bool bdshot_updated = process_bdshot_telemetry();

	_telemetry.publishSettings();

	if (serial_updated || bdshot_updated) {
		_esc_status.timestamp = hrt_absolute_time();
		_esc_status.esc_count = _motor_count;
		_esc_status.counter++;
		_esc_status_pub.publish(_esc_status);
	}

	if (_parameter_update_sub.updated()) {
		update_params();
	}

	// Telemetry init hook, retried until the motor mask is known
	if (!dragonfly_trtl::interlock().recoveryOwned()
	    && !dragonfly_trtl::interlock().directionPending(_trtl_requested, _trtl_core.configured(), _trtl_core.synced())
	    && _request_telemetry_init.load()
	    && init_telemetry(_serial_port_path, _telemetry_swap_rxtx)) {
		_request_telemetry_init.store(false);
	}

	handle_vehicle_commands();

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true);

	if (_trtl_capable || _trtl_requested || _trtl_core.ownsOutputs() || dragonfly_trtl::interlock().recoveryOwned()) {
		ScheduleDelayed(TURTLE_BACKUP_INTERVAL);
	}

	perf_end(_cycle_perf);
}

bool DShot::updateOutputs(float *outputs, unsigned num_outputs, unsigned num_control_groups_updated)
{
	_trtl_output_seen = true;

	if (!_hardware_initialized) {
		return false;
	}

	if (updateTurtleOutputs()) {
		return true;
	}

	if (_trtl_fallback || !_motor_count) {
		return false;
	}

	uint16_t hw_outputs[MAX_ACTUATORS] = {};

	for (unsigned i = 0; i < num_outputs; i++) {
		hw_outputs[i] = static_cast<uint16_t>(lroundf(outputs[i]));
	}

	if (dragonfly_trtl::interlock().directionPending(_trtl_requested, _trtl_core.configured(), _trtl_core.synced())) {
		// A pending command or telemetry response may delay startup normalization.
		// Let that transaction finish, but never pass through actuator-test throttle.
		for (unsigned i = 0; i < num_outputs; ++i) {
			hw_outputs[i] = DSHOT_DISARM_VALUE;
		}
	}

	_esc_status.esc_armed_flags = esc_armed_mask(hw_outputs, num_outputs);
	bool armed = _esc_status.esc_armed_flags != 0;

	// Bluejay drops EDT when the motor stops, so it has to be re-enabled after every flight.
	if (_armed_prev && !armed) {
		for (int i = 0; i < DSHOT_MAX_MOTORS; i++) {
			clear_edt_confirmation(i);
		}
	}

	_armed_prev = armed;

	if (!armed) {
		// Select next command to send (if any)
		if (_telemetry.telemetryResponseFinished() &&
		    _current_command.finished() && _telemetry.commandResponseFinished()) {
			select_next_command();
		}

		// Send command if available
		if (!_current_command.finished()) {
			update_motor_commands(num_outputs);

		} else {
			update_motor_outputs(hw_outputs, num_outputs);
		}

	} else {
		update_motor_outputs(hw_outputs, num_outputs);
	}

	up_dshot_trigger();

	return true;
}

bool DShot::updateTurtleOutputs()
{
	using namespace dragonfly_trtl;
	updateTurtleInputs(); // The mixer has consumed the newest arming and actuator-test state.

	if (_trtl_core.ownsOutputs() && (!_current_command.finished() || _dshot_programming_active)) {
		// An ESC in programming mode can interpret direction frames as settings data.
		// Arbitration should prevent this state; if violated, only STOP is safe.
		_trtl_core.fault(Reason::SAFETY);
	}

	_trtl_frame = _trtl_core.step(hrt_absolute_time(), _trtl_input);

	if (_trtl_frame.action == Action::NORMAL
	    && interlock().directionPending(_trtl_requested, _trtl_core.configured(), _trtl_core.synced())
	    && _trtl_input.commands_idle) {
		// Do not let actuator tests bypass startup direction normalization.
		_trtl_frame = {Action::STOP, 0, 0};
	}

	if (_trtl_frame.action == Action::NORMAL) {
		return false;
	}

	_esc_status.esc_armed_flags = 0;

	for (unsigned i = 0; i < DSHOT_MAXIMUM_CHANNELS; ++i) {
		const uint32_t bit = uint32_t{1} << i;

		// _output_mask is the mask with which DShot hardware was initialized. Write STOP
		// even when the current mixer mapping no longer identifies that channel as a motor.
		if (!(_output_mask & bit)) {
			continue;
		}

		if (!(_trtl_frame.outputs & bit)) {
			up_dshot_motor_command(i, DSHOT_CMD_MOTOR_STOP, false);

		} else if (_trtl_frame.action == Action::REVERSE) {
			up_dshot_motor_command(i, TURTLE_DIRECTION_REVERSE, true);

		} else if (_trtl_frame.action == Action::RESTORE) {
			up_dshot_motor_command(i, TURTLE_DIRECTION_NORMAL, true);

		} else if (_trtl_frame.action == Action::THROTTLE) {
			up_dshot_motor_data_set(i, _trtl_frame.level, false);

		} else {
			up_dshot_motor_command(i, DSHOT_CMD_MOTOR_STOP, false);
		}
	}

	up_dshot_trigger();
	return true;
}

bool DShot::turtleChannelConflicts(int32_t channel) const
{
	return dragonfly_trtl::channelConflict(_trtl_rc_mappings, TURTLE_RC_MAPPING_COUNT, channel)
	       || dragonfly_trtl::modeButtonConflict(_trtl_mode_buttons, channel);
}

void DShot::updateTurtleParameters()
{
	using namespace dragonfly_trtl;
	const hrt_abstime now = hrt_absolute_time();

	if (_trtl_last_params && now >= _trtl_last_params && now - _trtl_last_params < TURTLE_PARAMETER_INTERVAL) {
		return;
	}

	_trtl_last_params = now;

	if (_trtl_enable_handle == PARAM_INVALID) {
		_trtl_enable_handle = param_find("TRTL_EN");
	}

	int32_t requested = 0;
	const bool recovery_enabled = _trtl_enable_handle != PARAM_INVALID
				      && param_get(_trtl_enable_handle, &requested) == PX4_OK && requested == 1;

	if (!_trtl_capability_loaded) {
		if (_trtl_esc_handle == PARAM_INVALID) {
			_trtl_esc_handle = param_find("TRTL_ESC");
		}

		int32_t esc = 0;

		if (_trtl_esc_handle != PARAM_INVALID && param_get(_trtl_esc_handle, &esc) == PX4_OK) {
			_trtl_capability_loaded = true;
			_trtl_capable = esc != 0;
			_trtl_esc_profile = esc;

			if (_trtl_capable) {
				interlock().requireDirection();
			}
		}
	}

	// Enabling recovery without a compatible ESC profile must not gate normal output.
	const bool recovery_requested = recovery_enabled && _trtl_capable;

	if (recovery_requested != _trtl_requested) {
		_trtl_params_pending = true;
	}

	_trtl_requested = recovery_requested;

	// ESC profile and output assignment are a powered-session contract. EN only
	// controls recovery entry; it cannot remove startup or abort restoration.
	if (!_trtl_capable) {
		return;
	}

	// Current rc_channels has no function array. These shared RC configuration parameters
	// reserve every non-AUX assignment, including arm/kill and flight-mode controls.
	static constexpr const char *rc_names[] = {
		"RC_MAP_ROLL", "RC_MAP_PITCH", "RC_MAP_FAILSAFE", "RC_MAP_THROTTLE", "RC_MAP_YAW",
		"RC_MAP_PARAM1", "RC_MAP_PARAM2", "RC_MAP_PARAM3", "RC_MAP_ENG_MOT", "RC_MAP_FLTMODE",
		"RC_MAP_RETURN_SW", "RC_MAP_LOITER_SW", "RC_MAP_OFFB_SW", "RC_MAP_KILL_SW", "RC_MAP_TERM_SW",
		"RC_MAP_ARM_SW", "RC_MAP_FLAPS", "RC_MAP_TRANS_SW", "RC_MAP_GEAR_SW",
		"RC_MAP_PAY_SW"
	};
	static_assert(sizeof(rc_names) / sizeof(rc_names[0]) == TURTLE_RC_MAPPING_COUNT, "RC map handle count");

	if (!_trtl_handles_initialized) {
		for (auto &handle : _trtl_handles) {
			handle = PARAM_INVALID;
		}

		for (auto &handle : _trtl_rc_handles) {
			handle = PARAM_INVALID;
		}

		_trtl_handles_initialized = true;
	}

	for (unsigned i = 0; i < TURTLE_RC_MAPPING_COUNT; ++i) {
		if (_trtl_rc_handles[i] == PARAM_INVALID) {
			_trtl_rc_handles[i] = param_find(rc_names[i]);
		}

		// A missing or unreadable active mapping fails closed. AUX assignments are deliberately absent.
		_trtl_rc_mappings[i] = -1;

		if (_trtl_rc_handles[i] != PARAM_INVALID) {
			if (param_get(_trtl_rc_handles[i], &_trtl_rc_mappings[i]) != PX4_OK) {
				_trtl_rc_mappings[i] = -1;
			}
		}
	}

	if (_trtl_mode_buttons_handle == PARAM_INVALID) {
		_trtl_mode_buttons_handle = param_find("RC_MAP_FLTM_BTN");
	}

	_trtl_mode_buttons = -1;

	if (_trtl_mode_buttons_handle != PARAM_INVALID
	    && param_get(_trtl_mode_buttons_handle, &_trtl_mode_buttons) != PX4_OK) {
		_trtl_mode_buttons = -1;
	}

	_trtl_rc_conflict = turtleChannelConflicts(_trtl_config.channel);

	// Keep disable and RC-conflict checks responsive without reloading unchanged configuration.
	if (_trtl_loaded && !_trtl_params_pending) {
		return;
	}

	// Disabling reaches the controller on this poll through Input::recovery_enabled.
	// All other settings remain frozen until restoration and OFF release are complete.
	if (_trtl_core.busy() || interlock().owner() == Owner::FLIGHT) {
		return;
	}

	if (!fresh(now, _turtle_status.timestamp, TURTLE_STATE_TIMEOUT)
	    || !fresh(now, _turtle_armed.timestamp, TURTLE_STATE_TIMEOUT)
	    || _turtle_status.arming_state != vehicle_status_s::ARMING_STATE_DISARMED
	    || _turtle_armed.armed || _mixing_output.armed().armed) {
		return;
	}

	if (_trtl_loaded && !_trtl_input.switch_off) {
		return;
	}

	// The earlier FLIGHT check is only an optimisation; reserve atomically before
	// changing readiness or latching a configuration fault.
	ConfigurationGuard configuration_guard(interlock());

	if (!configuration_guard.allowed()) {
		return;
	}

	static constexpr const char *names[] = {
		"TRTL_EN", "TRTL_ESC", "TRTL_CH", "TRTL_POL", "TRTL_ANGLE", "TRTL_EXIT", "TRTL_MIN", "TRTL_MAX",
		"TRTL_RAMP", "TRTL_TIME", "TRTL_PRE", "TRTL_POST", "TRTL_SETTLE", "TRTL_COOL", "TRTL_DB", "TRTL_PWR",
		"TRTL_HOLD", "TRTL_FL", "TRTL_FR", "TRTL_RL", "TRTL_RR", "CA_ROTOR_COUNT"
	};
	static_assert(sizeof(names) / sizeof(names[0]) == TURTLE_PARAMETER_COUNT, "Recovery parameter handle count");
	int32_t values[TURTLE_PARAMETER_COUNT] {};

	for (unsigned i = 0; i < TURTLE_PARAMETER_COUNT; ++i) {
		if (_trtl_handles[i] == PARAM_INVALID) {
			_trtl_handles[i] = param_find(names[i]);
		}

		if (_trtl_handles[i] == PARAM_INVALID || param_get(_trtl_handles[i], &values[i]) != PX4_OK) {
			_trtl_loaded = false;
			_trtl_core.invalidate();
			return;
		}
	}

	Config config;
	config.enabled = values[0];
	config.esc = _trtl_esc_profile; // Latched until a complete FC/ESC power cycle.
	config.channel = values[2];
	config.polarity = values[3];
	config.entry_deg = values[4];
	config.exit_deg = values[5];
	config.start = values[6];
	config.peak = values[7];
	config.ramp_ms = values[8];
	config.drive_ms = values[9];
	config.pre_ms = values[10];
	config.post_ms = values[11];
	config.settle_ms = values[12];
	config.cooldown_ms = values[13];
	config.deadband_pct = values[14];
	config.proportional = values[15];
	config.hold_ms = values[16];
	dragonfly_mapping::Config corners;

	for (unsigned i = 0; i < 4; ++i) {
		corners.motor[i] = values[17 + i];
	}

	int32_t functions[DSHOT_MAXIMUM_CHANNELS] {};

	for (unsigned i = 0; i < DSHOT_MAXIMUM_CHANNELS; ++i) {
		functions[i] = static_cast<int32_t>(_mixing_output.outputFunction(i));
	}

	const auto resolved = dragonfly_mapping::resolve(corners, functions, DSHOT_MAXIMUM_CHANNELS, _output_mask);
	config.outputs = resolved.all;
	_trtl_geometry_ok = values[21] == 4 && resolved.valid();
	static constexpr const char *suffix[] = {"PX", "PY", "AX", "AY", "AZ"};

	for (unsigned i = 0; i < 4 && _trtl_geometry_ok; ++i) {
		float geometry[5] {};

		for (unsigned j = 0; j < 5; ++j) {
			char name[24];
			snprintf(name, sizeof(name), "CA_ROTOR%ld_%s", long(corners.motor[i] - 1), suffix[j]);
			const param_t handle = param_find(name);

			if (handle == PARAM_INVALID || param_get(handle, &geometry[j]) != PX4_OK) {
				_trtl_geometry_ok = false;
				break;
			}
		}

		_trtl_geometry_ok = _trtl_geometry_ok
				    && quadCorner(i, geometry[0], geometry[1], geometry[2], geometry[3], geometry[4]);
		char tilt_name[24];
		snprintf(tilt_name, sizeof(tilt_name), "CA_ROTOR%ld_TILT", long(corners.motor[i] - 1));
		const param_t tilt_handle = param_find(tilt_name);
		int32_t tilt = 0;

		if (tilt_handle == PARAM_INVALID || param_get(tilt_handle, &tilt) != PX4_OK || tilt != 0) {
			_trtl_geometry_ok = false;
		}
	}

	const bool raw_ok = dragonfly_turtle_rc::healthy(now, _turtle_rc_channels.timestamp,
			    _turtle_rc_channels.timestamp_last_valid, _turtle_rc_channels.signal_lost);
	const float raw = raw_ok ? dragonfly_turtle_rc::select(_turtle_rc_channels.channels,
			  _turtle_rc_channels.channel_count, config.channel) : NAN;

	if (_trtl_mapping_latched && (!std::isfinite(raw) || raw * float(config.polarity) >= TURTLE_SWITCH_OFF)) {
		return;
	}

	_trtl_loaded = false;
	_trtl_rc_conflict = turtleChannelConflicts(config.channel);

	if (!resolved.valid() || !_trtl_geometry_ok || !config.valid()) {
		_trtl_core.invalidate();
		return;
	}

	if (_trtl_mapping_latched && !dragonfly_mapping::same(_trtl_map, resolved)) {
		// Do not certify a newly selected set while the previous physical ESCs
		// might still hold temporary direction state.
		_trtl_core.fault(Reason::MAPPING);
		return;
	}

	_trtl_config = config;
	_trtl_map_config = corners;
	_trtl_map = resolved;
	_trtl_loaded = _trtl_core.configure(config);

	if (_trtl_loaded) {
		_trtl_params_pending = false;
	}

	_trtl_mapping_latched = _trtl_mapping_latched || _trtl_loaded;
}

void DShot::updateTurtleInputs()
{
	using namespace dragonfly_trtl;
	_turtle_manual_sub.update(&_turtle_manual);
	_turtle_status_sub.update(&_turtle_status);
	_turtle_armed_sub.update(&_turtle_armed);
	_turtle_att_sub.update(&_turtle_att);
	_turtle_rc_sub.update(&_turtle_rc_channels);
	const hrt_abstime now = hrt_absolute_time();
	Input input;
	const bool state_fresh = fresh(now, _turtle_status.timestamp, TURTLE_STATE_TIMEOUT)
				 && fresh(now, _turtle_armed.timestamp, TURTLE_STATE_TIMEOUT);
	const auto &armed = _mixing_output.armed();
	input.disarmed = state_fresh && _turtle_status.arming_state == vehicle_status_s::ARMING_STATE_DISARMED
			 && !_turtle_armed.armed && !armed.armed;
	input.hardware_ok = state_fresh && !_turtle_armed.lockdown && !_turtle_armed.kill && !_turtle_armed.termination
			    && !_turtle_armed.in_esc_calibration_mode && !armed.lockdown && !armed.kill && !armed.termination
			    && !armed.in_esc_calibration_mode && (!_turtle_status.safety_button_available || _turtle_status.safety_off)
			    && !_turtle_status.calibration_enabled && !_turtle_status.rc_calibration_in_progress
			    && !_turtle_status.failsafe && !_mixing_output.isActuatorTestRunning();
	input.commands_idle = _current_command.finished() && !_dshot_programming_active
			      && _telemetry.commandResponseFinished() && _telemetry.telemetryResponseFinished();
	input.recovery_enabled = _trtl_requested && !should_exit();
	int32_t functions[DSHOT_MAXIMUM_CHANNELS] {};

	for (unsigned i = 0; i < DSHOT_MAXIMUM_CHANNELS; ++i) {
		functions[i] = static_cast<int32_t>(_mixing_output.outputFunction(i));
	}

	const auto current = dragonfly_mapping::resolve(_trtl_map_config, functions, DSHOT_MAXIMUM_CHANNELS, _output_mask);
	_trtl_map_ok = _trtl_loaded && _trtl_geometry_ok && dragonfly_mapping::same(current, _trtl_map)
		       && !_turtle_status.is_vtol && _turtle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		       && !_3d_enabled && !_mixing_output.reversibleOutputs();
	input.mapping_ok = _trtl_map_ok;
	const bool raw_ok = dragonfly_turtle_rc::healthy(now, _turtle_rc_channels.timestamp,
			    _turtle_rc_channels.timestamp_last_valid, _turtle_rc_channels.signal_lost);
	const float raw = raw_ok ? dragonfly_turtle_rc::select(_turtle_rc_channels.channels,
			  _turtle_rc_channels.channel_count, _trtl_config.channel) : NAN;
	input.rc_ok = raw_ok && std::isfinite(raw) && !_trtl_rc_conflict
		      && manualInputFresh(now, _turtle_manual.timestamp, _turtle_manual.timestamp_sample)
		      && _turtle_manual.valid && _turtle_manual.data_source == manual_control_setpoint_s::SOURCE_RC;
	const float switch_value = raw * float(_trtl_config.polarity);
	input.switch_on = input.rc_ok && switch_value > TURTLE_SWITCH_ON;
	input.switch_off = input.rc_ok && switch_value < TURTLE_SWITCH_OFF;
	input.throttle_low = input.rc_ok && std::isfinite(_turtle_manual.throttle)
			     && _turtle_manual.throttle >= -TURTLE_STICK_TOLERANCE
			     && _turtle_manual.throttle < TURTLE_THROTTLE_LOW;
	input.vertical = vertical(_turtle_att.q);
	input.attitude_ok = fresh(now, _turtle_att.timestamp, TURTLE_ATTITUDE_TIMEOUT) && std::isfinite(input.vertical);
	const float roll = _turtle_manual.roll;
	const float pitch = _turtle_manual.pitch;

	if (std::isfinite(roll) && std::isfinite(pitch)
	    && std::fabs(roll) <= TURTLE_STICK_TOLERANCE && std::fabs(pitch) <= TURTLE_STICK_TOLERANCE) {
		input.selected = dragonfly_mapping::select(_trtl_map, roll, pitch, float(_trtl_config.deadband_pct) * 0.01f);
		input.strength = math::min(math::max(std::fabs(roll), std::fabs(pitch)), 1.f);

	} else {
		input.rc_ok = false;
	}

	_trtl_input = input;
}

// TODO: this needs a refactor
void DShot::select_next_command()
{
	// Settings Programming
	// NOTE: only update when we're not actively programming an ESC
	if (!_dshot_programming_active) {

		if (_esc_eeprom_write_sub.updated()) {
			auto last_generation = _esc_eeprom_write_sub.get_last_generation();
			_esc_eeprom_write_sub.copy(&_esc_eeprom_write);
			auto current_generation = _esc_eeprom_write_sub.get_last_generation();

			if (current_generation != last_generation + 1) {
				PX4_ERR("esc_eeprom_write lost, generation %u -> %u", last_generation, current_generation);
			}

			// A command for a motor without an output is never sent, so it never finishes and blocks every
			// later command and the serial telemetry.
			uint16_t write_mask = _motor_mask;

			if (_esc_eeprom_write.index != 255) {
				write_mask = _esc_eeprom_write.index < DSHOT_MAX_MOTORS ? (_motor_mask & (1u << _esc_eeprom_write.index)) : 0;
			}

			if (write_mask == 0) {
				PX4_ERR("esc_eeprom_write: invalid index %u", _esc_eeprom_write.index);

			} else {
				PX4_DEBUG("ESC%u: starting programming mode", _esc_eeprom_write.index + 1);
				_dshot_programming_active = true;
				_telemetry.invalidateSettings(write_mask);
			}
		}
	}

	// Command order or priority:
	// - EDT Request
	// - Settings Request
	// - Settings Programming
	// Programming, once started, runs to the save without the others: an ESC in programming mode takes the
	// next frames of any command as the address and value, and a read before the save returns old settings.

	// EDT Request mask
	uint16_t needs_edt_request_mask = _bdshot_telem_online_mask & ~_bdshot_edt_confirmed_mask;

	bool serial_telem_delay_elapsed = hrt_absolute_time() > _serial_telem_delay_until;

	_current_command.clear();

	// EDT Request: use motor-order masks since needs_edt_request_mask is in motor order
	uint16_t edt_motors_to_request = _bdshot_motor_mask & needs_edt_request_mask;

	if (_bdshot_edt_enabled && !_dshot_programming_active && edt_motors_to_request != 0) {
		// Find first motor that needs EDT request and has been online long enough
		hrt_abstime now = hrt_absolute_time();

		for (int motor_index = 0; motor_index < DSHOT_MAX_MOTORS; motor_index++) {
			if (!(edt_motors_to_request & (1 << motor_index))) {
				continue;
			}

			// The ESC only takes commands once it has seen zero throttle for a while (AM32: one second plus
			// its arming tune), so hold off after it comes online and space the retries.
			if (_bdshot_telem_online_timestamps[motor_index] == 0
			    || (now - _bdshot_telem_online_timestamps[motor_index]) < 1_s
			    || (now - _bdshot_edt_last_request[motor_index]) < BDSHOT_EDT_RETRY_INTERVAL) {
				continue;
			}

			if (_bdshot_edt_attempts[motor_index] >= BDSHOT_EDT_MAX_ATTEMPTS) {
				// Warn once, a retry interval after the last attempt went unanswered
				if (_bdshot_edt_attempts[motor_index] == BDSHOT_EDT_MAX_ATTEMPTS) {
					PX4_WARN("ESC%d: no EDT frames after %d enable requests", motor_index + 1, BDSHOT_EDT_MAX_ATTEMPTS);
					_bdshot_edt_attempts[motor_index]++;
				}

				continue;
			}

			_current_command.num_repetitions = 10;
			_current_command.command = DSHOT_EXTENDED_TELEMETRY_ENABLE;
			_current_command.motor_mask = (1 << motor_index);
			_bdshot_edt_attempts[motor_index]++;
			_bdshot_edt_last_request[motor_index] = now;
			PX4_DEBUG("ESC%d: requesting EDT (attempt %d) at time %.2fs", motor_index + 1,
				  _bdshot_edt_attempts[motor_index], (double)now / 1000000.);
			break;
		}
	}

	// A motor held off or out of EDT attempts leaves no EDT command, and must not keep the settings reads waiting.
	if (_current_command.finished() && _esc_type != 0 && _serial_telemetry_enabled && serial_telem_delay_elapsed
	    && !_dshot_programming_active && !_mixing_output.armed().armed) {
		const int motor_index = _telemetry.getSettingsRequest(_motor_mask & _serial_telem_online_mask);

		if (motor_index >= 0) {
			_current_command.num_repetitions = 6;
			_current_command.command = DSHOT_CMD_ESC_INFO;
			_current_command.motor_mask = (1 << motor_index);
			_current_command.expect_response = true;
			PX4_DEBUG("ESC%d: requesting settings", motor_index + 1);
		}

	} else if (_dshot_programming_active) {
		// Motor mask for programming: all motors or single motor
		const uint16_t programming_motor_mask = _esc_eeprom_write.index == 255
							? (uint16_t)((1u << DSHOT_MAX_MOTORS) - 1)
							: (uint16_t)(1u << _esc_eeprom_write.index);

		// Settings programming state machine
		if (_programming_state == ProgrammingState::Idle) {
			// Get next setting address/value to program
			int next_index = -1;

			// Find settings that need to be written but haven't been yet
			int max_length = math::min((int)_esc_eeprom_write.length, (int)sizeof(_esc_eeprom_write.data));

			for (int i = 0; i < max_length; i++) {
				int array_index = i / 32;
				int bit_index = i % 32;

				bool needs_write = _esc_eeprom_write.write_mask[array_index] & (1 << bit_index);
				bool already_written = _settings_written_mask[array_index] & (1 << bit_index);

				if (needs_write && !already_written) {
					next_index = i;
					break;
				}
			}

			if (next_index >= 0) {
				if (_esc_eeprom_write.index == 255) {
					PX4_DEBUG("ESC ALL: Writing setting at index %d, value %u", next_index, _esc_eeprom_write.data[next_index]);

				} else {
					PX4_DEBUG("ESC%d: Writing setting at index %d, value %u", _esc_eeprom_write.index + 1, next_index,
						  _esc_eeprom_write.data[next_index]);
				}

				_programming_address = next_index;
				_programming_value = _esc_eeprom_write.data[next_index];
				_programming_state = ProgrammingState::EnterMode;

				// Pre-emptively Mark this setting as written
				int array_index = next_index / 32;
				int bit_index = next_index % 32;
				_settings_written_mask[array_index] |= (1 << bit_index);

			} else {
				// All settings have been written
				PX4_DEBUG("All settings written at time %.2fs", (double)hrt_absolute_time() / 1000000.);

				_current_command.command = DSHOT_CMD_SAVE_SETTINGS;
				_current_command.is_save = true;
				_current_command.num_repetitions = 6;
				_current_command.motor_mask = programming_motor_mask;
				_programming_state = ProgrammingState::Idle;

				// Clear the written mask for this motor for next time
				_settings_written_mask[0] = 0;
				_settings_written_mask[1] = 0;
			}
		}

		switch (_programming_state) {
		case ProgrammingState::EnterMode:
			_current_command.command = DSHOT_CMD_ENTER_PROGRAMMING_MODE;
			_current_command.num_repetitions = 6;
			_current_command.motor_mask = programming_motor_mask;
			_programming_state = ProgrammingState::SendAddress;
			break;

		case ProgrammingState::SendAddress:
			_current_command.command = _programming_address;
			_current_command.num_repetitions = 1;
			_current_command.motor_mask = programming_motor_mask;
			_programming_state = ProgrammingState::SendValue;
			break;

		case ProgrammingState::SendValue:
			_current_command.command = _programming_value;
			_current_command.num_repetitions = 1;
			_current_command.motor_mask = programming_motor_mask;
			_programming_state = ProgrammingState::ExitMode;
			break;

		case ProgrammingState::ExitMode:
			_current_command.command = DSHOT_CMD_EXIT_PROGRAMMING_MODE;
			_current_command.num_repetitions = 1;
			_current_command.motor_mask = programming_motor_mask;
			_programming_state = ProgrammingState::Idle;
			break;

		default:
			break;
		}
	}
}

void DShot::update_motor_outputs(uint16_t outputs[MAX_ACTUATORS], int num_outputs)
{
	for (int i = 0; i < num_outputs; i++) {
		int motor_index = motor_index_from_output(i);

		if (motor_index < 0) {
			up_dshot_motor_command(i, DSHOT_CMD_MOTOR_STOP, false);
			continue;
		}

		bool set_telemetry_bit = (_telemetry_motor_index == motor_index
					  && _serial_telemetry_enabled
					  && _telemetry.telemetryResponseFinished()
					  && _telemetry.commandResponseFinished()
					  && hrt_absolute_time() > _serial_telem_delay_until);

		if (set_telemetry_bit) {
			_telemetry.startTelemetryRequest();
		}

		if (outputs[i] == DSHOT_DISARM_VALUE) {
			up_dshot_motor_command(i, DSHOT_CMD_MOTOR_STOP, set_telemetry_bit);

		} else {
			uint16_t output = calculate_output_value(outputs[i], i);

			// 3D deadzone: send motor stop to avoid MIN_throttle offset in up_dshot_motor_data_set
			if (output == DSHOT_DISARM_VALUE) {
				up_dshot_motor_command(i, DSHOT_CMD_MOTOR_STOP, set_telemetry_bit);

			} else {
				up_dshot_motor_data_set(i, output, set_telemetry_bit);
			}
		}
	}
}

void DShot::update_motor_commands(int num_outputs)
{
	bool command_sent = false;

	for (int i = 0; i < num_outputs; i++) {
		uint16_t command = DSHOT_CMD_MOTOR_STOP;
		int motor_index = motor_index_from_output(i);

		if (motor_index >= 0 && (_current_command.motor_mask & (1 << motor_index))) {
			if (_current_command.expect_response) {
				_telemetry.setExpectCommandResponse(motor_index, _current_command.command);
			}

			command = _current_command.command;
			command_sent = true;
		}

		// Bluejay/BLHeli_S discard commands unless the tlm bit is set. It stays clear on commands answered
		// over the telemetry UART: AM32 reads it as a KISS request and starts that frame on the same wire,
		// then aborts it a few loop iterations later to send the EEPROM dump.
		const bool request_telemetry = command != DSHOT_CMD_MOTOR_STOP && !_current_command.expect_response;
		up_dshot_motor_command(i, command, request_telemetry);
	}

	if (command_sent) {
		--_current_command.num_repetitions;

		if (_current_command.finished() && _current_command.is_save) {
			// Programming ends with the save: anything sent in its place leaves the ESC on unsaved settings.
			_dshot_programming_active = false;
			_telemetry.invalidateSettings(_current_command.motor_mask);

			_serial_telem_delay_until = hrt_absolute_time() + 500_ms;
		}

		// Queue a save command if it has been requested
		if (_current_command.num_repetitions == 0 && _current_command.save) {
			_current_command.save = false;
			_current_command.is_save = true;
			_current_command.num_repetitions = 10;
			_current_command.command = DSHOT_CMD_SAVE_SETTINGS;
		}
	}
}

void DShot::clear_edt_confirmation(int motor_index)
{
	_bdshot_edt_confirmed_mask &= ~(1 << motor_index);
	_bdshot_edt_attempts[motor_index] = 0;
	_bdshot_edt_last_request[motor_index] = hrt_absolute_time();
}

uint16_t DShot::esc_armed_mask(uint16_t *outputs, uint8_t num_outputs)
{
	uint16_t mask = 0;

	for (uint8_t i = 0; i < num_outputs; i++) {
		int motor_index = motor_index_from_output(i);

		if (motor_index >= 0 && outputs[i] != DSHOT_DISARM_VALUE) {
			mask |= (1 << motor_index);
		}
	}

	return mask;
}

uint16_t DShot::calculate_output_value(uint16_t raw, int index)
{
	uint16_t output = raw;
	const bool reversible = _mixing_output.reversibleOutputs() & (1u << index);

	// Reverse output if required
	if (_3d_enabled || reversible) {
		output = convert_output_to_3d_scaling(raw, reversible);
	}

	output = math::min(output, DSHOT_MAX_THROTTLE);

	return output;
}

bool DShot::process_serial_telemetry()
{
	if (!_serial_telemetry_enabled) {
		return false;
	}

	// Periodically retry skipped motors when disarmed so channels that came online late
	// (e.g. ESC power-cycled after boot) can recover. We don't retry while armed to avoid
	// timeout blips on healthy channels during flight.
	bool armed = _esc_status.esc_armed_flags != 0;

	if (!armed && _serial_telem_skip_mask != 0
	    && hrt_elapsed_time(&_serial_telem_last_retry) > SERIAL_TELEM_RETRY_INTERVAL) {
		_serial_telem_skip_mask = 0;

		for (int i = 0; i < DSHOT_MAX_MOTORS; i++) {
			_serial_telem_consecutive_timeouts[i] = 0;
		}

		_serial_telem_last_retry = hrt_absolute_time();
	}

	bool all_telem_sampled = false;

	if (!_telemetry.commandResponseFinished()) {
		if (_telemetry.commandResponseStarted()) {
			_telemetry.parseCommandResponse();
		}

	} else if (_telemetry_motor_index < 0) {
		// Bootstrap: pick the first motor to poll
		set_next_telemetry_index();

	} else {

		int motor_index = _telemetry_motor_index;

		EscData esc {};
		esc.source = TelemetrySource::Serial;
		esc.motor_index = motor_index;

		switch (_telemetry.parseTelemetryPacket(&esc)) {
		case TelemetryStatus::NotStarted:
			// no-op, should not hit this case
			break;

		case TelemetryStatus::NotReady:
			// no-op, will eventually timeout
			break;

		case TelemetryStatus::Ready:

			// Reset consecutive timeout counter on any successful response
			_serial_telem_consecutive_timeouts[motor_index] = 0;
			_serial_telem_skip_mask &= ~(1 << motor_index);

			// Online mask is in motor order for consistency with BDShot
			if (_serial_telem_online_mask & (1 << motor_index)) {
				consume_esc_data(esc);
				all_telem_sampled = set_next_telemetry_index();
				perf_count(_serial_telem_success_perf);

			} else {
				hrt_abstime now = hrt_absolute_time();

				// Timestamps are in actuator order to match _esc_status.esc[]
				if (_serial_telem_online_timestamps[motor_index] == 0) {
					_serial_telem_online_timestamps[motor_index] = now;
				}

				// Mark as online only after 100_ms without errors
				if (now - _serial_telem_online_timestamps[motor_index] > 100_ms) {
					_serial_telem_online_mask |= (1 << motor_index);
				}
			}

			break;

		case TelemetryStatus::Timeout:
			// Set ESC data to zeroes
			// Error counts and timestamps are in actuator order to match _esc_status.esc[]
			_serial_telem_errors[motor_index]++;
			_serial_telem_online_mask &= ~(1 << motor_index);
			_serial_telem_online_timestamps[motor_index] = 0;

			// Track consecutive timeouts for adaptive skip
			if (_serial_telem_consecutive_timeouts[motor_index] < SERIAL_TELEM_SKIP_THRESHOLD) {
				_serial_telem_consecutive_timeouts[motor_index]++;

				if (_serial_telem_consecutive_timeouts[motor_index] >= SERIAL_TELEM_SKIP_THRESHOLD) {
					_serial_telem_skip_mask |= (1 << motor_index);
					_telemetry.invalidateSettings(1u << motor_index);
					PX4_DEBUG("ESC%d serial telemetry lost, skipping", motor_index + 1);
				}
			}

			// Consume an empty EscData to zero the data
			consume_esc_data(esc);
			all_telem_sampled = set_next_telemetry_index();
			perf_count(_serial_telem_timeout_perf);
			break;

		case TelemetryStatus::ParseError:
			// Set ESC data to zeroes
			PX4_WARN("Telem parse error, ESC %d", motor_index);
			_serial_telem_errors[motor_index]++;
			_serial_telem_online_mask &= ~(1 << motor_index);
			_serial_telem_online_timestamps[motor_index] = 0;
			// Consume an empty EscData to zero the data
			consume_esc_data(esc);
			all_telem_sampled = set_next_telemetry_index();
			_serial_telem_delay_until = hrt_absolute_time() + 1_s;
			perf_count(_serial_telem_error_perf);
			break;
		}
	}

	return all_telem_sampled;
}

bool DShot::set_next_telemetry_index()
{
	// Round-robin through motor indices (Motor1=0, Motor2=1, ...).
	// _telemetry_motor_index and _telemetry_requested_mask are in motor-index domain.

	// Active motors are those that exist and aren't being skipped
	uint16_t active_motor_mask = _motor_mask & ~_serial_telem_skip_mask;

	if (active_motor_mask == 0) {
		_telemetry_motor_index = -1;
		_telemetry_requested_mask = 0;
		return true;
	}

	int start = (_telemetry_motor_index < 0) ? 0 : (_telemetry_motor_index + 1) % DSHOT_MAX_MOTORS;
	int motor = start;

	do {
		if ((active_motor_mask & (1 << motor)) && !(_telemetry_requested_mask & (1 << motor))) {
			_telemetry_motor_index = motor;
			_telemetry_requested_mask |= (1 << motor);
			return false;
		}

		motor = (motor + 1) % DSHOT_MAX_MOTORS;
	} while (motor != start);

	// All active motors have been sampled
	_telemetry_motor_index = -1;
	_telemetry_requested_mask = 0;
	perf_count(_serial_telem_allsampled_perf);
	return true;
}

bool DShot::process_bdshot_telemetry()
{
	if (!_bdshot_output_mask) {
		return false;
	}

	hrt_abstime now = hrt_absolute_time();

	// Don't try to process any telem data until after ESCs have been given time to boot
	if (now < ESC_INIT_TELEM_DELAY) {
		return false;
	}

	// We wait until all BDShot channels are ready.
	if ((up_bdshot_get_ready_mask() & _bdshot_output_mask) != _bdshot_output_mask) {
		return false;
	}

	for (uint8_t output_channel = 0; output_channel < DSHOT_MAXIMUM_CHANNELS; output_channel++) {
		if (!(_bdshot_output_mask & (1 << output_channel))) {
			continue;
		}

		int motor_index = motor_index_from_output(output_channel);

		if (motor_index < 0) {
			continue;
		}

		EscData esc = {};
		esc.source = TelemetrySource::BDShot;
		esc.motor_index = motor_index;
		esc.timestamp = now;

		_bdshot_telem_errors[motor_index] = up_bdshot_num_errors(output_channel);

		if (up_bdshot_channel_online(output_channel)) {

			// Record when this motor first came online
			if (!(_bdshot_telem_online_mask & (1 << motor_index))) {
				_bdshot_telem_online_timestamps[motor_index] = now;
			}

			// Online mask is in motor order for command/request logic
			_bdshot_telem_online_mask |= (1 << motor_index);

			// Only update RPM if online
			int erpm = 0;

			if (up_bdshot_get_erpm(output_channel, &erpm) == PX4_OK) {
				esc.erpm = erpm * 100;

			} else {
				// Use previous value (esc_status is indexed by motor_index)
				int pole_count = math::max(get_pole_count(motor_index), 2); // constrain to 2 to avoid divide by zero
				esc.erpm = _esc_status.esc[motor_index].esc_rpm * (pole_count / 2);
			}

			// Extended DShot Telemetry
			if (_bdshot_edt_enabled) {

				uint8_t value = 0;

				// The enable ack is a single state/event frame and can be lost, so any EDT frame counts
				bool edt_frame = up_bdshot_get_extended_telemetry(output_channel, DSHOT_EDT_STATE_EVENT, &value) == PX4_OK;

				if (up_bdshot_get_extended_telemetry(output_channel, DSHOT_EDT_TEMPERATURE, &value) == PX4_OK) {
					esc.temperature = value; // BDShot temperature is in C
					edt_frame = true;

				} else {
					esc.temperature = _esc_status.esc[motor_index].esc_temperature; // use previous
				}

				if (up_bdshot_get_extended_telemetry(output_channel, DSHOT_EDT_VOLTAGE, &value) == PX4_OK) {
					esc.voltage = static_cast<float>(value) / 4.f; // BDShot voltage is in 0.25V
					edt_frame = true;

				} else {
					esc.voltage = _esc_status.esc[motor_index].esc_voltage; // use previous
				}

				if (up_bdshot_get_extended_telemetry(output_channel, DSHOT_EDT_CURRENT, &value) == PX4_OK) {
					esc.current = static_cast<float>(value); // EDT current is in 1A steps
					edt_frame = true;

				} else {
					esc.current = _esc_status.esc[motor_index].esc_current; // use previous
				}

				// Only frames after our own request count, or one still in flight at the disarm edge would
				// hide Bluejay dropping EDT with the motor stop.
				if (edt_frame && _bdshot_edt_attempts[motor_index] > 0) {
					_bdshot_edt_confirmed_mask |= (1 << motor_index);
				}
			}

		} else {
			_bdshot_telem_online_mask &= ~(1 << motor_index);
			_bdshot_telem_online_timestamps[motor_index] = 0;
			clear_edt_confirmation(motor_index);
			perf_count(_bdshot_error_perf);
		}

		consume_esc_data(esc);
	}

	perf_count(_bdshot_recv_perf);

	return true;
}

void DShot::consume_esc_data(const EscData &esc)
{
	int motor_index = esc.motor_index;

	if (!math::isInRange(motor_index, 0, DSHOT_MAX_MOTORS - 1)) {
		return;
	}

	// Determine if this motor is online based on its telemetry sources.
	// Intentionally conservative: when both BDShot and serial telemetry are enabled,
	// both must report online. A motor reporting offline on either source indicates
	// a degraded setup that should be surfaced to the operator.
	uint16_t motor_bit = 1u << motor_index;
	bool is_bdshot = _bdshot_motor_mask & motor_bit;

	// Both sources must report online when enabled (conservative)
	bool motor_online = (!is_bdshot || (_bdshot_telem_online_mask & motor_bit))
			    && (!_serial_telemetry_enabled || (_serial_telem_online_mask & motor_bit));

	if (motor_online) {
		_esc_status.esc_online_flags |= motor_bit;

	} else {
		_esc_status.esc_online_flags &= ~motor_bit;
	}

	// esc_status is indexed by motor_index (Motor1=0, Motor2=1, ...)
	_esc_status.esc[motor_index].esc_errorcount = _serial_telem_errors[motor_index] +
			_bdshot_telem_errors[motor_index];

	if (esc.source == TelemetrySource::Serial) {
		// Only use SerialTelemetry eRPM when BDShot is disabled
		if (!is_bdshot) {
			_esc_status.esc[motor_index].timestamp = esc.timestamp;
			int pole_count = math::max(get_pole_count(motor_index), 2); // constrain to 2 to avoid divide by zero
			_esc_status.esc[motor_index].esc_rpm = esc.erpm / (pole_count / 2);
		}

		_esc_status.esc[motor_index].esc_voltage = esc.voltage;
		_esc_status.esc[motor_index].esc_current = esc.current;
		_esc_status.esc[motor_index].esc_temperature = esc.temperature;

	} else if (esc.source == TelemetrySource::BDShot) {
		_esc_status.esc[motor_index].timestamp = esc.timestamp;
		int pole_count = math::max(get_pole_count(motor_index), 2); // constrain to 2 to avoid divide by zero
		_esc_status.esc[motor_index].esc_rpm = esc.erpm / (pole_count / 2);

		// Only use BDShot Volt/Curr/Temp when Serial Telemetry is disabled
		if (!_serial_telemetry_enabled) {
			_esc_status.esc[motor_index].esc_voltage = esc.voltage;
			_esc_status.esc[motor_index].esc_current = esc.current;
			_esc_status.esc[motor_index].esc_temperature = esc.temperature;
		}
	}
}

uint16_t DShot::convert_output_to_3d_scaling(uint16_t output, bool reversible)
{
	// DShot 3D splits the throttle ranges in two.
	// This is in terms of DShot values, code below is in terms of actuator_output
	// Direction 1) 48 is the slowest, 1047 is the fastest.
	// Direction 2) 1049 is the slowest, 2047 is the fastest.

	// The deadband guards a neutral, which only a reversible output has.
	if (reversible && output >= _3d_dead_l && output <= _3d_dead_h) {
		return DSHOT_DISARM_VALUE;
	}

	bool upper_range = output >= DSHOT_3D_SPLIT;

	if (upper_range) {
		output -= DSHOT_3D_SPLIT;

	} else {
		output = DSHOT_3D_SPLIT - 1 - output; // lower range is inverted
	}

	float max_output = DSHOT_3D_SPLIT - 1;
	float min_output = max_output * _dshot_min;
	output = math::min(max_output, (min_output + output * (max_output - min_output) / max_output));

	if (upper_range) {
		output += DSHOT_3D_SPLIT;
	}

	return output;
}

void DShot::handle_vehicle_commands()
{
	vehicle_command_s command = {};

	// Wait for an outstanding settings read, which completes within COMMAND_RESPONSE_TIMEOUT: a setting changed
	// before it lands would have the read restore the old cache.
	while (_current_command.finished() && _telemetry.commandResponseFinished() && _vehicle_command_sub.update(&command)) {

		switch (command.command) {
		case vehicle_command_s::VEHICLE_CMD_CONFIGURE_ACTUATOR:
			handle_configure_actuator(command);
			break;

		case vehicle_command_s::VEHICLE_CMD_ESC_REQUEST_EEPROM:
			handle_esc_request_eeprom(command);
			break;

		default:
			break;
		}
	}
}

void DShot::rejectTurtleCommand(const vehicle_command_s &command)
{
	vehicle_command_ack_s ack{};
	ack.timestamp = hrt_absolute_time();
	ack.command = command.command;
	ack.target_system = command.source_system;
	ack.target_component = command.source_component;
	ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
	_command_ack_pub.publish(ack);
}

void DShot::handle_configure_actuator(const vehicle_command_s &command)
{
	if (dragonfly_trtl::interlock().recoveryOwned()
	    || dragonfly_trtl::interlock().directionPending(_trtl_requested, _trtl_core.configured(), _trtl_core.synced())) {
		rejectTurtleCommand(command);
		return;
	}

	int function = (int)(command.param5 + 0.5);

	if (function > 1000) {
		// NOTE: backwards compatibility for QGC - 1101=Motor1, 1102=Motor2, etc
		function -= 1000;

	} else if (function >= 1 && function <= 16) {
		// MAVLink standard: ACTUATOR_OUTPUT_FUNCTION_MOTOR1=1 .. MOTOR16=16
		// PX4 internal:     OutputFunction::Motor1=101 .. Motor12=112
		function += 100;
	}

	int motor_index = -1;

	if (function >= (int)OutputFunction::Motor1 && function < ((int)OutputFunction::Motor1 + DSHOT_MAXIMUM_CHANNELS)) {
		for (int i = 0; i < DSHOT_MAXIMUM_CHANNELS; ++i) {
			if ((int)_mixing_output.outputFunction(i) == function && _mixing_output.isMotor(i)) {
				motor_index = (int)_mixing_output.outputFunction(i) - (int)OutputFunction::Motor1;
			}
		}
	}

	vehicle_command_ack_s command_ack = {};
	command_ack.command = command.command;
	command_ack.target_system = command.source_system;
	command_ack.target_component = command.source_component;
	command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;

	if (_dshot_programming_active) {
		// Programming spans many commands and would take the next frames as an address and value.
		command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;

	} else if ((motor_index >= 0) && (motor_index < DSHOT_MAX_MOTORS)) {
		int type = lroundf(command.param1);
		PX4_DEBUG("motor_index: %i type: %i", motor_index, type);
		_current_command.clear();
		_current_command.command = DSHOT_CMD_MOTOR_STOP;
		_current_command.num_repetitions = 10;
		_current_command.save = false;


		switch (type) {
		case vehicle_command_s::ACTUATOR_CONFIGURATION_BEEP:
			_current_command.command = DSHOT_CMD_BEEP1;
			break;

		case vehicle_command_s::ACTUATOR_CONFIGURATION_3D_MODE_OFF:
			_current_command.command = DSHOT_CMD_3D_MODE_OFF;
			_current_command.save = true;
			break;

		case vehicle_command_s::ACTUATOR_CONFIGURATION_3D_MODE_ON:
			_current_command.command = DSHOT_CMD_3D_MODE_ON;
			_current_command.save = true;
			break;

		case vehicle_command_s::ACTUATOR_CONFIGURATION_SPIN_DIRECTION1:
			_current_command.command = DSHOT_CMD_SPIN_DIRECTION_1;
			_current_command.save = true;
			break;

		case vehicle_command_s::ACTUATOR_CONFIGURATION_SPIN_DIRECTION2:
			_current_command.command = DSHOT_CMD_SPIN_DIRECTION_2;
			_current_command.save = true;
			break;

		default:
			PX4_WARN("unknown command: %i", type);
			break;
		}

		if (_current_command.command != DSHOT_CMD_MOTOR_STOP) {
			command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
			_current_command.motor_mask = 1 << motor_index;

			if (_current_command.save) {
				_telemetry.invalidateSettings(_current_command.motor_mask);
			}
		}
	}

	command_ack.timestamp = hrt_absolute_time();
	_command_ack_pub.publish(command_ack);
}

void DShot::handle_esc_request_eeprom(const vehicle_command_s &command)
{
	if (dragonfly_trtl::interlock().recoveryOwned()
	    || dragonfly_trtl::interlock().directionPending(_trtl_requested, _trtl_core.configured(), _trtl_core.synced())) {
		rejectTurtleCommand(command);
		return;
	}

	PX4_DEBUG("Received ESC_REQUEST_EEPROM");
	PX4_DEBUG("esc_index: %d", (int)command.param2);

	int esc_index = lroundf(command.param2);

	if (esc_index != 255 && (esc_index < 0 || esc_index >= DSHOT_MAX_MOTORS)) {
		PX4_ERR("ESC_REQUEST_EEPROM: invalid esc_index %d", esc_index);
		return;
	}

	_telemetry.requestSettings(esc_index == 255 ? _motor_mask : (1u << esc_index));
}

int DShot::get_pole_count(int motor_index) const
{
	if (motor_index >= 0 && motor_index < DSHOT_MAX_MOTORS) {
		return _pole_count_params[motor_index];
	}

	return 14;
}

void DShot::apply_min_values(uint32_t reversible)
{
	for (uint8_t i = 0; i < DSHOT_MAXIMUM_CHANNELS; i++) {
		if (reversible & (1u << i)) {
			// Spans both halves, so neutral has to land on the split.
			_mixing_output.minValue(i) = DSHOT_MIN_THROTTLE;

		} else if (_3d_enabled) {
			_mixing_output.minValue(i) = DSHOT_3D_FORWARD_START;

		} else {
			const float min_value = _dshot_min * (float)DSHOT_MAX_THROTTLE;
			_mixing_output.minValue(i) = math::constrain((uint16_t)min_value, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
		}
	}
}

void DShot::reversibleMaskChanged(uint32_t reversible_mask)
{
	apply_min_values(reversible_mask);
}

void DShot::update_params()
{
	_trtl_params_pending = true;

	parameter_update_s pupdate;
	_parameter_update_sub.copy(&pupdate);

	updateParams();

	// Cache params used in hot path
	_bdshot_edt_enabled = _param_dshot_bidir_edt.get();
	_3d_enabled = _param_dshot_3d_enable.get();
	_3d_dead_l = _param_dshot_3d_dead_l.get();
	_3d_dead_h = _param_dshot_3d_dead_h.get();
	_dshot_min = _param_dshot_min.get();
	_esc_type = _param_dshot_esc_type.get();

	// Re-apply after updateParams(), which reloads the minimums from the shared output params.
	apply_min_values(_mixing_output.reversibleOutputs());

	// Update per-motor pole count param handles and cached values
	for (int i = 0; i < DSHOT_MAX_MOTORS; i++) {
		char name[20];
		snprintf(name, sizeof(name), "DSHOT_MOT_POL%d", i + 1);
		_param_pole_count_handles[i] = param_find(name);

		int32_t pole_count = 14;
		param_get(_param_pole_count_handles[i], &pole_count);
		_pole_count_params[i] = pole_count;
	}
}

void DShot::mixerChanged()
{
	// Output assignments can be applied after the parameter notification was consumed.
	_trtl_params_pending = true;

	PX4_DEBUG("mixerChanged");

	_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_DSHOT;

	// Build actuator-order and motor-order masks from actual motor assignments
	uint32_t new_output_mask = 0;
	uint32_t new_motor_mask = 0;

	for (int actuator_channel = 0; actuator_channel < DSHOT_MAXIMUM_CHANNELS; actuator_channel++) {
		int motor_index = motor_index_from_output(actuator_channel);

		if (motor_index >= 0) {
			new_output_mask |= (1 << actuator_channel);
			new_motor_mask |= (1 << motor_index);
			_esc_status.esc[motor_index].actuator_function = (uint8_t)_mixing_output.outputFunction(actuator_channel);
		}
	}

	// Check if we need to (re)initialize hardware
	// bool needs_init = !_hardware_initialized || (new_output_mask != _output_mask);

	if (!_hardware_initialized) {
		PX4_DEBUG("Output mask changed: 0x%" PRIx32 " -> 0x%" PRIx32, _output_mask, new_output_mask);
		_output_mask = new_output_mask;
		_motor_mask = new_motor_mask;
		_motor_count = __builtin_popcount(_output_mask);

		uint32_t new_bdshot_output_mask = _bdshot_timer_channels & _output_mask;
		PX4_DEBUG("BDShot Output mask changed: 0x%" PRIx32 " -> 0x%" PRIx32, _bdshot_output_mask, new_bdshot_output_mask);
		_bdshot_output_mask = new_bdshot_output_mask;

		// Compute motor-order BDShot mask
		uint32_t new_bdshot_motor_mask = 0;

		for (int actuator_channel = 0; actuator_channel < DSHOT_MAXIMUM_CHANNELS; actuator_channel++) {
			int motor_index = motor_index_from_output(actuator_channel);

			if (motor_index >= 0 && (new_bdshot_output_mask & (1 << actuator_channel))) {
				new_bdshot_motor_mask |= (1 << motor_index);
			}
		}

		_bdshot_motor_mask = new_bdshot_motor_mask;
		PX4_DEBUG("Motor mask: 0x%" PRIx32 ", BDShot motor mask: 0x%" PRIx32, _motor_mask, _bdshot_motor_mask);

		up_dshot_init(_output_mask, _bdshot_output_mask, _dshot_frequency, _bdshot_edt_enabled);
		up_dshot_arm(true);
		_hardware_initialized = true;
	}
}

bool DShot::initialize_dshot()
{
	uint32_t dshot_timer_channels = 0;  // Channels on DShot-enabled timers

	// Iterate through timers to determine DShot frequency and BDShot channels
	for (uint8_t timer_index = 0; timer_index < MAX_IO_TIMERS; timer_index++) {

		// Get mask of actuator channels associated with this timer group
		uint32_t timer_channels = io_timer_get_group(timer_index);

		if (timer_channels == 0) {
			continue;
		}

		char param_name[17] = {};
		snprintf(param_name, sizeof(param_name), "%s_TIM%u", _mixing_output.paramPrefix(), timer_index);

		int32_t tim_config = 0;
		param_t handle = param_find(param_name);
		param_get(handle, &tim_config);

		// Map timer config to DShot frequency and BDShot flag
		uint32_t freq = 0;
		bool is_bdshot = false;

		switch (tim_config) {
		case TIM_CONFIG_DSHOT150:  freq = DSHOT150; break;

		case TIM_CONFIG_DSHOT300:  freq = DSHOT300; break;

		case TIM_CONFIG_DSHOT600:  freq = DSHOT600; break;

		case TIM_CONFIG_BDSHOT150: freq = DSHOT150; is_bdshot = true; break;

		case TIM_CONFIG_BDSHOT300: freq = DSHOT300; is_bdshot = true; break;

		case TIM_CONFIG_BDSHOT600: freq = DSHOT600; is_bdshot = true; break;

		default: break;
		}

		if (freq > 0) {
			if (_dshot_frequency != 0 && _dshot_frequency != freq) {
				PX4_WARN("Mixed DShot frequencies across timers, using freq: %" PRIu32, freq);
			}

			_dshot_frequency = freq;
			dshot_timer_channels |= timer_channels;

			if (is_bdshot) {
				_bdshot_timer_channels |= timer_channels;
			}
		}
	}

	if (dshot_timer_channels == 0) {
		PX4_INFO("No channels configured");
		return false;
	}

	return true;
}

bool DShot::init_telemetry(const char *device, bool swap_rxtx)
{
	if (!device) {
		return true;
	}

	// The settings handlers are created per motor, and the mask is only known once the mixer has run
	if (_motor_mask == 0) {
		return false;
	}

	if (_telemetry.init(device, swap_rxtx) != PX4_OK) {
		PX4_ERR("telemetry init failed");
		return true;
	}

	// Enable serial telemetry now that we've successfully initialized
	_serial_telemetry_enabled = true;

	// Initialize ESC settings handlers based on ESC type
	ESCType esc_type = static_cast<ESCType>(_param_dshot_esc_type.get());
	_telemetry.initSettingsHandlers(esc_type, _motor_mask);

	return true;
}

static void print_spacer()
{
	for (int i = 0; i < 64; i++) {
		PX4_INFO_RAW("-");
	}

	PX4_INFO_RAW("\n");
}

int DShot::print_status()
{
	print_spacer();
	PX4_INFO("DShot Driver Status");
	print_spacer();
	PX4_INFO("Turtle: configured=%d enabled=%d synced=%d phase=%s reason=%s",
		 int(_trtl_loaded), int(_trtl_core.enabled()), int(_trtl_core.synced()),
		 dragonfly_trtl::phaseName(_trtl_core.phase), dragonfly_trtl::reasonName(_trtl_core.reason));
	PX4_INFO("TRTL: capability=%d pending=%d", int(_trtl_capable),
		 int(dragonfly_trtl::interlock().directionPending(_trtl_requested, _trtl_core.configured(), _trtl_core.synced())));
	PX4_INFO("TRTL: map=%d geometry=%d rc_conflict=%d owner=%u ready=%d",
		 int(_trtl_map_ok), int(_trtl_geometry_ok), int(_trtl_rc_conflict),
		 unsigned(dragonfly_trtl::interlock().owner()), int(dragonfly_trtl::interlock().ready()));
	PX4_INFO("TRTL: outputs FL=%u FR=%u RL=%u RR=%u CH=%ld POL=%ld",
		 dragonfly_mapping::outputNumber(_trtl_map.corner[0]), dragonfly_mapping::outputNumber(_trtl_map.corner[1]),
		 dragonfly_mapping::outputNumber(_trtl_map.corner[2]), dragonfly_mapping::outputNumber(_trtl_map.corner[3]),
		 long(_trtl_config.channel), long(_trtl_config.polarity));
	PX4_INFO("TRTL: entry=%ld exit=%ld start=%ld peak=%ld ramp=%ld drive=%ld",
		 long(_trtl_config.entry_deg), long(_trtl_config.exit_deg), long(_trtl_config.start),
		 long(_trtl_config.peak), long(_trtl_config.ramp_ms), long(_trtl_config.drive_ms));
	PX4_INFO("TRTL: attempts=%u restores_sent=%u reverse_frames=%u restore_frames=%u throttle_frames=%u last_drive_ms=%u",
		 unsigned(_trtl_core.attempts), unsigned(_trtl_core.restore_sequences_sent), unsigned(_trtl_core.reverse_frames),
		 unsigned(_trtl_core.restore_frames), unsigned(_trtl_core.throttle_frames), unsigned(_trtl_core.last_drive_ms));
	PX4_INFO("TRTL: rc=%d on=%d off=%d low=%d disarmed=%d hardware=%d commands_idle=%d tilt_z=%.3f selected=0x%x level=%u",
		 int(_trtl_input.rc_ok), int(_trtl_input.switch_on), int(_trtl_input.switch_off), int(_trtl_input.throttle_low),
		 int(_trtl_input.disarmed), int(_trtl_input.hardware_ok), int(_trtl_input.commands_idle), double(_trtl_input.vertical),
		 unsigned(_trtl_frame.outputs), unsigned(_trtl_frame.level));

	// Configuration
	PX4_INFO("Configuration:");
	PX4_INFO("  Output Mask:        0x%02lx (%u channels)", (unsigned long)_output_mask, _motor_count);
	PX4_INFO("  BDShot Telemetry:   %s", _bdshot_output_mask ? "Enabled" : "Disabled");
	PX4_INFO("  Serial Telemetry:   %s", _serial_telemetry_enabled ? "Enabled" : "Disabled");
	PX4_INFO("  Extended DShot:     %s", _bdshot_edt_enabled ? "Enabled" : "Disabled");

	const char *esc_type_str = "Unknown";

	switch (_param_dshot_esc_type.get()) {
	case 1: esc_type_str = "AM32"; break;

	default: break;
	}

	PX4_INFO("  ESC Type:           %s (%" PRId32 ")", esc_type_str, _param_dshot_esc_type.get());
	PX4_INFO("  3D Mode:            %s", _param_dshot_3d_enable.get() ? "Enabled" : "Disabled");

	// Per-motor pole counts
	PX4_INFO("  Motor Poles:");

	for (int i = 0; i < DSHOT_MAX_MOTORS; i++) {
		if (_motor_mask & (1 << i)) {
			PX4_INFO("    Motor %d: %d poles", i + 1, get_pole_count(i));
		}
	}

	// Telemetry Status
	if (_bdshot_output_mask || _serial_telemetry_enabled) {
		print_spacer();
		PX4_INFO("Telemetry Status:");
		PX4_INFO("  %-4s %-6s %-8s %-8s %-12s %-12s", "Ch", "Motor", "BDShot", "Serial", "BDShot Err", "Serial Err");

		for (int actuator_channel = 0; actuator_channel < DSHOT_MAXIMUM_CHANNELS; actuator_channel++) {
			int motor_index = motor_index_from_output(actuator_channel);

			if (motor_index < 0) {
				continue;
			}

			const char *bdshot_status = "-";
			const char *serial_status = "-";

			// Online masks are in motor order
			if (_bdshot_output_mask & (1 << actuator_channel)) {
				if (!up_bdshot_channel_capture_supported(actuator_channel)) {
					bdshot_status = "No DMA";

				} else {
					bdshot_status = (_bdshot_telem_online_mask & (1 << motor_index)) ? "Online" : "Offline";
				}
			}

			if (_serial_telemetry_enabled) {
				serial_status = (_serial_telem_online_mask & (1 << motor_index)) ? "Online" : "Offline";
			}

			// Error arrays are in actuator order
			PX4_INFO("  %-4d %-6d %-8s %-8s %-12lu %-12lu",
				 actuator_channel + 1,
				 motor_index + 1,
				 bdshot_status,
				 serial_status,
				 (unsigned long)_bdshot_telem_errors[motor_index],
				 (unsigned long)_serial_telem_errors[motor_index]);
		}

		if (_bdshot_output_mask && _bdshot_edt_enabled) {
			PX4_INFO("  EDT Confirmed Mask (motor order): 0x%02x", _bdshot_edt_confirmed_mask);
		}
	}

	// Bidirectional DShot hardware status
	if (_bdshot_output_mask) {
		print_spacer();
		PX4_INFO("Bidirectional DShot Hardware:");
		up_bdshot_status();
	}

	// Serial telemetry stats
	if (_serial_telemetry_enabled) {
		print_spacer();
		PX4_INFO("Serial Telemetry Stats:");
		_telemetry.printStatus();
	}

	print_spacer();
	PX4_INFO("Mixer Information:");
	_mixing_output.printStatus();

	print_spacer();
	PX4_INFO("Performance Counters:");
	perf_print_counter(_cycle_perf);

	if (_bdshot_output_mask) {
		perf_print_counter(_bdshot_recv_perf);
		perf_print_counter(_bdshot_error_perf);
	}

	if (_serial_telemetry_enabled) {
		perf_print_counter(_serial_telem_success_perf);
		perf_print_counter(_serial_telem_error_perf);
		perf_print_counter(_serial_telem_timeout_perf);
		perf_print_counter(_serial_telem_allsampled_perf);
	}

	print_spacer();

	return 0;
}

int DShot::custom_command(int argc, char *argv[])
{
	if (dragonfly_trtl::interlock().recoveryOwned()
	    || (dragonfly_trtl::interlock().directionRequired() && !dragonfly_trtl::interlock().ready())) {
		PX4_WARN("DShot command denied during turtle recovery/restoration");
		return PX4_ERROR;
	}

	const char *verb = argv[0];

	int myoptind = 1;
	bool swap_rxtx = false;
	const char *device_name = nullptr;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "xd:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'x':
			swap_rxtx = true;
			break;

		case 'd':
			device_name = myoptarg;
			break;

		default:
			return print_usage("unrecognized flag");
		}
	}

	if (!strcmp(verb, "telemetry")) {
		if (device_name) {
			// telemetry can be requested before the module is started
			strncpy(_serial_port_path, device_name, sizeof(_serial_port_path) - 1);
			_serial_port_path[sizeof(_serial_port_path) - 1] = '\0';
			_telemetry_swap_rxtx = swap_rxtx;
			_request_telemetry_init.store(true);
		}

		return 0;
	}

	if (!is_running(desc)) {
		int ret = DShot::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int DShot::task_spawn(int argc, char *argv[])
{
	int ret = PX4_ERROR;
	DShot *instance = new DShot();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

		PX4_INFO("Exiting");
		ret = PX4_OK;

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return ret;
}

int DShot::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This is the DShot output driver. It can be used as drop-in replacement
to use DShot as ESC communication protocol instead of PWM.

It supports:
- DShot150, DShot300, DShot600
- telemetry via separate UART and publishing as esc_status message

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dshot", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_COMMAND_DESCR("telemetry", "Enable Telemetry on a UART");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<device>", "UART device", false);
	PRINT_MODULE_USAGE_PARAM_FLAG('x', "Swap RX/TX pins", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int dshot_main(int argc, char *argv[])
{
	return ModuleBase::main(DShot::desc, argc, argv);
}
