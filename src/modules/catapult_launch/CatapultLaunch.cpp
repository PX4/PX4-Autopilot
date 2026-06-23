/****************************************************************************
 *
 * CatapultLaunch.cpp
 * Catapult launch automation for tube-launched fixed-wing aircraft.
 *
 * Approach B (PoC): independent module, active only while nav_state is
 * AUTO_TAKEOFF or AUTO_MISSION (with Takeoff waypoint).
 *
 * Tail servos (MAIN5/MAIN6) are controlled via MAV_CMD_DO_SET_ACTUATOR
 * targeting Peripheral_via_Actuator_Set1 and Set2, avoiding conflicts
 * with existing flight control surface outputs on actuator_servos.
 *
 * Motor (MAIN3) auto-start is implemented by publishing DO_SET_ACTUATOR
 * for Peripheral_via_Actuator_Set3 when CAT_MOT_AUTO is enabled.
 * The existing Takeoff throttle control is NOT overridden.
 *
 ****************************************************************************/

#include "CatapultLaunch.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/events.h>

using namespace time_literals;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

/* Run loop interval: 20ms = 50Hz */
static constexpr uint32_t SCHEDULE_INTERVAL_US = 20000;

/* PWM midpoint and half-range for normalization */
static constexpr float PWM_MID_US  = 1500.f;
static constexpr float PWM_HALF_US = 500.f;

/* FunctionActuatorSet only handles index==0 (param7=0).
 * All six slots live in one group, distinguished by param1-param6:
 *   param1 -> Peripheral_via_Actuator_Set1 -> MAIN5 (tail servo 1)
 *   param2 -> Peripheral_via_Actuator_Set2 -> MAIN6 (tail servo 2)
 *   param3 -> Peripheral_via_Actuator_Set3 -> MAIN3 (propulsion motor)
 * Send NAN for slots that should not change.
 */
static constexpr float ACTUATOR_SET_INDEX = 0.f;

CatapultLaunch::CatapultLaunch() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

bool CatapultLaunch::init()
{
	ScheduleOnInterval(SCHEDULE_INTERVAL_US);
	return true;
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  Main loop                                                                 */
/* ────────────────────────────────────────────────────────────────────────── */

void CatapultLaunch::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	updateSubscriptions();
	updateParams();

	const hrt_abstime now = hrt_absolute_time();

	/* Disabled globally: stay in IDLE, keep tail servos at lock position */
	if (!_param_cat_en.get()) {
		if (_state != State::IDLE) {
			transitionTo(State::IDLE);
		}

		return;
	}

	/* ── IDLE → ARMED_WAIT ── */
	if (_state == State::IDLE) {
		if (isArmed() && isInTakeoffNavState()) {
			transitionTo(State::ARMED_WAIT);
			/* Output lock PWM now that we are about to arm the sequence */
			commandTailServos(_param_cat_tail5_lock.get(), _param_cat_tail6_lock.get());
		}

		return;
	}

	/* ── Check abort conditions in any active state ── */
	if (_state != State::ABORTED && _state != State::COMPLETE) {
		if (checkAbortConditions()) {
			transitionTo(State::ABORTED);
			return;
		}
	}

	/* ── ABORTED → IDLE (after disarm / reset) ── */
	if (_state == State::ABORTED) {
		if (!isArmed() || !isInTakeoffNavState()) {
			transitionTo(State::IDLE);
		}

		return;
	}

	/* ── COMPLETE → IDLE ── */
	if (_state == State::COMPLETE) {
		if (!isArmed() || !isInTakeoffNavState()) {
			transitionTo(State::IDLE);
		}

		return;
	}

	/* ── Check manual failsafe in ARMED_WAIT and LAUNCHED ── */
	if (_state == State::ARMED_WAIT || _state == State::LAUNCHED) {
		if (checkManualFailsafe()) {
			transitionTo(State::MANUAL_FAILSAFE);
			return;
		}
	}

	/* ── MANUAL_FAILSAFE ── */
	if (_state == State::MANUAL_FAILSAFE) {
		/* Release tail if not already done */
		if (!_tail_released) {
			commandTailServos(_param_cat_tail5_rel.get(), _param_cat_tail6_rel.get());
			_tail_released = true;
			_tail_release_time = now;
			PX4_WARN("[CatapultLaunch] Manual failsafe: tail released");
		}

		/* Request Stabilized mode if configured */
		if (_param_cat_fs_to_stab.get()) {
			commandStabilizedMode();
		}

		/* Motor auto-start in failsafe only if explicitly enabled */
		if (_param_cat_fs_mot_en.get() && checkMotorAutoEnabled() && !_motor_started) {
			commandMotor(pwmToNormalized(_param_cat_mot_strt_pwm.get()));
			_motor_started = true;
			_motor_ramp_start_time = now;
			PX4_INFO("[CatapultLaunch] Manual failsafe: motor start commanded");
		}

		transitionTo(State::COMPLETE);
		return;
	}

	/* ── ARMED_WAIT: wait for launch detection ── */
	if (_state == State::ARMED_WAIT) {
		if (_param_cat_trig_mode.get() == 1) {
			/* IMU acceleration trigger */
			const float acc_g = getSelectedAccelG();
			const float threshold_g = _param_cat_acc_thr_g.get();

			if (acc_g >= threshold_g) {
				/* Accumulate hold time */
				if (_mot_rc_active_since == 0) {
					_mot_rc_active_since = now; /* reuse as accel hold timer */
				}

				const uint32_t hold_ms = (uint32_t)((now - _mot_rc_active_since) / 1000ULL);

				if (hold_ms >= (uint32_t)_param_cat_acc_hold_ms.get()) {
					_t0 = now;
					_mot_rc_active_since = 0;
					transitionTo(State::LAUNCHED);
					PX4_INFO("[CatapultLaunch] Launch detected: %.2fG (T0 set)", (double)acc_g);
				}

			} else {
				_mot_rc_active_since = 0; /* reset hold timer on drop below threshold */
			}
		}

		return;
	}

	/* ── LAUNCHED: wait for tail delay and motor delay ── */
	if (_state == State::LAUNCHED) {
		const float elapsed_s = (float)(now - _t0) / 1e6f;

		/* Tail release: execute first (even if t == t') */
		if (!_tail_released && elapsed_s >= _param_cat_tail_dly.get()) {
			commandTailServos(_param_cat_tail5_rel.get(), _param_cat_tail6_rel.get());
			_tail_released = true;
			_tail_release_time = now;
			transitionTo(State::TAIL_RELEASED);
			PX4_INFO("[CatapultLaunch] Tail released at T0+%.2fs", (double)elapsed_s);
		}

		return;
	}

	/* ── TAIL_RELEASED: wait for motor delay ── */
	if (_state == State::TAIL_RELEASED) {
		const float elapsed_s = (float)(now - _t0) / 1e6f;

		const bool motor_delay_met = elapsed_s >= _param_cat_mot_dly.get();
		const bool tail_req_met    = !_param_cat_mot_req_tail.get() || _tail_released;
		const bool auto_enabled    = checkMotorAutoEnabled();

		if (motor_delay_met && tail_req_met && auto_enabled) {
			_motor_ramp_start_pwm = (float)(_param_cat_mot_stop_pwm.get() > 0
						       ? _param_cat_mot_stop_pwm.get() : 1000);
			_motor_ramp_start_time = now;
			commandMotor(pwmToNormalized((int)_motor_ramp_start_pwm));
			_motor_started = true;
			transitionTo(State::MOTOR_STARTED);
			PX4_INFO("[CatapultLaunch] Motor start initiated at T0+%.2fs", (double)elapsed_s);

		} else if (motor_delay_met && !auto_enabled) {
			/* Motor auto disabled: sequence complete */
			transitionTo(State::COMPLETE);
		}

		return;
	}

	/* ── MOTOR_STARTED: ramp motor and transition to COMPLETE ── */
	if (_state == State::MOTOR_STARTED) {
		const float ramp_value = motorRampValue(now);
		commandMotor(ramp_value);

		const float ramp_done = pwmToNormalized(_param_cat_mot_strt_pwm.get());

		if (ramp_value >= ramp_done) {
			transitionTo(State::COMPLETE);
			PX4_INFO("[CatapultLaunch] Motor ramp complete");
		}

		return;
	}
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  Helpers                                                                    */
/* ────────────────────────────────────────────────────────────────────────── */

void CatapultLaunch::updateSubscriptions()
{
	_vehicle_status_sub.update(&_vehicle_status);
	_vehicle_local_position_sub.update(&_local_pos);
	_vehicle_attitude_sub.update(&_vehicle_attitude);
	_rc_channels_sub.update(&_rc_channels);
	_vehicle_land_detected_sub.update(&_land_detected);
}

void CatapultLaunch::transitionTo(State new_state)
{
	if (new_state == _state) {
		return;
	}

	PX4_DEBUG("[CatapultLaunch] state %d -> %d", (int)_state, (int)new_state);

	/* On abort: maintain tail servo position if already released */
	if (new_state == State::ABORTED) {
		if (_tail_released) {
			commandTailServos(_param_cat_tail5_rel.get(), _param_cat_tail6_rel.get());

		} else {
			commandTailServos(_param_cat_tail5_lock.get(), _param_cat_tail6_lock.get());
		}
	}

	/* On reset to IDLE: release all state */
	if (new_state == State::IDLE) {
		_t0                  = 0;
		_tail_release_time   = 0;
		_tail_released       = false;
		_motor_started       = false;
		_mot_rc_active_since = 0;
		_fs_rc_active_since  = 0;
		_motor_ramp_start_time = 0;
	}

	_state = new_state;
	_state_entry_time = hrt_absolute_time();
}

bool CatapultLaunch::isInTakeoffNavState() const
{
	return _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF
	       || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
}

bool CatapultLaunch::isArmed() const
{
	return _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
}

bool CatapultLaunch::isKilled() const
{
	/* Kill switch triggers a disarm with KILL_SWITCH reason — covered by isArmed() */
	return false;
}

bool CatapultLaunch::isFailsafe() const
{
	return _vehicle_status.failsafe;
}

bool CatapultLaunch::checkAbortConditions()
{
	if (!isArmed())           { PX4_WARN("[CatapultLaunch] Abort: disarmed"); return true; }
	if (isKilled())           { PX4_WARN("[CatapultLaunch] Abort: kill switch"); return true; }
	if (!_param_cat_en.get()) { PX4_WARN("[CatapultLaunch] Abort: CAT_EN=0"); return true; }

	if (_param_cat_abort_fs.get() && isFailsafe()) {
		PX4_WARN("[CatapultLaunch] Abort: PX4 failsafe");
		return true;
	}

	return false;
}

bool CatapultLaunch::checkManualFailsafe()
{
	const int ch = _param_cat_fs_rc_ch.get();

	if (ch < 1 || ch > 18) {
		return false; /* -1 = disabled */
	}

	const float val = _rc_channels.channels[ch - 1];
	const bool active = PX4_ISFINITE(val) && val >= _param_cat_fs_rc_thr.get();

	const hrt_abstime now = hrt_absolute_time();

	if (active) {
		if (_fs_rc_active_since == 0) {
			_fs_rc_active_since = now;
		}

		const uint32_t hold_ms = (uint32_t)((now - _fs_rc_active_since) / 1000ULL);

		if (hold_ms >= (uint32_t)_param_cat_fs_rc_hold.get()) {
			return true;
		}

	} else {
		_fs_rc_active_since = 0;
	}

	return false;
}

bool CatapultLaunch::checkMotorAutoEnabled()
{
	if (!_param_cat_mot_auto.get()) {
		return false;
	}

	/* RC gate */
	const int rc_ch = _param_cat_mot_rc_ch.get();

	if (rc_ch >= 1 && rc_ch <= 18) {
		const float val = _rc_channels.channels[rc_ch - 1];
		const bool active = PX4_ISFINITE(val) && val >= _param_cat_mot_rc_thr.get();
		const hrt_abstime now = hrt_absolute_time();

		if (active) {
			if (_mot_rc_active_since == 0) {
				_mot_rc_active_since = now;
			}

			const uint32_t hold_ms = (uint32_t)((now - _mot_rc_active_since) / 1000ULL);

			if (hold_ms < (uint32_t)_param_cat_mot_rc_hold.get()) {
				return false; /* hold time not yet met */
			}

		} else {
			_mot_rc_active_since = 0;
			return false; /* RC gate not satisfied */
		}
	}

	/* MAVLink gate */
	if (_param_cat_mav_en.get() && !_mavlink_auto_start_allowed) {
		return false;
	}

	return true;
}

float CatapultLaunch::getSelectedAccelG() const
{
	/* Rotate NED acceleration into body frame using vehicle_attitude quaternion (body→NED) */
	const matrix::Dcmf R(matrix::Quatf(_vehicle_attitude.q));
	const Vector3f accel_body = R.transpose() * Vector3f{_local_pos.ax, _local_pos.ay, _local_pos.az};

	float selected = 0.f;

	switch (_param_cat_acc_axis.get()) {
	case 0: selected = accel_body.norm();  break; /* magnitude */
	case 1: selected = accel_body(0);      break; /* Body X */
	case 2: selected = accel_body(1);      break; /* Body Y */
	case 3: selected = accel_body(2);      break; /* Body Z */

	default: selected = accel_body(0);     break;
	}

	return fabsf(selected) / 9.80665f;
}

float CatapultLaunch::pwmToNormalized(int pwm_us) const
{
	return ((float)pwm_us - PWM_MID_US) / PWM_HALF_US;
}

void CatapultLaunch::commandTailServos(int pwm5_us, int pwm6_us)
{
	vehicle_command_s cmd{};
	cmd.timestamp        = hrt_absolute_time();
	cmd.command          = vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR;
	cmd.param1           = pwmToNormalized(pwm5_us); /* Set1 -> MAIN5 */
	cmd.param2           = pwmToNormalized(pwm6_us); /* Set2 -> MAIN6 */
	cmd.param3           = NAN; /* motor slot: do not change */
	cmd.param4           = NAN;
	cmd.param5           = NAN;
	cmd.param6           = NAN;
	cmd.param7           = ACTUATOR_SET_INDEX;
	cmd.target_system    = 1;
	cmd.target_component = 1;
	cmd.source_system    = 1;
	cmd.source_component = 1;
	cmd.from_external    = false;
	_vehicle_command_pub.publish(cmd);
}

void CatapultLaunch::commandMotor(float normalized_value)
{
	vehicle_command_s cmd{};
	cmd.timestamp        = hrt_absolute_time();
	cmd.command          = vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR;
	cmd.param1           = NAN; /* tail servo slots: do not change */
	cmd.param2           = NAN;
	cmd.param3           = normalized_value; /* Set3 -> MAIN3 */
	cmd.param4           = NAN;
	cmd.param5           = NAN;
	cmd.param6           = NAN;
	cmd.param7           = ACTUATOR_SET_INDEX;
	cmd.target_system    = 1;
	cmd.target_component = 1;
	cmd.source_system    = 1;
	cmd.source_component = 1;
	cmd.from_external    = false;
	_vehicle_command_pub.publish(cmd);
}

void CatapultLaunch::commandStabilizedMode()
{
	vehicle_command_s cmd{};
	cmd.timestamp     = hrt_absolute_time();
	cmd.command       = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	cmd.param1        = 1.f;  /* base mode: custom */
	cmd.param2        = (float)vehicle_status_s::NAVIGATION_STATE_STAB;
	cmd.target_system    = 1;
	cmd.target_component = 1;
	cmd.source_system    = 1;
	cmd.source_component = 1;
	cmd.from_external    = false;
	_vehicle_command_pub.publish(cmd);
	PX4_INFO("[CatapultLaunch] Stabilized mode transition requested");
}

float CatapultLaunch::motorRampValue(hrt_abstime now) const
{
	const float ramp_s = _param_cat_mot_ramp.get();

	if (ramp_s < 1e-3f || _motor_ramp_start_time == 0) {
		return pwmToNormalized(_param_cat_mot_strt_pwm.get());
	}

	const float elapsed = (float)(now - _motor_ramp_start_time) / 1e6f;
	const float t       = math::constrain(elapsed / ramp_s, 0.f, 1.f);
	const float start   = pwmToNormalized((int)_motor_ramp_start_pwm);
	const float target  = pwmToNormalized(_param_cat_mot_strt_pwm.get());
	return start + t * (target - start);
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  Module boilerplate                                                         */
/* ────────────────────────────────────────────────────────────────────────── */

int CatapultLaunch::task_spawn(int argc, char *argv[])
{
	CatapultLaunch *instance = new CatapultLaunch();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int CatapultLaunch::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CatapultLaunch::print_status()
{
	static const char *const state_names[] = {
		"IDLE", "ARMED_WAIT", "LAUNCHED", "TAIL_RELEASED",
		"MOTOR_STARTED", "MANUAL_FAILSAFE", "ABORTED", "COMPLETE"
	};

	PX4_INFO("state: %s", state_names[(int)_state]);
	PX4_INFO("CAT_EN: %d  armed: %d  takeoff_nav: %d  landed: %d  failsafe: %d",
		 (int)_param_cat_en.get(), (int)isArmed(), (int)isInTakeoffNavState(),
		 (int)_land_detected.landed, (int)isFailsafe());
	PX4_INFO("nav_state: %d  accel(sel): %.2fG  tail_released: %d  motor_started: %d",
		 (int)_vehicle_status.nav_state, (double)getSelectedAccelG(),
		 (int)_tail_released, (int)_motor_started);

	return 0;
}

int CatapultLaunch::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Catapult launch automation for tube-launched fixed-wing aircraft (PoC module).

Controls tail lock servos (MAIN5/MAIN6) and propulsion motor (MAIN3) via
MAV_CMD_DO_SET_ACTUATOR during Takeoff / Mission Takeoff nav states.

Requires CAT_EN=1 to activate.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("catapult_launch", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int catapult_launch_main(int argc, char *argv[])
{
	return CatapultLaunch::main(argc, argv);
}
