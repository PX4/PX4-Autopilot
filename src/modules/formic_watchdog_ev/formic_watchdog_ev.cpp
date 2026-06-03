#include "formic_watchdog_ev.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

// -----------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------

FormicWatchdogEv::FormicWatchdogEv() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	parameters_update(true);

}

bool FormicWatchdogEv::init()
{
	ScheduleOnInterval(33_ms); // Run at ~30 Hz
	parameters_update();
	_ev_pos_deriv_filter.setCutoffFreq(30.0f, 1.0f);
	return true;
}

void FormicWatchdogEv::parameters_update(bool force)
{
	if (force || _parameter_update_sub.updated()) {
		parameter_update_s param_update{};
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	_ev_vel_enabled = (_param_ekf2_ev_ctrl.get() & (1 << 2)) != 0;
	_aux_switch = static_cast<user_aux_switch_t>(_param_formic_wdev_aux.get());

	// Start with VIO allowed. With no AUX channel configured this stays true
	// (VIO active all the time); otherwise handel_user_aux_control() follows
	// the selected switch.
	_formic_state.user_aux_control = true;
}

void FormicWatchdogEv::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	parameters_update();

	if (_param_formic_wdev_en.get() == 0) {
		return;
	}
	handel_user_aux_control();

	if (_formic_pos_req.updated()) {
		formic_pos_req_s pos_req{};
		_formic_pos_req.copy(&pos_req);
		_pos_requested = pos_req.pos_req;
		// PX4_INFO("Position request: %s", _pos_requested ? "true" : "false");
	}


	// Forward odometry from formic_odom to the visual odometry topic
	vehicle_odometry_s odometry{};

	if (_odometry_sub_formic.update(&odometry)) {
		_last_ev_timestamp = hrt_absolute_time();

		// Mark the start of this EV session. no_EvData() resets it to 0 on a
		// dropout, so a stop/re-arrive restarts the settle window every session.
		if (_first_ev_timestamp == 0) {
			_first_ev_timestamp = _last_ev_timestamp;
		}

		check_EV_z_velocity(odometry); // try get the ev vilo data
		if (multy_sensor_z_velocity_check()) {
			_formic_state.multy_sensor_z_velocity_error = true;
			_formic_state.error_find = true;
		}

		// Only forward EV odometry once the stream has been arriving for the
		// configured settle time (FORMIC_WDEV_INIT, in seconds). During the
		// settle window we only build the quality average (the data is not
		// forwarded); the final average is computed exactly once, right before3
		// the first sample is fused.
		const hrt_abstime settle_us = (hrt_abstime)_param_formic_wdev_init.get() * 1_s;
		if (hrt_elapsed_time(&_first_ev_timestamp) < settle_us) {
			accumulate_quality(odometry); // still settling: build the average, don't forward yet
			accumulate_3d_velocity(odometry); // also accumulate the EV z-velocity for the init average (mirroring the quality)
		}
		else if (!_formic_state.error_find && _formic_state.user_aux_control) { // only forward the data if the aux switch is active (if configured) and if no error has been found, otherwise we keep publishing the state machine with the error flag set but we don't forward the possibly bad data to the rest of the system
			if (!_init_check_done) {
				// settle window just elapsed: compute the init average once, before the first fuse
				_quality_average_init = finalize_quality_average();
				_vel_average_init     = finalize_3d_velocity_average();
				_init_check_done = true;
				_formic_state.quality_init_check_fail = (_quality_average_init < _param_ekf2_ev_qmin.get());
				_formic_state.vel_3d_init_check_fail = (_vel_average_init > _param_formic_wdev_vini.get()); // if the average EV all_vel during the settle window is very low, it's likely that the EV data is not good (e.g. bad EV fusion configuration, or EV not really moving which makes the quality metric less meaningful). In this case we also set the error flag and skip forwarding the data.


				if (_formic_state.quality_init_check_fail || _formic_state.vel_3d_init_check_fail) {
					_formic_state.error_find = true;
					PX4_WARN("EV data did not pass the init checks! quality average: %.2f, vel average: %.2f", (double)_quality_average_init, (double)_vel_average_init);
				}
			}
			/* The `raw_yaw` variable in the `check_EV_z_velocity` function is calculated by extracting
			the yaw angle from the quaternion orientation data in the `vehicle_odometry_s` structure.
			The yaw angle is obtained by converting the quaternion to Euler angles and then extracting
			the yaw component. This `raw_yaw` value is used for further processing or checks related to
			the yaw orientation of the vehicle. */
			float raw_yaw = get_yaw_from_quat(odometry); // try get the ev vilo data
			resetcounter_heading(raw_yaw); // (relies on _formic_state.ev_data_arrived being fresh)

			copy_odometry_msg(odometry);
		}
	}

	no_EvData(); // updates _formic_state.ev_data_arrived every cycle
	update_baro_deriv();

	update_pipeline_status(); // decide _formic_state.status from the flags set above

	_formic_state.timestamp = hrt_absolute_time();
	_formic_state_machine_pub.publish(_formic_state);
}

// -----------------------------------------------------------------------------
// Data handling helpers
// -----------------------------------------------------------------------------

void FormicWatchdogEv::copy_odometry_msg(vehicle_odometry_s &odometry)
{
	// Forward the EV odometry only while a position mode is requested. The
	// resulting pipeline state (VALID_POS vs MANUAL/etc.) is decided centrally
	// in update_pipeline_status(); this helper only handles the forwarding.

	odometry.reset_counter = _formic_state.heading_reset_counter;
	_odometry_pub.publish(odometry);

	// Mark when forwarding first started this session. Hold the status below
	// VALID_POS for 1 s after the first forwarded sample, so the EKF has time to
	// settle on the freshly-forwarded EV pose before we advertise it as a trusted
	// position source. During this window the status stays INIT (see
	// update_pipeline_status). Reset to 0 on EV dropout so each session re-waits.
	if (_forwarding_start_time == 0) {
		_forwarding_start_time = hrt_absolute_time();
	}

	if (hrt_elapsed_time(&_forwarding_start_time) >= 1_s) {
		_formic_state.status = (uint8_t)pipline_status::VALID_POS;
	}

}

// Single source of truth for the pipeline state. Runs once per cycle, after all
// the checks have set their flags. The status reports EV-pipeline health only;
// it is intentionally INDEPENDENT of the position request (_pos_requested) so
// the commander can read it to decide whether to enter a position mode without
// creating a feedback loop. _pos_requested only gates whether the odometry is
// forwarded to the EKF (see copy_odometry_msg). Evaluated in strict priority:
//   ERROR        - a latched fault was found this session (cleared on EV dropout)
//   MANUAL       - the user AUX gate is off (VIO disabled by the operator)
//   WAIT_TO_DATA - no fresh EV stream yet
//   INIT         - EV data arriving, still inside the settle window
//   VALID_POS    - past settle, no error: EV is a trusted position source
void FormicWatchdogEv::update_pipeline_status()



{

	if(!_pos_requested){
		_formic_state.status = (uint8_t)pipline_status::MANUAL;
		return;
	}

	if (_formic_state.error_find) {
		_formic_state.status = (uint8_t)pipline_status::ERROR;
		return;
	}

	if (!_formic_state.ev_data_arrived) {
		_formic_state.status = (uint8_t)pipline_status::WAIT_TO_DATA;
		return;
	}

	if (!_init_check_done) {
		_formic_state.status = (uint8_t)pipline_status::INIT;
		return;
	}

	// Init checks passed, but stay in INIT during the 1 s post-forwarding settle
	// window. copy_odometry_msg() promotes the status to VALID_POS once the window
	// elapses (and only on cycles where EV is actually forwarded).
	if (_forwarding_start_time == 0 || hrt_elapsed_time(&_forwarding_start_time) < 1_s) {
		_formic_state.status = (uint8_t)pipline_status::INIT;
		return;
	}

}



void FormicWatchdogEv::handel_user_aux_control()
{
	// No AUX channel configured -> VIO is active all the time, nothing to gate.
	if (_aux_switch == user_aux_switch_t::AUX_NONE) {
		return;
	}

	if (_manual_control_setpoint_sub.updated()) {
		manual_control_setpoint_s manual_control_setpoint{};

		if (_manual_control_setpoint_sub.copy(&manual_control_setpoint)) {
			if (_formic_state.user_aux_control) {
				// check the aux switch and set the error_find flag accordingly
				const bool aux_active = ((manual_control_setpoint.aux1 > 0.5f) && (_aux_switch == user_aux_switch_t::AUX1))
				                        || ((manual_control_setpoint.aux2 > 0.5f) && (_aux_switch == user_aux_switch_t::AUX2))
				                        || ((manual_control_setpoint.aux3 > 0.5f) && (_aux_switch == user_aux_switch_t::AUX3))
				                        || ((manual_control_setpoint.aux4 > 0.5f) && (_aux_switch == user_aux_switch_t::AUX4))
				                        || ((manual_control_setpoint.aux5 > 0.5f) && (_aux_switch == user_aux_switch_t::AUX5))
				                        || ((manual_control_setpoint.aux6 > 0.5f) && (_aux_switch == user_aux_switch_t::AUX6));

				_formic_state.user_aux_control = aux_active; // only check the aux switch until it's released once, then ignore it for the rest of the session (until next restart or until the EV stream drops and restarts, which resets all the state)
				// PX4_INFO("AUX switch %s -> user_aux_control: %d", aux_active ? "active" : "inactive", _formic_state.user_aux_control);
			}
		}
	}
}


/*
get the raw ev 
check if the rkf2_ev_ctrl have ev_velocity - if have use the velo 
if not have do derivative of the ev position and use the deriv as the ev velocity
*/
void FormicWatchdogEv::check_EV_z_velocity(vehicle_odometry_s &odometry)
{
	if (_ev_vel_enabled) {
		_ev_vel_data_z = odometry.velocity[2];

	} else {
		const hrt_abstime now = odometry.timestamp;

		if (_prev_ev_pos_time != 0) {
			const float dt = (now - _prev_ev_pos_time) * 1e-6f;

			if (dt > 0.001f) {
				const float deriv = (odometry.position[2] - _prev_ev_z) / dt;
				_ev_vel_data_z = _ev_pos_deriv_filter.update(deriv);
			}
		}

		_prev_ev_z        = odometry.position[2];
		_prev_ev_pos_time = now;
	}
}

void FormicWatchdogEv::update_baro_deriv()
{
	vehicle_air_data_s air_data{};

	if (_air_data_sub.update(&air_data)) {
		_baro_filtered = _baro_deriv_filter.apply(air_data.baro_alt_meter_derivative);
	}
}


// Accumulate one quality sample during the settle window (data is not forwarded yet).
void FormicWatchdogEv::accumulate_quality(const vehicle_odometry_s &odometry)
{
	_quality_count += 1;
	_quality_sum   += odometry.quality;
}

// Mean quality over the samples collected during the settle window.
float FormicWatchdogEv::finalize_quality_average() const
{
	if (_quality_count == 0) {
		return 0.0f; // no samples accumulated during the window
	}

	return _quality_sum / _quality_count;
}

// Accumulate one EV 3D speed sample during the settle window. Uses the full
// odometry velocity vector (||vx,vy,vz||). Samples with a non-finite velocity
// (e.g. EV velocity fusion disabled) are skipped.
void FormicWatchdogEv::accumulate_3d_velocity(const vehicle_odometry_s &odometry)
{
	const matrix::Vector3f vel(odometry.velocity);

	if (!vel.isAllFinite()) {
		return ; // velocity not available skip this sample
	}

	_vel_count += 1;
	_vel_sum   += vel.norm();
}

// Mean EV 3D speed over the samples collected during the settle window.
float FormicWatchdogEv::finalize_3d_velocity_average() const
{
	if (_vel_count == 0) {
		return 0.0f; // no samples accumulated during the window
	}

	return _vel_sum / _vel_count;
}



// -----------------------------------------------------------------------------
// Checks
// -----------------------------------------------------------------------------

/**
 * check the dz at multy sensor
 */
bool FormicWatchdogEv::multy_sensor_z_velocity_check()
{
	// check all the sensor z velocity
	float dv = fabsf(_ev_vel_data_z - _baro_filtered);
	bool ret = (dv > _param_formic_wdev_dvel.get());
	if (ret){
		PX4_INFO("EV z velocity: %.2f, baro deriv: %.2f, dv: %.2f", (double)_ev_vel_data_z, (double)_baro_filtered, (double)dv);
	}
	return (ret);
}


float FormicWatchdogEv::get_yaw_from_quat(const vehicle_odometry_s &odometry)
{
	const matrix::Quatf quat(odometry.q);
	if (!quat.isAllFinite() || quat.length() < 0.9f) {
		return NAN;
	}
	return matrix::Eulerf(quat).psi();
}

/*
checking if the heading is and the distance at the heading between the EV_rad heading and the current heading at the current session
*/
bool FormicWatchdogEv::check_EV_aid_src_heading(float raw_yaw)
{
	estimator_aid_source1d_s ev_yaw{};

	if (!_estimator_aid_src_heading_sub.copy(&ev_yaw)) {
		return false;
	}

	// EV yaw must be actively fused (and not rejected) to be trusted.
	if (!ev_yaw.fused || ev_yaw.innovation_rejected) {
		return false;
	}

	vehicle_local_position_s local_pos{};

	if (!_local_position_sub.copy(&local_pos)) {
		return false;
	}

	// Heading difference between the local estimate and the EV yaw observation (radians).
	const float d_yaw = matrix::wrap_pi(local_pos.heading - ev_yaw.observation); /// need fix

	// Only count a heading reset when the difference exceeds the configured
	// threshold (FORMIC_WDEV_DYAW). When d_yaw is small the heading is already
	// aligned with the EV and no reset is needed for this session.
	bool ret = fabsf(d_yaw) > _param_formic_wdev_dyaw.get();
	if (ret){
		// print ony when need a reset
		PX4_INFO("EV yaw: %.2f, local heading: %.2f, d_yaw: %.2f", (double)ev_yaw.observation, (double)local_pos.heading, (double)d_yaw);
	}

	// Error check is only armed once we have LEFT the reset phase (at_reset_counter
	// == false, i.e. the heading was aligned at least once this session). During the
	// reset phase a large d_yaw is expected (we are still aligning) and must not be
	// flagged as an error.
	if (!at_reset_counter && fabsf(d_yaw) > _param_formic_wdev_dyaw.get()*2) {
		PX4_WARN("Large heading difference detected (%.2f deg). Please check the EV yaw configuration and the local heading estimate.", (double)math::degrees(d_yaw));
		_formic_state.error_find = true;
	}
	return ret;
}

/*
 * Heading reset counter. Each EV session runs in two phases:
 *
 *   RESET phase  (at_reset_counter == true): while the heading is NOT yet aligned
 *                with the EV (d_yaw large), keep incrementing the reset counter to
 *                force the EKF to re-align to the EV heading. Throttled to one
 *                increment every 3 s. A large d_yaw here is expected, not an error.
 *   CHECK phase  (at_reset_counter == false): the heading was aligned at least once,
 *                so we stop resetting and start watching for errors (a large d_yaw
 *                now means the EV/heading disagree -> error_find, see
 *                check_EV_aid_src_heading).
 *
 * On EV dropout the counter is cleared and the RESET phase is re-armed for the next
 * session (see no_EvData / the ev_data_arrived guard below).
 */
void FormicWatchdogEv::resetcounter_heading(float raw_yaw)
{
	// EV session stopped -> clear the counter, the throttle timer, and re-arm the
	// reset phase so the next session starts aligning from zero.
	if (!_formic_state.ev_data_arrived) {
		_formic_state.heading_reset_counter = 0;
		_last_heading_reset_time = 0;
		at_reset_counter = true;
		return;
	}

	const bool needs_reset = check_EV_aid_src_heading(raw_yaw); // true while d_yaw > FORMIC_WDEV_DYAW

	// CHECK phase: reset phase already finished this session. Keep evaluating the
	// heading (the error gate lives inside check_EV_aid_src_heading) but never reset.
	if (!at_reset_counter) {
		_formic_state.heading_alligned_with_ev = !needs_reset;
		return;
	}

	// RESET phase: heading is now aligned -> leave the reset phase for the rest of
	// this session and switch to error checking.
	if (!needs_reset) {
		at_reset_counter = false;
		_formic_state.heading_alligned_with_ev = true;
		return;
	}

	// Heading still misaligned. Throttle: only one reset every 3 seconds.
	if (_last_heading_reset_time != 0 && hrt_elapsed_time(&_last_heading_reset_time) < 3_s) {
		return;
	}

	// Force another EKF heading reset to the EV.
	_formic_state.heading_reset_counter++;
	_formic_state.heading_alligned_with_ev = false;
	_last_heading_reset_time = hrt_absolute_time();
}



void FormicWatchdogEv::no_EvData()
{
	/* Determine if there has been a dropout in the EV (Extended Visual) data stream. */
	if ((_last_ev_timestamp == 0) ||
	    ((hrt_absolute_time() - _last_ev_timestamp) > 500_ms)) {
		_formic_state.ev_data_arrived = false;
		_formic_state.error_find = false; // dropout = end of session: clear latched error so the next session may use EV
		// No EV data: fall back to WAIT_TO_DATA if a position mode is still requested, otherwise MANUAL.
		_first_ev_timestamp = 0; // EV dropped out: restart the settle window on re-arrival
		_quality_count = 0;    // reset quality average accumulator
		_quality_sum   = 0.0f;
		_quality_average_init = 0.0f; // reset computed init average
		_init_check_done = false;   // recompute the init average next session
		_vel_count = 0;        // reset 3D speed average accumulator
		_vel_sum   = 0.0f;
		_vel_average_init = 0.0f; // reset computed init 3D speed average
		_formic_state.quality_init_check_fail = false; // clear latched init-check flags for the next session
		_formic_state.vel_3d_init_check_fail  = false;
		_formic_state.heading_reset_counter = 0; // reset the heading reset counter at dropout, so the next session starts from zero
		_last_heading_reset_time = 0; // clear the 3 s reset throttle timer
		at_reset_counter = true; // re-arm the reset phase: next session resets until the heading aligns, then checks for errors
		_forwarding_start_time = 0; // restart the 1 s post-forwarding settle window next session

	}
	else {
		// Fresh EV data has arrived within the timeout window.
		_formic_state.ev_data_arrived = true;
	}
}

// Return true when the EKF is actively fusing the EV pose: the EV position and
// yaw aid sources (and velocity, when EV velocity fusion is enabled) must each
// have fused recently and not be innovation-rejected. This confirms the EKF is
// really consuming the EV data this module forwards, not just that data arrived.

// -----------------------------------------------------------------------------
// Module boilerplate
// -----------------------------------------------------------------------------

int FormicWatchdogEv::task_spawn(int argc, char *argv[])
{
	FormicWatchdogEv *instance = new FormicWatchdogEv();

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init()) {
		PX4_INFO("started");
		return PX4_OK;
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

FormicWatchdogEv *FormicWatchdogEv::instantiate(int argc, char *argv[])
{
	return new FormicWatchdogEv();
}

int FormicWatchdogEv::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FormicWatchdogEv::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Minimal skeleton module ready for custom logic.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("formic_watchdog_ev", "module");
	PRINT_MODULE_USAGE_COMMAND("start");
	return PX4_OK;
}

extern "C" __EXPORT int formic_watchdog_ev_main(int argc, char *argv[])
{
	return FormicWatchdogEv::main(argc, argv);
}
