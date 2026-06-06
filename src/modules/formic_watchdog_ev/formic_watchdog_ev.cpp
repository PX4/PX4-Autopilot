#include "formic_watchdog_ev.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

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

	handel_pos_req_user_intention(); // check if the commander requested to be in a position mode (e.g. by moving the AUX switch or by requesting a position mode while EV was not healthy, which triggers the position request as a fallback)

	update_baro_deriv();


	vehicle_odometry_s odometry{};

	if (_odometry_sub_formic.update(&odometry)) {
		_last_ev_timestamp = hrt_absolute_time();

		if (_first_ev_timestamp == 0) {
			_first_ev_timestamp = _last_ev_timestamp;
		}

		check_EV_z_velocity(odometry); // try get the ev vilo data
		if (multy_sensor_z_velocity_check()) {
			_formic_state.multy_sensor_z_velocity_error = true;
			_formic_state.error_find = true;
		}

		const hrt_abstime settle_us = (hrt_abstime)_param_formic_wdev_init.get() * 1_s;
		if (hrt_elapsed_time(&_first_ev_timestamp) < settle_us) {
			accumulate_quality(odometry); // still settling: build the average, don't forward yet
			accumulate_3d_velocity(odometry); // also accumulate the EV z-velocity for the init average (mirroring the quality)
		}
		else if (!_formic_state.error_find && _formic_state.user_aux_control) { // only forward the data if the aux switch is active (if configured) and if no error has been found, otherwise we keep publishing the state machine with the error flag set but we don't forward the possibly bad data to the rest of the system
			if (!_init_check_done) {
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
			resetcounter(odometry); // (relies on _formic_state.ev_data_arrived being fresh)
			copy_odometry_msg(odometry);
		}
	}

	no_EvData(); // updates _formic_state.ev_data_arrived every cycle

	update_pipeline_status(); // decide _formic_state.status from the flags set above

	_formic_state.timestamp = hrt_absolute_time();
	_formic_state_machine_pub.publish(_formic_state);
}

// -----------------------------------------------------------------------------
// Data handling helpers
// -----------------------------------------------------------------------------

void FormicWatchdogEv::copy_odometry_msg(vehicle_odometry_s &odometry)
{
	odometry.reset_counter = _formic_state.reset_counter;
	_odometry_pub.publish(odometry);
	if (_forwarding_start_time == 0) {
		_forwarding_start_time = hrt_absolute_time();
	}

	if (hrt_elapsed_time(&_forwarding_start_time) >= 2_s) {
		// maby check the pos is alighend if not reset counter ??
		_formic_state.status = (uint8_t)pipline_status::VALID_POS;
	}

}

void FormicWatchdogEv::update_pipeline_status()

{

	if(!_pos_requested){
		_formic_state.status = (uint8_t)pipline_status::MANUAL;
		return;
	}

	if (_formic_state.error_find) {
		_formic_state.status = (uint8_t)pipline_status::EV_ERROR;
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

	// Init checks passed, but stay in INIT during the 2 s post-forwarding settle
	// window. copy_odometry_msg() promotes the status to VALID_POS once the window
	// elapses (and only on cycles where EV is actually forwarded).
	if (_forwarding_start_time == 0 || hrt_elapsed_time(&_forwarding_start_time) < 2_s) {
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


void FormicWatchdogEv::handel_pos_req_user_intention()
// handel the pos_req that came from the commander 
{
	if (_formic_pos_req.updated()) {
		formic_pos_req_s pos_req{};
		_formic_pos_req.copy(&pos_req);
		_pos_requested = pos_req.pos_req;
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



/*
applide a low pass the the baro div that i have (at the vehicle_air_Data )
*/
void FormicWatchdogEv::update_baro_deriv()
{
	vehicle_air_data_s air_data{};
	if (_air_data_sub.update(&air_data)) {
		_baro_filtered = _baro_deriv_filter.apply(air_data.baro_alt_meter_derivative);
		_formic_state.baro_data = _baro_filtered;
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
bool FormicWatchdogEv::check_EV_aid_src_heading(float vio_yaw, float estimator_yaw)
{
	estimator_aid_source1d_s ev_yaw{};

	// No fused EV yaw aid this cycle -> alignment is unknown. Report "data not valid"
	// so the caller keeps the current phase instead of treating it as aligned.
	if (!_estimator_aid_src_heading_sub.copy(&ev_yaw)) {
		_formic_state.heading_alligned_with_ev = false;
		return false;
	}
	if (!ev_yaw.fused || ev_yaw.innovation_rejected) {
		_formic_state.heading_alligned_with_ev = false;
		return false;
	}

	const float d_yaw = matrix::wrap_pi(vio_yaw - estimator_yaw);
	const float dyaw_thr = _param_formic_wdev_dyaw.get();
	const bool aligned = fabsf(d_yaw) <= dyaw_thr;
	_formic_state.heading_alligned_with_ev = aligned;

	if (!at_reset_counter && fabsf(d_yaw) > dyaw_thr * 2.f) {
		PX4_INFO("Large heading difference detected (%.2f deg). Please check the EV yaw configuration and the local heading estimate.", (double)math::degrees(d_yaw));
		_formic_state.error_find = true;
	}
	return true; // had fused EV yaw aid data this cycle
}


// handel a pos reset counter // 
bool FormicWatchdogEv::handel_pos_reset(const float vio_pos[2], const float estimator_pos[2])
{
	estimator_aid_source2d_s ev_pos{};

	if (!_estimator_aid_src_pos_sub.copy(&ev_pos)) {
		_formic_state.pos_alligned_with_ev = false;
		return false;
	}

	if (!ev_pos.fused || ev_pos.innovation_rejected) {
		_formic_state.pos_alligned_with_ev = false;
		return false;
	}

	matrix::Vector2f d_pos;
	for (int i = 0; i < 2; i++) {
		d_pos(i) = fabsf(vio_pos[i] - estimator_pos[i]);
	}

	const bool aligned = d_pos.norm() <= _param_formic_wdev_dpos.get();
	_formic_state.pos_alligned_with_ev = aligned;

	if (!aligned) {
		PX4_INFO("EV pos: [%.2f, %.2f], local pos: [%.2f, %.2f], d_pos: [%.2f, %.2f]", (double)vio_pos[0], (double)vio_pos[1], (double)estimator_pos[0], (double)estimator_pos[1], (double)d_pos(0), (double)d_pos(1));
	}

	return true; // had fused EV position aid data this cycle
}


void FormicWatchdogEv::resetcounter(vehicle_odometry_s &odometry)
{

	
	vehicle_odometry_s esti_odom{};

	if (!_estimtor_odometry_sub.copy(&esti_odom)) {
		return;
	}

	const float raw_yaw      = get_yaw_from_quat(odometry);  // EV (VIO) yaw
	const float estimtor_yaw = get_yaw_from_quat(esti_odom); // EKF yaw

	// New session (EV just (re)arrived): clear the counter and re-arm the reset phase.
	if (!_formic_state.ev_data_arrived) {
		_formic_state.reset_counter = 0;
		_last_reset_time = 0;
		at_reset_counter = true;
		return;
	}

	// Evaluate alignment. Each helper sets the matching *_alligned_with_ev flag and
	// returns false when it had NO fused EV aid data this cycle (alignment unknown).
	const bool yaw_data_valid = check_EV_aid_src_heading(raw_yaw, estimtor_yaw);
	const bool pos_data_valid = handel_pos_reset(odometry.position, esti_odom.position);

	if (!at_reset_counter) {
		return;
	}
	if (!yaw_data_valid || !pos_data_valid) {
		return;
	}

	const bool aligned = _formic_state.heading_alligned_with_ev
			     && _formic_state.pos_alligned_with_ev;

	// Both heading AND position aligned -> leave the reset phase for the rest of this
	// session and switch to error checking.
	if (aligned) {
		at_reset_counter = false;
		return;
	}

	// Still misaligned. Throttle: only one reset every 2 seconds.
	if (_last_reset_time != 0 && hrt_elapsed_time(&_last_reset_time) < 2_s) {
		return;
	}

	_formic_state.reset_counter++;
	_last_reset_time = hrt_absolute_time();
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
		_formic_state.reset_counter = 0; // reset the EV reset counter at dropout, so the next session starts from zero
		_last_reset_time = 0; // clear the 3 s reset throttle timer
		at_reset_counter = true; // re-arm the reset phase: next session resets until the heading aligns, then checks for errors
		_forwarding_start_time = 0; // restart the 1 s post-forwarding settle window next session

	}
	else {
		// Fresh EV data has arrived within the timeout window.
		_formic_state.ev_data_arrived = true;
	}
}




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
