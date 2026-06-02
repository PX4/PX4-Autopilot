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
		_formic_state.multy_sensor_z_velocity_error = multy_sensor_z_velocity_check();
		if (_formic_state.multy_sensor_z_velocity_error) {
			_formic_state.error_find = true;
		}

		// Only forward EV odometry once the stream has been arriving for the
		// configured settle time (FORMIC_WDEV_INIT, in seconds). During the
		// settle window we only build the quality average (the data is not
		// forwarded); the final average is computed exactly once, right before
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
				_formic_state.vel_3d_init_check_fail = (_vel_average_init > 5.0f); // if the average EV all_vel during the settle window is very low, it's likely that the EV data is not good (e.g. bad EV fusion configuration, or EV not really moving which makes the quality metric less meaningful). In this case we also set the error flag and skip forwarding the data.


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

	_formic_state.timestamp = hrt_absolute_time();
	_formic_state_machine_pub.publish(_formic_state);
}

// -----------------------------------------------------------------------------
// Data handling helpers
// -----------------------------------------------------------------------------

void FormicWatchdogEv::copy_odometry_msg(vehicle_odometry_s &odometry)
{
	odometry.reset_counter = _formic_state.heading_reset_counter;
	_odometry_pub.publish(odometry);
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
	PX4_INFO("EV z velocity: %.2f, baro deriv: %.2f, dv: %.2f", (double)_ev_vel_data_z, (double)_baro_filtered, (double)dv);
	return (dv > 1.5f);
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
	const float dt_yaw = matrix::wrap_pi(local_pos.heading - ev_yaw.observation); /// need fix 
	PX4_INFO("EV yaw: %.2f, local heading: %.2f, dt_yaw: %.2f", (double)ev_yaw.observation, (double)local_pos.heading, (double)dt_yaw);

	return (fabsf(dt_yaw) < 0.3f);
}

/*
 * Heading reset counter:
 *   1) when the EV heading is aligned, increment the reset counter
 *   2) when the EV session stops (dropout), reset the counter to zero
 *   3) allow at most one increment every 3 seconds
 */
void FormicWatchdogEv::resetcounter_heading(float raw_yaw)
{
	// (2) EV session stopped -> clear the counter and the throttle timer.
	if (!_formic_state.ev_data_arrived) {
		_formic_state.heading_reset_counter = 0;
		_last_heading_reset_time = 0;
		return;
	}

	// (3) Throttle: only one reset every 3 seconds.
	if (_last_heading_reset_time != 0 && hrt_elapsed_time(&_last_heading_reset_time) < 3_s) {
		return;
	}

	// (1) Heading aligned with EV -> count one reset.
	if (check_EV_aid_src_heading(raw_yaw)) {
		_formic_state.heading_reset_counter++;
		_formic_state.heading_alligned_with_ev = true;
		_last_heading_reset_time = hrt_absolute_time();
	}
	else {
		_formic_state.heading_alligned_with_ev = false;
	}
}



void FormicWatchdogEv::no_EvData()
{
	/* Determine if there has been a dropout in the EV (Extended Visual) data stream. */
	if ((_last_ev_timestamp == 0) ||
	    ((hrt_absolute_time() - _last_ev_timestamp) > 500_ms)) {
		_formic_state.ev_data_arrived = false;
		_formic_state.error_find = false; // dropout = end of session: clear latched error so the next session may use EV
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
	}
	else {
		// Fresh EV data has arrived within the timeout window.
		_formic_state.ev_data_arrived = true;
	}
}

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
