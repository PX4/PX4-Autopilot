/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/events.h>
#include "navputer.hpp"

using namespace time_literals;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

pthread_mutex_t navputer_module_mutex = PTHREAD_MUTEX_INITIALIZER;
static px4::atomic<Navputer *> _instance {};

Navputer::Navputer(const px4::wq_config_t &config, bool replay_mode):
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, config),
	_params(_ekf.getParamHandle()),
	_fc(*_ekf.getFusionControlHandle()),
	_param_npt_rngbc_ctrl(_params->ekf2_rngbc_ctrl),
	_param_npt_rngbc_delay(_params->ekf2_rngbc_delay),
	_param_npt_rngbc_noise(_params->ekf2_rngbc_noise),
	_param_npt_rngbc_gate(_params->ekf2_rngbc_gate)
{
	AdvertiseTopics();

	// TODO: hardcoding required params for now
	auto* fc = _ekf.getFusionControlHandle();
	fc->baro.enabled = true;
	fc->mag.enabled = true;
	fc->rngbcn.enabled = true;
	fc->agp[0].enabled = true;

	// TODO: temp solution, should be provided externally or from Aux aid src
	_ekf.resetGlobalPositionTo(49.796766, 24.347826, 270);
}

Navputer::~Navputer()
{

}

void Navputer::AdvertiseTopics()
{
	_attitude_pub.advertise();
	_local_position_pub.advertise();
}


int Navputer::task_spawn(int argc, char *argv[])
{
	bool success = false;
	bool replay_mode = false;

	if (argc > 1 && !strcmp(argv[1], "-r")) {
		PX4_INFO("replay mode enabled");
		replay_mode = true;
	}

	Navputer *_inst = new Navputer(px4::wq_configurations::INS0, replay_mode);

	if (_inst) {
		_instance.store(_inst);
		_inst->ScheduleNow();
		success = true;
	}

	return success ? PX4_OK : PX4_ERROR;
}

int Navputer::print_status(bool verbose)
{
	PX4_INFO_RAW("Navputer is OK\n");

	return 0;
}

void Navputer::Run()
{
	if (should_exit()) {
		_sensor_combined_sub.unregisterCallback();
		_vehicle_imu_sub.unregisterCallback();

		return;
	}

	// check for parameter updates
	if (_parameter_update_sub.updated() || !_callback_registered) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		_ekf.updateParameters();
	}

	if(!_callback_registered) {
		_callback_registered = _sensor_combined_sub.registerCallback();

		if (!_callback_registered) {
			ScheduleDelayed(10_ms);
			return;
		}
	}

	bool imu_updated = false;
	imuSample imu_sample_new {};

	hrt_abstime imu_dt = 0; // for tracking time slip later

	sensor_combined_s sensor_combined;
	imu_updated = _sensor_combined_sub.update(&sensor_combined);

	if (imu_updated) {
		imu_sample_new.time_us = sensor_combined.timestamp;
		imu_sample_new.delta_ang_dt = sensor_combined.gyro_integral_dt * 1.e-6f;
		imu_sample_new.delta_ang = Vector3f{sensor_combined.gyro_rad} * imu_sample_new.delta_ang_dt;
		imu_sample_new.delta_vel_dt = sensor_combined.accelerometer_integral_dt * 1.e-6f;
		imu_sample_new.delta_vel = Vector3f{sensor_combined.accelerometer_m_s2} * imu_sample_new.delta_vel_dt;

		if (sensor_combined.accelerometer_clipping > 0) {
			imu_sample_new.delta_vel_clipping[0] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_X;
			imu_sample_new.delta_vel_clipping[1] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_Y;
			imu_sample_new.delta_vel_clipping[2] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_Z;
		}

		imu_dt = sensor_combined.gyro_integral_dt;

		if (sensor_combined.accel_calibration_count != _accel_calibration_count) {

			PX4_DEBUG("%d - resetting accelerometer bias", _instance);

			_ekf.resetAccelBias();
			_accel_calibration_count = sensor_combined.accel_calibration_count;

			// reset bias learning
			_accel_cal = {};
		}

		if (sensor_combined.gyro_calibration_count != _gyro_calibration_count) {

			PX4_DEBUG("%d - resetting rate gyro bias", _instance);

			_ekf.resetGyroBias();
			_gyro_calibration_count = sensor_combined.gyro_calibration_count;

			// reset bias learning
			_gyro_cal = {};
		}
	}

	if (imu_updated) {
		const hrt_abstime now = imu_sample_new.time_us;

		// push imu data into estimator
		_ekf.setIMUData(imu_sample_new);

		// integrate time to monitor time slippage
		if (_start_time_us > 0) {
			_integrated_time_us += imu_dt;
			_last_time_slip_us = (imu_sample_new.time_us - _start_time_us) - _integrated_time_us;

		} else {
			_start_time_us = imu_sample_new.time_us;
			_last_time_slip_us = 0;
		}

		// ekf2_timestamps (using 0.1 ms relative timestamps)
		ekf2_timestamps_s ekf2_timestamps {
			.timestamp = now,
			.airspeed_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.airspeed_validated_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.distance_sensor_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.optical_flow_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.vehicle_air_data_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.vehicle_magnetometer_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.visual_odometry_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
		};

		UpdateBaroSample(ekf2_timestamps);
		UpdateMagSample(ekf2_timestamps);
		UpdateRangingBeaconSample(ekf2_timestamps);


		if (_ekf.update()) {
			PublishLocalPosition(now);
			PublishStatusFlags(now);
			PublishFusionControl(now);

			UpdateAccelCalibration(now);
			UpdateGyroCalibration(now);
			UpdateMagCalibration(now);
		}

		PublishAttitude(now); // publish attitude immediately (uses quaternion from output predictor)
	}

	// re-schedule as backup timeout
	ScheduleDelayed(100_ms);
}

void Navputer::PublishAttitude(const hrt_abstime &timestamp)
{
	if (_ekf.attitude_valid()) {
		// generate vehicle attitude quaternion data
		navput_attitude_s att;
		att.timestamp_sample = timestamp;
		_ekf.getQuaternion().copyTo(att.q);

		_ekf.get_quat_reset(&att.delta_q_reset[0], &att.quat_reset_counter);
		att.timestamp = hrt_absolute_time();
		_attitude_pub.publish(att);
	}
}

void Navputer::PublishLocalPosition(const hrt_abstime &timestamp)
{
	navput_local_position_s lpos{};
	// generate vehicle local position data
	lpos.timestamp_sample = timestamp;

	// Position of body origin in local NED frame
	const Vector3f position{_ekf.getPosition()};
	lpos.x = position(0);
	lpos.y = position(1);
	lpos.z = position(2);

	// Velocity of body origin in local NED frame (m/s)
	const Vector3f velocity{_ekf.getVelocity()};
	lpos.vx = velocity(0);
	lpos.vy = velocity(1);
	lpos.vz = velocity(2);

	// vertical position time derivative (m/s)
	lpos.z_deriv = _ekf.getVerticalPositionDerivative();

	// Acceleration of body origin in local frame
	const Vector3f vel_deriv{_ekf.getVelocityDerivative()};
	_ekf.resetVelocityDerivativeAccumulation();
	lpos.ax = vel_deriv(0);
	lpos.ay = vel_deriv(1);
	lpos.az = vel_deriv(2);

	lpos.xy_valid = _ekf.isLocalHorizontalPositionValid();
	lpos.v_xy_valid = _ekf.isLocalHorizontalPositionValid();

	// TODO: some modules (e.g.: mc_pos_control) don't handle v_z_valid != z_valid properly
	lpos.z_valid = _ekf.isLocalVerticalPositionValid() || _ekf.isLocalVerticalVelocityValid();
	lpos.v_z_valid = _ekf.isLocalVerticalVelocityValid() || _ekf.isLocalVerticalPositionValid();

	// Position of local NED origin in GPS / WGS84 frame
	if (_ekf.global_origin_valid()) {
		lpos.ref_timestamp = _ekf.global_origin().getProjectionReferenceTimestamp();
		lpos.ref_lat = _ekf.global_origin().getProjectionReferenceLat(); // Reference point latitude in degrees
		lpos.ref_lon = _ekf.global_origin().getProjectionReferenceLon(); // Reference point longitude in degrees
		lpos.ref_alt = _ekf.getEkfGlobalOriginAltitude();           // Reference point in MSL altitude meters
		lpos.xy_global = true;
		lpos.z_global = true;

	} else {
		lpos.ref_timestamp = 0;
		lpos.ref_lat = static_cast<double>(NAN);
		lpos.ref_lon = static_cast<double>(NAN);
		lpos.ref_alt = NAN;
		lpos.xy_global = false;
		lpos.z_global = false;
	}

	Quatf delta_q_reset;
	_ekf.get_quat_reset(&delta_q_reset(0), &lpos.heading_reset_counter);

	lpos.heading = Eulerf(_ekf.getQuaternion()).psi();
	lpos.unaided_heading = _ekf.getUnaidedYaw();
	lpos.heading_var = _ekf.getYawVar();
	lpos.delta_heading = Eulerf(delta_q_reset).psi();
	lpos.heading_good_for_control = _ekf.isYawFinalAlignComplete();
	lpos.tilt_var = _ekf.getTiltVariance();

#if defined(CONFIG_EKF2_TERRAIN)
	// Distance to bottom surface (ground) in meters, must be positive
	lpos.dist_bottom_valid = _ekf.isHeightAboveGroundEstimateValid();
	lpos.dist_bottom = math::max(_ekf.getHagl(), 0.f);
	lpos.dist_bottom_var = _ekf.getHaglVariance();
	_ekf.get_hagl_reset(&lpos.delta_dist_bottom, &lpos.dist_bottom_reset_counter);

	lpos.dist_bottom_sensor_bitfield = vehicle_local_position_s::DIST_BOTTOM_SENSOR_NONE;

	if (_ekf.control_status_flags().rng_terrain || _ekf.control_status_flags().rng_hgt) {
		lpos.dist_bottom_sensor_bitfield |= vehicle_local_position_s::DIST_BOTTOM_SENSOR_RANGE;
	}

	if (_ekf.control_status_flags().opt_flow_terrain) {
		lpos.dist_bottom_sensor_bitfield |= vehicle_local_position_s::DIST_BOTTOM_SENSOR_FLOW;
	}

#endif // CONFIG_EKF2_TERRAIN

	_ekf.get_ekf_lpos_accuracy(&lpos.eph, &lpos.epv);
	_ekf.get_ekf_vel_accuracy(&lpos.evh, &lpos.evv);

	// get state reset information of position and velocity
	_ekf.get_posD_reset(&lpos.delta_z, &lpos.z_reset_counter);
	_ekf.get_velD_reset(&lpos.delta_vz, &lpos.vz_reset_counter);
	_ekf.get_posNE_reset(&lpos.delta_xy[0], &lpos.xy_reset_counter);
	_ekf.get_velNE_reset(&lpos.delta_vxy[0], &lpos.vxy_reset_counter);

	lpos.dead_reckoning = _ekf.control_status_flags().inertial_dead_reckoning
			      || _ekf.control_status_flags().wind_dead_reckoning;

	// get control limit information
	_ekf.get_ekf_ctrl_limits(&lpos.vxy_max, &lpos.vz_max, &lpos.hagl_min, &lpos.hagl_max_z, &lpos.hagl_max_xy);

	// convert NaN to INFINITY
	if (!PX4_ISFINITE(lpos.vxy_max)) {
		lpos.vxy_max = INFINITY;
	}

	if (!PX4_ISFINITE(lpos.vz_max)) {
		lpos.vz_max = INFINITY;
	}

	if (!PX4_ISFINITE(lpos.hagl_min)) {
		lpos.hagl_min = INFINITY;
	}

	if (!PX4_ISFINITE(lpos.hagl_max_z)) {
		lpos.hagl_max_z = INFINITY;
	}

	if (!PX4_ISFINITE(lpos.hagl_max_xy)) {
		lpos.hagl_max_xy = INFINITY;
	}

	// publish vehicle local position data
	lpos.timestamp = hrt_absolute_time();
	_local_position_pub.publish(lpos);
}

void Navputer::PublishFusionControl(const hrt_abstime &timestamp)
{
	navput_fusion_control_s msg{};
	msg.gps_intended[0] = _fc.gps.intended();
	msg.of_intended     = _fc.of.intended();
	msg.ev_intended     = _fc.ev.intended();

	for (uint8_t i = 0; i < MAX_AGP_INSTANCES; i++) {
		msg.agp_intended[i] = _fc.agp[i].intended();
	}

	msg.baro_intended   = _fc.baro.intended();
	msg.rng_intended    = _fc.rng.intended();
	msg.mag_intended    = _fc.mag.intended();
	msg.aspd_intended   = _fc.aspd.intended();
	msg.rngbcn_intended = _fc.rngbcn.intended();

	const auto &cs = _ekf.control_status_flags();
	msg.gps_active[0] = cs.gnss_pos || cs.gps_hgt || cs.gnss_vel || cs.gnss_yaw;
	msg.of_active     = cs.opt_flow;
	msg.ev_active     = cs.ev_pos || cs.ev_hgt || cs.ev_vel || cs.ev_yaw;
	msg.baro_active   = cs.baro_hgt;
	msg.rng_active    = cs.rng_hgt;
	msg.mag_active    = cs.mag;
	msg.aspd_active   = cs.fuse_aspd;
	msg.rngbcn_active = cs.rngbcn_fusion;

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION)
	{
		const uint8_t agp_mask = _ekf.getAgpFusingBitmask();

		for (uint8_t i = 0; i < MAX_AGP_INSTANCES; i++) {
			msg.agp_active[i] = agp_mask & (1u << i);
		}
	}
#endif

	msg.timestamp = hrt_absolute_time();
	_fc_pub.publish(msg);
}

void Navputer::PublishStatusFlags(const hrt_abstime &timestamp)
{
	// publish at ~ 1 Hz (or immediately if filter control status or fault status changes)
	bool update = (timestamp >= _last_status_flags_publish + 1_s);

	// filter control status
	if (_ekf.control_status().value != _filter_control_status) {
		update = true;
		_filter_control_status = _ekf.control_status().value;
		_filter_control_status_changes++;
	}

	// filter fault status
	if (_ekf.fault_status().value != _filter_fault_status) {
		update = true;
		_filter_fault_status = _ekf.fault_status().value;
		_filter_fault_status_changes++;
	}

	if (update) {
		navput_status_flags_s status_flags{};
		status_flags.timestamp_sample = _ekf.time_delayed_us();

		status_flags.control_status_changes   = _filter_control_status_changes;
		status_flags.cs_tilt_align            = _ekf.control_status_flags().tilt_align;
		status_flags.cs_yaw_align             = _ekf.control_status_flags().yaw_align;
		status_flags.cs_gnss_pos              = _ekf.control_status_flags().gnss_pos;
		status_flags.cs_opt_flow              = _ekf.control_status_flags().opt_flow;
		status_flags.cs_mag_hdg               = _ekf.control_status_flags().mag_hdg;
		status_flags.cs_mag_3d                = _ekf.control_status_flags().mag_3D;
		status_flags.cs_mag_dec               = _ekf.control_status_flags().mag_dec;
		status_flags.cs_in_air                = _ekf.control_status_flags().in_air;
		status_flags.cs_wind                  = _ekf.control_status_flags().wind;
		status_flags.cs_baro_hgt              = _ekf.control_status_flags().baro_hgt;
		status_flags.cs_rng_hgt               = _ekf.control_status_flags().rng_hgt;
		status_flags.cs_gps_hgt               = _ekf.control_status_flags().gps_hgt;
		status_flags.cs_ev_pos                = _ekf.control_status_flags().ev_pos;
		status_flags.cs_ev_yaw                = _ekf.control_status_flags().ev_yaw;
		status_flags.cs_ev_hgt                = _ekf.control_status_flags().ev_hgt;
		status_flags.cs_fuse_beta             = _ekf.control_status_flags().fuse_beta;
		status_flags.cs_mag_field_disturbed   = _ekf.control_status_flags().mag_field_disturbed;
		status_flags.cs_fixed_wing            = _ekf.control_status_flags().fixed_wing;
		status_flags.cs_mag_fault             = _ekf.control_status_flags().mag_fault;
		status_flags.cs_fuse_aspd             = _ekf.control_status_flags().fuse_aspd;
		status_flags.cs_gnd_effect            = _ekf.control_status_flags().gnd_effect;
		status_flags.cs_rng_stuck             = _ekf.control_status_flags().rng_stuck;
		status_flags.cs_gnss_yaw               = _ekf.control_status_flags().gnss_yaw;
		status_flags.cs_mag_aligned_in_flight = _ekf.control_status_flags().mag_aligned_in_flight;
		status_flags.cs_ev_vel                = _ekf.control_status_flags().ev_vel;
		status_flags.cs_synthetic_mag_z       = _ekf.control_status_flags().synthetic_mag_z;
		status_flags.cs_vehicle_at_rest       = _ekf.control_status_flags().vehicle_at_rest;
		status_flags.cs_gnss_yaw_fault         = _ekf.control_status_flags().gnss_yaw_fault;
		status_flags.cs_rng_fault             = _ekf.control_status_flags().rng_fault;
		status_flags.cs_inertial_dead_reckoning = _ekf.control_status_flags().inertial_dead_reckoning;
		status_flags.cs_wind_dead_reckoning     = _ekf.control_status_flags().wind_dead_reckoning;
		status_flags.cs_rng_kin_consistent      = _ekf.control_status_flags().rng_kin_consistent;
		status_flags.cs_fake_pos                = _ekf.control_status_flags().fake_pos;
		status_flags.cs_fake_hgt                = _ekf.control_status_flags().fake_hgt;
		status_flags.cs_gravity_vector          = _ekf.control_status_flags().gravity_vector;
		status_flags.cs_mag                     = _ekf.control_status_flags().mag;
		status_flags.cs_ev_yaw_fault            = _ekf.control_status_flags().ev_yaw_fault;
		status_flags.cs_mag_heading_consistent  = _ekf.control_status_flags().mag_heading_consistent;
		status_flags.cs_aux_gpos                = _ekf.control_status_flags().aux_gpos;
		status_flags.cs_rng_terrain    = _ekf.control_status_flags().rng_terrain;
		status_flags.cs_opt_flow_terrain    = _ekf.control_status_flags().opt_flow_terrain;
		status_flags.cs_valid_fake_pos      = _ekf.control_status_flags().valid_fake_pos;
		status_flags.cs_constant_pos        = _ekf.control_status_flags().constant_pos;
		status_flags.cs_baro_fault	    = _ekf.control_status_flags().baro_fault;
		status_flags.cs_gnss_vel            = _ekf.control_status_flags().gnss_vel;
		status_flags.cs_gnss_fault          = _ekf.control_status_flags().gnss_fault;
		status_flags.cs_yaw_manual          = _ekf.control_status_flags().yaw_manual;
		status_flags.cs_gnss_hgt_fault      = _ekf.control_status_flags().gnss_hgt_fault;
		status_flags.cs_in_transition       = _ekf.control_status_flags().in_transition;
		status_flags.cs_heading_observable  = _ekf.control_status_flags().heading_observable;

		status_flags.fault_status_changes     = _filter_fault_status_changes;
		status_flags.fs_bad_mag_x             = _ekf.fault_status_flags().bad_mag_x;
		status_flags.fs_bad_mag_y             = _ekf.fault_status_flags().bad_mag_y;
		status_flags.fs_bad_mag_z             = _ekf.fault_status_flags().bad_mag_z;
		status_flags.fs_bad_hdg               = _ekf.fault_status_flags().bad_hdg;
		status_flags.fs_bad_mag_decl          = _ekf.fault_status_flags().bad_mag_decl;
		status_flags.fs_bad_airspeed          = _ekf.fault_status_flags().bad_airspeed;
		status_flags.fs_bad_sideslip          = _ekf.fault_status_flags().bad_sideslip;
		status_flags.fs_bad_optflow_x         = _ekf.fault_status_flags().bad_optflow_X;
		status_flags.fs_bad_optflow_y         = _ekf.fault_status_flags().bad_optflow_Y;
		status_flags.fs_bad_acc_vertical      = _ekf.fault_status_flags().bad_acc_vertical;
		status_flags.fs_bad_acc_clipping      = _ekf.fault_status_flags().bad_acc_clipping;

		status_flags.timestamp = hrt_absolute_time();
		_status_flags_pub.publish(status_flags);

		_last_status_flags_publish = status_flags.timestamp;
	}
}

void Navputer::UpdateBaroSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF baro sample
	vehicle_air_data_s airdata;

	if (_airdata_sub.update(&airdata)) {

		bool reset = false;

		// check if barometer has changed
		if (airdata.baro_device_id != _device_id_baro) {
			if (_device_id_baro != 0) {
				PX4_DEBUG("baro sensor ID changed %" PRIu32 " -> %" PRIu32, _device_id_baro, airdata.baro_device_id);
			}

			reset = true;

		} else if (airdata.calibration_count != _baro_calibration_count) {
			// existing calibration has changed, reset saved baro bias
			PX4_DEBUG("baro %" PRIu32 " calibration updated, resetting bias", _device_id_baro);
			reset = true;
		}

		if (reset) {
			_device_id_baro = airdata.baro_device_id;
			_baro_calibration_count = airdata.calibration_count;
		}

		_ekf.set_air_density(airdata.rho);

		_ekf.setBaroData(baroSample{airdata.timestamp_sample, airdata.baro_alt_meter, reset});

		ekf2_timestamps.vehicle_air_data_timestamp_rel = (int16_t)((int64_t)airdata.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}
}

void Navputer::UpdateMagSample(ekf2_timestamps_s &ekf2_timestamps)
{
	vehicle_magnetometer_s magnetometer;

	if (_magnetometer_sub.update(&magnetometer)) {

		bool reset = false;

		// check if magnetometer has changed
		if (magnetometer.device_id != _device_id_mag) {
			if (_device_id_mag != 0) {
				PX4_DEBUG("mag sensor ID changed %" PRIu32 " -> %" PRIu32, _device_id_mag, magnetometer.device_id);
			}

			reset = true;

		} else if (magnetometer.calibration_count != _mag_calibration_count) {
			// existing calibration has changed, reset saved mag bias
			PX4_DEBUG("mag %" PRIu32 " calibration updated, resetting bias", _device_id_mag);
			reset = true;
		}

		if (reset) {
			_device_id_mag = magnetometer.device_id;
			_mag_calibration_count = magnetometer.calibration_count;

			// reset magnetometer bias learning
			_mag_cal = {};
		}

		_ekf.setMagData(magSample{magnetometer.timestamp_sample, Vector3f{magnetometer.magnetometer_ga}, reset});

		ekf2_timestamps.vehicle_magnetometer_timestamp_rel = (int16_t)((int64_t)magnetometer.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}
}

void Navputer::UpdateRangingBeaconSample(ekf2_timestamps_s &ekf2_timestamps)
{
	ranging_beacon_s ranging_beacon;

	if (_ranging_beacon_sub.update(&ranging_beacon)) {
		const float range_var = PX4_ISFINITE(ranging_beacon.range_accuracy)
					? sq(ranging_beacon.range_accuracy) : 50.f; // TODO: expose in params sq(_param_ekf2_rngbc_noise.get());

		rangingBeaconSample sample{
			.time_us = ranging_beacon.timestamp_sample,
			.beacon_id = ranging_beacon.beacon_id,
			.range_m = ranging_beacon.range,
			.range_var = range_var,
			.beacon_lat = ranging_beacon.lat,
			.beacon_lon = ranging_beacon.lon,
			.beacon_alt = ranging_beacon.alt,
		};

		_ekf.setRangingBeaconData(sample);
	}
}

void Navputer::UpdateCalibration(const hrt_abstime &timestamp, InFlightCalibration &cal, const matrix::Vector3f &bias,
			     const matrix::Vector3f &bias_variance, float bias_limit, bool bias_valid, bool learning_valid)
{
	// reset existing cal on takeoff
	if (!_ekf.control_status_prev_flags().in_air && _ekf.control_status_flags().in_air) {
		cal = {};
	}

	// Check if conditions are OK for learning of accelerometer bias values
	// the EKF is operating in the correct mode and there are no filter faults
	static constexpr float max_var_allowed = 1e-3f;
	static constexpr float max_var_ratio = 1e2f;

	const bool valid = bias_valid
			   && (bias_variance.max() < max_var_allowed)
			   && (bias_variance.max() < max_var_ratio * bias_variance.min());

	if (valid && learning_valid) {
		// consider bias estimates stable when all checks pass consistently and bias hasn't changed more than 10% of the limit
		const float bias_change_limit = 0.1f * bias_limit;

		if (!(cal.bias - bias).longerThan(bias_change_limit)) {
			if (cal.last_us != 0) {
				cal.total_time_us += timestamp - cal.last_us;
			}

			if (cal.total_time_us > 10_s) {
				cal.cal_available = true;
			}

		} else {
			cal.total_time_us = 0;
			cal.bias = bias;
			cal.cal_available = false;
		}

		cal.last_us = timestamp;

	} else {
		// conditions are NOT OK for learning bias, reset timestamp
		// but keep the accumulated calibration time
		cal.last_us = 0;

		if (!valid && (cal.total_time_us != 0)) {
			// if a filter fault has occurred, assume previous learning was invalid and do not
			// count it towards total learning time.
			cal = {};
		}
	}
}

void Navputer::UpdateAccelCalibration(const hrt_abstime &timestamp)
{
	// the EKF is operating in the correct mode and there are no filter faults
	const bool bias_valid = true // TODO: (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::AccelBias))
				&& _ekf.control_status_flags().tilt_align
				&& (_ekf.fault_status().value == 0)
				&& !_ekf.fault_status_flags().bad_acc_clipping
				&& !_ekf.fault_status_flags().bad_acc_vertical;

	const bool learning_valid = bias_valid && !_ekf.accel_bias_inhibited();

	UpdateCalibration(timestamp, _accel_cal, _ekf.getAccelBias(), _ekf.getAccelBiasVariance(), _ekf.getAccelBiasLimit(),
			  bias_valid, learning_valid);
}

void Navputer::UpdateGyroCalibration(const hrt_abstime &timestamp)
{
	// the EKF is operating in the correct mode and there are no filter faults
	const bool bias_valid = true // TODO: (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::GyroBias))
				&& _ekf.control_status_flags().tilt_align
				&& (_ekf.fault_status().value == 0);

	const bool learning_valid = bias_valid && !_ekf.gyro_bias_inhibited();

	UpdateCalibration(timestamp, _gyro_cal, _ekf.getGyroBias(), _ekf.getGyroBiasVariance(), _ekf.getGyroBiasLimit(),
			  bias_valid, learning_valid);
}

void Navputer::UpdateMagCalibration(const hrt_abstime &timestamp)
{
	const Vector3f mag_bias = _ekf.getMagBias();
	const Vector3f mag_bias_var = _ekf.getMagBiasVariance();

	const bool bias_valid = (_ekf.fault_status().value == 0)
				&& _ekf.control_status_flags().yaw_align
				&& mag_bias_var.longerThan(0.f) && !mag_bias_var.longerThan(0.02f);

	const bool learning_valid = bias_valid && _ekf.control_status_flags().mag;

	UpdateCalibration(timestamp, _mag_cal, mag_bias, mag_bias_var, _ekf.getMagBiasLimit(), bias_valid, learning_valid);
}

int Navputer::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Navputer::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	return 0;
}

extern "C" __EXPORT int navputer_main(int argc, char *argv[])
{
	if (argc <= 1 || strcmp(argv[1], "-h") == 0) {
		return Navputer::print_usage();
	}

	if (strcmp(argv[1], "start") == 0) {
		int ret = 0;
		Navputer::lock_module();

		ret = Navputer::task_spawn(argc - 1, argv + 1);

		if (ret < 0) {
			PX4_ERR("start failed (%i)", ret);
		}

		Navputer::unlock_module();
		return ret;
	} else if (strcmp(argv[1], "status") == 0) {
		if (Navputer::trylock_module()) {
			bool verbose_status = false;

			_instance.load()->print_status(verbose_status);

			Navputer::unlock_module();

		} else {
			PX4_WARN("module locked, try again later");
		}

		return 0;

	} else if (strcmp(argv[1], "stop") == 0) {
		Navputer::lock_module();

		// otherwise stop everything
		bool was_running = false;

		Navputer *inst = _instance.load();

		if (inst) {
			PX4_INFO("stopping Navputer");
			was_running = true;
			inst->request_stop();
			px4_usleep(20000); // 20 ms
			delete inst;
			_instance.store(nullptr);
		}

		if (!was_running) {
			PX4_WARN("not running");
		}

		Navputer::unlock_module();
		return PX4_OK;
	}

	Navputer::lock_module(); // Lock here, as the method could access _instance.
	int ret = Navputer::custom_command(argc - 1, argv + 1);
	Navputer::unlock_module();

	return ret;
}
