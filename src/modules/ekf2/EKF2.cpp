/****************************************************************************
 *
 *   Copyright (c) 2015-2020 PX4 Development Team. All rights reserved.
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

#include "EKF2.hpp"

using namespace time_literals;
using math::constrain;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

pthread_mutex_t ekf2_module_mutex = PTHREAD_MUTEX_INITIALIZER;
static px4::atomic<EKF2 *> _objects[EKF2_MAX_INSTANCES] {};
#if !defined(CONSTRAINED_FLASH)
static px4::atomic<EKF2Selector *> _ekf2_selector {nullptr};
#endif // !CONSTRAINED_FLASH

EKF2::EKF2(int instance, const px4::wq_config_t &config, int imu, int mag, bool replay_mode):
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, config),
	_replay_mode(replay_mode && instance < 0),
	_multi_mode(instance >= 0),
	_instance(math::constrain(instance, 0, EKF2_MAX_INSTANCES - 1)),
	_ekf_update_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": update")),
	_attitude_pub(_multi_mode ? ORB_ID(estimator_attitude) : ORB_ID(vehicle_attitude)),
	_local_position_pub(_multi_mode ? ORB_ID(estimator_local_position) : ORB_ID(vehicle_local_position)),
	_global_position_pub(_multi_mode ? ORB_ID(estimator_global_position) : ORB_ID(vehicle_global_position)),
	_odometry_pub(_multi_mode ? ORB_ID(estimator_odometry) : ORB_ID(vehicle_odometry)),
	_params(_ekf.getParamHandle()),
	_param_ekf2_min_obs_dt(_params->sensor_interval_min_ms),
	_param_ekf2_mag_delay(_params->mag_delay_ms),
	_param_ekf2_baro_delay(_params->baro_delay_ms),
	_param_ekf2_gps_delay(_params->gps_delay_ms),
	_param_ekf2_of_delay(_params->flow_delay_ms),
	_param_ekf2_rng_delay(_params->range_delay_ms),
	_param_ekf2_asp_delay(_params->airspeed_delay_ms),
	_param_ekf2_ev_delay(_params->ev_delay_ms),
	_param_ekf2_avel_delay(_params->auxvel_delay_ms),
	_param_ekf2_gyr_noise(_params->gyro_noise),
	_param_ekf2_acc_noise(_params->accel_noise),
	_param_ekf2_gyr_b_noise(_params->gyro_bias_p_noise),
	_param_ekf2_acc_b_noise(_params->accel_bias_p_noise),
	_param_ekf2_mag_e_noise(_params->mage_p_noise),
	_param_ekf2_mag_b_noise(_params->magb_p_noise),
	_param_ekf2_wind_noise(_params->wind_vel_p_noise),
	_param_ekf2_terr_noise(_params->terrain_p_noise),
	_param_ekf2_terr_grad(_params->terrain_gradient),
	_param_ekf2_gps_v_noise(_params->gps_vel_noise),
	_param_ekf2_gps_p_noise(_params->gps_pos_noise),
	_param_ekf2_noaid_noise(_params->pos_noaid_noise),
	_param_ekf2_baro_noise(_params->baro_noise),
	_param_ekf2_baro_gate(_params->baro_innov_gate),
	_param_ekf2_gnd_eff_dz(_params->gnd_effect_deadzone),
	_param_ekf2_gnd_max_hgt(_params->gnd_effect_max_hgt),
	_param_ekf2_gps_p_gate(_params->gps_pos_innov_gate),
	_param_ekf2_gps_v_gate(_params->gps_vel_innov_gate),
	_param_ekf2_tas_gate(_params->tas_innov_gate),
	_param_ekf2_head_noise(_params->mag_heading_noise),
	_param_ekf2_mag_noise(_params->mag_noise),
	_param_ekf2_eas_noise(_params->eas_noise),
	_param_ekf2_beta_gate(_params->beta_innov_gate),
	_param_ekf2_beta_noise(_params->beta_noise),
	_param_ekf2_mag_decl(_params->mag_declination_deg),
	_param_ekf2_hdg_gate(_params->heading_innov_gate),
	_param_ekf2_mag_gate(_params->mag_innov_gate),
	_param_ekf2_decl_type(_params->mag_declination_source),
	_param_ekf2_mag_type(_params->mag_fusion_type),
	_param_ekf2_mag_acclim(_params->mag_acc_gate),
	_param_ekf2_mag_yawlim(_params->mag_yaw_rate_gate),
	_param_ekf2_gps_check(_params->gps_check_mask),
	_param_ekf2_req_eph(_params->req_hacc),
	_param_ekf2_req_epv(_params->req_vacc),
	_param_ekf2_req_sacc(_params->req_sacc),
	_param_ekf2_req_nsats(_params->req_nsats),
	_param_ekf2_req_pdop(_params->req_pdop),
	_param_ekf2_req_hdrift(_params->req_hdrift),
	_param_ekf2_req_vdrift(_params->req_vdrift),
	_param_ekf2_aid_mask(_params->fusion_mode),
	_param_ekf2_hgt_mode(_params->vdist_sensor_type),
	_param_ekf2_terr_mask(_params->terrain_fusion_mode),
	_param_ekf2_noaid_tout(_params->valid_timeout_max),
	_param_ekf2_rng_noise(_params->range_noise),
	_param_ekf2_rng_sfe(_params->range_noise_scaler),
	_param_ekf2_rng_gate(_params->range_innov_gate),
	_param_ekf2_min_rng(_params->rng_gnd_clearance),
	_param_ekf2_rng_pitch(_params->rng_sens_pitch),
	_param_ekf2_rng_aid(_params->range_aid),
	_param_ekf2_rng_a_vmax(_params->max_vel_for_range_aid),
	_param_ekf2_rng_a_hmax(_params->max_hagl_for_range_aid),
	_param_ekf2_rng_a_igate(_params->range_aid_innov_gate),
	_param_ekf2_evv_gate(_params->ev_vel_innov_gate),
	_param_ekf2_evp_gate(_params->ev_pos_innov_gate),
	_param_ekf2_of_n_min(_params->flow_noise),
	_param_ekf2_of_n_max(_params->flow_noise_qual_min),
	_param_ekf2_of_qmin(_params->flow_qual_min),
	_param_ekf2_of_gate(_params->flow_innov_gate),
	_param_ekf2_imu_pos_x(_params->imu_pos_body(0)),
	_param_ekf2_imu_pos_y(_params->imu_pos_body(1)),
	_param_ekf2_imu_pos_z(_params->imu_pos_body(2)),
	_param_ekf2_gps_pos_x(_params->gps_pos_body(0)),
	_param_ekf2_gps_pos_y(_params->gps_pos_body(1)),
	_param_ekf2_gps_pos_z(_params->gps_pos_body(2)),
	_param_ekf2_rng_pos_x(_params->rng_pos_body(0)),
	_param_ekf2_rng_pos_y(_params->rng_pos_body(1)),
	_param_ekf2_rng_pos_z(_params->rng_pos_body(2)),
	_param_ekf2_of_pos_x(_params->flow_pos_body(0)),
	_param_ekf2_of_pos_y(_params->flow_pos_body(1)),
	_param_ekf2_of_pos_z(_params->flow_pos_body(2)),
	_param_ekf2_ev_pos_x(_params->ev_pos_body(0)),
	_param_ekf2_ev_pos_y(_params->ev_pos_body(1)),
	_param_ekf2_ev_pos_z(_params->ev_pos_body(2)),
	_param_ekf2_tau_vel(_params->vel_Tau),
	_param_ekf2_tau_pos(_params->pos_Tau),
	_param_ekf2_gbias_init(_params->switch_on_gyro_bias),
	_param_ekf2_abias_init(_params->switch_on_accel_bias),
	_param_ekf2_angerr_init(_params->initial_tilt_err),
	_param_ekf2_abl_lim(_params->acc_bias_lim),
	_param_ekf2_abl_acclim(_params->acc_bias_learn_acc_lim),
	_param_ekf2_abl_gyrlim(_params->acc_bias_learn_gyr_lim),
	_param_ekf2_abl_tau(_params->acc_bias_learn_tc),
	_param_ekf2_drag_noise(_params->drag_noise),
	_param_ekf2_bcoef_x(_params->bcoef_x),
	_param_ekf2_bcoef_y(_params->bcoef_y),
	_param_ekf2_aspd_max(_params->max_correction_airspeed),
	_param_ekf2_pcoef_xp(_params->static_pressure_coef_xp),
	_param_ekf2_pcoef_xn(_params->static_pressure_coef_xn),
	_param_ekf2_pcoef_yp(_params->static_pressure_coef_yp),
	_param_ekf2_pcoef_yn(_params->static_pressure_coef_yn),
	_param_ekf2_pcoef_z(_params->static_pressure_coef_z),
	_param_ekf2_move_test(_params->is_moving_scaler),
	_param_ekf2_mag_check(_params->check_mag_strength),
	_param_ekf2_gsf_tas_default(_params->EKFGSF_tas_default)
{
	// initialise parameter cache
	updateParams();

	_ekf.set_min_required_gps_health_time(_param_ekf2_req_gps_h.get() * 1_s);

	if (_multi_mode) {
		// advertise immediately to ensure consistent uORB instance numbering
		_attitude_pub.advertise();
		_local_position_pub.advertise();
		_global_position_pub.advertise();
		_odometry_pub.advertise();

		_ekf2_timestamps_pub.advertise();
		_ekf_gps_drift_pub.advertise();
		_estimator_innovation_test_ratios_pub.advertise();
		_estimator_innovation_variances_pub.advertise();
		_estimator_innovations_pub.advertise();
		_estimator_optical_flow_vel_pub.advertise();
		_estimator_sensor_bias_pub.advertise();
		_estimator_states_pub.advertise();
		_estimator_status_pub.advertise();
		_estimator_visual_odometry_aligned_pub.advertised();
		_wind_pub.advertise();
		_yaw_est_pub.advertise();


		_vehicle_imu_sub.ChangeInstance(imu);
		_magnetometer_sub.ChangeInstance(mag);
	}
}

EKF2::~EKF2()
{
	if (!_multi_mode) {
		px4_lockstep_unregister_component(_lockstep_component);
	}

	perf_free(_ekf_update_perf);
}

int EKF2::print_status()
{
	PX4_INFO_RAW("ekf2:%d attitude: %d, local position: %d, global position: %d\n", _instance, _ekf.attitude_valid(),
		     _ekf.local_position_is_valid(), _ekf.global_position_is_valid());
	perf_print_counter(_ekf_update_perf);
	return 0;
}

template<typename Param>
void EKF2::update_mag_bias(Param &mag_bias_param, int axis_index)
{
	if (_valid_cal_available[axis_index]) {

		// calculate weighting using ratio of variances and update stored bias values
		const float weighting = constrain(_param_ekf2_magb_vref.get() / (_param_ekf2_magb_vref.get() +
						  _last_valid_variance[axis_index]), 0.0f, _param_ekf2_magb_k.get());
		const float mag_bias_saved = mag_bias_param.get();

		_last_valid_mag_cal[axis_index] = weighting * _last_valid_mag_cal[axis_index] + mag_bias_saved;

		mag_bias_param.set(_last_valid_mag_cal[axis_index]);

		// save new parameters unless in multi-instance mode
		if (!_multi_mode) {
			mag_bias_param.commit_no_notification();
		}

		_valid_cal_available[axis_index] = false;
	}
}

template<typename Param>
bool EKF2::update_mag_decl(Param &mag_decl_param)
{
	// update stored declination value
	float declination_deg;

	if (_ekf.get_mag_decl_deg(&declination_deg)) {
		mag_decl_param.set(declination_deg);

		if (!_multi_mode) {
			mag_decl_param.commit_no_notification();
		}

		return true;
	}

	return false;
}

void EKF2::Run()
{
	if (should_exit()) {
		_sensor_combined_sub.unregisterCallback();
		_vehicle_imu_sub.unregisterCallback();

		return;
	}

	if (!_callback_registered) {
		if (_multi_mode) {
			_callback_registered = _vehicle_imu_sub.registerCallback();

		} else {
			_callback_registered = _sensor_combined_sub.registerCallback();
		}

		if (!_callback_registered) {
			PX4_WARN("%d failed to register callback, retrying", _instance);
			ScheduleDelayed(1_s);
			return;
		}
	}

	bool updated = false;
	imuSample imu_sample_new {};

	hrt_abstime imu_dt = 0; // for tracking time slip later

	if (_multi_mode) {
		vehicle_imu_s imu;
		updated = _vehicle_imu_sub.update(&imu);

		imu_sample_new.time_us = imu.timestamp_sample;
		imu_sample_new.delta_ang_dt = imu.delta_angle_dt * 1.e-6f;
		imu_sample_new.delta_ang = Vector3f{imu.delta_angle};
		imu_sample_new.delta_vel_dt = imu.delta_velocity_dt * 1.e-6f;
		imu_sample_new.delta_vel = Vector3f{imu.delta_velocity};

		if (imu.delta_velocity_clipping > 0) {
			imu_sample_new.delta_vel_clipping[0] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_X;
			imu_sample_new.delta_vel_clipping[1] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_Y;
			imu_sample_new.delta_vel_clipping[2] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_Z;
		}

		imu_dt = imu.delta_angle_dt;

		_device_id_accel = imu.accel_device_id;
		_device_id_gyro = imu.gyro_device_id;

	} else {
		sensor_combined_s sensor_combined;
		updated = _sensor_combined_sub.update(&sensor_combined);

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
	}

	if (updated) {

		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
		}

		const hrt_abstime now = imu_sample_new.time_us;

		// ekf2_timestamps (using 0.1 ms relative timestamps)
		ekf2_timestamps_s ekf2_timestamps{};
		ekf2_timestamps.timestamp = now;

		ekf2_timestamps.airspeed_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.distance_sensor_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.optical_flow_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.vehicle_air_data_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.vehicle_magnetometer_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.visual_odometry_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;

		// update all other topics if they have new data
		if (_status_sub.updated()) {
			vehicle_status_s vehicle_status;

			if (_status_sub.copy(&vehicle_status)) {

				const bool is_fixed_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
				_can_observe_heading_in_flight = (vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

				// only fuse synthetic sideslip measurements if conditions are met
				_ekf.set_fuse_beta_flag(is_fixed_wing && (_param_ekf2_fuse_beta.get() == 1));

				// let the EKF know if the vehicle motion is that of a fixed wing (forward flight only relative to wind)
				_ekf.set_is_fixed_wing(is_fixed_wing);
			}
		}

		// Always update sensor selction first time through if time stamp is non zero
		if (!_multi_mode && (_sensor_selection_sub.updated() || (_device_id_accel == 0 || _device_id_gyro == 0))) {
			sensor_selection_s sensor_selection;

			if (_sensor_selection_sub.copy(&sensor_selection)) {
				if (_device_id_accel != sensor_selection.accel_device_id) {
					_imu_bias_reset_request = true;
					_device_id_accel = sensor_selection.accel_device_id;
				}

				if (_device_id_gyro != sensor_selection.gyro_device_id) {
					_imu_bias_reset_request = true;
					_device_id_gyro = sensor_selection.gyro_device_id;
				}
			}
		}

		// attempt reset until successful
		if (_imu_bias_reset_request) {
			_imu_bias_reset_request = !_ekf.reset_imu_bias();
		}

		// push imu data into estimator
		_ekf.setIMUData(imu_sample_new);

		// publish attitude immediately (uses quaternion from output predictor)
		publish_attitude(now);

		// read mag data
		if (_magnetometer_sub.updated()) {
			vehicle_magnetometer_s magnetometer;

			if (_magnetometer_sub.copy(&magnetometer)) {
				// Reset learned bias parameters if there has been a persistant change in magnetometer ID
				// Do not reset parmameters when armed to prevent potential time slips casued by parameter set
				// and notification events
				// Check if there has been a persistant change in magnetometer ID
				if (magnetometer.device_id != 0
				    && (magnetometer.device_id != (uint32_t)_param_ekf2_magbias_id.get())) {

					if (_invalid_mag_id_count < 200) {
						_invalid_mag_id_count++;
					}

				} else {
					if (_invalid_mag_id_count > 0) {
						_invalid_mag_id_count--;
					}
				}

				_device_id_mag = magnetometer.device_id;

				if (!_armed && (_invalid_mag_id_count > 100)) {
					// the sensor ID used for the last saved mag bias is not confirmed to be the same as the current sensor ID
					// this means we need to reset the learned bias values to zero
					_param_ekf2_magbias_x.set(0.f);
					_param_ekf2_magbias_y.set(0.f);
					_param_ekf2_magbias_z.set(0.f);
					_param_ekf2_magbias_id.set(magnetometer.device_id);

					if (!_multi_mode) {
						_param_ekf2_magbias_x.reset();
						_param_ekf2_magbias_y.reset();
						_param_ekf2_magbias_z.reset();
						_param_ekf2_magbias_id.commit();
						PX4_INFO("Mag sensor ID changed to %i", _param_ekf2_magbias_id.get());
					}

					_invalid_mag_id_count = 0;
				}

				magSample mag_sample {};
				mag_sample.mag(0) = magnetometer.magnetometer_ga[0] - _param_ekf2_magbias_x.get();
				mag_sample.mag(1) = magnetometer.magnetometer_ga[1] - _param_ekf2_magbias_y.get();
				mag_sample.mag(2) = magnetometer.magnetometer_ga[2] - _param_ekf2_magbias_z.get();
				mag_sample.time_us = magnetometer.timestamp_sample;

				_ekf.setMagData(mag_sample);
				ekf2_timestamps.vehicle_magnetometer_timestamp_rel = (int16_t)((int64_t)magnetometer.timestamp / 100 -
						(int64_t)ekf2_timestamps.timestamp / 100);
			}
		}

		// read baro data
		if (_airdata_sub.updated()) {
			vehicle_air_data_s airdata;

			if (_airdata_sub.copy(&airdata)) {
				_ekf.set_air_density(airdata.rho);
				const baroSample baro_sample {airdata.baro_alt_meter, airdata.timestamp_sample};
				_ekf.setBaroData(baro_sample);
				ekf2_timestamps.vehicle_air_data_timestamp_rel = (int16_t)((int64_t)airdata.timestamp / 100 -
						(int64_t)ekf2_timestamps.timestamp / 100);

				_device_id_baro = airdata.baro_device_id;
			}
		}

		if (_vehicle_gps_position_sub.updated()) {
			vehicle_gps_position_s gps;

			if (_vehicle_gps_position_sub.copy(&gps)) {
				gps_message gps_msg{};

				fillGpsMsgWithVehicleGpsPosData(gps_msg, gps);

				_ekf.setGpsData(gps_msg);

				_gps_time_usec = gps_msg.time_usec;
				_gps_alttitude_ellipsoid = gps.alt_ellipsoid;
			}
		}

		if (_airspeed_sub.updated()) {
			airspeed_s airspeed;

			if (_airspeed_sub.copy(&airspeed)) {
				// only set airspeed data if condition for airspeed fusion are met
				if ((_param_ekf2_arsp_thr.get() > FLT_EPSILON) && (airspeed.true_airspeed_m_s > _param_ekf2_arsp_thr.get())) {

					airspeedSample airspeed_sample {};
					airspeed_sample.time_us = airspeed.timestamp;
					airspeed_sample.eas2tas = airspeed.true_airspeed_m_s / airspeed.indicated_airspeed_m_s;
					airspeed_sample.true_airspeed = airspeed.true_airspeed_m_s;
					_ekf.setAirspeedData(airspeed_sample);
				}

				ekf2_timestamps.airspeed_timestamp_rel = (int16_t)((int64_t)airspeed.timestamp / 100 -
						(int64_t)ekf2_timestamps.timestamp / 100);
			}
		}

		bool new_optical_flow_data_received = false;
		optical_flow_s optical_flow;

		if (_optical_flow_sub.update(&optical_flow)) {
			flowSample flow {};
			// NOTE: the EKF uses the reverse sign convention to the flow sensor. EKF assumes positive LOS rate
			// is produced by a RH rotation of the image about the sensor axis.
			flow.flow_xy_rad(0) = -optical_flow.pixel_flow_x_integral;
			flow.flow_xy_rad(1) = -optical_flow.pixel_flow_y_integral;
			flow.gyro_xyz(0) = -optical_flow.gyro_x_rate_integral;
			flow.gyro_xyz(1) = -optical_flow.gyro_y_rate_integral;
			flow.gyro_xyz(2) = -optical_flow.gyro_z_rate_integral;
			flow.quality = optical_flow.quality;
			flow.dt = 1e-6f * (float)optical_flow.integration_timespan;
			flow.time_us = optical_flow.timestamp;

			if (PX4_ISFINITE(optical_flow.pixel_flow_y_integral) &&
			    PX4_ISFINITE(optical_flow.pixel_flow_x_integral) &&
			    flow.dt < 1) {

				_ekf.setOpticalFlowData(flow);

				new_optical_flow_data_received = true;
			}

			// Save sensor limits reported by the optical flow sensor
			_ekf.set_optical_flow_limits(optical_flow.max_flow_rate, optical_flow.min_ground_distance,
						     optical_flow.max_ground_distance);

			ekf2_timestamps.optical_flow_timestamp_rel = (int16_t)((int64_t)optical_flow.timestamp / 100 -
					(int64_t)ekf2_timestamps.timestamp / 100);
		}

		if (_range_finder_sub_index >= 0) {

			if (_distance_sensor_subs[_range_finder_sub_index].updated()) {
				distance_sensor_s range_finder;

				if (_distance_sensor_subs[_range_finder_sub_index].copy(&range_finder)) {
					rangeSample range_sample {};
					range_sample.rng = range_finder.current_distance;
					range_sample.quality = range_finder.signal_quality;
					range_sample.time_us = range_finder.timestamp;
					_ekf.setRangeData(range_sample);

					// Save sensor limits reported by the rangefinder
					_ekf.set_rangefinder_limits(range_finder.min_distance, range_finder.max_distance);

					ekf2_timestamps.distance_sensor_timestamp_rel = (int16_t)((int64_t)range_finder.timestamp / 100 -
							(int64_t)ekf2_timestamps.timestamp / 100);
				}
			}

		} else {
			_range_finder_sub_index = getRangeSubIndex();
		}

		// get external vision data
		bool new_ev_data_received = false;
		vehicle_odometry_s ev_odom;

		if (_ev_odom_sub.update(&ev_odom)) {
			extVisionSample ev_data{};

			// if error estimates are unavailable, use parameter defined defaults

			// check for valid velocity data
			if (PX4_ISFINITE(ev_odom.vx) && PX4_ISFINITE(ev_odom.vy) && PX4_ISFINITE(ev_odom.vz)) {
				ev_data.vel(0) = ev_odom.vx;
				ev_data.vel(1) = ev_odom.vy;
				ev_data.vel(2) = ev_odom.vz;

				if (ev_odom.velocity_frame == vehicle_odometry_s::BODY_FRAME_FRD) {
					ev_data.vel_frame = estimator::BODY_FRAME_FRD;

				} else {
					ev_data.vel_frame = estimator::LOCAL_FRAME_FRD;
				}

				// velocity measurement error from ev_data or parameters
				float param_evv_noise_var = sq(_param_ekf2_evv_noise.get());

				if (!_param_ekf2_ev_noise_md.get() && PX4_ISFINITE(ev_odom.velocity_covariance[ev_odom.COVARIANCE_MATRIX_VX_VARIANCE])
				    && PX4_ISFINITE(ev_odom.velocity_covariance[ev_odom.COVARIANCE_MATRIX_VY_VARIANCE])
				    && PX4_ISFINITE(ev_odom.velocity_covariance[ev_odom.COVARIANCE_MATRIX_VZ_VARIANCE])) {
					ev_data.velCov(0, 0) = ev_odom.velocity_covariance[ev_odom.COVARIANCE_MATRIX_VX_VARIANCE];
					ev_data.velCov(0, 1) = ev_data.velCov(1, 0) = ev_odom.velocity_covariance[1];
					ev_data.velCov(0, 2) = ev_data.velCov(2, 0) = ev_odom.velocity_covariance[2];
					ev_data.velCov(1, 1) = ev_odom.velocity_covariance[ev_odom.COVARIANCE_MATRIX_VY_VARIANCE];
					ev_data.velCov(1, 2) = ev_data.velCov(2, 1) = ev_odom.velocity_covariance[7];
					ev_data.velCov(2, 2) = ev_odom.velocity_covariance[ev_odom.COVARIANCE_MATRIX_VZ_VARIANCE];

				} else {
					ev_data.velCov = matrix::eye<float, 3>() * param_evv_noise_var;
				}
			}

			// check for valid position data
			if (PX4_ISFINITE(ev_odom.x) && PX4_ISFINITE(ev_odom.y) && PX4_ISFINITE(ev_odom.z)) {
				ev_data.pos(0) = ev_odom.x;
				ev_data.pos(1) = ev_odom.y;
				ev_data.pos(2) = ev_odom.z;

				float param_evp_noise_var = sq(_param_ekf2_evp_noise.get());

				// position measurement error from ev_data or parameters
				if (!_param_ekf2_ev_noise_md.get() && PX4_ISFINITE(ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_X_VARIANCE])
				    && PX4_ISFINITE(ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_Y_VARIANCE])
				    && PX4_ISFINITE(ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_Z_VARIANCE])) {
					ev_data.posVar(0) = fmaxf(param_evp_noise_var, ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_X_VARIANCE]);
					ev_data.posVar(1) = fmaxf(param_evp_noise_var, ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_Y_VARIANCE]);
					ev_data.posVar(2) = fmaxf(param_evp_noise_var, ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_Z_VARIANCE]);

				} else {
					ev_data.posVar.setAll(param_evp_noise_var);
				}
			}

			// check for valid orientation data
			if (PX4_ISFINITE(ev_odom.q[0])) {
				ev_data.quat = Quatf(ev_odom.q);

				// orientation measurement error from ev_data or parameters
				float param_eva_noise_var = sq(_param_ekf2_eva_noise.get());

				if (!_param_ekf2_ev_noise_md.get() && PX4_ISFINITE(ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_YAW_VARIANCE])) {
					ev_data.angVar = fmaxf(param_eva_noise_var, ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_YAW_VARIANCE]);

				} else {
					ev_data.angVar = param_eva_noise_var;
				}
			}

			// use timestamp from external computer, clocks are synchronized when using MAVROS
			ev_data.time_us = ev_odom.timestamp_sample;
			_ekf.setExtVisionData(ev_data);

			ekf2_timestamps.visual_odometry_timestamp_rel = (int16_t)((int64_t)ev_odom.timestamp / 100 -
					(int64_t)ekf2_timestamps.timestamp / 100);

			new_ev_data_received = true;
		}

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_ekf.set_in_air_status(!vehicle_land_detected.landed);
				_landed = vehicle_land_detected.landed;
				_in_ground_effect = vehicle_land_detected.in_ground_effect;
			}
		}

		// use the landing target pose estimate as another source of velocity data
		if (_landing_target_pose_sub.updated()) {
			landing_target_pose_s landing_target_pose;

			if (_landing_target_pose_sub.copy(&landing_target_pose)) {
				// we can only use the landing target if it has a fixed position and  a valid velocity estimate
				if (landing_target_pose.is_static && landing_target_pose.rel_vel_valid) {
					// velocity of vehicle relative to target has opposite sign to target relative to vehicle
					auxVelSample auxvel_sample {};
					auxvel_sample.vel = Vector3f{-landing_target_pose.vx_rel, -landing_target_pose.vy_rel, 0.0f};
					auxvel_sample.velVar = Vector3f{landing_target_pose.cov_vx_rel, landing_target_pose.cov_vy_rel, 0.0f};
					auxvel_sample.time_us = landing_target_pose.timestamp;
					_ekf.setAuxVelData(auxvel_sample);
				}
			}
		}

		// run the EKF update and output
		perf_begin(_ekf_update_perf);
		const bool ekf_updated = _ekf.update();
		perf_end(_ekf_update_perf);

		// integrate time to monitor time slippage
		if (_start_time_us == 0) {
			_start_time_us = now;
			_last_time_slip_us = 0;

		} else if (_start_time_us > 0) {
			_integrated_time_us += imu_dt;
			_last_time_slip_us = (now - _start_time_us) - _integrated_time_us;
		}

		if (ekf_updated) {

			vehicle_local_position_s lpos{};
			_ekf.get_ekf_lpos_accuracy(&lpos.eph, &lpos.epv);

			filter_control_status_u control_status;
			_ekf.get_control_mode(&control_status.value);

			// only publish position after successful alignment
			if (control_status.flags.tilt_align) {
				// generate vehicle local position data
				lpos.timestamp_sample = imu_sample_new.time_us;

				// Position of body origin in local NED frame
				const Vector3f position = _ekf.getPosition();
				lpos.x = position(0);
				lpos.y = position(1);
				lpos.z = position(2);

				// Velocity of body origin in local NED frame (m/s)
				const Vector3f velocity = _ekf.getVelocity();
				lpos.vx = velocity(0);
				lpos.vy = velocity(1);
				lpos.vz = velocity(2);

				// vertical position time derivative (m/s)
				lpos.z_deriv = _ekf.getVerticalPositionDerivative();

				// Acceleration of body origin in local frame
				Vector3f vel_deriv = _ekf.getVelocityDerivative();
				lpos.ax = vel_deriv(0);
				lpos.ay = vel_deriv(1);
				lpos.az = vel_deriv(2);

				// TODO: better status reporting
				lpos.xy_valid = _ekf.local_position_is_valid() && !_preflt_checker.hasHorizFailed();
				lpos.z_valid = !_preflt_checker.hasVertFailed();
				lpos.v_xy_valid = _ekf.local_position_is_valid() && !_preflt_checker.hasHorizFailed();
				lpos.v_z_valid = !_preflt_checker.hasVertFailed();

				// Position of local NED origin in GPS / WGS84 frame
				map_projection_reference_s ekf_origin;
				uint64_t origin_time;

				// true if position (x,y,z) has a valid WGS-84 global reference (ref_lat, ref_lon, alt)
				const bool ekf_origin_valid = _ekf.get_ekf_origin(&origin_time, &ekf_origin, &lpos.ref_alt);
				lpos.xy_global = ekf_origin_valid;
				lpos.z_global = ekf_origin_valid;

				if (ekf_origin_valid && (origin_time > lpos.ref_timestamp)) {
					lpos.ref_timestamp = origin_time;
					lpos.ref_lat = ekf_origin.lat_rad * 180.0 / M_PI; // Reference point latitude in degrees
					lpos.ref_lon = ekf_origin.lon_rad * 180.0 / M_PI; // Reference point longitude in degrees
				}

				// The rotation of the tangent plane vs. geographical north
				const Quatf q = _ekf.getQuaternion();

				Quatf delta_q_reset;
				_ekf.get_quat_reset(&delta_q_reset(0), &lpos.heading_reset_counter);

				lpos.heading = Eulerf(q).psi();
				lpos.delta_heading = Eulerf(delta_q_reset).psi();

				lpos.dist_bottom_valid = _ekf.isTerrainEstimateValid();

				float terrain_vpos = _ekf.getTerrainVertPos();
				lpos.dist_bottom = terrain_vpos - lpos.z; // Distance to bottom surface (ground) in meters

				// constrain the distance to ground to _rng_gnd_clearance
				if (lpos.dist_bottom < _param_ekf2_min_rng.get()) {
					lpos.dist_bottom = _param_ekf2_min_rng.get();
				}

				if (!_had_valid_terrain) {
					_had_valid_terrain = lpos.dist_bottom_valid;
				}

				// only consider ground effect if compensation is configured and the vehicle is armed (props spinning)
				if ((_param_ekf2_gnd_eff_dz.get() > 0.0f) && _armed) {
					// set ground effect flag if vehicle is closer than a specified distance to the ground
					if (lpos.dist_bottom_valid) {
						_ekf.set_gnd_effect_flag(lpos.dist_bottom < _param_ekf2_gnd_max_hgt.get());

						// if we have no valid terrain estimate and never had one then use ground effect flag from land detector
						// _had_valid_terrain is used to make sure that we don't fall back to using this option
						// if we temporarily lose terrain data due to the distance sensor getting out of range

					} else if (!_had_valid_terrain) {
						// update ground effect flag based on land detector state
						_ekf.set_gnd_effect_flag(_in_ground_effect);
					}

				} else {
					_ekf.set_gnd_effect_flag(false);
				}

				_ekf.get_ekf_vel_accuracy(&lpos.evh, &lpos.evv);

				// get state reset information of position and velocity
				_ekf.get_posD_reset(&lpos.delta_z, &lpos.z_reset_counter);
				_ekf.get_velD_reset(&lpos.delta_vz, &lpos.vz_reset_counter);
				_ekf.get_posNE_reset(&lpos.delta_xy[0], &lpos.xy_reset_counter);
				_ekf.get_velNE_reset(&lpos.delta_vxy[0], &lpos.vxy_reset_counter);

				// get control limit information
				_ekf.get_ekf_ctrl_limits(&lpos.vxy_max, &lpos.vz_max, &lpos.hagl_min, &lpos.hagl_max);

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

				if (!PX4_ISFINITE(lpos.hagl_max)) {
					lpos.hagl_max = INFINITY;
				}

				// publish vehicle local position data
				lpos.timestamp = _replay_mode ? now : hrt_absolute_time();
				_local_position_pub.publish(lpos);

				// publish vehicle_odometry
				publish_odometry(now, imu_sample_new, lpos);

				// publish vehicle_global_position if valid
				if (_ekf.global_position_is_valid() && !_preflt_checker.hasFailed()) {
					// only publish if position has changed by at least 1 mm (map_projection_reproject is relatively expensive)
					if ((_last_local_position_for_gpos - position).longerThan(0.001f)) {

						// generate and publish global position data
						vehicle_global_position_s global_pos{};
						global_pos.timestamp_sample = imu_sample_new.time_us;

						map_projection_reproject(&ekf_origin, lpos.x, lpos.y, &global_pos.lat, &global_pos.lon);

						global_pos.lat_lon_reset_counter = lpos.xy_reset_counter;

						global_pos.alt = -lpos.z + lpos.ref_alt; // Altitude AMSL in meters
						global_pos.alt_ellipsoid = filter_altitude_ellipsoid(global_pos.alt);

						// global altitude has opposite sign of local down position
						global_pos.delta_alt = -lpos.delta_z;

						_ekf.get_ekf_gpos_accuracy(&global_pos.eph, &global_pos.epv);

						global_pos.terrain_alt_valid = lpos.dist_bottom_valid;

						if (global_pos.terrain_alt_valid) {
							global_pos.terrain_alt = lpos.ref_alt - terrain_vpos; // Terrain altitude in m, WGS84

						} else {
							global_pos.terrain_alt = 0.0f; // Terrain altitude in m, WGS84
						}

						global_pos.dead_reckoning = _ekf.inertial_dead_reckoning(); // True if this position is estimated through dead-reckoning
						global_pos.timestamp = _replay_mode ? now : hrt_absolute_time();
						_global_position_pub.publish(global_pos);

						_last_local_position_for_gpos = position;
					}
				}
			}

			// publish estimator states
			estimator_states_s states;
			states.timestamp_sample = imu_sample_new.time_us;
			states.n_states = 24;
			_ekf.getStateAtFusionHorizonAsVector().copyTo(states.states);
			_ekf.covariances_diagonal().copyTo(states.covariances);
			states.timestamp = _replay_mode ? now : hrt_absolute_time();
			_estimator_states_pub.publish(states);

			// publish estimator status
			estimator_status_s status{};
			status.timestamp_sample = imu_sample_new.time_us;
			_ekf.getOutputTrackingError().copyTo(status.output_tracking_error);
			_ekf.get_gps_check_status(&status.gps_check_fail_flags);
			// only report enabled GPS check failures (the param indexes are shifted by 1 bit, because they don't include
			// the GPS Fix bit, which is always checked)
			status.gps_check_fail_flags &= ((uint16_t)_params->gps_check_mask << 1) | 1;
			status.control_mode_flags = control_status.value;
			_ekf.get_filter_fault_status(&status.filter_fault_flags);
			_ekf.get_innovation_test_status(status.innovation_check_flags, status.mag_test_ratio,
							status.vel_test_ratio, status.pos_test_ratio,
							status.hgt_test_ratio, status.tas_test_ratio,
							status.hagl_test_ratio, status.beta_test_ratio);

			status.pos_horiz_accuracy = lpos.eph;
			status.pos_vert_accuracy = lpos.epv;
			_ekf.get_ekf_soln_status(&status.solution_status_flags);
			_ekf.getImuVibrationMetrics().copyTo(status.vibe);
			status.time_slip = _last_time_slip_us * 1e-6f;
			status.pre_flt_fail_innov_heading = _preflt_checker.hasHeadingFailed();
			status.pre_flt_fail_innov_vel_horiz = _preflt_checker.hasHorizVelFailed();
			status.pre_flt_fail_innov_vel_vert = _preflt_checker.hasVertVelFailed();
			status.pre_flt_fail_innov_height = _preflt_checker.hasHeightFailed();
			status.pre_flt_fail_mag_field_disturbed = control_status.flags.mag_field_disturbed;
			status.accel_device_id = _device_id_accel;
			status.baro_device_id = _device_id_baro;
			status.gyro_device_id = _device_id_gyro;
			status.mag_device_id = _device_id_mag;
			status.timestamp = _replay_mode ? now : hrt_absolute_time();
			_estimator_status_pub.publish(status);

			// estimator_sensor_bias
			if (status.filter_fault_flags == 0) {
				// publish all corrected sensor readings and bias estimates after mag calibration is updated above
				estimator_sensor_bias_s bias;
				bias.timestamp_sample = imu_sample_new.time_us;

				// take device ids from sensor_selection_s if not using specific vehicle_imu_s
				bias.gyro_device_id = _device_id_gyro;
				bias.accel_device_id = _device_id_accel;
				bias.mag_device_id = _device_id_mag;

				_ekf.getGyroBias().copyTo(bias.gyro_bias);
				_ekf.getAccelBias().copyTo(bias.accel_bias);

				bias.mag_bias[0] = _last_valid_mag_cal[0];
				bias.mag_bias[1] = _last_valid_mag_cal[1];
				bias.mag_bias[2] = _last_valid_mag_cal[2];

				bias.gyro_bias_variance[0] = states.covariances[10];
				bias.gyro_bias_variance[1] = states.covariances[11];
				bias.gyro_bias_variance[2] = states.covariances[12];
				bias.accel_bias_variance[0] = states.covariances[13];
				bias.accel_bias_variance[1] = states.covariances[14];
				bias.accel_bias_variance[2] = states.covariances[15];
				bias.mag_bias_variance[0] = states.covariances[19];
				bias.mag_bias_variance[1] = states.covariances[20];
				bias.mag_bias_variance[2] = states.covariances[21];

				bias.timestamp = _replay_mode ? now : hrt_absolute_time();
				_estimator_sensor_bias_pub.publish(bias);
			}

			// publish GPS drift data only when updated to minimise overhead
			float gps_drift[3];
			bool blocked;

			if (_ekf.get_gps_drift_metrics(gps_drift, &blocked)) {
				ekf_gps_drift_s drift_data;
				drift_data.hpos_drift_rate = gps_drift[0];
				drift_data.vpos_drift_rate = gps_drift[1];
				drift_data.hspd = gps_drift[2];
				drift_data.blocked = blocked;
				drift_data.timestamp = _replay_mode ? now : hrt_absolute_time();

				_ekf_gps_drift_pub.publish(drift_data);
			}

			{
				/* Check and save learned magnetometer bias estimates */

				// Check if conditions are OK for learning of magnetometer bias values
				if (!_landed && _armed &&
				    !status.filter_fault_flags && // there are no filter faults
				    control_status.flags.mag_3D) { // the EKF is operating in the correct mode

					if (_last_magcal_us == 0) {
						_last_magcal_us = now;

					} else {
						_total_cal_time_us += now - _last_magcal_us;
						_last_magcal_us = now;
					}

				} else if (status.filter_fault_flags != 0) {
					// if a filter fault has occurred, assume previous learning was invalid and do not
					// count it towards total learning time.
					_total_cal_time_us = 0;

					for (bool &cal_available : _valid_cal_available) {
						cal_available = false;
					}

				} else {
					// conditions are NOT OK for learning magnetometer bias, reset timestamp
					// but keep the accumulated calibration time
					_last_magcal_us = now;
				}

				// Start checking mag bias estimates when we have accumulated sufficient calibration time
				if (_total_cal_time_us > 30_s) {
					// we have sufficient accumulated valid flight time to form a reliable bias estimate
					// check that the state variance for each axis is within a range indicating filter convergence
					const float max_var_allowed = 100.0f * _param_ekf2_magb_vref.get();
					const float min_var_allowed = 0.01f * _param_ekf2_magb_vref.get();

					// Declare all bias estimates invalid if any variances are out of range
					bool all_estimates_invalid = false;

					for (uint8_t axis_index = 0; axis_index <= 2; axis_index++) {
						if (states.covariances[axis_index + 19] < min_var_allowed
						    || states.covariances[axis_index + 19] > max_var_allowed) {
							all_estimates_invalid = true;
						}
					}

					// Store valid estimates and their associated variances
					if (!all_estimates_invalid) {
						for (uint8_t axis_index = 0; axis_index <= 2; axis_index++) {
							_last_valid_mag_cal[axis_index] = states.states[axis_index + 19];
							_valid_cal_available[axis_index] = true;
							_last_valid_variance[axis_index] = states.covariances[axis_index + 19];
						}
					}
				}

				// Check and save the last valid calibration when we are disarmed
				if (!_armed && _standby && (status.filter_fault_flags == 0)) {
					update_mag_bias(_param_ekf2_magbias_x, 0);
					update_mag_bias(_param_ekf2_magbias_y, 1);
					update_mag_bias(_param_ekf2_magbias_z, 2);

					// reset to prevent data being saved too frequently
					_total_cal_time_us = 0;
				}
			}

			publish_wind_estimate(now);

			publish_yaw_estimator_status(now);

			if (!_mag_decl_saved && _standby) {
				_mag_decl_saved = update_mag_decl(_param_ekf2_mag_decl);
			}

			{
				// publish estimator innovation data
				estimator_innovations_s innovations;
				innovations.timestamp_sample = imu_sample_new.time_us;
				_ekf.getGpsVelPosInnov(&innovations.gps_hvel[0], innovations.gps_vvel, &innovations.gps_hpos[0],
						       innovations.gps_vpos);
				_ekf.getEvVelPosInnov(&innovations.ev_hvel[0], innovations.ev_vvel, &innovations.ev_hpos[0], innovations.ev_vpos);
				_ekf.getBaroHgtInnov(innovations.baro_vpos);
				_ekf.getRngHgtInnov(innovations.rng_vpos);
				_ekf.getAuxVelInnov(&innovations.aux_hvel[0]);
				_ekf.getFlowInnov(&innovations.flow[0]);
				_ekf.getHeadingInnov(innovations.heading);
				_ekf.getMagInnov(innovations.mag_field);
				_ekf.getDragInnov(&innovations.drag[0]);
				_ekf.getAirspeedInnov(innovations.airspeed);
				_ekf.getBetaInnov(innovations.beta);
				_ekf.getHaglInnov(innovations.hagl);
				// Not yet supported
				innovations.aux_vvel = NAN;
				innovations.fake_hpos[0] = innovations.fake_hpos[1] = innovations.fake_vpos = NAN;
				innovations.fake_hvel[0] = innovations.fake_hvel[1] = innovations.fake_vvel = NAN;

				// publish estimator innovation variance data
				estimator_innovations_s innovation_var;
				innovation_var.timestamp_sample = imu_sample_new.time_us;
				_ekf.getGpsVelPosInnovVar(&innovation_var.gps_hvel[0], innovation_var.gps_vvel, &innovation_var.gps_hpos[0],
							  innovation_var.gps_vpos);
				_ekf.getEvVelPosInnovVar(&innovation_var.ev_hvel[0], innovation_var.ev_vvel, &innovation_var.ev_hpos[0],
							 innovation_var.ev_vpos);
				_ekf.getBaroHgtInnovVar(innovation_var.baro_vpos);
				_ekf.getRngHgtInnovVar(innovation_var.rng_vpos);
				_ekf.getAuxVelInnovVar(&innovation_var.aux_hvel[0]);
				_ekf.getFlowInnovVar(&innovation_var.flow[0]);
				_ekf.getHeadingInnovVar(innovation_var.heading);
				_ekf.getMagInnovVar(&innovation_var.mag_field[0]);
				_ekf.getDragInnovVar(&innovation_var.drag[0]);
				_ekf.getAirspeedInnovVar(innovation_var.airspeed);
				_ekf.getBetaInnovVar(innovation_var.beta);
				_ekf.getHaglInnovVar(innovation_var.hagl);
				// Not yet supported
				innovation_var.aux_vvel = NAN;
				innovation_var.fake_hpos[0] = innovation_var.fake_hpos[1] = innovation_var.fake_vpos = NAN;
				innovation_var.fake_hvel[0] = innovation_var.fake_hvel[1] = innovation_var.fake_vvel = NAN;


				// publish estimator innovation test ratio data
				estimator_innovations_s test_ratios;
				test_ratios.timestamp_sample = imu_sample_new.time_us;
				_ekf.getGpsVelPosInnovRatio(test_ratios.gps_hvel[0], test_ratios.gps_vvel, test_ratios.gps_hpos[0],
							    test_ratios.gps_vpos);
				_ekf.getEvVelPosInnovRatio(test_ratios.ev_hvel[0], test_ratios.ev_vvel, test_ratios.ev_hpos[0],
							   test_ratios.ev_vpos);
				_ekf.getBaroHgtInnovRatio(test_ratios.baro_vpos);
				_ekf.getRngHgtInnovRatio(test_ratios.rng_vpos);
				_ekf.getAuxVelInnovRatio(test_ratios.aux_hvel[0]);
				_ekf.getFlowInnovRatio(test_ratios.flow[0]);
				_ekf.getHeadingInnovRatio(test_ratios.heading);
				_ekf.getMagInnovRatio(test_ratios.mag_field[0]);
				_ekf.getDragInnovRatio(&test_ratios.drag[0]);
				_ekf.getAirspeedInnovRatio(test_ratios.airspeed);
				_ekf.getBetaInnovRatio(test_ratios.beta);
				_ekf.getHaglInnovRatio(test_ratios.hagl);
				// Not yet supported
				test_ratios.aux_vvel = NAN;
				test_ratios.fake_hpos[0] = test_ratios.fake_hpos[1] = test_ratios.fake_vpos = NAN;
				test_ratios.fake_hvel[0] = test_ratios.fake_hvel[1] = test_ratios.fake_vvel = NAN;

				// calculate noise filtered velocity innovations which are used for pre-flight checking
				if (_standby) {
					float dt_seconds = imu_sample_new.delta_ang_dt;
					runPreFlightChecks(dt_seconds, control_status, innovations, _can_observe_heading_in_flight);

				} else {
					resetPreFlightChecks();
				}

				innovations.timestamp = _replay_mode ? now : hrt_absolute_time();
				_estimator_innovations_pub.publish(innovations);

				innovation_var.timestamp = _replay_mode ? now : hrt_absolute_time();
				_estimator_innovation_variances_pub.publish(innovation_var);

				test_ratios.timestamp = _replay_mode ? now : hrt_absolute_time();
				_estimator_innovation_test_ratios_pub.publish(test_ratios);
			}
		}

		if (new_optical_flow_data_received) {
			publish_estimator_optical_flow_vel(now);
		}

		// publish external visual odometry after fixed frame alignment if new odometry is received
		if (new_ev_data_received) {
			const Quatf quat_ev2ekf = _ekf.getVisionAlignmentQuaternion(); // rotates from EV to EKF navigation frame
			const Dcmf ev_rot_mat(quat_ev2ekf);

			vehicle_odometry_s aligned_ev_odom{ev_odom};

			// Rotate external position and velocity into EKF navigation frame
			const Vector3f aligned_pos = ev_rot_mat * Vector3f(ev_odom.x, ev_odom.y, ev_odom.z);
			aligned_ev_odom.x = aligned_pos(0);
			aligned_ev_odom.y = aligned_pos(1);
			aligned_ev_odom.z = aligned_pos(2);

			switch (ev_odom.velocity_frame) {
			case vehicle_odometry_s::BODY_FRAME_FRD: {
					const Vector3f aligned_vel = Dcmf(_ekf.getQuaternion()) * Vector3f(ev_odom.vx, ev_odom.vy, ev_odom.vz);
					aligned_ev_odom.vx = aligned_vel(0);
					aligned_ev_odom.vy = aligned_vel(1);
					aligned_ev_odom.vz = aligned_vel(2);
					break;
				}

			case vehicle_odometry_s::LOCAL_FRAME_FRD: {
					const Vector3f aligned_vel = ev_rot_mat * Vector3f(ev_odom.vx, ev_odom.vy, ev_odom.vz);
					aligned_ev_odom.vx = aligned_vel(0);
					aligned_ev_odom.vy = aligned_vel(1);
					aligned_ev_odom.vz = aligned_vel(2);
					break;
				}
			}

			aligned_ev_odom.velocity_frame = vehicle_odometry_s::LOCAL_FRAME_NED;

			// Compute orientation in EKF navigation frame
			Quatf ev_quat_aligned = quat_ev2ekf * Quatf(ev_odom.q) ;
			ev_quat_aligned.normalize();

			ev_quat_aligned.copyTo(aligned_ev_odom.q);
			quat_ev2ekf.copyTo(aligned_ev_odom.q_offset);

			_estimator_visual_odometry_aligned_pub.publish(aligned_ev_odom);
		}

		// publish ekf2_timestamps
		_ekf2_timestamps_pub.publish(ekf2_timestamps);

		if (!_multi_mode) {
			if (_lockstep_component == -1) {
				_lockstep_component = px4_lockstep_register_component();
			}

			px4_lockstep_progress(_lockstep_component);
		}
	}
}

void EKF2::fillGpsMsgWithVehicleGpsPosData(gps_message &msg, const vehicle_gps_position_s &data)
{
	msg.time_usec = data.timestamp;
	msg.lat = data.lat;
	msg.lon = data.lon;
	msg.alt = data.alt;
	msg.yaw = data.heading;
	msg.yaw_offset = data.heading_offset;
	msg.fix_type = data.fix_type;
	msg.eph = data.eph;
	msg.epv = data.epv;
	msg.sacc = data.s_variance_m_s;
	msg.vel_m_s = data.vel_m_s;
	msg.vel_ned(0) = data.vel_n_m_s;
	msg.vel_ned(1) = data.vel_e_m_s;
	msg.vel_ned(2) = data.vel_d_m_s;
	msg.vel_ned_valid = data.vel_ned_valid;
	msg.nsats = data.satellites_used;
	msg.pdop = sqrtf(data.hdop * data.hdop + data.vdop * data.vdop);
}

void EKF2::runPreFlightChecks(const float dt,
			      const filter_control_status_u &control_status,
			      const estimator_innovations_s &innov,
			      const bool can_observe_heading_in_flight)
{
	_preflt_checker.setVehicleCanObserveHeadingInFlight(can_observe_heading_in_flight);
	_preflt_checker.setUsingGpsAiding(control_status.flags.gps);
	_preflt_checker.setUsingFlowAiding(control_status.flags.opt_flow);
	_preflt_checker.setUsingEvPosAiding(control_status.flags.ev_pos);
	_preflt_checker.setUsingEvVelAiding(control_status.flags.ev_vel);

	_preflt_checker.update(dt, innov);
}

void EKF2::resetPreFlightChecks()
{
	_preflt_checker.reset();
}

int EKF2::getRangeSubIndex()
{
	for (unsigned i = 0; i < _distance_sensor_subs.size(); i++) {
		distance_sensor_s report;

		if (_distance_sensor_subs[i].update(&report)) {
			// only use the first instace which has the correct orientation
			if (report.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING) {
				PX4_INFO("Found range finder with instance %d", i);
				return i;
			}
		}
	}

	return -1;
}

void EKF2::publish_attitude(const hrt_abstime &timestamp)
{
	if (_ekf.attitude_valid()) {
		// generate vehicle attitude quaternion data
		vehicle_attitude_s att;
		att.timestamp_sample = timestamp;
		const Quatf q{_ekf.calculate_quaternion()};
		q.copyTo(att.q);

		_ekf.get_quat_reset(&att.delta_q_reset[0], &att.quat_reset_counter);
		att.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_attitude_pub.publish(att);

	}  else if (_replay_mode) {
		// in replay mode we have to tell the replay module not to wait for an update
		// we do this by publishing an attitude with zero timestamp
		vehicle_attitude_s att{};
		_attitude_pub.publish(att);
	}
}

void EKF2::publish_odometry(const hrt_abstime &timestamp, const imuSample &imu, const vehicle_local_position_s &lpos)
{
	// generate vehicle odometry data
	vehicle_odometry_s odom{};
	odom.timestamp_sample = imu.time_us;

	odom.local_frame = vehicle_odometry_s::LOCAL_FRAME_NED;

	// Vehicle odometry position
	odom.x = lpos.x;
	odom.y = lpos.y;
	odom.z = lpos.z;

	// Vehicle odometry linear velocity
	odom.velocity_frame = vehicle_odometry_s::LOCAL_FRAME_FRD;
	odom.vx = lpos.vx;
	odom.vy = lpos.vy;
	odom.vz = lpos.vz;

	// Vehicle odometry quaternion
	_ekf.getQuaternion().copyTo(odom.q);

	// Vehicle odometry angular rates
	const Vector3f gyro_bias = _ekf.getGyroBias();
	const Vector3f rates(imu.delta_ang / imu.delta_ang_dt);
	odom.rollspeed = rates(0) - gyro_bias(0);
	odom.pitchspeed = rates(1) - gyro_bias(1);
	odom.yawspeed = rates(2) - gyro_bias(2);

	// get the covariance matrix size
	const size_t POS_URT_SIZE = sizeof(odom.pose_covariance) / sizeof(odom.pose_covariance[0]);
	const size_t VEL_URT_SIZE = sizeof(odom.velocity_covariance) / sizeof(odom.velocity_covariance[0]);

	// Get covariances to vehicle odometry
	float covariances[24];
	_ekf.covariances_diagonal().copyTo(covariances);

	// initially set pose covariances to 0
	for (size_t i = 0; i < POS_URT_SIZE; i++) {
		odom.pose_covariance[i] = 0.0;
	}

	// set the position variances
	odom.pose_covariance[odom.COVARIANCE_MATRIX_X_VARIANCE] = covariances[7];
	odom.pose_covariance[odom.COVARIANCE_MATRIX_Y_VARIANCE] = covariances[8];
	odom.pose_covariance[odom.COVARIANCE_MATRIX_Z_VARIANCE] = covariances[9];

	// TODO: implement propagation from quaternion covariance to Euler angle covariance
	// by employing the covariance law

	// initially set velocity covariances to 0
	for (size_t i = 0; i < VEL_URT_SIZE; i++) {
		odom.velocity_covariance[i] = 0.0;
	}

	// set the linear velocity variances
	odom.velocity_covariance[odom.COVARIANCE_MATRIX_VX_VARIANCE] = covariances[4];
	odom.velocity_covariance[odom.COVARIANCE_MATRIX_VY_VARIANCE] = covariances[5];
	odom.velocity_covariance[odom.COVARIANCE_MATRIX_VZ_VARIANCE] = covariances[6];

	// publish vehicle odometry data
	odom.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_odometry_pub.publish(odom);
}

void EKF2::publish_yaw_estimator_status(const hrt_abstime &timestamp)
{
	yaw_estimator_status_s yaw_est_test_data{};

	static_assert(sizeof(yaw_estimator_status_s::yaw) / sizeof(float) == N_MODELS_EKFGSF,
		      "yaw_estimator_status_s::yaw wrong size");

	if (_ekf.getDataEKFGSF(&yaw_est_test_data.yaw_composite, &yaw_est_test_data.yaw_variance,
			       &yaw_est_test_data.yaw[0],
			       &yaw_est_test_data.innov_vn[0], &yaw_est_test_data.innov_ve[0],
			       &yaw_est_test_data.weight[0])) {

		yaw_est_test_data.timestamp_sample = timestamp;
		yaw_est_test_data.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

		_yaw_est_pub.publish(yaw_est_test_data);
	}
}

void EKF2::publish_wind_estimate(const hrt_abstime &timestamp)
{
	if (_ekf.get_wind_status()) {
		// Publish wind estimate only if ekf declares them valid
		wind_estimate_s wind_estimate{};
		wind_estimate.timestamp_sample = timestamp;

		const Vector2f wind_vel = _ekf.getWindVelocity();
		const Vector2f wind_vel_var = _ekf.getWindVelocityVariance();
		_ekf.getAirspeedInnov(wind_estimate.tas_innov);
		_ekf.getAirspeedInnovVar(wind_estimate.tas_innov_var);
		_ekf.getBetaInnov(wind_estimate.beta_innov);
		_ekf.getBetaInnovVar(wind_estimate.beta_innov_var);

		wind_estimate.windspeed_north = wind_vel(0);
		wind_estimate.windspeed_east = wind_vel(1);
		wind_estimate.variance_north = wind_vel_var(0);
		wind_estimate.variance_east = wind_vel_var(1);
		wind_estimate.tas_scale = 0.0f; //leave at 0 as scale is not estimated in ekf
		wind_estimate.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

		_wind_pub.publish(wind_estimate);
	}
}

void EKF2::publish_estimator_optical_flow_vel(const hrt_abstime &timestamp)
{
	estimator_optical_flow_vel_s flow_vel{};
	flow_vel.timestamp_sample = timestamp;

	_ekf.getFlowVelBody().copyTo(flow_vel.vel_body);
	_ekf.getFlowVelNE().copyTo(flow_vel.vel_ne);
	_ekf.getFlowUncompensated().copyTo(flow_vel.flow_uncompensated_integral);
	_ekf.getFlowCompensated().copyTo(flow_vel.flow_compensated_integral);
	_ekf.getFlowGyro().copyTo(flow_vel.gyro_rate_integral);
	flow_vel.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

	_estimator_optical_flow_vel_pub.publish(flow_vel);
}

float EKF2::filter_altitude_ellipsoid(float amsl_hgt)
{
	float height_diff = static_cast<float>(_gps_alttitude_ellipsoid) * 1e-3f - amsl_hgt;

	if (_gps_alttitude_ellipsoid_previous_timestamp == 0) {

		_wgs84_hgt_offset = height_diff;
		_gps_alttitude_ellipsoid_previous_timestamp = _gps_time_usec;

	} else if (_gps_time_usec != _gps_alttitude_ellipsoid_previous_timestamp) {

		// apply a 10 second first order low pass filter to baro offset
		float dt = 1e-6f * (_gps_time_usec - _gps_alttitude_ellipsoid_previous_timestamp);
		_gps_alttitude_ellipsoid_previous_timestamp = _gps_time_usec;
		float offset_rate_correction = 0.1f * (height_diff - _wgs84_hgt_offset);
		_wgs84_hgt_offset += dt * constrain(offset_rate_correction, -0.1f, 0.1f);
	}

	return amsl_hgt + _wgs84_hgt_offset;
}

int EKF2::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int EKF2::task_spawn(int argc, char *argv[])
{
	bool success = false;
	bool replay_mode = false;

	if (argc > 1 && !strcmp(argv[1], "-r")) {
		PX4_INFO("replay mode enabled");
		replay_mode = true;
	}

#if !defined(CONSTRAINED_FLASH)
	bool multi_mode = false;
	int32_t imu_instances = 0;
	int32_t mag_instances = 0;

	int32_t sens_imu_mode = 1;
	param_get(param_find("SENS_IMU_MODE"), &sens_imu_mode);

	if (sens_imu_mode == 0) {
		// ekf selector requires SENS_IMU_MODE = 0
		multi_mode = true;

		// IMUs (1 - 4 supported)
		param_get(param_find("EKF2_MULTI_IMU"), &imu_instances);

		if (imu_instances < 1 || imu_instances > 4) {
			const int32_t imu_instances_limited = math::constrain(imu_instances, 1, 4);
			PX4_WARN("EKF2_MULTI_IMU limited %d -> %d", imu_instances, imu_instances_limited);
			param_set_no_notification(param_find("EKF2_MULTI_IMU"), &imu_instances_limited);
			imu_instances = imu_instances_limited;
		}

		int32_t sens_mag_mode = 1;
		param_get(param_find("SENS_MAG_MODE"), &sens_mag_mode);

		if (sens_mag_mode == 0) {
			param_get(param_find("EKF2_MULTI_MAG"), &mag_instances);

			// Mags (1 - 4 supported)
			if (mag_instances < 1 || mag_instances > 4) {
				const int32_t mag_instances_limited = math::constrain(mag_instances, 1, 4);
				PX4_WARN("EKF2_MULTI_MAG limited %d -> %d", mag_instances, mag_instances_limited);
				param_set_no_notification(param_find("EKF2_MULTI_MAG"), &mag_instances_limited);
				mag_instances = mag_instances_limited;
			}

		} else {
			mag_instances = 1;
		}
	}

	if (multi_mode) {
		// Start EKF2Selector if it's not already running
		if (_ekf2_selector.load() == nullptr) {
			EKF2Selector *inst = new EKF2Selector();

			if (inst) {
				_ekf2_selector.store(inst);
				inst->Start();

			} else {
				PX4_ERR("Failed to start EKF2 selector");
			}
		}

		const hrt_abstime time_started = hrt_absolute_time();
		const int multi_instances = math::min(imu_instances * mag_instances, (int)EKF2_MAX_INSTANCES);
		int multi_instances_allocated = 0;

		// allocate EKF2 instances until all found or arming
		uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};

		while ((multi_instances_allocated < multi_instances)
		       && (vehicle_status_sub.get().arming_state != vehicle_status_s::ARMING_STATE_ARMED)
		       && (hrt_elapsed_time(&time_started) < 30_s)) {

			vehicle_status_sub.update();

			for (uint8_t mag = 0; mag < mag_instances; mag++) {
				uORB::SubscriptionData<vehicle_magnetometer_s> vehicle_mag_sub{ORB_ID(vehicle_magnetometer), mag};

				for (uint8_t imu = 0; imu < imu_instances; imu++) {

					uORB::SubscriptionData<vehicle_imu_s> vehicle_imu_sub{ORB_ID(vehicle_imu), imu};
					vehicle_mag_sub.update();

					// Mag & IMU data must be valid, first mag can be ignored initially
					if ((vehicle_mag_sub.get().device_id != 0 || mag == 0)
					    && (vehicle_imu_sub.get().accel_device_id != 0)
					    && (vehicle_imu_sub.get().gyro_device_id != 0)) {

						const int instance = imu + mag * imu_instances;

						if (_objects[instance].load() == nullptr) {
							EKF2 *ekf2_inst = new EKF2(instance, px4::ins_instance_to_wq(imu), imu, mag, false);

							if (ekf2_inst) {
								PX4_INFO("starting instance %d, IMU:%d (%d), MAG:%d (%d)", instance,
									 imu, vehicle_imu_sub.get().accel_device_id,
									 mag, vehicle_mag_sub.get().device_id);

								_objects[instance].store(ekf2_inst);
								ekf2_inst->ScheduleNow();
								success = true;
								multi_instances_allocated++;

							} else {
								PX4_ERR("instance %d alloc failed", instance);
								px4_usleep(1000000);
								break;
							}
						}

					} else {
						px4_usleep(50000); // give the sensors extra time to start
						continue;
					}
				}
			}

			if (multi_instances_allocated < multi_instances) {
				px4_usleep(100000);
			}
		}

	}

#endif // !CONSTRAINED_FLASH

	else {
		// otherwise launch regular
		int instance = -1;
		int imu = 0;
		int mag = 0;
		EKF2 *ekf2_inst = new EKF2(instance, px4::wq_configurations::INS0, imu, mag, replay_mode);

		if (ekf2_inst) {
			_objects[0].store(ekf2_inst);
			ekf2_inst->ScheduleNow();
			success = true;
		}
	}

	return success ? PX4_OK : PX4_ERROR;
}

int EKF2::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Attitude and position estimator using an Extended Kalman Filter. It is used for Multirotors and Fixed-Wing.

The documentation can be found on the [ECL/EKF Overview & Tuning](https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html) page.

ekf2 can be started in replay mode (`-r`): in this mode it does not access the system time, but only uses the
timestamps from the sensor topics.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ekf2", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('r', "Enable replay mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ekf2_main(int argc, char *argv[])
{
	if (argc <= 1 || strcmp(argv[1], "-h") == 0) {
		return EKF2::print_usage();
	}

	if (strcmp(argv[1], "start") == 0) {
		int ret = 0;
		EKF2::lock_module();

		ret = EKF2::task_spawn(argc - 1, argv + 1);

		if (ret < 0) {
			PX4_ERR("start failed (%i)", ret);
		}

		EKF2::unlock_module();
		return ret;

	} else if (strcmp(argv[1], "status") == 0) {
		if (EKF2::trylock_module()) {
#if !defined(CONSTRAINED_FLASH)
			if (_ekf2_selector.load()) {
				_ekf2_selector.load()->PrintStatus();
			}
#endif // !CONSTRAINED_FLASH

			for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
				if (_objects[i].load()) {
					PX4_INFO_RAW("\n");
					_objects[i].load()->print_status();
				}
			}

			EKF2::unlock_module();

		} else {
			PX4_WARN("module locked, try again later");
		}

		return 0;

	} else if (strcmp(argv[1], "stop") == 0) {
		EKF2::lock_module();

		if (argc > 2) {
			int instance = atoi(argv[2]);

			PX4_INFO("stopping %d", instance);

			if (instance > 0 && instance < EKF2_MAX_INSTANCES) {
				EKF2 *inst = _objects[instance].load();

				if (inst) {
					inst->request_stop();
					px4_usleep(20000); // 20 ms
					delete inst;
					_objects[instance].store(nullptr);
				}
			}

		} else {
			// otherwise stop everything
			bool was_running = false;

#if !defined(CONSTRAINED_FLASH)
			if (_ekf2_selector.load()) {
				PX4_INFO("stopping ekf2 selector");
				_ekf2_selector.load()->Stop();
				delete _ekf2_selector.load();
				_ekf2_selector.store(nullptr);
				was_running = true;
			}
#endif // !CONSTRAINED_FLASH

			for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
				EKF2 *inst = _objects[i].load();

				if (inst) {
					PX4_INFO("stopping ekf2 instance %d", i);
					was_running = true;
					inst->request_stop();
					px4_usleep(20000); // 20 ms
					delete inst;
					_objects[i].store(nullptr);
				}
			}

			if (!was_running) {
				PX4_WARN("not running");
			}
		}

		EKF2::unlock_module();
		return PX4_OK;
	}

	EKF2::lock_module(); // Lock here, as the method could access _object.
	int ret = EKF2::custom_command(argc - 1, argv + 1);
	EKF2::unlock_module();

	return ret;
}
