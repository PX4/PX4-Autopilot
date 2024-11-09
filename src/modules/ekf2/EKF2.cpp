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
#include "EKF2.hpp"

using namespace time_literals;
using math::constrain;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

static constexpr float kDefaultExternalPosAccuracy = 50.0f; // [m]
static constexpr float kMaxDelaySecondsExternalPosMeasurement = 15.0f; // [s]

pthread_mutex_t ekf2_module_mutex = PTHREAD_MUTEX_INITIALIZER;
static px4::atomic<EKF2 *> _objects[EKF2_MAX_INSTANCES] {};
#if defined(CONFIG_EKF2_MULTI_INSTANCE)
static px4::atomic<EKF2Selector *> _ekf2_selector {nullptr};
#endif // CONFIG_EKF2_MULTI_INSTANCE

EKF2::EKF2(bool multi_mode, const px4::wq_config_t &config, bool replay_mode):
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, config),
	_replay_mode(replay_mode && !multi_mode),
	_multi_mode(multi_mode),
	_instance(multi_mode ? -1 : 0),
	_attitude_pub(multi_mode ? ORB_ID(estimator_attitude) : ORB_ID(vehicle_attitude)),
	_local_position_pub(multi_mode ? ORB_ID(estimator_local_position) : ORB_ID(vehicle_local_position)),
	_global_position_pub(multi_mode ? ORB_ID(estimator_global_position) : ORB_ID(vehicle_global_position)),
	_odometry_pub(multi_mode ? ORB_ID(estimator_odometry) : ORB_ID(vehicle_odometry)),
#if defined(CONFIG_EKF2_WIND)
	_wind_pub(multi_mode ? ORB_ID(estimator_wind) : ORB_ID(wind)),
#endif // CONFIG_EKF2_WIND
	_params(_ekf.getParamHandle()),
	_param_ekf2_predict_us(_params->filter_update_interval_us),
	_param_ekf2_delay_max(_params->delay_max_ms),
	_param_ekf2_imu_ctrl(_params->imu_ctrl),
	_param_ekf2_vel_lim(_params->velocity_limit),
#if defined(CONFIG_EKF2_AUXVEL)
	_param_ekf2_avel_delay(_params->auxvel_delay_ms),
#endif // CONFIG_EKF2_AUXVEL
	_param_ekf2_gyr_noise(_params->gyro_noise),
	_param_ekf2_acc_noise(_params->accel_noise),
	_param_ekf2_gyr_b_noise(_params->gyro_bias_p_noise),
	_param_ekf2_acc_b_noise(_params->accel_bias_p_noise),
#if defined(CONFIG_EKF2_WIND)
	_param_ekf2_wind_nsd(_params->wind_vel_nsd),
#endif // CONFIG_EKF2_WIND
	_param_ekf2_noaid_noise(_params->pos_noaid_noise),
#if defined(CONFIG_EKF2_GNSS)
	_param_ekf2_gps_ctrl(_params->gnss_ctrl),
	_param_ekf2_gps_delay(_params->gps_delay_ms),
	_param_ekf2_gps_pos_x(_params->gps_pos_body(0)),
	_param_ekf2_gps_pos_y(_params->gps_pos_body(1)),
	_param_ekf2_gps_pos_z(_params->gps_pos_body(2)),
	_param_ekf2_gps_v_noise(_params->gps_vel_noise),
	_param_ekf2_gps_p_noise(_params->gps_pos_noise),
	_param_ekf2_gps_p_gate(_params->gps_pos_innov_gate),
	_param_ekf2_gps_v_gate(_params->gps_vel_innov_gate),
	_param_ekf2_gps_check(_params->gps_check_mask),
	_param_ekf2_req_eph(_params->req_hacc),
	_param_ekf2_req_epv(_params->req_vacc),
	_param_ekf2_req_sacc(_params->req_sacc),
	_param_ekf2_req_nsats(_params->req_nsats),
	_param_ekf2_req_pdop(_params->req_pdop),
	_param_ekf2_req_hdrift(_params->req_hdrift),
	_param_ekf2_req_vdrift(_params->req_vdrift),
	_param_ekf2_gsf_tas_default(_params->EKFGSF_tas_default),
#endif // CONFIG_EKF2_GNSS
#if defined(CONFIG_EKF2_BAROMETER)
	_param_ekf2_baro_ctrl(_params->baro_ctrl),
	_param_ekf2_baro_delay(_params->baro_delay_ms),
	_param_ekf2_baro_noise(_params->baro_noise),
	_param_ekf2_baro_gate(_params->baro_innov_gate),
	_param_ekf2_gnd_eff_dz(_params->gnd_effect_deadzone),
	_param_ekf2_gnd_max_hgt(_params->gnd_effect_max_hgt),
# if defined(CONFIG_EKF2_BARO_COMPENSATION)
	_param_ekf2_aspd_max(_params->max_correction_airspeed),
	_param_ekf2_pcoef_xp(_params->static_pressure_coef_xp),
	_param_ekf2_pcoef_xn(_params->static_pressure_coef_xn),
	_param_ekf2_pcoef_yp(_params->static_pressure_coef_yp),
	_param_ekf2_pcoef_yn(_params->static_pressure_coef_yn),
	_param_ekf2_pcoef_z(_params->static_pressure_coef_z),
# endif // CONFIG_EKF2_BARO_COMPENSATION
#endif // CONFIG_EKF2_BAROMETER
#if defined(CONFIG_EKF2_AIRSPEED)
	_param_ekf2_asp_delay(_params->airspeed_delay_ms),
	_param_ekf2_tas_gate(_params->tas_innov_gate),
	_param_ekf2_eas_noise(_params->eas_noise),
	_param_ekf2_arsp_thr(_params->arsp_thr),
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
	_param_ekf2_beta_gate(_params->beta_innov_gate),
	_param_ekf2_beta_noise(_params->beta_noise),
	_param_ekf2_fuse_beta(_params->beta_fusion_enabled),
#endif // CONFIG_EKF2_SIDESLIP
#if defined(CONFIG_EKF2_MAGNETOMETER)
	_param_ekf2_mag_delay(_params->mag_delay_ms),
	_param_ekf2_mag_e_noise(_params->mage_p_noise),
	_param_ekf2_mag_b_noise(_params->magb_p_noise),
	_param_ekf2_head_noise(_params->mag_heading_noise),
	_param_ekf2_mag_noise(_params->mag_noise),
	_param_ekf2_mag_decl(_params->mag_declination_deg),
	_param_ekf2_hdg_gate(_params->heading_innov_gate),
	_param_ekf2_mag_gate(_params->mag_innov_gate),
	_param_ekf2_decl_type(_params->mag_declination_source),
	_param_ekf2_mag_type(_params->mag_fusion_type),
	_param_ekf2_mag_acclim(_params->mag_acc_gate),
	_param_ekf2_mag_check(_params->mag_check),
	_param_ekf2_mag_chk_str(_params->mag_check_strength_tolerance_gs),
	_param_ekf2_mag_chk_inc(_params->mag_check_inclination_tolerance_deg),
	_param_ekf2_synthetic_mag_z(_params->synthesize_mag_z),
#endif // CONFIG_EKF2_MAGNETOMETER
	_param_ekf2_hgt_ref(_params->height_sensor_ref),
	_param_ekf2_noaid_tout(_params->valid_timeout_max),
#if defined(CONFIG_EKF2_TERRAIN) || defined(CONFIG_EKF2_OPTICAL_FLOW) || defined(CONFIG_EKF2_RANGE_FINDER)
	_param_ekf2_min_rng(_params->rng_gnd_clearance),
#endif // CONFIG_EKF2_TERRAIN || CONFIG_EKF2_OPTICAL_FLOW || CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_TERRAIN)
	_param_ekf2_terr_noise(_params->terrain_p_noise),
	_param_ekf2_terr_grad(_params->terrain_gradient),
#endif // CONFIG_EKF2_TERRAIN
#if defined(CONFIG_EKF2_RANGE_FINDER)
	_param_ekf2_rng_ctrl(_params->rng_ctrl),
	_param_ekf2_rng_delay(_params->range_delay_ms),
	_param_ekf2_rng_noise(_params->range_noise),
	_param_ekf2_rng_sfe(_params->range_noise_scaler),
	_param_ekf2_rng_gate(_params->range_innov_gate),
	_param_ekf2_rng_pitch(_params->rng_sens_pitch),
	_param_ekf2_rng_a_vmax(_params->max_vel_for_range_aid),
	_param_ekf2_rng_a_hmax(_params->max_hagl_for_range_aid),
	_param_ekf2_rng_a_igate(_params->range_aid_innov_gate),
	_param_ekf2_rng_qlty_t(_params->range_valid_quality_s),
	_param_ekf2_rng_k_gate(_params->range_kin_consistency_gate),
	_param_ekf2_rng_fog(_params->rng_fog),
	_param_ekf2_rng_pos_x(_params->rng_pos_body(0)),
	_param_ekf2_rng_pos_y(_params->rng_pos_body(1)),
	_param_ekf2_rng_pos_z(_params->rng_pos_body(2)),
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	_param_ekf2_ev_delay(_params->ev_delay_ms),
	_param_ekf2_ev_ctrl(_params->ev_ctrl),
	_param_ekf2_ev_qmin(_params->ev_quality_minimum),
	_param_ekf2_evp_noise(_params->ev_pos_noise),
	_param_ekf2_evv_noise(_params->ev_vel_noise),
	_param_ekf2_eva_noise(_params->ev_att_noise),
	_param_ekf2_evv_gate(_params->ev_vel_innov_gate),
	_param_ekf2_evp_gate(_params->ev_pos_innov_gate),
	_param_ekf2_ev_pos_x(_params->ev_pos_body(0)),
	_param_ekf2_ev_pos_y(_params->ev_pos_body(1)),
	_param_ekf2_ev_pos_z(_params->ev_pos_body(2)),
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	_param_ekf2_of_ctrl(_params->flow_ctrl),
	_param_ekf2_of_gyr_src(_params->flow_gyro_src),
	_param_ekf2_of_delay(_params->flow_delay_ms),
	_param_ekf2_of_n_min(_params->flow_noise),
	_param_ekf2_of_n_max(_params->flow_noise_qual_min),
	_param_ekf2_of_qmin(_params->flow_qual_min),
	_param_ekf2_of_qmin_gnd(_params->flow_qual_min_gnd),
	_param_ekf2_of_gate(_params->flow_innov_gate),
	_param_ekf2_of_pos_x(_params->flow_pos_body(0)),
	_param_ekf2_of_pos_y(_params->flow_pos_body(1)),
	_param_ekf2_of_pos_z(_params->flow_pos_body(2)),
#endif // CONFIG_EKF2_OPTICAL_FLOW
#if defined(CONFIG_EKF2_DRAG_FUSION)
	_param_ekf2_drag_ctrl(_params->drag_ctrl),
	_param_ekf2_drag_noise(_params->drag_noise),
	_param_ekf2_bcoef_x(_params->bcoef_x),
	_param_ekf2_bcoef_y(_params->bcoef_y),
	_param_ekf2_mcoef(_params->mcoef),
#endif // CONFIG_EKF2_DRAG_FUSION
#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	_param_ekf2_grav_noise(_params->gravity_noise),
#endif // CONFIG_EKF2_GRAVITY_FUSION
	_param_ekf2_imu_pos_x(_params->imu_pos_body(0)),
	_param_ekf2_imu_pos_y(_params->imu_pos_body(1)),
	_param_ekf2_imu_pos_z(_params->imu_pos_body(2)),
	_param_ekf2_gbias_init(_params->switch_on_gyro_bias),
	_param_ekf2_abias_init(_params->switch_on_accel_bias),
	_param_ekf2_angerr_init(_params->initial_tilt_err),
	_param_ekf2_abl_lim(_params->acc_bias_lim),
	_param_ekf2_abl_acclim(_params->acc_bias_learn_acc_lim),
	_param_ekf2_abl_gyrlim(_params->acc_bias_learn_gyr_lim),
	_param_ekf2_abl_tau(_params->acc_bias_learn_tc),
	_param_ekf2_gyr_b_lim(_params->gyro_bias_lim)
{
	AdvertiseTopics();
}

EKF2::~EKF2()
{
	perf_free(_ekf_update_perf);
	perf_free(_msg_missed_imu_perf);
}

void EKF2::AdvertiseTopics()
{
	// advertise expected minimal topic set immediately for logging
	_attitude_pub.advertise();
	_local_position_pub.advertise();
	_estimator_event_flags_pub.advertise();
	_estimator_sensor_bias_pub.advertise();
	_estimator_status_pub.advertise();
	_estimator_status_flags_pub.advertise();

	if (_multi_mode) {
		// only force advertise these in multi mode to ensure consistent uORB instance numbering
		_global_position_pub.advertise();
		_odometry_pub.advertise();

#if defined(CONFIG_EKF2_WIND)
		_wind_pub.advertise();
#endif // CONFIG_EKF2_WIND
	}

#if defined(CONFIG_EKF2_GNSS)

	if (_param_ekf2_gps_ctrl.get()) {
		_estimator_gps_status_pub.advertise();
		_yaw_est_pub.advertise();
	}

#endif // CONFIG_EKF2_GNSS

	// verbose logging
	if (_param_ekf2_log_verbose.get()) {
		_estimator_innovation_test_ratios_pub.advertise();
		_estimator_innovation_variances_pub.advertise();
		_estimator_innovations_pub.advertise();
		_estimator_states_pub.advertise();

#if defined(CONFIG_EKF2_AIRSPEED)

		if (_param_ekf2_arsp_thr.get() > 0.f) {
			_estimator_aid_src_airspeed_pub.advertise();
		}

#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_BAROMETER)

		if (_param_ekf2_baro_ctrl.get()) {
			_estimator_aid_src_baro_hgt_pub.advertise();
			_estimator_baro_bias_pub.advertise();
		}

#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_DRAG_FUSION)

		if (_param_ekf2_drag_ctrl.get()) {
			_estimator_aid_src_drag_pub.advertise();
		}

#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

		if (_param_ekf2_ev_ctrl.get() & static_cast<int32_t>(EvCtrl::VPOS)) {
			_estimator_aid_src_ev_hgt_pub.advertise();
			_estimator_ev_pos_bias_pub.advertise();
		}

		if (_param_ekf2_ev_ctrl.get() & static_cast<int32_t>(EvCtrl::HPOS)) {
			_estimator_aid_src_ev_pos_pub.advertise();
			_estimator_ev_pos_bias_pub.advertise();
		}

		if (_param_ekf2_ev_ctrl.get() & static_cast<int32_t>(EvCtrl::VEL)) {
			_estimator_aid_src_ev_vel_pub.advertise();
		}

		if (_param_ekf2_ev_ctrl.get() & static_cast<int32_t>(EvCtrl::YAW)) {
			_estimator_aid_src_ev_yaw_pub.advertise();
		}

#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)

		if (_param_ekf2_gps_ctrl.get()) {
			if (_param_ekf2_gps_ctrl.get() & static_cast<int32_t>(GnssCtrl::VPOS)) {
				_estimator_aid_src_gnss_hgt_pub.advertise();
				_estimator_gnss_hgt_bias_pub.advertise();
			}

			if (_param_ekf2_gps_ctrl.get() & static_cast<int32_t>(GnssCtrl::HPOS)) {
				_estimator_aid_src_gnss_pos_pub.advertise();
			}

			if (_param_ekf2_gps_ctrl.get() & static_cast<int32_t>(GnssCtrl::VEL)) {
				_estimator_aid_src_gnss_vel_pub.advertise();
			}

# if defined(CONFIG_EKF2_GNSS_YAW)

			if (_param_ekf2_gps_ctrl.get() & static_cast<int32_t>(GnssCtrl::YAW)) {
				_estimator_aid_src_gnss_yaw_pub.advertise();
			}

# endif // CONFIG_EKF2_GNSS_YAW
		}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_GRAVITY_FUSION)

		if (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::GravityVector)) {
			_estimator_aid_src_gravity_pub.advertise();
		}

#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_MAGNETOMETER)

		if (_param_ekf2_mag_type.get() != MagFuseType::NONE) {
			_estimator_aid_src_mag_pub.advertise();
		}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

		if (_param_ekf2_of_ctrl.get()) {
			_estimator_optical_flow_vel_pub.advertise();
			_estimator_aid_src_optical_flow_pub.advertise();
		}

#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_RANGE_FINDER)

		// RNG advertise
		if (_param_ekf2_rng_ctrl.get()) {
			_estimator_aid_src_rng_hgt_pub.advertise();
		}

#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_SIDESLIP)

		if (_param_ekf2_fuse_beta.get()) {
			_estimator_aid_src_sideslip_pub.advertise();
		}

#endif // CONFIG_EKF2_SIDESLIP

	} // end verbose logging
}

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
bool EKF2::multi_init(int imu, int mag)
{
	bool changed_instance = _vehicle_imu_sub.ChangeInstance(imu);

#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (!_magnetometer_sub.ChangeInstance(mag)) {
		changed_instance = false;
	}

#endif // CONFIG_EKF2_MAGNETOMETER

	const int status_instance = _estimator_states_pub.get_instance();

	if ((status_instance >= 0) && changed_instance
	    && (_attitude_pub.get_instance() == status_instance)
	    && (_local_position_pub.get_instance() == status_instance)
	    && (_global_position_pub.get_instance() == status_instance)) {

		_instance = status_instance;

		ScheduleNow();
		return true;
	}

	PX4_ERR("publication instance problem: %d att: %d lpos: %d gpos: %d", status_instance,
		_attitude_pub.get_instance(), _local_position_pub.get_instance(), _global_position_pub.get_instance());

	return false;
}
#endif // CONFIG_EKF2_MULTI_INSTANCE

int EKF2::print_status(bool verbose)
{
	PX4_INFO_RAW("ekf2:%d EKF dt: %.4fs, attitude: %d, local position: %d, global position: %d\n",
		     _instance, (double)_ekf.get_dt_ekf_avg(), _ekf.attitude_valid(),
		     _ekf.isLocalHorizontalPositionValid(), _ekf.isGlobalHorizontalPositionValid());

	perf_print_counter(_ekf_update_perf);
	perf_print_counter(_msg_missed_imu_perf);

	if (verbose) {
#if defined(CONFIG_EKF2_VERBOSE_STATUS)
		_ekf.print_status();
#endif // CONFIG_EKF2_VERBOSE_STATUS
	}

	return 0;
}

void EKF2::Run()
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

		VerifyParams();

		// force advertise topics immediately for logging (EKF2_LOG_VERBOSE, per aid source control)
		AdvertiseTopics();

#if defined(CONFIG_EKF2_GNSS)
		_ekf.set_min_required_gps_health_time(_param_ekf2_req_gps_h.get() * 1_s);
#endif // CONFIG_EKF2_GNSS

		const matrix::Vector3f imu_pos_body(_param_ekf2_imu_pos_x.get(),
						    _param_ekf2_imu_pos_y.get(),
						    _param_ekf2_imu_pos_z.get());
		_ekf.output_predictor().set_imu_offset(imu_pos_body);
		_ekf.output_predictor().set_pos_correction_tc(_param_ekf2_tau_pos.get());
		_ekf.output_predictor().set_vel_correction_tc(_param_ekf2_tau_vel.get());

#if defined(CONFIG_EKF2_AIRSPEED)
		// The airspeed scale factor correcton is only available via parameter as used by the airspeed module
		param_t param_aspd_scale = param_find("ASPD_SCALE_1");

		if (param_aspd_scale != PARAM_INVALID) {
			param_get(param_aspd_scale, &_airspeed_scale_factor);
		}

#endif // CONFIG_EKF2_AIRSPEED

		_ekf.updateParameters();
	}

	if (!_callback_registered) {
#if defined(CONFIG_EKF2_MULTI_INSTANCE)

		if (_multi_mode) {
			_callback_registered = _vehicle_imu_sub.registerCallback();

		} else
#endif // CONFIG_EKF2_MULTI_INSTANCE
		{
			_callback_registered = _sensor_combined_sub.registerCallback();
		}

		if (!_callback_registered) {
			ScheduleDelayed(10_ms);
			return;
		}
	}

	if (_vehicle_command_sub.updated()) {
		vehicle_command_s vehicle_command;

		if (_vehicle_command_sub.update(&vehicle_command)) {

			vehicle_command_ack_s command_ack{};
			command_ack.command = vehicle_command.command;
			command_ack.target_system = vehicle_command.source_system;
			command_ack.target_component = vehicle_command.source_component;

			if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN) {
				double latitude = vehicle_command.param5;
				double longitude = vehicle_command.param6;
				float altitude = vehicle_command.param7;

				if (_ekf.setEkfGlobalOrigin(latitude, longitude, altitude)) {
					// Validate the ekf origin status.
					uint64_t origin_time {};
					_ekf.getEkfGlobalOrigin(origin_time, latitude, longitude, altitude);
					PX4_INFO("%d - New NED origin (LLA): %3.10f, %3.10f, %4.3f\n",
						 _instance, latitude, longitude, static_cast<double>(altitude));

					command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					PX4_ERR("%d - Failed to set new NED origin (LLA): %3.10f, %3.10f, %4.3f\n",
						_instance, latitude, longitude, static_cast<double>(altitude));

					command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
				}

				command_ack.timestamp = hrt_absolute_time();
				_vehicle_command_ack_pub.publish(command_ack);

			} else if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_EXTERNAL_POSITION_ESTIMATE) {

				if ((_ekf.control_status_flags().wind_dead_reckoning || _ekf.control_status_flags().inertial_dead_reckoning
				     || (!_ekf.control_status_flags().in_air && !_ekf.control_status_flags().gps)) && PX4_ISFINITE(vehicle_command.param2)
				    && PX4_ISFINITE(vehicle_command.param5) && PX4_ISFINITE(vehicle_command.param6)
				   ) {

					const float measurement_delay_seconds = math::constrain(vehicle_command.param2, 0.0f,
										kMaxDelaySecondsExternalPosMeasurement);
					const uint64_t timestamp_observation = vehicle_command.timestamp - measurement_delay_seconds * 1_s;

					float accuracy = kDefaultExternalPosAccuracy;

					if (PX4_ISFINITE(vehicle_command.param3) && vehicle_command.param3 > FLT_EPSILON) {
						accuracy = vehicle_command.param3;
					}

					if (_ekf.resetGlobalPosToExternalObservation(vehicle_command.param5, vehicle_command.param6, vehicle_command.param7,
							accuracy, accuracy, timestamp_observation)
					   ) {
						command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

					} else {
						command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
					}

				} else {
					command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED; // TODO: expand
				}

				command_ack.timestamp = hrt_absolute_time();
				_vehicle_command_ack_pub.publish(command_ack);
			}

			if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_EXTERNAL_WIND_ESTIMATE) {
#if defined(CONFIG_EKF2_WIND)
				// wind direction is given as azimuth where wind blows FROM
				// PX4 backend expects direction where wind blows TO
				const float wind_direction_rad = wrap_pi(math::radians(vehicle_command.param3) + M_PI_F);
				const float wind_direction_accuracy_rad = math::radians(vehicle_command.param4);
				_ekf.resetWindToExternalObservation(vehicle_command.param1, wind_direction_rad, vehicle_command.param2,
								    wind_direction_accuracy_rad);
				command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
#else
				command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
#endif // CONFIG_EKF2_WIND
				command_ack.timestamp = hrt_absolute_time();
				_vehicle_command_ack_pub.publish(command_ack);
			}
		}
	}

	bool imu_updated = false;
	imuSample imu_sample_new {};

	hrt_abstime imu_dt = 0; // for tracking time slip later

#if defined(CONFIG_EKF2_MULTI_INSTANCE)

	if (_multi_mode) {
		const unsigned last_generation = _vehicle_imu_sub.get_last_generation();
		vehicle_imu_s imu;
		imu_updated = _vehicle_imu_sub.update(&imu);

		if (imu_updated && (_vehicle_imu_sub.get_last_generation() != last_generation + 1)) {
			perf_count(_msg_missed_imu_perf);
		}

		if (imu_updated) {
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

			if ((_device_id_accel == 0) || (_device_id_gyro == 0)) {
				_device_id_accel = imu.accel_device_id;
				_device_id_gyro = imu.gyro_device_id;
				_accel_calibration_count = imu.accel_calibration_count;
				_gyro_calibration_count = imu.gyro_calibration_count;

			} else {
				if ((imu.accel_calibration_count != _accel_calibration_count)
				    || (imu.accel_device_id != _device_id_accel)) {

					PX4_DEBUG("%d - resetting accelerometer bias", _instance);
					_device_id_accel = imu.accel_device_id;

					_ekf.resetAccelBias();
					_accel_calibration_count = imu.accel_calibration_count;

					// reset bias learning
					_accel_cal = {};
				}

				if ((imu.gyro_calibration_count != _gyro_calibration_count)
				    || (imu.gyro_device_id != _device_id_gyro)) {

					PX4_DEBUG("%d - resetting rate gyro bias", _instance);
					_device_id_gyro = imu.gyro_device_id;

					_ekf.resetGyroBias();
					_gyro_calibration_count = imu.gyro_calibration_count;

					// reset bias learning
					_gyro_cal = {};
				}
			}
		}

	} else
#endif // CONFIG_EKF2_MULTI_INSTANCE
	{
		const unsigned last_generation = _sensor_combined_sub.get_last_generation();
		sensor_combined_s sensor_combined;
		imu_updated = _sensor_combined_sub.update(&sensor_combined);

		if (imu_updated && (_sensor_combined_sub.get_last_generation() != last_generation + 1)) {
			perf_count(_msg_missed_imu_perf);
		}

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

		if (_sensor_selection_sub.updated() || (_device_id_accel == 0 || _device_id_gyro == 0)) {
			sensor_selection_s sensor_selection;

			if (_sensor_selection_sub.copy(&sensor_selection)) {
				if (_device_id_accel != sensor_selection.accel_device_id) {

					_device_id_accel = sensor_selection.accel_device_id;

					_ekf.resetAccelBias();

					// reset bias learning
					_accel_cal = {};
				}

				if (_device_id_gyro != sensor_selection.gyro_device_id) {

					_device_id_gyro = sensor_selection.gyro_device_id;

					_ekf.resetGyroBias();

					// reset bias learning
					_gyro_cal = {};
				}
			}
		}
	}

	if (imu_updated) {
		const hrt_abstime now = imu_sample_new.time_us;

		// push imu data into estimator
		_ekf.setIMUData(imu_sample_new);
		PublishAttitude(now); // publish attitude immediately (uses quaternion from output predictor)

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
			.distance_sensor_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.optical_flow_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.vehicle_air_data_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.vehicle_magnetometer_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.visual_odometry_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
		};

#if defined(CONFIG_EKF2_AIRSPEED)
		UpdateAirspeedSample(ekf2_timestamps);
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_AUXVEL)
		UpdateAuxVelSample(ekf2_timestamps);
#endif // CONFIG_EKF2_AUXVEL
#if defined(CONFIG_EKF2_BAROMETER)
		UpdateBaroSample(ekf2_timestamps);
#endif // CONFIG_EKF2_BAROMETER
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		UpdateExtVisionSample(ekf2_timestamps);
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
		UpdateFlowSample(ekf2_timestamps);
#endif // CONFIG_EKF2_OPTICAL_FLOW
#if defined(CONFIG_EKF2_GNSS)
		UpdateGpsSample(ekf2_timestamps);
#endif // CONFIG_EKF2_GNSS
#if defined(CONFIG_EKF2_MAGNETOMETER)
		UpdateMagSample(ekf2_timestamps);
#endif // CONFIG_EKF2_MAGNETOMETER
#if defined(CONFIG_EKF2_RANGE_FINDER)
		UpdateRangeSample(ekf2_timestamps);
#endif // CONFIG_EKF2_RANGE_FINDER
		UpdateSystemFlagsSample(ekf2_timestamps);

		// run the EKF update and output
		const hrt_abstime ekf_update_start = hrt_absolute_time();

		if (_ekf.update()) {
			perf_set_elapsed(_ekf_update_perf, hrt_elapsed_time(&ekf_update_start));

			PublishLocalPosition(now);
			PublishOdometry(now, imu_sample_new);
			PublishGlobalPosition(now);
			PublishSensorBias(now);

#if defined(CONFIG_EKF2_WIND)
			PublishWindEstimate(now);
#endif // CONFIG_EKF2_WIND

			// publish status/logging messages
			PublishEventFlags(now);
			PublishStatus(now);
			PublishStatusFlags(now);

			if (_param_ekf2_log_verbose.get()) {
				PublishAidSourceStatus(now);
				PublishInnovations(now);
				PublishInnovationTestRatios(now);
				PublishInnovationVariances(now);
				PublishStates(now);

#if defined(CONFIG_EKF2_BAROMETER)
				PublishBaroBias(now);
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
				PublishEvPosBias(now);
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
				PublishGnssHgtBias(now);
#endif // CONFIG_EKF2_GNSS

			}

#if defined(CONFIG_EKF2_GNSS)
			PublishGpsStatus(now);
			PublishYawEstimatorStatus(now);
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
			PublishOpticalFlowVel(now);
#endif // CONFIG_EKF2_OPTICAL_FLOW

			UpdateAccelCalibration(now);
			UpdateGyroCalibration(now);
#if defined(CONFIG_EKF2_MAGNETOMETER)
			UpdateMagCalibration(now);
#endif // CONFIG_EKF2_MAGNETOMETER
		}

		// publish ekf2_timestamps
		_ekf2_timestamps_pub.publish(ekf2_timestamps);
	}

	// re-schedule as backup timeout
	ScheduleDelayed(100_ms);
}

void EKF2::VerifyParams()
{
#if defined(CONFIG_EKF2_MAGNETOMETER)

	// EKF2_MAG_TYPE obsolete options
	if ((_param_ekf2_mag_type.get() != MagFuseType::AUTO)
	    && (_param_ekf2_mag_type.get() != MagFuseType::HEADING)
	    && (_param_ekf2_mag_type.get() != MagFuseType::NONE)
	    && (_param_ekf2_mag_type.get() != MagFuseType::INIT)
	   ) {

		mavlink_log_critical(&_mavlink_log_pub, "EKF2_MAG_TYPE invalid, resetting to default");
		/* EVENT
		 * @description <param>EKF2_MAG_TYPE</param> is set to {1:.0}.
		 */
		events::send<float>(events::ID("ekf2_mag_type_invalid"), events::Log::Warning,
				    "EKF2_MAG_TYPE invalid, resetting to default", _param_ekf2_mag_type.get());

		_param_ekf2_mag_type.set(0);
		_param_ekf2_mag_type.commit();
	}

#endif // CONFIG_EKF2_MAGNETOMETER

	float delay_max = _param_ekf2_delay_max.get();

#if defined(CONFIG_EKF2_AUXVEL)

	if (_param_ekf2_avel_delay.get() > delay_max) {
		delay_max = _param_ekf2_avel_delay.get();
	}

#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_BAROMETER)

	if (_param_ekf2_baro_delay.get() > delay_max) {
		delay_max = _param_ekf2_baro_delay.get();
	}

#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_AIRSPEED)

	if (_param_ekf2_asp_delay.get() > delay_max) {
		delay_max = _param_ekf2_asp_delay.get();
	}

#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (_param_ekf2_mag_delay.get() > delay_max) {
		delay_max = _param_ekf2_mag_delay.get();
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_RANGE_FINDER)

	if (_param_ekf2_rng_delay.get() > delay_max) {
		delay_max = _param_ekf2_rng_delay.get();
	}

#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_GNSS)

	if (_param_ekf2_gps_delay.get() > delay_max) {
		delay_max = _param_ekf2_gps_delay.get();
	}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

	if (_param_ekf2_of_delay.get() > delay_max) {
		delay_max = _param_ekf2_of_delay.get();
	}

#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_param_ekf2_ev_delay.get() > delay_max) {
		delay_max = _param_ekf2_ev_delay.get();
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	if (delay_max > _param_ekf2_delay_max.get()) {
		/* EVENT
		 * @description EKF2_DELAY_MAX({1}ms) is too small compared to the maximum sensor delay ({2})
		 */
		events::send<float, float>(events::ID("nf_delay_max_too_small"), events::Log::Warning,
					   "EKF2_DELAY_MAX increased to {2}ms, please reboot", _param_ekf2_delay_max.get(),
					   delay_max);
		_param_ekf2_delay_max.commit_no_notification(delay_max);
	}
}

void EKF2::PublishAidSourceStatus(const hrt_abstime &timestamp)
{
#if defined(CONFIG_EKF2_AIRSPEED)
	// airspeed
	PublishAidSourceStatus(_ekf.aid_src_airspeed(), _status_airspeed_pub_last, _estimator_aid_src_airspeed_pub);
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
	// sideslip
	PublishAidSourceStatus(_ekf.aid_src_sideslip(), _status_sideslip_pub_last, _estimator_aid_src_sideslip_pub);
#endif // CONFIG_EKF2_SIDESLIP
#if defined(CONFIG_EKF2_BAROMETER)
	// baro height
	PublishAidSourceStatus(_ekf.aid_src_baro_hgt(), _status_baro_hgt_pub_last, _estimator_aid_src_baro_hgt_pub);
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// drag
	PublishAidSourceStatus(_ekf.aid_src_drag(), _status_drag_pub_last, _estimator_aid_src_drag_pub);
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// RNG height
	PublishAidSourceStatus(_ekf.aid_src_rng_hgt(), _status_rng_hgt_pub_last, _estimator_aid_src_rng_hgt_pub);
#endif // CONFIG_EKF2_RANGE_FINDER

	// fake position
	PublishAidSourceStatus(_ekf.aid_src_fake_pos(), _status_fake_pos_pub_last, _estimator_aid_src_fake_pos_pub);
	PublishAidSourceStatus(_ekf.aid_src_fake_hgt(), _status_fake_hgt_pub_last, _estimator_aid_src_fake_hgt_pub);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// external vision (EV) hgt/pos/vel/yaw
	PublishAidSourceStatus(_ekf.aid_src_ev_hgt(), _status_ev_hgt_pub_last, _estimator_aid_src_ev_hgt_pub);
	PublishAidSourceStatus(_ekf.aid_src_ev_pos(), _status_ev_pos_pub_last, _estimator_aid_src_ev_pos_pub);
	PublishAidSourceStatus(_ekf.aid_src_ev_vel(), _status_ev_vel_pub_last, _estimator_aid_src_ev_vel_pub);
	PublishAidSourceStatus(_ekf.aid_src_ev_yaw(), _status_ev_yaw_pub_last, _estimator_aid_src_ev_yaw_pub);
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
	// GNSS hgt/pos/vel/yaw
	PublishAidSourceStatus(_ekf.aid_src_gnss_hgt(), _status_gnss_hgt_pub_last, _estimator_aid_src_gnss_hgt_pub);
	PublishAidSourceStatus(_ekf.aid_src_gnss_pos(), _status_gnss_pos_pub_last, _estimator_aid_src_gnss_pos_pub);
	PublishAidSourceStatus(_ekf.aid_src_gnss_vel(), _status_gnss_vel_pub_last, _estimator_aid_src_gnss_vel_pub);
# if defined(CONFIG_EKF2_GNSS_YAW)
	PublishAidSourceStatus(_ekf.aid_src_gnss_yaw(), _status_gnss_yaw_pub_last, _estimator_aid_src_gnss_yaw_pub);
# endif // CONFIG_EKF2_GNSS_YAW
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// mag 3d
	PublishAidSourceStatus(_ekf.aid_src_mag(), _status_mag_pub_last, _estimator_aid_src_mag_pub);
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	// gravity
	PublishAidSourceStatus(_ekf.aid_src_gravity(), _status_gravity_pub_last, _estimator_aid_src_gravity_pub);
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_AUXVEL)
	// aux velocity
	PublishAidSourceStatus(_ekf.aid_src_aux_vel(), _status_aux_vel_pub_last, _estimator_aid_src_aux_vel_pub);
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// optical flow
	PublishAidSourceStatus(_ekf.aid_src_optical_flow(), _status_optical_flow_pub_last, _estimator_aid_src_optical_flow_pub);
#endif // CONFIG_EKF2_OPTICAL_FLOW
}

void EKF2::PublishAttitude(const hrt_abstime &timestamp)
{
	if (_ekf.attitude_valid()) {
		// generate vehicle attitude quaternion data
		vehicle_attitude_s att;
		att.timestamp_sample = timestamp;
		_ekf.getQuaternion().copyTo(att.q);

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

#if defined(CONFIG_EKF2_BAROMETER)
void EKF2::PublishBaroBias(const hrt_abstime &timestamp)
{
	if (_ekf.aid_src_baro_hgt().timestamp_sample != 0) {
		const BiasEstimator::status &status = _ekf.getBaroBiasEstimatorStatus();

		if (fabsf(status.bias - _last_baro_bias_published) > 0.001f) {
			_estimator_baro_bias_pub.publish(fillEstimatorBiasMsg(status, _ekf.aid_src_baro_hgt().timestamp_sample, timestamp,
							 _device_id_baro));

			_last_baro_bias_published = status.bias;
		}
	}
}
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GNSS)
void EKF2::PublishGnssHgtBias(const hrt_abstime &timestamp)
{
	if (_ekf.get_gps_sample_delayed().time_us != 0) {
		const BiasEstimator::status &status = _ekf.getGpsHgtBiasEstimatorStatus();

		if (fabsf(status.bias - _last_gnss_hgt_bias_published) > 0.001f) {
			_estimator_gnss_hgt_bias_pub.publish(fillEstimatorBiasMsg(status, _ekf.get_gps_sample_delayed().time_us, timestamp));

			_last_gnss_hgt_bias_published = status.bias;
		}
	}
}
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
void EKF2::PublishEvPosBias(const hrt_abstime &timestamp)
{
	if (_ekf.aid_src_ev_hgt().timestamp_sample) {

		estimator_bias3d_s bias{};

		// height
		BiasEstimator::status bias_est_status[3];
		bias_est_status[0] = _ekf.getEvPosBiasEstimatorStatus(0);
		bias_est_status[1] = _ekf.getEvPosBiasEstimatorStatus(1);
		bias_est_status[2] = _ekf.getEvHgtBiasEstimatorStatus();

		for (int i = 0; i < 3; i++) {
			bias.bias[i] = bias_est_status[i].bias;
			bias.bias_var[i] = bias_est_status[i].bias_var;

			bias.innov[i] = bias_est_status[i].innov;
			bias.innov_var[i] = bias_est_status[i].innov_var;
			bias.innov_test_ratio[i] = bias_est_status[i].innov_test_ratio;
		}

		const Vector3f bias_vec{bias.bias};

		if ((bias_vec - _last_ev_bias_published).longerThan(0.01f)) {
			bias.timestamp_sample = _ekf.aid_src_ev_hgt().timestamp_sample;
			bias.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
			_estimator_ev_pos_bias_pub.publish(bias);

			_last_ev_bias_published = Vector3f(bias.bias);
		}
	}
}
#endif // CONFIG_EKF2_EXTERNAL_VISION

estimator_bias_s EKF2::fillEstimatorBiasMsg(const BiasEstimator::status &status, uint64_t timestamp_sample_us,
		uint64_t timestamp, uint32_t device_id)
{
	estimator_bias_s bias{};
	bias.timestamp_sample = timestamp_sample_us;
	bias.device_id = device_id;
	bias.bias = status.bias;
	bias.bias_var = status.bias_var;
	bias.innov = status.innov;
	bias.innov_var = status.innov_var;
	bias.innov_test_ratio = status.innov_test_ratio;
	bias.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

	return bias;
}

void EKF2::PublishEventFlags(const hrt_abstime &timestamp)
{
	// information events
	uint32_t information_events = _ekf.information_event_status().value;
	bool information_event_updated = false;

	if (information_events != 0) {
		information_event_updated = true;
		_filter_information_event_changes++;
	}

	if (information_event_updated) {
		estimator_event_flags_s event_flags{};
		event_flags.timestamp_sample = _ekf.time_delayed_us();

		event_flags.information_event_changes           = _filter_information_event_changes;
		event_flags.gps_checks_passed                   = _ekf.information_event_flags().gps_checks_passed;
		event_flags.reset_vel_to_gps                    = _ekf.information_event_flags().reset_vel_to_gps;
		event_flags.reset_vel_to_flow                   = _ekf.information_event_flags().reset_vel_to_flow;
		event_flags.reset_vel_to_vision                 = _ekf.information_event_flags().reset_vel_to_vision;
		event_flags.reset_vel_to_zero                   = _ekf.information_event_flags().reset_vel_to_zero;
		event_flags.reset_pos_to_last_known             = _ekf.information_event_flags().reset_pos_to_last_known;
		event_flags.reset_pos_to_gps                    = _ekf.information_event_flags().reset_pos_to_gps;
		event_flags.reset_pos_to_vision                 = _ekf.information_event_flags().reset_pos_to_vision;
		event_flags.starting_gps_fusion                 = _ekf.information_event_flags().starting_gps_fusion;
		event_flags.starting_vision_pos_fusion          = _ekf.information_event_flags().starting_vision_pos_fusion;
		event_flags.starting_vision_vel_fusion          = _ekf.information_event_flags().starting_vision_vel_fusion;
		event_flags.starting_vision_yaw_fusion          = _ekf.information_event_flags().starting_vision_yaw_fusion;
		event_flags.yaw_aligned_to_imu_gps              = _ekf.information_event_flags().yaw_aligned_to_imu_gps;
		event_flags.reset_hgt_to_baro                   = _ekf.information_event_flags().reset_hgt_to_baro;
		event_flags.reset_hgt_to_gps                    = _ekf.information_event_flags().reset_hgt_to_gps;
		event_flags.reset_hgt_to_rng                    = _ekf.information_event_flags().reset_hgt_to_rng;
		event_flags.reset_hgt_to_ev                     = _ekf.information_event_flags().reset_hgt_to_ev;

		event_flags.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_estimator_event_flags_pub.update(event_flags);

		_last_event_flags_publish = event_flags.timestamp;

		_ekf.clear_information_events();

	} else if ((_last_event_flags_publish != 0) && (timestamp >= _last_event_flags_publish + 1_s)) {
		// continue publishing periodically
		_estimator_event_flags_pub.get().timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_estimator_event_flags_pub.update();
		_last_event_flags_publish = _estimator_event_flags_pub.get().timestamp;
	}
}

void EKF2::PublishGlobalPosition(const hrt_abstime &timestamp)
{
	if (_ekf.global_origin_valid() && _ekf.control_status().flags.yaw_align) {
		const Vector3f position{_ekf.getPosition()};

		// generate and publish global position data
		vehicle_global_position_s global_pos{};
		global_pos.timestamp_sample = timestamp;

		// Position of local NED origin in GPS / WGS84 frame
		_ekf.global_origin().reproject(position(0), position(1), global_pos.lat, global_pos.lon);
		global_pos.lat_lon_valid = _ekf.isGlobalHorizontalPositionValid();

		global_pos.alt = -position(2) + _ekf.getEkfGlobalOriginAltitude(); // Altitude AMSL in meters
		global_pos.alt_valid = _ekf.isGlobalVerticalPositionValid();

#if defined(CONFIG_EKF2_GNSS)
		global_pos.alt_ellipsoid = altAmslToEllipsoid(global_pos.alt);
#endif

		// global altitude has opposite sign of local down position
		float delta_z = 0.f;
		uint8_t z_reset_counter = 0;
		_ekf.get_posD_reset(&delta_z, &z_reset_counter);
		global_pos.delta_alt = -delta_z;
		global_pos.alt_reset_counter = z_reset_counter;

		float delta_xy[2] {};
		uint8_t xy_reset_counter = 0;
		_ekf.get_posNE_reset(delta_xy, &xy_reset_counter);
		global_pos.lat_lon_reset_counter = xy_reset_counter;

		_ekf.get_ekf_gpos_accuracy(&global_pos.eph, &global_pos.epv);

#if defined(CONFIG_EKF2_TERRAIN)

		// Terrain altitude in m, WGS84
		global_pos.terrain_alt = _ekf.getEkfGlobalOriginAltitude() - _ekf.getTerrainVertPos();
		global_pos.terrain_alt_valid = _ekf.isTerrainEstimateValid();

		float delta_hagl = 0.f;
		_ekf.get_hagl_reset(&delta_hagl, &global_pos.terrain_reset_counter);
		global_pos.delta_terrain = -delta_z;
#endif // CONFIG_EKF2_TERRAIN

		global_pos.dead_reckoning = _ekf.control_status_flags().inertial_dead_reckoning
					    || _ekf.control_status_flags().wind_dead_reckoning;

		global_pos.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_global_position_pub.publish(global_pos);
	}
}

#if defined(CONFIG_EKF2_GNSS)
void EKF2::PublishGpsStatus(const hrt_abstime &timestamp)
{
	const hrt_abstime timestamp_sample = _ekf.get_gps_sample_delayed().time_us;

	if (timestamp_sample == _last_gps_status_published) {
		return;
	}

	estimator_gps_status_s estimator_gps_status{};
	estimator_gps_status.timestamp_sample = timestamp_sample;

	estimator_gps_status.position_drift_rate_horizontal_m_s = _ekf.gps_horizontal_position_drift_rate_m_s();
	estimator_gps_status.position_drift_rate_vertical_m_s   = _ekf.gps_vertical_position_drift_rate_m_s();
	estimator_gps_status.filtered_horizontal_speed_m_s      = _ekf.gps_filtered_horizontal_velocity_m_s();

	estimator_gps_status.checks_passed = _ekf.gps_checks_passed();

	estimator_gps_status.check_fail_gps_fix          = _ekf.gps_check_fail_status_flags().fix;
	estimator_gps_status.check_fail_min_sat_count    = _ekf.gps_check_fail_status_flags().nsats;
	estimator_gps_status.check_fail_max_pdop         = _ekf.gps_check_fail_status_flags().pdop;
	estimator_gps_status.check_fail_max_horz_err     = _ekf.gps_check_fail_status_flags().hacc;
	estimator_gps_status.check_fail_max_vert_err     = _ekf.gps_check_fail_status_flags().vacc;
	estimator_gps_status.check_fail_max_spd_err      = _ekf.gps_check_fail_status_flags().sacc;
	estimator_gps_status.check_fail_max_horz_drift   = _ekf.gps_check_fail_status_flags().hdrift;
	estimator_gps_status.check_fail_max_vert_drift   = _ekf.gps_check_fail_status_flags().vdrift;
	estimator_gps_status.check_fail_max_horz_spd_err = _ekf.gps_check_fail_status_flags().hspeed;
	estimator_gps_status.check_fail_max_vert_spd_err = _ekf.gps_check_fail_status_flags().vspeed;
	estimator_gps_status.check_fail_spoofed_gps      = _ekf.gps_check_fail_status_flags().spoofed;

	estimator_gps_status.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_gps_status_pub.publish(estimator_gps_status);


	_last_gps_status_published = timestamp_sample;
}
#endif // CONFIG_EKF2_GNSS

void EKF2::PublishInnovations(const hrt_abstime &timestamp)
{
	// publish estimator innovation data
	estimator_innovations_s innovations{};
	innovations.timestamp_sample = _ekf.time_delayed_us();

#if defined(CONFIG_EKF2_GNSS)
	// GPS
	innovations.gps_hvel[0] = _ekf.aid_src_gnss_vel().innovation[0];
	innovations.gps_hvel[1] = _ekf.aid_src_gnss_vel().innovation[1];
	innovations.gps_vvel    = _ekf.aid_src_gnss_vel().innovation[2];
	innovations.gps_hpos[0] = _ekf.aid_src_gnss_pos().innovation[0];
	innovations.gps_hpos[1] = _ekf.aid_src_gnss_pos().innovation[1];
	innovations.gps_vpos    = _ekf.aid_src_gnss_hgt().innovation;
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// External Vision
	innovations.ev_hvel[0] = _ekf.aid_src_ev_vel().innovation[0];
	innovations.ev_hvel[1] = _ekf.aid_src_ev_vel().innovation[1];
	innovations.ev_vvel    = _ekf.aid_src_ev_vel().innovation[2];
	innovations.ev_hpos[0] = _ekf.aid_src_ev_pos().innovation[0];
	innovations.ev_hpos[1] = _ekf.aid_src_ev_pos().innovation[1];
	innovations.ev_vpos    = _ekf.aid_src_ev_hgt().innovation;
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// Height sensors
#if defined(CONFIG_EKF2_RANGE_FINDER)
	innovations.rng_vpos = _ekf.aid_src_rng_hgt().innovation;
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_BAROMETER)
	innovations.baro_vpos = _ekf.aid_src_baro_hgt().innovation;
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_AUXVEL)
	// Auxiliary velocity
	innovations.aux_hvel[0] = _ekf.aid_src_aux_vel().innovation[0];
	innovations.aux_hvel[1] = _ekf.aid_src_aux_vel().innovation[1];
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// Optical flow
	innovations.flow[0] = _ekf.aid_src_optical_flow().innovation[0];
	innovations.flow[1] = _ekf.aid_src_optical_flow().innovation[1];
#endif // CONFIG_EKF2_OPTICAL_FLOW

	// heading
	innovations.heading = _ekf.getHeadingInnov();

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// mag_field
	innovations.mag_field[0] = _ekf.aid_src_mag().innovation[0];
	innovations.mag_field[1] = _ekf.aid_src_mag().innovation[1];
	innovations.mag_field[2] = _ekf.aid_src_mag().innovation[2];
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	// gravity
	innovations.gravity[0] = _ekf.aid_src_gravity().innovation[0];
	innovations.gravity[1] = _ekf.aid_src_gravity().innovation[1];
	innovations.gravity[2] = _ekf.aid_src_gravity().innovation[2];
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// drag
	innovations.drag[0] = _ekf.aid_src_drag().innovation[0];
	innovations.drag[1] = _ekf.aid_src_drag().innovation[1];
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_AIRSPEED)
	// airspeed
	innovations.airspeed = _ekf.aid_src_airspeed().innovation;
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	// beta
	innovations.beta = _ekf.aid_src_sideslip().innovation;
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_TERRAIN) && defined(CONFIG_EKF2_RANGE_FINDER)
	// hagl
	innovations.hagl = _ekf.aid_src_rng_hgt().innovation;
#endif // CONFIG_EKF2_TERRAIN && CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// hagl_rate
	innovations.hagl_rate = _ekf.getHaglRateInnov();
#endif // CONFIG_EKF2_RANGE_FINDER

	innovations.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_innovations_pub.publish(innovations);
}

void EKF2::PublishInnovationTestRatios(const hrt_abstime &timestamp)
{
	// publish estimator innovation test ratio data
	estimator_innovations_s test_ratios{};
	test_ratios.timestamp_sample = _ekf.time_delayed_us();

#if defined(CONFIG_EKF2_GNSS)
	// GPS
	test_ratios.gps_hvel[0] = _ekf.aid_src_gnss_vel().test_ratio[0];
	test_ratios.gps_hvel[1] = _ekf.aid_src_gnss_vel().test_ratio[1];
	test_ratios.gps_vvel    = _ekf.aid_src_gnss_vel().test_ratio[2];
	test_ratios.gps_hpos[0] = _ekf.aid_src_gnss_pos().test_ratio[0];
	test_ratios.gps_hpos[1] = _ekf.aid_src_gnss_pos().test_ratio[1];
	test_ratios.gps_vpos    = _ekf.aid_src_gnss_hgt().test_ratio;
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// External Vision
	test_ratios.ev_hvel[0] = _ekf.aid_src_ev_vel().test_ratio[0];
	test_ratios.ev_hvel[1] = _ekf.aid_src_ev_vel().test_ratio[1];
	test_ratios.ev_vvel    = _ekf.aid_src_ev_vel().test_ratio[2];
	test_ratios.ev_hpos[0] = _ekf.aid_src_ev_pos().test_ratio[0];
	test_ratios.ev_hpos[1] = _ekf.aid_src_ev_pos().test_ratio[1];
	test_ratios.ev_vpos    = _ekf.aid_src_ev_hgt().test_ratio;
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// Height sensors
#if defined(CONFIG_EKF2_RANGE_FINDER)
	test_ratios.rng_vpos = _ekf.aid_src_rng_hgt().test_ratio;
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_BAROMETER)
	test_ratios.baro_vpos = _ekf.aid_src_baro_hgt().test_ratio;
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_AUXVEL)
	// Auxiliary velocity
	test_ratios.aux_hvel[0] = _ekf.aid_src_aux_vel().test_ratio[0];
	test_ratios.aux_hvel[1] = _ekf.aid_src_aux_vel().test_ratio[1];
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// Optical flow
	test_ratios.flow[0] = _ekf.aid_src_optical_flow().test_ratio[0];
	test_ratios.flow[1] = _ekf.aid_src_optical_flow().test_ratio[1];
#endif // CONFIG_EKF2_OPTICAL_FLOW

	// heading
	test_ratios.heading = _ekf.getHeadingInnovRatio();

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// mag_field
	test_ratios.mag_field[0] = _ekf.aid_src_mag().test_ratio[0];
	test_ratios.mag_field[1] = _ekf.aid_src_mag().test_ratio[1];
	test_ratios.mag_field[2] = _ekf.aid_src_mag().test_ratio[2];
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	// gravity
	test_ratios.gravity[0] = _ekf.aid_src_gravity().test_ratio[0];
	test_ratios.gravity[1] = _ekf.aid_src_gravity().test_ratio[1];
	test_ratios.gravity[2] = _ekf.aid_src_gravity().test_ratio[2];
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// drag
	test_ratios.drag[0] = _ekf.aid_src_drag().test_ratio[0];
	test_ratios.drag[1] = _ekf.aid_src_drag().test_ratio[1];
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_AIRSPEED)
	// airspeed
	test_ratios.airspeed = _ekf.aid_src_airspeed().test_ratio;
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	// beta
	test_ratios.beta = _ekf.aid_src_sideslip().test_ratio;
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_TERRAIN) && defined(CONFIG_EKF2_RANGE_FINDER)
	// hagl
	test_ratios.hagl = _ekf.aid_src_rng_hgt().test_ratio;
#endif // CONFIG_EKF2_TERRAIN && CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// hagl_rate
	test_ratios.hagl_rate = _ekf.getHaglRateInnovRatio();
#endif // CONFIG_EKF2_RANGE_FINDER

	test_ratios.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_innovation_test_ratios_pub.publish(test_ratios);
}

void EKF2::PublishInnovationVariances(const hrt_abstime &timestamp)
{
	// publish estimator innovation variance data
	estimator_innovations_s variances{};
	variances.timestamp_sample = _ekf.time_delayed_us();

#if defined(CONFIG_EKF2_GNSS)
	// GPS
	variances.gps_hvel[0] = _ekf.aid_src_gnss_vel().innovation_variance[0];
	variances.gps_hvel[1] = _ekf.aid_src_gnss_vel().innovation_variance[1];
	variances.gps_vvel    = _ekf.aid_src_gnss_vel().innovation_variance[2];
	variances.gps_hpos[0] = _ekf.aid_src_gnss_pos().innovation_variance[0];
	variances.gps_hpos[1] = _ekf.aid_src_gnss_pos().innovation_variance[1];
	variances.gps_vpos    = _ekf.aid_src_gnss_hgt().innovation_variance;
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// External Vision
	variances.ev_hvel[0] = _ekf.aid_src_ev_vel().innovation_variance[0];
	variances.ev_hvel[1] = _ekf.aid_src_ev_vel().innovation_variance[1];
	variances.ev_vvel    = _ekf.aid_src_ev_vel().innovation_variance[2];
	variances.ev_hpos[0] = _ekf.aid_src_ev_pos().innovation_variance[0];
	variances.ev_hpos[1] = _ekf.aid_src_ev_pos().innovation_variance[1];
	variances.ev_vpos    = _ekf.aid_src_ev_hgt().innovation_variance;
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// Height sensors
#if defined(CONFIG_EKF2_RANGE_FINDER)
	variances.rng_vpos = _ekf.aid_src_rng_hgt().innovation_variance;
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_BAROMETER)
	variances.baro_vpos = _ekf.aid_src_baro_hgt().innovation_variance;
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_AUXVEL)
	// Auxiliary velocity
	variances.aux_hvel[0] = _ekf.aid_src_aux_vel().innovation_variance[0];
	variances.aux_hvel[1] = _ekf.aid_src_aux_vel().innovation_variance[1];
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// Optical flow
	variances.flow[0] = _ekf.aid_src_optical_flow().innovation_variance[0];
	variances.flow[1] = _ekf.aid_src_optical_flow().innovation_variance[1];
#endif // CONFIG_EKF2_OPTICAL_FLOW

	// heading
	variances.heading = _ekf.getHeadingInnovVar();

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// mag_field
	variances.mag_field[0] = _ekf.aid_src_mag().innovation_variance[0];
	variances.mag_field[1] = _ekf.aid_src_mag().innovation_variance[1];
	variances.mag_field[2] = _ekf.aid_src_mag().innovation_variance[2];
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	// gravity
	variances.gravity[0] = _ekf.aid_src_gravity().innovation_variance[0];
	variances.gravity[1] = _ekf.aid_src_gravity().innovation_variance[1];
	variances.gravity[2] = _ekf.aid_src_gravity().innovation_variance[2];
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// drag
	variances.drag[0] = _ekf.aid_src_drag().innovation_variance[0];
	variances.drag[1] = _ekf.aid_src_drag().innovation_variance[1];
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_AIRSPEED)
	// airspeed
	variances.airspeed = _ekf.aid_src_airspeed().innovation_variance;
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	// beta
	variances.beta = _ekf.aid_src_sideslip().innovation_variance;
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_TERRAIN) && defined(CONFIG_EKF2_RANGE_FINDER)
	// hagl
	variances.hagl = _ekf.aid_src_rng_hgt().innovation_variance;
#endif // CONFIG_EKF2_TERRAIN && CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// hagl_rate
	variances.hagl_rate = _ekf.getHaglRateInnovVar();
#endif // CONFIG_EKF2_RANGE_FINDER

	variances.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_innovation_variances_pub.publish(variances);
}

void EKF2::PublishLocalPosition(const hrt_abstime &timestamp)
{
	vehicle_local_position_s lpos{};
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
	lpos.dist_bottom_valid = _ekf.isTerrainEstimateValid();
	lpos.dist_bottom = math::max(_ekf.getHagl(), 0.f);
	lpos.dist_bottom_var = _ekf.getTerrainVariance();
	_ekf.get_hagl_reset(&lpos.delta_dist_bottom, &lpos.dist_bottom_reset_counter);

	lpos.dist_bottom_sensor_bitfield = vehicle_local_position_s::DIST_BOTTOM_SENSOR_NONE;

	if (_ekf.control_status_flags().rng_terrain) {
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
	lpos.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_local_position_pub.publish(lpos);
}

void EKF2::PublishOdometry(const hrt_abstime &timestamp, const imuSample &imu_sample)
{
	// generate vehicle odometry data
	vehicle_odometry_s odom;
	odom.timestamp_sample = imu_sample.time_us;

	// position
	odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
	_ekf.getPosition().copyTo(odom.position);

	// orientation quaternion
	_ekf.getQuaternion().copyTo(odom.q);

	// velocity
	odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;
	_ekf.getVelocity().copyTo(odom.velocity);

	// angular_velocity
	const Vector3f rates{imu_sample.delta_ang / imu_sample.delta_ang_dt};
	const Vector3f angular_velocity = rates - _ekf.getGyroBias();
	angular_velocity.copyTo(odom.angular_velocity);

	// velocity covariances
	_ekf.getVelocityVariance().copyTo(odom.velocity_variance);

	// position covariances
	_ekf.getPositionVariance().copyTo(odom.position_variance);

	// orientation covariance
	_ekf.getRotVarBody().copyTo(odom.orientation_variance);

	odom.reset_counter = _ekf.get_quat_reset_count()
			     + _ekf.get_velNE_reset_count() + _ekf.get_velD_reset_count()
			     + _ekf.get_posNE_reset_count() + _ekf.get_posD_reset_count();

	odom.quality = 0;

	// publish vehicle odometry data
	odom.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_odometry_pub.publish(odom);
}

void EKF2::PublishSensorBias(const hrt_abstime &timestamp)
{
	// estimator_sensor_bias
	const Vector3f gyro_bias{_ekf.getGyroBias()};
	const Vector3f accel_bias{_ekf.getAccelBias()};

#if defined(CONFIG_EKF2_MAGNETOMETER)
	const Vector3f mag_bias {_ekf.getMagBias()};
#endif // CONFIG_EKF2_MAGNETOMETER

	// publish at ~1 Hz, or sooner if there's a change
	if ((gyro_bias - _last_gyro_bias_published).longerThan(0.001f)
	    || (accel_bias - _last_accel_bias_published).longerThan(0.001f)
#if defined(CONFIG_EKF2_MAGNETOMETER)
	    || (mag_bias - _last_mag_bias_published).longerThan(0.001f)
#endif // CONFIG_EKF2_MAGNETOMETER
	    || (timestamp >= _last_sensor_bias_published + 1_s)) {

		estimator_sensor_bias_s bias{};
		bias.timestamp_sample = _ekf.time_delayed_us();

		// take device ids from sensor_selection_s if not using specific vehicle_imu_s
		if ((_device_id_gyro != 0) && (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::GyroBias))) {
			const Vector3f bias_var{_ekf.getGyroBiasVariance()};

			bias.gyro_device_id = _device_id_gyro;
			gyro_bias.copyTo(bias.gyro_bias);
			bias.gyro_bias_limit = _ekf.getGyroBiasLimit();
			bias_var.copyTo(bias.gyro_bias_variance);
			bias.gyro_bias_valid = bias_var.longerThan(0.f) && !bias_var.longerThan(0.1f);
			bias.gyro_bias_stable = _gyro_cal.cal_available;
			_last_gyro_bias_published = gyro_bias;
		}

		if ((_device_id_accel != 0) && (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::AccelBias))) {
			const Vector3f bias_var{_ekf.getAccelBiasVariance()};

			bias.accel_device_id = _device_id_accel;
			accel_bias.copyTo(bias.accel_bias);
			bias.accel_bias_limit = _ekf.getAccelBiasLimit();
			bias_var.copyTo(bias.accel_bias_variance);
			bias.accel_bias_valid = bias_var.longerThan(0.f) && !bias_var.longerThan(0.1f);
			bias.accel_bias_stable = _accel_cal.cal_available;
			_last_accel_bias_published = accel_bias;
		}

#if defined(CONFIG_EKF2_MAGNETOMETER)

		if (_device_id_mag != 0) {
			const Vector3f bias_var{_ekf.getMagBiasVariance()};

			bias.mag_device_id = _device_id_mag;
			mag_bias.copyTo(bias.mag_bias);
			bias.mag_bias_limit = _ekf.getMagBiasLimit();
			bias_var.copyTo(bias.mag_bias_variance);
			bias.mag_bias_valid = bias_var.longerThan(0.f) && !bias_var.longerThan(0.1f);
			bias.mag_bias_stable = _mag_cal.cal_available;
			_last_mag_bias_published = mag_bias;
		}

#endif // CONFIG_EKF2_MAGNETOMETER

		bias.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_estimator_sensor_bias_pub.publish(bias);

		_last_sensor_bias_published = bias.timestamp;
	}
}

void EKF2::PublishStates(const hrt_abstime &timestamp)
{
	// publish estimator states
	estimator_states_s states;
	states.timestamp_sample = _ekf.time_delayed_us();
	const auto state_vector = _ekf.state().vector();
	state_vector.copyTo(states.states);
	states.n_states = state_vector.size();
	_ekf.covariances_diagonal().copyTo(states.covariances);
	states.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_states_pub.publish(states);
}

void EKF2::PublishStatus(const hrt_abstime &timestamp)
{
	estimator_status_s status{};
	status.timestamp_sample = _ekf.time_delayed_us();

	_ekf.getOutputTrackingError().copyTo(status.output_tracking_error);

#if defined(CONFIG_EKF2_GNSS)
	// only report enabled GPS check failures (the param indexes are shifted by 1 bit, because they don't include
	// the GPS Fix bit, which is always checked)
	status.gps_check_fail_flags = _ekf.gps_check_fail_status().value & (((uint16_t)_params->gps_check_mask << 1) | 1);
#endif // CONFIG_EKF2_GNSS

	status.control_mode_flags = _ekf.control_status().value;
	status.filter_fault_flags = _ekf.fault_status().value;

	// vel_test_ratio
	float vel_xy_test_ratio = _ekf.getHorizontalVelocityInnovationTestRatio();
	float vel_z_test_ratio = _ekf.getVerticalVelocityInnovationTestRatio();

	if (PX4_ISFINITE(vel_xy_test_ratio) && PX4_ISFINITE(vel_z_test_ratio)) {
		status.vel_test_ratio = math::max(vel_xy_test_ratio, vel_z_test_ratio);

	} else if (PX4_ISFINITE(vel_xy_test_ratio)) {
		status.vel_test_ratio = vel_xy_test_ratio;

	} else if (PX4_ISFINITE(vel_z_test_ratio)) {
		status.vel_test_ratio = vel_z_test_ratio;

	} else {
		status.vel_test_ratio = NAN;
	}

	status.hdg_test_ratio = _ekf.getHeadingInnovationTestRatio();
	status.pos_test_ratio = _ekf.getHorizontalPositionInnovationTestRatio();
	status.hgt_test_ratio = _ekf.getVerticalPositionInnovationTestRatio();
	status.tas_test_ratio = _ekf.getAirspeedInnovationTestRatio();
	status.hagl_test_ratio = _ekf.getHeightAboveGroundInnovationTestRatio();
	status.beta_test_ratio = _ekf.getSyntheticSideslipInnovationTestRatio();

	_ekf.get_ekf_lpos_accuracy(&status.pos_horiz_accuracy, &status.pos_vert_accuracy);
	status.solution_status_flags = _ekf.get_ekf_soln_status();

	// reset counters
	status.reset_count_vel_ne = _ekf.state_reset_status().reset_count.velNE;
	status.reset_count_vel_d = _ekf.state_reset_status().reset_count.velD;
	status.reset_count_pos_ne = _ekf.state_reset_status().reset_count.posNE;
	status.reset_count_pod_d = _ekf.state_reset_status().reset_count.posD;
	status.reset_count_quat = _ekf.state_reset_status().reset_count.quat;

	status.time_slip = _last_time_slip_us * 1e-6f;

	static constexpr float kMinTestRatioPreflight = 0.5f;
	status.pre_flt_fail_innov_heading   = (kMinTestRatioPreflight < status.hdg_test_ratio);
	status.pre_flt_fail_innov_height    = (kMinTestRatioPreflight < status.hgt_test_ratio);
	status.pre_flt_fail_innov_pos_horiz = (kMinTestRatioPreflight < status.pos_test_ratio);
	status.pre_flt_fail_innov_vel_horiz = (kMinTestRatioPreflight < vel_xy_test_ratio);
	status.pre_flt_fail_innov_vel_vert  = (kMinTestRatioPreflight < vel_z_test_ratio);

	status.pre_flt_fail_mag_field_disturbed = _ekf.control_status_flags().mag_field_disturbed;

	status.accel_device_id = _device_id_accel;
#if defined(CONFIG_EKF2_BAROMETER)
	status.baro_device_id = _device_id_baro;
#endif // CONFIG_EKF2_BAROMETER
	status.gyro_device_id = _device_id_gyro;

#if defined(CONFIG_EKF2_MAGNETOMETER)
	status.mag_device_id = _device_id_mag;

	_ekf.get_mag_checks(status.mag_inclination_deg, status.mag_inclination_ref_deg, status.mag_strength_gs,
			    status.mag_strength_ref_gs);
#endif // CONFIG_EKF2_MAGNETOMETER

	status.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_status_pub.publish(status);
}

void EKF2::PublishStatusFlags(const hrt_abstime &timestamp)
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

	// innovation check fail status
	if (_ekf.innov_check_fail_status().value != _innov_check_fail_status) {
		update = true;
		_innov_check_fail_status = _ekf.innov_check_fail_status().value;
		_innov_check_fail_status_changes++;
	}

	if (update) {
		estimator_status_flags_s status_flags{};
		status_flags.timestamp_sample = _ekf.time_delayed_us();

		status_flags.control_status_changes   = _filter_control_status_changes;
		status_flags.cs_tilt_align            = _ekf.control_status_flags().tilt_align;
		status_flags.cs_yaw_align             = _ekf.control_status_flags().yaw_align;
		status_flags.cs_gps                   = _ekf.control_status_flags().gps;
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

		status_flags.innovation_fault_status_changes = _innov_check_fail_status_changes;
		status_flags.reject_hor_vel                  = _ekf.innov_check_fail_status_flags().reject_hor_vel;
		status_flags.reject_ver_vel                  = _ekf.innov_check_fail_status_flags().reject_ver_vel;
		status_flags.reject_hor_pos                  = _ekf.innov_check_fail_status_flags().reject_hor_pos;
		status_flags.reject_ver_pos                  = _ekf.innov_check_fail_status_flags().reject_ver_pos;
		status_flags.reject_yaw                      = _ekf.innov_check_fail_status_flags().reject_yaw;
		status_flags.reject_airspeed                 = _ekf.innov_check_fail_status_flags().reject_airspeed;
		status_flags.reject_sideslip                 = _ekf.innov_check_fail_status_flags().reject_sideslip;
		status_flags.reject_hagl                     = _ekf.innov_check_fail_status_flags().reject_hagl;
		status_flags.reject_optflow_x                = _ekf.innov_check_fail_status_flags().reject_optflow_X;
		status_flags.reject_optflow_y                = _ekf.innov_check_fail_status_flags().reject_optflow_Y;

		status_flags.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_estimator_status_flags_pub.publish(status_flags);

		_last_status_flags_publish = status_flags.timestamp;
	}
}

#if defined(CONFIG_EKF2_GNSS)
void EKF2::PublishYawEstimatorStatus(const hrt_abstime &timestamp)
{
	static_assert(sizeof(yaw_estimator_status_s::yaw) / sizeof(float) == N_MODELS_EKFGSF,
		      "yaw_estimator_status_s::yaw wrong size");

	yaw_estimator_status_s yaw_est_test_data;

	if (_ekf.getDataEKFGSF(&yaw_est_test_data.yaw_composite, &yaw_est_test_data.yaw_variance,
			       yaw_est_test_data.yaw,
			       yaw_est_test_data.innov_vn, yaw_est_test_data.innov_ve,
			       yaw_est_test_data.weight)) {

		yaw_est_test_data.yaw_composite_valid = _ekf.isYawEmergencyEstimateAvailable();
		yaw_est_test_data.timestamp_sample = _ekf.time_delayed_us();
		yaw_est_test_data.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

		_yaw_est_pub.publish(yaw_est_test_data);
	}
}
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_WIND)
void EKF2::PublishWindEstimate(const hrt_abstime &timestamp)
{
	if (_ekf.get_wind_status()) {
		// Publish wind estimate only if ekf declares them valid
		wind_s wind{};
		wind.timestamp_sample = _ekf.time_delayed_us();

		const Vector2f wind_vel = _ekf.getWindVelocity();
		const Vector2f wind_vel_var = _ekf.getWindVelocityVariance();

#if defined(CONFIG_EKF2_AIRSPEED)
		wind.tas_innov = _ekf.aid_src_airspeed().innovation;
		wind.tas_innov_var = _ekf.aid_src_airspeed().innovation_variance;
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
		wind.beta_innov = _ekf.aid_src_sideslip().innovation;
		wind.beta_innov = _ekf.aid_src_sideslip().innovation_variance;
#endif // CONFIG_EKF2_SIDESLIP

		wind.windspeed_north = wind_vel(0);
		wind.windspeed_east = wind_vel(1);
		wind.variance_north = wind_vel_var(0);
		wind.variance_east = wind_vel_var(1);
		wind.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

		_wind_pub.publish(wind);
	}
}
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
void EKF2::PublishOpticalFlowVel(const hrt_abstime &timestamp)
{
	const hrt_abstime timestamp_sample = _ekf.aid_src_optical_flow().timestamp_sample;

	if ((timestamp_sample != 0) && (timestamp_sample > _optical_flow_vel_pub_last)) {

		vehicle_optical_flow_vel_s flow_vel{};
		flow_vel.timestamp_sample = _ekf.aid_src_optical_flow().timestamp_sample;

		_ekf.getFlowVelBody().copyTo(flow_vel.vel_body);
		_ekf.getFlowVelNE().copyTo(flow_vel.vel_ne);

		_ekf.getFilteredFlowVelBody().copyTo(flow_vel.vel_body_filtered);
		_ekf.getFilteredFlowVelNE().copyTo(flow_vel.vel_ne_filtered);

		_ekf.getFlowUncompensated().copyTo(flow_vel.flow_rate_uncompensated);
		_ekf.getFlowCompensated().copyTo(flow_vel.flow_rate_compensated);

		_ekf.getFlowGyro().copyTo(flow_vel.gyro_rate);

		_ekf.getFlowGyroBias().copyTo(flow_vel.gyro_bias);
		_ekf.getFlowRefBodyRate().copyTo(flow_vel.ref_gyro);

		flow_vel.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

		_estimator_optical_flow_vel_pub.publish(flow_vel);

		_optical_flow_vel_pub_last = timestamp_sample;
	}
}
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_AIRSPEED)
void EKF2::UpdateAirspeedSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF airspeed sample
	// prefer ORB_ID(airspeed_validated) if available, otherwise fallback to raw airspeed ORB_ID(airspeed)
	if (_airspeed_validated_sub.updated()) {
		airspeed_validated_s airspeed_validated;

		if (_airspeed_validated_sub.update(&airspeed_validated)) {

			if (PX4_ISFINITE(airspeed_validated.true_airspeed_m_s)
			    && (airspeed_validated.selected_airspeed_index > 0)
			   ) {
				float cas2tas = 1.f;

				if (PX4_ISFINITE(airspeed_validated.calibrated_airspeed_m_s)
				    && (airspeed_validated.calibrated_airspeed_m_s > FLT_EPSILON)) {
					cas2tas = airspeed_validated.true_airspeed_m_s / airspeed_validated.calibrated_airspeed_m_s;
				}

				airspeedSample airspeed_sample {
					.time_us = airspeed_validated.timestamp,
					.true_airspeed = airspeed_validated.true_airspeed_m_s,
					.eas2tas = cas2tas,
				};
				_ekf.setAirspeedData(airspeed_sample);
			}

			_airspeed_validated_timestamp_last = airspeed_validated.timestamp;
		}

	} else if (((ekf2_timestamps.timestamp - _airspeed_validated_timestamp_last) > 3_s) && _airspeed_sub.updated()) {
		// use ORB_ID(airspeed) if ORB_ID(airspeed_validated) is unavailable
		airspeed_s airspeed;

		if (_airspeed_sub.update(&airspeed)) {
			// The airspeed measurement received via ORB_ID(airspeed) topic has not been corrected
			// for scale factor errors and requires the ASPD_SCALE correction to be applied.
			const float true_airspeed_m_s = airspeed.true_airspeed_m_s * _airspeed_scale_factor;

			if (PX4_ISFINITE(airspeed.true_airspeed_m_s)
			    && PX4_ISFINITE(airspeed.indicated_airspeed_m_s)
			    && (airspeed.indicated_airspeed_m_s > 0.f)
			   ) {
				airspeedSample airspeed_sample {
					.time_us = airspeed.timestamp_sample,
					.true_airspeed = true_airspeed_m_s,
					.eas2tas = airspeed.true_airspeed_m_s / airspeed.indicated_airspeed_m_s,
				};
				_ekf.setAirspeedData(airspeed_sample);
			}

			ekf2_timestamps.airspeed_timestamp_rel = (int16_t)((int64_t)airspeed.timestamp / 100 -
					(int64_t)ekf2_timestamps.timestamp / 100);
		}
	}
}
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_AUXVEL)
void EKF2::UpdateAuxVelSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF auxiliary velocity sample
	//  - use the landing target pose estimate as another source of velocity data
	landing_target_pose_s landing_target_pose;

	if (_landing_target_pose_sub.update(&landing_target_pose)) {
		// we can only use the landing target if it has a fixed position and  a valid velocity estimate
		if (landing_target_pose.is_static && landing_target_pose.rel_vel_valid) {
			// velocity of vehicle relative to target has opposite sign to target relative to vehicle
			auxVelSample auxvel_sample{
				.time_us = landing_target_pose.timestamp,
				.vel = Vector2f{-landing_target_pose.vx_rel, -landing_target_pose.vy_rel},
				.velVar = Vector2f{landing_target_pose.cov_vx_rel, landing_target_pose.cov_vy_rel},
			};
			_ekf.setAuxVelData(auxvel_sample);
		}
	}
}
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_BAROMETER)
void EKF2::UpdateBaroSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF baro sample
	vehicle_air_data_s airdata;

	if (_airdata_sub.update(&airdata)) {

		bool reset = false;

		// check if barometer has changed
		if (airdata.baro_device_id != _device_id_baro) {
			if (_device_id_baro != 0) {
				PX4_DEBUG("%d - baro sensor ID changed %" PRIu32 " -> %" PRIu32, _instance, _device_id_baro, airdata.baro_device_id);
			}

			reset = true;

		} else if (airdata.calibration_count != _baro_calibration_count) {
			// existing calibration has changed, reset saved baro bias
			PX4_DEBUG("%d - baro %" PRIu32 " calibration updated, resetting bias", _instance, _device_id_baro);
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
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
bool EKF2::UpdateExtVisionSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF external vision sample
	bool new_ev_odom = false;

	vehicle_odometry_s ev_odom;

	if (_ev_odom_sub.update(&ev_odom)) {

		extVisionSample ev_data{};
		ev_data.pos.setNaN();
		ev_data.vel.setNaN();
		ev_data.quat.setNaN();

		// check for valid velocity data
		const Vector3f ev_odom_vel(ev_odom.velocity);
		const Vector3f ev_odom_vel_var(ev_odom.velocity_variance);

		if (ev_odom_vel.isAllFinite()) {
			bool velocity_frame_valid = false;

			switch (ev_odom.velocity_frame) {
			case vehicle_odometry_s::VELOCITY_FRAME_NED:
				ev_data.vel_frame = VelocityFrame::LOCAL_FRAME_NED;
				velocity_frame_valid = true;
				break;

			case vehicle_odometry_s::VELOCITY_FRAME_FRD:
				ev_data.vel_frame = VelocityFrame::LOCAL_FRAME_FRD;
				velocity_frame_valid = true;
				break;

			case vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD:
				ev_data.vel_frame = VelocityFrame::BODY_FRAME_FRD;
				velocity_frame_valid = true;
				break;
			}

			if (velocity_frame_valid) {
				ev_data.vel = ev_odom_vel;

				const float evv_noise_var = sq(_param_ekf2_evv_noise.get());

				// velocity measurement error from ev_data or parameters
				if ((_param_ekf2_ev_noise_md.get() == 0) && ev_odom_vel_var.isAllFinite()) {

					ev_data.velocity_var(0) = fmaxf(evv_noise_var, ev_odom_vel_var(0));
					ev_data.velocity_var(1) = fmaxf(evv_noise_var, ev_odom_vel_var(1));
					ev_data.velocity_var(2) = fmaxf(evv_noise_var, ev_odom_vel_var(2));

				} else {
					ev_data.velocity_var.setAll(evv_noise_var);
				}

				new_ev_odom = true;
			}
		}

		// check for valid position data
		const Vector3f ev_odom_pos(ev_odom.position);
		const Vector3f ev_odom_pos_var(ev_odom.position_variance);

		if (ev_odom_pos.isAllFinite()) {
			bool position_frame_valid = false;

			switch (ev_odom.pose_frame) {
			case vehicle_odometry_s::POSE_FRAME_NED:
				ev_data.pos_frame = PositionFrame::LOCAL_FRAME_NED;
				position_frame_valid = true;
				break;

			case vehicle_odometry_s::POSE_FRAME_FRD:
				ev_data.pos_frame = PositionFrame::LOCAL_FRAME_FRD;
				position_frame_valid = true;
				break;
			}

			if (position_frame_valid) {
				ev_data.pos = ev_odom_pos;

				const float evp_noise_var = sq(_param_ekf2_evp_noise.get());

				// position measurement error from ev_data or parameters
				if ((_param_ekf2_ev_noise_md.get() == 0) && ev_odom_pos_var.isAllFinite()) {

					ev_data.position_var(0) = fmaxf(evp_noise_var, ev_odom_pos_var(0));
					ev_data.position_var(1) = fmaxf(evp_noise_var, ev_odom_pos_var(1));
					ev_data.position_var(2) = fmaxf(evp_noise_var, ev_odom_pos_var(2));

				} else {
					ev_data.position_var.setAll(evp_noise_var);
				}

				new_ev_odom = true;
			}
		}

		// check for valid orientation data
		const Quatf ev_odom_q(ev_odom.q);
		const Vector3f ev_odom_q_var(ev_odom.orientation_variance);
		const bool non_zero = (fabsf(ev_odom_q(0)) > 0.f) || (fabsf(ev_odom_q(1)) > 0.f)
				      || (fabsf(ev_odom_q(2)) > 0.f) || (fabsf(ev_odom_q(3)) > 0.f);
		const float eps = 1e-5f;
		const bool no_element_larger_than_one = (fabsf(ev_odom_q(0)) <= 1.f + eps)
							&& (fabsf(ev_odom_q(1)) <= 1.f + eps)
							&& (fabsf(ev_odom_q(2)) <= 1.f + eps)
							&& (fabsf(ev_odom_q(3)) <= 1.f + eps);
		const bool norm_in_tolerance = fabsf(1.f - ev_odom_q.norm()) <= eps;

		const bool orientation_valid = ev_odom_q.isAllFinite() && non_zero && no_element_larger_than_one && norm_in_tolerance;

		if (orientation_valid) {
			ev_data.quat = ev_odom_q;
			ev_data.quat.normalize();

			// orientation measurement error from ev_data or parameters
			const float eva_noise_var = sq(_param_ekf2_eva_noise.get());

			if ((_param_ekf2_ev_noise_md.get() == 0) && ev_odom_q_var.isAllFinite()) {

				ev_data.orientation_var(0) = fmaxf(eva_noise_var, ev_odom_q_var(0));
				ev_data.orientation_var(1) = fmaxf(eva_noise_var, ev_odom_q_var(1));
				ev_data.orientation_var(2) = fmaxf(eva_noise_var, ev_odom_q_var(2));

			} else {
				ev_data.orientation_var.setAll(eva_noise_var);
			}

			new_ev_odom = true;
		}

		// use timestamp from external computer, clocks are synchronized when using MAVROS
		ev_data.time_us = ev_odom.timestamp_sample;
		ev_data.reset_counter = ev_odom.reset_counter;
		ev_data.quality = ev_odom.quality;

		if (new_ev_odom)  {
			_ekf.setExtVisionData(ev_data);
		}

		ekf2_timestamps.visual_odometry_timestamp_rel = (int16_t)((int64_t)ev_odom.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}

	return new_ev_odom;
}
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
bool EKF2::UpdateFlowSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF flow sample
	bool new_optical_flow = false;
	vehicle_optical_flow_s optical_flow;

	if (_vehicle_optical_flow_sub.update(&optical_flow)) {

		const float dt = 1e-6f * (float)optical_flow.integration_timespan_us;
		Vector2f flow_rate;
		Vector3f gyro_rate;

		if (dt > FLT_EPSILON) {
			// NOTE: the EKF uses the reverse sign convention to the flow sensor. EKF assumes positive LOS rate
			// is produced by a RH rotation of the image about the sensor axis.
			flow_rate = Vector2f(-optical_flow.pixel_flow[0], -optical_flow.pixel_flow[1]) / dt;
			gyro_rate = Vector3f(-optical_flow.delta_angle[0], -optical_flow.delta_angle[1], -optical_flow.delta_angle[2]) / dt;

		} else if (optical_flow.quality == 0) {
			// handle special case of SITL and PX4Flow where dt is forced to zero when the quaity is 0
			flow_rate.zero();
			gyro_rate.zero();
		}

		flowSample flow {
			.time_us = optical_flow.timestamp_sample - optical_flow.integration_timespan_us / 2, // correct timestamp to midpoint of integration interval as the data is converted to rates
			.flow_rate = flow_rate,
			.gyro_rate = gyro_rate,
			.quality = optical_flow.quality
		};

		if (Vector2f(optical_flow.pixel_flow).isAllFinite() && optical_flow.integration_timespan_us < 1e6) {

			// Save sensor limits reported by the optical flow sensor
			_ekf.set_optical_flow_limits(optical_flow.max_flow_rate, optical_flow.min_ground_distance,
						     optical_flow.max_ground_distance);

			_ekf.setOpticalFlowData(flow);

			new_optical_flow = true;
		}

#if defined(CONFIG_EKF2_RANGE_FINDER)

		// use optical_flow distance as range sample if distance_sensor unavailable
		if (PX4_ISFINITE(optical_flow.distance_m) && (ekf2_timestamps.timestamp > _last_range_sensor_update + 1_s)) {

			int8_t quality = static_cast<float>(optical_flow.quality) / static_cast<float>(UINT8_MAX) * 100.f;

			estimator::sensor::rangeSample range_sample {
				.time_us = optical_flow.timestamp_sample,
				.rng = optical_flow.distance_m,
				.quality = quality,
			};
			_ekf.setRangeData(range_sample);

			// set sensor limits
			_ekf.set_rangefinder_limits(optical_flow.min_ground_distance, optical_flow.max_ground_distance);
		}

#endif // CONFIG_EKF2_RANGE_FINDER

		ekf2_timestamps.optical_flow_timestamp_rel = (int16_t)((int64_t)optical_flow.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}

	return new_optical_flow;
}
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_GNSS)
void EKF2::UpdateGpsSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF GPS message
	sensor_gps_s vehicle_gps_position;

	if (_vehicle_gps_position_sub.update(&vehicle_gps_position)) {

		Vector3f vel_ned;

		if (vehicle_gps_position.vel_ned_valid) {
			vel_ned = Vector3f(vehicle_gps_position.vel_n_m_s,
					   vehicle_gps_position.vel_e_m_s,
					   vehicle_gps_position.vel_d_m_s);

		} else {
			return; //TODO: change and set to NAN
		}

		if (fabsf(_param_ekf2_gps_yaw_off.get()) > 0.f) {
			if (!PX4_ISFINITE(vehicle_gps_position.heading_offset) && PX4_ISFINITE(vehicle_gps_position.heading)) {
				// Apply offset
				float yaw_offset = matrix::wrap_pi(math::radians(_param_ekf2_gps_yaw_off.get()));
				vehicle_gps_position.heading_offset = yaw_offset;
				vehicle_gps_position.heading = matrix::wrap_pi(vehicle_gps_position.heading - yaw_offset);
			}
		}

		const float altitude_amsl = static_cast<float>(vehicle_gps_position.altitude_msl_m);
		const float altitude_ellipsoid = static_cast<float>(vehicle_gps_position.altitude_ellipsoid_m);

		gnssSample gnss_sample{
			.time_us = vehicle_gps_position.timestamp,
			.lat = vehicle_gps_position.latitude_deg,
			.lon = vehicle_gps_position.longitude_deg,
			.alt = altitude_amsl,
			.vel = vel_ned,
			.hacc = vehicle_gps_position.eph,
			.vacc = vehicle_gps_position.epv,
			.sacc = vehicle_gps_position.s_variance_m_s,
			.fix_type = vehicle_gps_position.fix_type,
			.nsats = vehicle_gps_position.satellites_used,
			.pdop = sqrtf(vehicle_gps_position.hdop *vehicle_gps_position.hdop
				      + vehicle_gps_position.vdop * vehicle_gps_position.vdop),
			.yaw = vehicle_gps_position.heading, //TODO: move to different message
			.yaw_acc = vehicle_gps_position.heading_accuracy,
			.yaw_offset = vehicle_gps_position.heading_offset,
			.spoofed = vehicle_gps_position.spoofing_state == sensor_gps_s::SPOOFING_STATE_MULTIPLE,
		};

		_ekf.setGpsData(gnss_sample);

		const float geoid_height = altitude_ellipsoid - altitude_amsl;

		if (_last_geoid_height_update_us == 0) {
			_geoid_height_lpf.reset(geoid_height);
			_last_geoid_height_update_us = gnss_sample.time_us;

		} else if (gnss_sample.time_us > _last_geoid_height_update_us) {
			const float dt = 1e-6f * (gnss_sample.time_us - _last_geoid_height_update_us);
			_geoid_height_lpf.setParameters(dt, kGeoidHeightLpfTimeConstant);
			_geoid_height_lpf.update(geoid_height);
			_last_geoid_height_update_us = gnss_sample.time_us;
		}

	}
}

float EKF2::altEllipsoidToAmsl(float ellipsoid_alt) const
{
	return ellipsoid_alt - _geoid_height_lpf.getState();
}

float EKF2::altAmslToEllipsoid(float amsl_alt) const
{
	return amsl_alt + _geoid_height_lpf.getState();
}
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_MAGNETOMETER)
void EKF2::UpdateMagSample(ekf2_timestamps_s &ekf2_timestamps)
{
	vehicle_magnetometer_s magnetometer;

	if (_magnetometer_sub.update(&magnetometer)) {

		bool reset = false;

		// check if magnetometer has changed
		if (magnetometer.device_id != _device_id_mag) {
			if (_device_id_mag != 0) {
				PX4_DEBUG("%d - mag sensor ID changed %" PRIu32 " -> %" PRIu32, _instance, _device_id_mag, magnetometer.device_id);
			}

			reset = true;

		} else if (magnetometer.calibration_count != _mag_calibration_count) {
			// existing calibration has changed, reset saved mag bias
			PX4_DEBUG("%d - mag %" PRIu32 " calibration updated, resetting bias", _instance, _device_id_mag);
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
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_RANGE_FINDER)
void EKF2::UpdateRangeSample(ekf2_timestamps_s &ekf2_timestamps)
{
	distance_sensor_s distance_sensor;

	if (_distance_sensor_selected < 0) {

		// only consider distance sensors that have updated within the last 0.1s
		const hrt_abstime timestamp_stale = math::max(ekf2_timestamps.timestamp, 100_ms) - 100_ms;

		if (_distance_sensor_subs.advertised()) {
			for (unsigned i = 0; i < _distance_sensor_subs.size(); i++) {

				if (_distance_sensor_subs[i].update(&distance_sensor)) {
					// only use the first instace which has the correct orientation
					if ((distance_sensor.timestamp != 0) && (distance_sensor.timestamp > timestamp_stale)
					    && (distance_sensor.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING)) {

						int ndist = orb_group_count(ORB_ID(distance_sensor));

						if (ndist > 1) {
							PX4_INFO("%d - selected distance_sensor:%d (%d advertised)", _instance, i, ndist);
						}

						_distance_sensor_selected = i;
						_last_range_sensor_update = distance_sensor.timestamp;
						break;
					}
				}
			}
		}
	}

	if (_distance_sensor_selected >= 0 && _distance_sensor_subs[_distance_sensor_selected].update(&distance_sensor)) {
		// EKF range sample
		if (distance_sensor.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING) {
			estimator::sensor::rangeSample range_sample {
				.time_us = distance_sensor.timestamp,
				.rng = distance_sensor.current_distance,
				.quality = distance_sensor.signal_quality,
			};
			_ekf.setRangeData(range_sample);

			// Save sensor limits reported by the rangefinder
			_ekf.set_rangefinder_limits(distance_sensor.min_distance, distance_sensor.max_distance);

			_last_range_sensor_update = ekf2_timestamps.timestamp;
		}

		ekf2_timestamps.distance_sensor_timestamp_rel = (int16_t)((int64_t)distance_sensor.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}

	if (_last_range_sensor_update < ekf2_timestamps.timestamp - 1_s) {
		// force reselection after timeout
		_distance_sensor_selected = -1;
	}
}
#endif // CONFIG_EKF2_RANGE_FINDER

void EKF2::UpdateSystemFlagsSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF system flags
	if (_status_sub.updated() || _vehicle_land_detected_sub.updated()) {

		systemFlagUpdate flags{};
		flags.time_us = ekf2_timestamps.timestamp;

		// vehicle_status
		vehicle_status_s vehicle_status;

		if (_status_sub.copy(&vehicle_status)
		    && (ekf2_timestamps.timestamp < vehicle_status.timestamp + 3_s)) {

			// initially set in_air from arming_state (will be overridden if land detector is available)
			flags.in_air = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			// let the EKF know if the vehicle motion is that of a fixed wing (forward flight only relative to wind)
			flags.is_fixed_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);

#if defined(CONFIG_EKF2_SIDESLIP)

			if (vehicle_status.is_vtol_tailsitter && _params->beta_fusion_enabled) {
				PX4_WARN("Disable EKF beta fusion as unsupported for tailsitter");
				_param_ekf2_fuse_beta.set(0);
				_param_ekf2_fuse_beta.commit_no_notification();
			}

#endif // CONFIG_EKF2_SIDESLIP
		}

		// vehicle_land_detected
		vehicle_land_detected_s vehicle_land_detected;

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)
		    && (ekf2_timestamps.timestamp < vehicle_land_detected.timestamp + 3_s)) {

			flags.at_rest = vehicle_land_detected.at_rest;
			flags.in_air = !vehicle_land_detected.landed;
			flags.gnd_effect = vehicle_land_detected.in_ground_effect;
		}

		launch_detection_status_s launch_detection_status;

		if (_launch_detection_status_sub.copy(&launch_detection_status)
		    && (ekf2_timestamps.timestamp < launch_detection_status.timestamp + 3_s)) {

			flags.constant_pos = (launch_detection_status.launch_detection_state ==
					      launch_detection_status_s::STATE_WAITING_FOR_LAUNCH);
		}

		_ekf.setSystemFlagData(flags);
	}
}

void EKF2::UpdateCalibration(const hrt_abstime &timestamp, InFlightCalibration &cal, const matrix::Vector3f &bias,
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

void EKF2::UpdateAccelCalibration(const hrt_abstime &timestamp)
{
	// the EKF is operating in the correct mode and there are no filter faults
	const bool bias_valid = (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::AccelBias))
				&& _ekf.control_status_flags().tilt_align
				&& (_ekf.fault_status().value == 0)
				&& !_ekf.fault_status_flags().bad_acc_clipping
				&& !_ekf.fault_status_flags().bad_acc_vertical;

	const bool learning_valid = bias_valid && !_ekf.accel_bias_inhibited();

	UpdateCalibration(timestamp, _accel_cal, _ekf.getAccelBias(), _ekf.getAccelBiasVariance(), _ekf.getAccelBiasLimit(),
			  bias_valid, learning_valid);
}

void EKF2::UpdateGyroCalibration(const hrt_abstime &timestamp)
{
	// the EKF is operating in the correct mode and there are no filter faults
	const bool bias_valid = (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::GyroBias))
				&& _ekf.control_status_flags().tilt_align
				&& (_ekf.fault_status().value == 0);

	const bool learning_valid = bias_valid && !_ekf.gyro_bias_inhibited();

	UpdateCalibration(timestamp, _gyro_cal, _ekf.getGyroBias(), _ekf.getGyroBiasVariance(), _ekf.getGyroBiasLimit(),
			  bias_valid, learning_valid);
}

#if defined(CONFIG_EKF2_MAGNETOMETER)
void EKF2::UpdateMagCalibration(const hrt_abstime &timestamp)
{
	const Vector3f mag_bias = _ekf.getMagBias();
	const Vector3f mag_bias_var = _ekf.getMagBiasVariance();

	const bool bias_valid = (_ekf.fault_status().value == 0)
				&& _ekf.control_status_flags().yaw_align
				&& mag_bias_var.longerThan(0.f) && !mag_bias_var.longerThan(0.02f);

	const bool learning_valid = bias_valid && _ekf.control_status_flags().mag;

	UpdateCalibration(timestamp, _mag_cal, mag_bias, mag_bias_var, _ekf.getMagBiasLimit(), bias_valid, learning_valid);

	// update stored declination value
	if (!_mag_decl_saved) {
		float declination_deg;

		if (_ekf.get_mag_decl_deg(declination_deg)) {
			_param_ekf2_mag_decl.update();

			if (PX4_ISFINITE(declination_deg) && (fabsf(declination_deg - _param_ekf2_mag_decl.get()) > 0.1f)) {
				_param_ekf2_mag_decl.set(declination_deg);
				_param_ekf2_mag_decl.commit_no_notification();
			}

			_mag_decl_saved = true;
		}
	}
}
#endif // CONFIG_EKF2_MAGNETOMETER

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

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	bool multi_mode = false;
	int32_t imu_instances = 0;
	int32_t mag_instances = 0;

	int32_t sens_imu_mode = 1;
	param_get(param_find("SENS_IMU_MODE"), &sens_imu_mode);

	if (sens_imu_mode == 0) {
		// ekf selector requires SENS_IMU_MODE = 0
		multi_mode = true;

		// IMUs (1 - MAX_NUM_IMUS supported)
		param_get(param_find("EKF2_MULTI_IMU"), &imu_instances);

		if (imu_instances < 1 || imu_instances > MAX_NUM_IMUS) {
			const int32_t imu_instances_limited = math::constrain(imu_instances, static_cast<int32_t>(1),
							      static_cast<int32_t>(MAX_NUM_IMUS));
			PX4_WARN("EKF2_MULTI_IMU limited %" PRId32 " -> %" PRId32, imu_instances, imu_instances_limited);
			param_set_no_notification(param_find("EKF2_MULTI_IMU"), &imu_instances_limited);
			imu_instances = imu_instances_limited;
		}

#if defined(CONFIG_EKF2_MAGNETOMETER)
		int32_t sens_mag_mode = 1;
		const param_t param_sens_mag_mode = param_find("SENS_MAG_MODE");
		param_get(param_sens_mag_mode, &sens_mag_mode);

		if (sens_mag_mode == 0) {
			const param_t param_ekf2_mult_mag = param_find("EKF2_MULTI_MAG");
			param_get(param_ekf2_mult_mag, &mag_instances);

			// Mags (1 - MAX_NUM_MAGS supported)
			if (mag_instances > MAX_NUM_MAGS) {
				const int32_t mag_instances_limited = math::constrain(mag_instances, static_cast<int32_t>(1),
								      static_cast<int32_t>(MAX_NUM_MAGS));
				PX4_WARN("EKF2_MULTI_MAG limited %" PRId32 " -> %" PRId32, mag_instances, mag_instances_limited);
				param_set_no_notification(param_ekf2_mult_mag, &mag_instances_limited);
				mag_instances = mag_instances_limited;

			} else if (mag_instances <= 1) {
				// properly disable multi-magnetometer at sensors hub level
				PX4_WARN("EKF2_MULTI_MAG disabled, resetting SENS_MAG_MODE");

				// re-enable at sensors level
				sens_mag_mode = 1;
				param_set(param_sens_mag_mode, &sens_mag_mode);

				mag_instances = 1;
			}

		} else {
			mag_instances = 1;
		}

#endif // CONFIG_EKF2_MAGNETOMETER
	}

	if (multi_mode && !replay_mode) {
		// Start EKF2Selector if it's not already running
		if (_ekf2_selector.load() == nullptr) {
			EKF2Selector *inst = new EKF2Selector();

			if (inst) {
				_ekf2_selector.store(inst);

			} else {
				PX4_ERR("Failed to create EKF2 selector");
				return PX4_ERROR;
			}
		}

		const hrt_abstime time_started = hrt_absolute_time();
		const int multi_instances = math::min(imu_instances * mag_instances, static_cast<int32_t>(EKF2_MAX_INSTANCES));
		int multi_instances_allocated = 0;

		// allocate EKF2 instances until all found or arming
		uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};

		bool ekf2_instance_created[MAX_NUM_IMUS][MAX_NUM_MAGS] {}; // IMUs * mags

		while ((multi_instances_allocated < multi_instances)
		       && (vehicle_status_sub.get().arming_state != vehicle_status_s::ARMING_STATE_ARMED)
		       && ((hrt_elapsed_time(&time_started) < 30_s)
			   || (vehicle_status_sub.get().hil_state == vehicle_status_s::HIL_STATE_ON))) {

			vehicle_status_sub.update();

			for (uint8_t mag = 0; mag < mag_instances; mag++) {
				uORB::SubscriptionData<vehicle_magnetometer_s> vehicle_mag_sub{ORB_ID(vehicle_magnetometer), mag};

				for (uint8_t imu = 0; imu < imu_instances; imu++) {

					uORB::SubscriptionData<vehicle_imu_s> vehicle_imu_sub{ORB_ID(vehicle_imu), imu};
					vehicle_mag_sub.update();

					// Mag & IMU data must be valid, first mag can be ignored initially
					if ((vehicle_mag_sub.advertised() || mag == 0) && (vehicle_imu_sub.advertised())) {

						if (!ekf2_instance_created[imu][mag]) {
							EKF2 *ekf2_inst = new EKF2(true, px4::ins_instance_to_wq(imu), false);

							if (ekf2_inst && ekf2_inst->multi_init(imu, mag)) {
								int actual_instance = ekf2_inst->instance(); // match uORB instance numbering

								if ((actual_instance >= 0) && (_objects[actual_instance].load() == nullptr)) {
									_objects[actual_instance].store(ekf2_inst);
									success = true;
									multi_instances_allocated++;
									ekf2_instance_created[imu][mag] = true;

									PX4_DEBUG("starting instance %d, IMU:%" PRIu8 " (%" PRIu32 "), MAG:%" PRIu8 " (%" PRIu32 ")", actual_instance,
										  imu, vehicle_imu_sub.get().accel_device_id,
										  mag, vehicle_mag_sub.get().device_id);

									_ekf2_selector.load()->ScheduleNow();

								} else {
									PX4_ERR("instance numbering problem instance: %d", actual_instance);
									delete ekf2_inst;
									break;
								}

							} else {
								PX4_ERR("alloc and init failed imu: %" PRIu8 " mag:%" PRIu8, imu, mag);
								px4_usleep(100000);
								break;
							}
						}

					} else {
						px4_usleep(1000); // give the sensors extra time to start
						break;
					}
				}
			}

			if (multi_instances_allocated < multi_instances) {
				px4_usleep(10000);
			}
		}

	} else

#endif // CONFIG_EKF2_MULTI_INSTANCE

	{
		// otherwise launch regular
		EKF2 *ekf2_inst = new EKF2(false, px4::wq_configurations::INS0, replay_mode);

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

The documentation can be found on the [ECL/EKF Overview & Tuning](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html) page.

ekf2 can be started in replay mode (`-r`): in this mode, it does not access the system time, but only uses the
timestamps from the sensor topics.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ekf2", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('r', "Enable replay mode", true);
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "print status info");
#if defined(CONFIG_EKF2_VERBOSE_STATUS)
	PRINT_MODULE_USAGE_ARG("-v", "verbose (print all states and full covariance matrix)", true);
#endif // CONFIG_EKF2_VERBOSE_STATUS
#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	PRINT_MODULE_USAGE_COMMAND_DESCR("select_instance", "Request switch to new estimator instance");
	PRINT_MODULE_USAGE_ARG("<instance>", "Specify desired estimator instance", false);
#endif // CONFIG_EKF2_MULTI_INSTANCE
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

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	} else if (strcmp(argv[1], "select_instance") == 0) {

		if (EKF2::trylock_module()) {
			if (_ekf2_selector.load()) {
				if (argc > 2) {
					int instance = atoi(argv[2]);
					_ekf2_selector.load()->RequestInstance(instance);
				} else {
					EKF2::unlock_module();
					return EKF2::print_usage("instance required");
				}

			} else {
				PX4_ERR("multi-EKF not active, unable to select instance");
			}

			EKF2::unlock_module();

		} else {
			PX4_WARN("module locked, try again later");
		}

		return 0;
#endif // CONFIG_EKF2_MULTI_INSTANCE
	} else if (strcmp(argv[1], "status") == 0) {
		if (EKF2::trylock_module()) {
#if defined(CONFIG_EKF2_MULTI_INSTANCE)
			if (_ekf2_selector.load()) {
				_ekf2_selector.load()->PrintStatus();
			}
#endif // CONFIG_EKF2_MULTI_INSTANCE

			bool verbose_status = false;

#if defined(CONFIG_EKF2_VERBOSE_STATUS)
			if (argc > 2 && (strcmp(argv[2], "-v") == 0)) {
				verbose_status = true;
			}
#endif // CONFIG_EKF2_VERBOSE_STATUS

			for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
				if (_objects[i].load()) {
					PX4_INFO_RAW("\n");
					_objects[i].load()->print_status(verbose_status);
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

			if (instance >= 0 && instance < EKF2_MAX_INSTANCES) {
				PX4_INFO("stopping instance %d", instance);
				EKF2 *inst = _objects[instance].load();

				if (inst) {
					inst->request_stop();
					px4_usleep(20000); // 20 ms
					delete inst;
					_objects[instance].store(nullptr);
				}
			} else {
				PX4_ERR("invalid instance %d", instance);
			}

		} else {
			// otherwise stop everything
			bool was_running = false;

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
			if (_ekf2_selector.load()) {
				PX4_INFO("stopping ekf2 selector");
				_ekf2_selector.load()->Stop();
				delete _ekf2_selector.load();
				_ekf2_selector.store(nullptr);
				was_running = true;
			}
#endif // CONFIG_EKF2_MULTI_INSTANCE

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
