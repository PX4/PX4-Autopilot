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

/**
 * @file EKF2.cpp
 * Implementation of the attitude and position estimator.
 *
 * @author Roman Bapst
 */

#ifndef EKF2_HPP
#define EKF2_HPP

#include "EKF/ekf.h"
#include "Utility/PreFlightChecker.hpp"

#include "EKF2Selector.hpp"

#include <float.h>

#include <containers/LockGuard.hpp>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/estimator_bias.h>
#include <uORB/topics/estimator_bias3d.h>
#include <uORB/topics/estimator_event_flags.h>
#include <uORB/topics/estimator_innovations.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/estimator_states.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/yaw_estimator_status.h>

#if defined(CONFIG_EKF2_AIRSPEED)
# include <uORB/topics/airspeed.h>
# include <uORB/topics/airspeed_validated.h>
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_AUXVEL)
# include <uORB/topics/landing_target_pose.h>
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_BAROMETER)
# include <uORB/topics/vehicle_air_data.h>
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GNSS)
# include <uORB/topics/estimator_gps_status.h>
# include <uORB/topics/sensor_gps.h>
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_MAGNETOMETER)
# include <uORB/topics/vehicle_magnetometer.h>
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
# include <uORB/topics/vehicle_optical_flow.h>
# include <uORB/topics/vehicle_optical_flow_vel.h>
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_RANGE_FINDER)
# include <uORB/topics/distance_sensor.h>
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_WIND)
# include <uORB/topics/wind.h>
#endif // CONFIG_EKF2_WIND

extern pthread_mutex_t ekf2_module_mutex;

class EKF2 final : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	EKF2() = delete;
	EKF2(bool multi_mode, const px4::wq_config_t &config, bool replay_mode);
	~EKF2() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int print_status(bool verbose = false);

	bool should_exit() const { return _task_should_exit.load(); }

	void request_stop() { _task_should_exit.store(true); }

	static void lock_module() { pthread_mutex_lock(&ekf2_module_mutex); }
	static bool trylock_module() { return (pthread_mutex_trylock(&ekf2_module_mutex) == 0); }
	static void unlock_module() { pthread_mutex_unlock(&ekf2_module_mutex); }

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	bool multi_init(int imu, int mag);
#endif // CONFIG_EKF2_MULTI_INSTANCE

	int instance() const { return _instance; }

private:

	static constexpr uint8_t MAX_NUM_IMUS = 4;
	static constexpr uint8_t MAX_NUM_MAGS = 4;

	void Run() override;

	void VerifyParams();

	void PublishAidSourceStatus(const hrt_abstime &timestamp);
	void PublishAttitude(const hrt_abstime &timestamp);

#if defined(CONFIG_EKF2_BAROMETER)
	void PublishBaroBias(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_RANGE_FINDER)
	void PublishRngHgtBias(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	void PublishEvPosBias(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_EXTERNAL_VISION
	estimator_bias_s fillEstimatorBiasMsg(const BiasEstimator::status &status, uint64_t timestamp_sample_us,
					      uint64_t timestamp, uint32_t device_id = 0);
	void PublishEventFlags(const hrt_abstime &timestamp);
	void PublishGlobalPosition(const hrt_abstime &timestamp);
	void PublishInnovations(const hrt_abstime &timestamp);
	void PublishInnovationTestRatios(const hrt_abstime &timestamp);
	void PublishInnovationVariances(const hrt_abstime &timestamp);
	void PublishLocalPosition(const hrt_abstime &timestamp);
	void PublishOdometry(const hrt_abstime &timestamp, const imuSample &imu_sample);
	void PublishSensorBias(const hrt_abstime &timestamp);
	void PublishStates(const hrt_abstime &timestamp);
	void PublishStatus(const hrt_abstime &timestamp);
	void PublishStatusFlags(const hrt_abstime &timestamp);
#if defined(CONFIG_EKF2_WIND)
	void PublishWindEstimate(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_AIRSPEED)
	void UpdateAirspeedSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_AUXVEL)
	void UpdateAuxVelSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_AUXVEL
#if defined(CONFIG_EKF2_BAROMETER)
	void UpdateBaroSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_BAROMETER
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	bool UpdateExtVisionSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_GNSS)
	/*
	 * Calculate filtered WGS84 height from estimated AMSL height
	 */
	float filter_altitude_ellipsoid(float amsl_hgt);

	void PublishGpsStatus(const hrt_abstime &timestamp);
	void PublishGnssHgtBias(const hrt_abstime &timestamp);
	void PublishYawEstimatorStatus(const hrt_abstime &timestamp);
	void UpdateGpsSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_GNSS
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	bool UpdateFlowSample(ekf2_timestamps_s &ekf2_timestamps);
	void PublishOpticalFlowVel(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_OPTICAL_FLOW
#if defined(CONFIG_EKF2_MAGNETOMETER)
	void UpdateMagSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_MAGNETOMETER
#if defined(CONFIG_EKF2_RANGE_FINDER)
	void UpdateRangeSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_RANGE_FINDER

	void UpdateSystemFlagsSample(ekf2_timestamps_s &ekf2_timestamps);

	// Used to check, save and use learned accel/gyro/mag biases
	struct InFlightCalibration {
		hrt_abstime last_us{0};         ///< last time the EKF was operating a mode that estimates accelerometer biases (uSec)
		hrt_abstime total_time_us{0};   ///< accumulated calibration time since the last save
		matrix::Vector3f bias{};
		bool cal_available{false};      ///< true when an unsaved valid calibration for the XYZ accelerometer bias is available
	};

	void UpdateCalibration(const hrt_abstime &timestamp, InFlightCalibration &cal, const matrix::Vector3f &bias,
			       const matrix::Vector3f &bias_variance, float bias_limit, bool bias_valid, bool learning_valid);
	void UpdateAccelCalibration(const hrt_abstime &timestamp);
	void UpdateGyroCalibration(const hrt_abstime &timestamp);
#if defined(CONFIG_EKF2_MAGNETOMETER)
	void UpdateMagCalibration(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_MAGNETOMETER

	// publish helper for estimator_aid_source topics
	template <typename T>
	void PublishAidSourceStatus(const T &status, hrt_abstime &status_publish_last, uORB::PublicationMulti<T> &pub)
	{
		if (status.timestamp_sample > status_publish_last) {
			// publish if updated
			T status_out{status};
			status_out.estimator_instance = _instance;
			status_out.timestamp = hrt_absolute_time();
			pub.publish(status_out);

			// record timestamp sample
			status_publish_last = status.timestamp_sample;
		}
	}

	static constexpr float sq(float x) { return x * x; };

	const bool _replay_mode{false};			///< true when we use replay data from a log
	const bool _multi_mode;
	int _instance{0};

	px4::atomic_bool _task_should_exit{false};

	// time slip monitoring
	uint64_t _integrated_time_us = 0;	///< integral of gyro delta time from start (uSec)
	uint64_t _start_time_us = 0;		///< system time at EKF start (uSec)
	int64_t _last_time_slip_us = 0;		///< Last time slip (uSec)

	perf_counter_t _ekf_update_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": EKF update")};
	perf_counter_t _msg_missed_imu_perf{perf_alloc(PC_COUNT, MODULE_NAME": IMU message missed")};

	InFlightCalibration _accel_cal{};
	InFlightCalibration _gyro_cal{};

	uint8_t _accel_calibration_count{0};
	uint8_t _gyro_calibration_count{0};

	uint32_t _device_id_accel{0};
	uint32_t _device_id_gyro{0};

	Vector3f _last_accel_bias_published{};
	Vector3f _last_gyro_bias_published{};

	hrt_abstime _last_sensor_bias_published{0};

	hrt_abstime _status_fake_hgt_pub_last{0};
	hrt_abstime _status_fake_pos_pub_last{0};

#if defined(CONFIG_EKF2_MAGNETOMETER)
	uint32_t _device_id_mag {0};

	// Used to control saving of mag declination to be used on next startup
	bool _mag_decl_saved = false;	///< true when the magnetic declination has been saved

	InFlightCalibration _mag_cal{};
	uint8_t _mag_calibration_count{0};
	Vector3f _last_mag_bias_published{};

	hrt_abstime _status_mag_pub_last{0};

	uORB::Subscription _magnetometer_sub{ORB_ID(vehicle_magnetometer)};

	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_mag_pub{ORB_ID(estimator_aid_src_mag)};
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_ev_hgt_pub {ORB_ID(estimator_aid_src_ev_hgt)};
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_ev_pos_pub{ORB_ID(estimator_aid_src_ev_pos)};
	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_ev_vel_pub{ORB_ID(estimator_aid_src_ev_vel)};
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_ev_yaw_pub{ORB_ID(estimator_aid_src_ev_yaw)};
	hrt_abstime _status_ev_hgt_pub_last{0};
	hrt_abstime _status_ev_pos_pub_last{0};
	hrt_abstime _status_ev_vel_pub_last{0};
	hrt_abstime _status_ev_yaw_pub_last{0};

	matrix::Vector3f _last_ev_bias_published{};

	uORB::Subscription _ev_odom_sub{ORB_ID(vehicle_visual_odometry)};

	uORB::PublicationMulti<estimator_bias3d_s> _estimator_ev_pos_bias_pub{ORB_ID(estimator_ev_pos_bias)};
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUXVEL)
	uORB::Subscription _landing_target_pose_sub {ORB_ID(landing_target_pose)};

	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_aux_vel_pub{ORB_ID(estimator_aid_src_aux_vel)};
	hrt_abstime _status_aux_vel_pub_last{0};
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_TERRAIN)

# if defined(CONFIG_EKF2_RANGE_FINDER)
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_terrain_range_finder_pub {ORB_ID(estimator_aid_src_terrain_range_finder)};
	hrt_abstime _status_terrain_range_finder_pub_last{0};
# endif // CONFIG_EKF2_RANGE_FINDER

# if defined(CONFIG_EKF2_OPTICAL_FLOW)
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_terrain_optical_flow_pub {ORB_ID(estimator_aid_src_terrain_optical_flow)};
	hrt_abstime _status_terrain_optical_flow_pub_last{0};
# endif // CONFIG_EKF2_OPTICAL_FLOW
#endif // CONFIG_EKF2_TERRAIN

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	uORB::Subscription _vehicle_optical_flow_sub {ORB_ID(vehicle_optical_flow)};
	uORB::PublicationMulti<vehicle_optical_flow_vel_s> _estimator_optical_flow_vel_pub{ORB_ID(estimator_optical_flow_vel)};

	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_optical_flow_pub{ORB_ID(estimator_aid_src_optical_flow)};
	hrt_abstime _status_optical_flow_pub_last{0};
	hrt_abstime _optical_flow_vel_pub_last{0};
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_BAROMETER)
	uint8_t _baro_calibration_count {0};
	uint32_t _device_id_baro{0};
	hrt_abstime _status_baro_hgt_pub_last{0};

	float _last_baro_bias_published{};

	uORB::Subscription _airdata_sub{ORB_ID(vehicle_air_data)};

	uORB::PublicationMulti<estimator_bias_s> _estimator_baro_bias_pub{ORB_ID(estimator_baro_bias)};
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_baro_hgt_pub {ORB_ID(estimator_aid_src_baro_hgt)};
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_DRAG_FUSION)
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_drag_pub {ORB_ID(estimator_aid_src_drag)};
	hrt_abstime _status_drag_pub_last{0};
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_AIRSPEED)
	uORB::Subscription _airspeed_sub {ORB_ID(airspeed)};
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};

	float _airspeed_scale_factor{1.0f}; ///< scale factor correction applied to airspeed measurements
	hrt_abstime _airspeed_validated_timestamp_last{0};

	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_airspeed_pub {ORB_ID(estimator_aid_src_airspeed)};
	hrt_abstime _status_airspeed_pub_last{0};
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_sideslip_pub {ORB_ID(estimator_aid_src_sideslip)};
	hrt_abstime _status_sideslip_pub_last {0};
#endif // CONFIG_EKF2_SIDESLIP

	orb_advert_t _mavlink_log_pub{nullptr};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	uORB::SubscriptionCallbackWorkItem _sensor_combined_sub{this, ORB_ID(sensor_combined)};
	uORB::SubscriptionCallbackWorkItem _vehicle_imu_sub{this, ORB_ID(vehicle_imu)};

#if defined(CONFIG_EKF2_RANGE_FINDER)
	hrt_abstime _status_rng_hgt_pub_last {0};
	float _last_rng_hgt_bias_published{};

	uORB::PublicationMulti<estimator_bias_s> _estimator_rng_hgt_bias_pub {ORB_ID(estimator_rng_hgt_bias)};
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_rng_hgt_pub{ORB_ID(estimator_aid_src_rng_hgt)};

	uORB::SubscriptionMultiArray<distance_sensor_s> _distance_sensor_subs{ORB_ID::distance_sensor};
	hrt_abstime _last_range_sensor_update{0};
	int _distance_sensor_selected{-1}; // because we can have several distance sensor instances with different orientations
#endif // CONFIG_EKF2_RANGE_FINDER

	bool _callback_registered{false};

	hrt_abstime _last_event_flags_publish{0};
	hrt_abstime _last_status_flags_publish{0};

	uint64_t _filter_control_status{0};
	uint32_t _filter_fault_status{0};
	uint32_t _innov_check_fail_status{0};

	uint32_t _filter_control_status_changes{0};
	uint32_t _filter_fault_status_changes{0};
	uint32_t _innov_check_fail_status_changes{0};
	uint32_t _filter_warning_event_changes{0};
	uint32_t _filter_information_event_changes{0};

	uORB::PublicationMulti<ekf2_timestamps_s>            _ekf2_timestamps_pub{ORB_ID(ekf2_timestamps)};
	uORB::PublicationMultiData<estimator_event_flags_s>  _estimator_event_flags_pub{ORB_ID(estimator_event_flags)};
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovation_test_ratios_pub{ORB_ID(estimator_innovation_test_ratios)};
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovation_variances_pub{ORB_ID(estimator_innovation_variances)};
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovations_pub{ORB_ID(estimator_innovations)};
	uORB::PublicationMulti<estimator_sensor_bias_s>      _estimator_sensor_bias_pub{ORB_ID(estimator_sensor_bias)};
	uORB::PublicationMulti<estimator_states_s>           _estimator_states_pub{ORB_ID(estimator_states)};
	uORB::PublicationMulti<estimator_status_flags_s>     _estimator_status_flags_pub{ORB_ID(estimator_status_flags)};
	uORB::PublicationMulti<estimator_status_s>           _estimator_status_pub{ORB_ID(estimator_status)};

	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_fake_hgt_pub{ORB_ID(estimator_aid_src_fake_hgt)};
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_fake_pos_pub{ORB_ID(estimator_aid_src_fake_pos)};

	// publications with topic dependent on multi-mode
	uORB::PublicationMulti<vehicle_attitude_s>           _attitude_pub;
	uORB::PublicationMulti<vehicle_local_position_s>     _local_position_pub;
	uORB::PublicationMulti<vehicle_global_position_s>    _global_position_pub;
	uORB::PublicationMulti<vehicle_odometry_s>           _odometry_pub;

#if defined(CONFIG_EKF2_WIND)
	uORB::PublicationMulti<wind_s>              _wind_pub;
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_GNSS)
	uint64_t _gps_time_usec {0};
	int32_t _gps_alttitude_ellipsoid{0};			///< altitude in 1E-3 meters (millimeters) above ellipsoid
	uint64_t _gps_alttitude_ellipsoid_previous_timestamp{0}; ///< storage for previous timestamp to compute dt
	float   _wgs84_hgt_offset = 0;  ///< height offset between AMSL and WGS84

	hrt_abstime _last_gps_status_published{0};

	hrt_abstime _status_gnss_hgt_pub_last{0};
	hrt_abstime _status_gnss_pos_pub_last{0};
	hrt_abstime _status_gnss_vel_pub_last{0};

	float _last_gnss_hgt_bias_published{};

	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};

	uORB::PublicationMulti<estimator_bias_s> _estimator_gnss_hgt_bias_pub{ORB_ID(estimator_gnss_hgt_bias)};
	uORB::PublicationMulti<estimator_gps_status_s> _estimator_gps_status_pub{ORB_ID(estimator_gps_status)};
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_gnss_hgt_pub{ORB_ID(estimator_aid_src_gnss_hgt)};
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_gnss_pos_pub{ORB_ID(estimator_aid_src_gnss_pos)};
	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_gnss_vel_pub{ORB_ID(estimator_aid_src_gnss_vel)};

	uORB::PublicationMulti<yaw_estimator_status_s> _yaw_est_pub{ORB_ID(yaw_estimator_status)};

# if defined(CONFIG_EKF2_GNSS_YAW)
	hrt_abstime _status_gnss_yaw_pub_last {0};
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_gnss_yaw_pub {ORB_ID(estimator_aid_src_gnss_yaw)};
# endif // CONFIG_EKF2_GNSS_YAW
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	hrt_abstime _status_gravity_pub_last {0};
	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_gravity_pub{ORB_ID(estimator_aid_src_gravity)};
#endif // CONFIG_EKF2_GRAVITY_FUSION

	PreFlightChecker _preflt_checker;

	Ekf _ekf;

	parameters *_params;	///< pointer to ekf parameter struct (located in _ekf class instance)

	DEFINE_PARAMETERS(
		(ParamExtInt<px4::params::EKF2_PREDICT_US>) _param_ekf2_predict_us,
		(ParamExtFloat<px4::params::EKF2_DELAY_MAX>) _param_ekf2_delay_max,
		(ParamExtInt<px4::params::EKF2_IMU_CTRL>) _param_ekf2_imu_ctrl,

#if defined(CONFIG_EKF2_AUXVEL)
		(ParamExtFloat<px4::params::EKF2_AVEL_DELAY>)
		_param_ekf2_avel_delay,	///< auxiliary velocity measurement delay relative to the IMU (mSec)
#endif // CONFIG_EKF2_AUXVEL

		(ParamExtFloat<px4::params::EKF2_GYR_NOISE>)
		_param_ekf2_gyr_noise,	///< IMU angular rate noise used for covariance prediction (rad/sec)
		(ParamExtFloat<px4::params::EKF2_ACC_NOISE>)
		_param_ekf2_acc_noise,	///< IMU acceleration noise use for covariance prediction (m/sec**2)

		// process noise
		(ParamExtFloat<px4::params::EKF2_GYR_B_NOISE>)
		_param_ekf2_gyr_b_noise,	///< process noise for IMU rate gyro bias prediction (rad/sec**2)
		(ParamExtFloat<px4::params::EKF2_ACC_B_NOISE>)
		_param_ekf2_acc_b_noise,///< process noise for IMU accelerometer bias prediction (m/sec**3)

#if defined(CONFIG_EKF2_WIND)
		(ParamExtFloat<px4::params::EKF2_WIND_NSD>) _param_ekf2_wind_nsd,
#endif // CONFIG_EKF2_WIND

		(ParamExtFloat<px4::params::EKF2_NOAID_NOISE>) _param_ekf2_noaid_noise,

#if defined(CONFIG_EKF2_GNSS)
		(ParamExtInt<px4::params::EKF2_GPS_CTRL>) _param_ekf2_gps_ctrl,
		(ParamExtFloat<px4::params::EKF2_GPS_DELAY>) _param_ekf2_gps_delay,

		(ParamExtFloat<px4::params::EKF2_GPS_POS_X>) _param_ekf2_gps_pos_x,
		(ParamExtFloat<px4::params::EKF2_GPS_POS_Y>) _param_ekf2_gps_pos_y,
		(ParamExtFloat<px4::params::EKF2_GPS_POS_Z>) _param_ekf2_gps_pos_z,

		(ParamExtFloat<px4::params::EKF2_GPS_V_NOISE>) _param_ekf2_gps_v_noise,
		(ParamExtFloat<px4::params::EKF2_GPS_P_NOISE>) _param_ekf2_gps_p_noise,

		(ParamExtFloat<px4::params::EKF2_GPS_P_GATE>) _param_ekf2_gps_p_gate,
		(ParamExtFloat<px4::params::EKF2_GPS_V_GATE>) _param_ekf2_gps_v_gate,

		(ParamExtInt<px4::params::EKF2_GPS_CHECK>) _param_ekf2_gps_check,
		(ParamExtFloat<px4::params::EKF2_REQ_EPH>)    _param_ekf2_req_eph,
		(ParamExtFloat<px4::params::EKF2_REQ_EPV>)    _param_ekf2_req_epv,
		(ParamExtFloat<px4::params::EKF2_REQ_SACC>)   _param_ekf2_req_sacc,
		(ParamExtInt<px4::params::EKF2_REQ_NSATS>)    _param_ekf2_req_nsats,
		(ParamExtFloat<px4::params::EKF2_REQ_PDOP>)   _param_ekf2_req_pdop,
		(ParamExtFloat<px4::params::EKF2_REQ_HDRIFT>) _param_ekf2_req_hdrift,
		(ParamExtFloat<px4::params::EKF2_REQ_VDRIFT>) _param_ekf2_req_vdrift,
		(ParamFloat<px4::params::EKF2_REQ_GPS_H>)     _param_ekf2_req_gps_h,

		// Used by EKF-GSF experimental yaw estimator
		(ParamExtFloat<px4::params::EKF2_GSF_TAS>) _param_ekf2_gsf_tas_default,
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_BAROMETER)
		(ParamExtInt<px4::params::EKF2_BARO_CTRL>) _param_ekf2_baro_ctrl,///< barometer control selection
		(ParamExtFloat<px4::params::EKF2_BARO_DELAY>) _param_ekf2_baro_delay,
		(ParamExtFloat<px4::params::EKF2_BARO_NOISE>) _param_ekf2_baro_noise,
		(ParamExtFloat<px4::params::EKF2_BARO_GATE>) _param_ekf2_baro_gate,
		(ParamExtFloat<px4::params::EKF2_GND_EFF_DZ>) _param_ekf2_gnd_eff_dz,
		(ParamExtFloat<px4::params::EKF2_GND_MAX_HGT>) _param_ekf2_gnd_max_hgt,

# if defined(CONFIG_EKF2_BARO_COMPENSATION)
		// Corrections for static pressure position error where Ps_error = Ps_meas - Ps_truth
		(ParamExtFloat<px4::params::EKF2_ASPD_MAX>) _param_ekf2_aspd_max,
		(ParamExtFloat<px4::params::EKF2_PCOEF_XP>) _param_ekf2_pcoef_xp,
		(ParamExtFloat<px4::params::EKF2_PCOEF_XN>) _param_ekf2_pcoef_xn,
		(ParamExtFloat<px4::params::EKF2_PCOEF_YP>) _param_ekf2_pcoef_yp,
		(ParamExtFloat<px4::params::EKF2_PCOEF_YN>) _param_ekf2_pcoef_yn,
		(ParamExtFloat<px4::params::EKF2_PCOEF_Z>) _param_ekf2_pcoef_z,
# endif // CONFIG_EKF2_BARO_COMPENSATION
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_AIRSPEED)
		(ParamExtFloat<px4::params::EKF2_ASP_DELAY>)
		_param_ekf2_asp_delay, ///< airspeed measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_TAS_GATE>)
		_param_ekf2_tas_gate, ///< True Airspeed innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_EAS_NOISE>)
		_param_ekf2_eas_noise, ///< measurement noise used for airspeed fusion (m/sec)

		// control of airspeed fusion
		(ParamExtFloat<px4::params::EKF2_ARSP_THR>)
		_param_ekf2_arsp_thr, ///< A value of zero will disabled airspeed fusion. Any positive value sets the minimum airspeed which will be used (m/sec)
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
		(ParamExtFloat<px4::params::EKF2_BETA_GATE>)
		_param_ekf2_beta_gate, ///< synthetic sideslip innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_BETA_NOISE>) _param_ekf2_beta_noise, ///< synthetic sideslip noise (rad)

		(ParamExtInt<px4::params::EKF2_FUSE_BETA>)
		_param_ekf2_fuse_beta, ///< Controls synthetic sideslip fusion, 0 disables, 1 enables
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_MAGNETOMETER)
		(ParamExtFloat<px4::params::EKF2_MAG_DELAY>) _param_ekf2_mag_delay,
		(ParamExtFloat<px4::params::EKF2_MAG_E_NOISE>) _param_ekf2_mag_e_noise,
		(ParamExtFloat<px4::params::EKF2_MAG_B_NOISE>) _param_ekf2_mag_b_noise,
		(ParamExtFloat<px4::params::EKF2_HEAD_NOISE>) _param_ekf2_head_noise,
		(ParamExtFloat<px4::params::EKF2_MAG_NOISE>) _param_ekf2_mag_noise,
		(ParamExtFloat<px4::params::EKF2_MAG_DECL>) _param_ekf2_mag_decl,
		(ParamExtFloat<px4::params::EKF2_HDG_GATE>) _param_ekf2_hdg_gate,
		(ParamExtFloat<px4::params::EKF2_MAG_GATE>) _param_ekf2_mag_gate,
		(ParamExtInt<px4::params::EKF2_DECL_TYPE>) _param_ekf2_decl_type,
		(ParamExtInt<px4::params::EKF2_MAG_TYPE>) _param_ekf2_mag_type,
		(ParamExtFloat<px4::params::EKF2_MAG_ACCLIM>) _param_ekf2_mag_acclim,
		(ParamExtInt<px4::params::EKF2_MAG_CHECK>) _param_ekf2_mag_check,
		(ParamExtFloat<px4::params::EKF2_MAG_CHK_STR>) _param_ekf2_mag_chk_str,
		(ParamExtFloat<px4::params::EKF2_MAG_CHK_INC>) _param_ekf2_mag_chk_inc,
		(ParamExtInt<px4::params::EKF2_SYNT_MAG_Z>) _param_ekf2_synthetic_mag_z,
#endif // CONFIG_EKF2_MAGNETOMETER

		(ParamExtInt<px4::params::EKF2_HGT_REF>) _param_ekf2_hgt_ref,    ///< selects the primary source for height data

		(ParamExtInt<px4::params::EKF2_NOAID_TOUT>)
		_param_ekf2_noaid_tout,	///< maximum lapsed time from last fusion of measurements that constrain drift before the EKF will report the horizontal nav solution invalid (uSec)

#if defined(CONFIG_EKF2_TERRAIN) || defined(CONFIG_EKF2_OPTICAL_FLOW) || defined(CONFIG_EKF2_RANGE_FINDER)
		(ParamExtFloat<px4::params::EKF2_MIN_RNG>) _param_ekf2_min_rng,
#endif // CONFIG_EKF2_TERRAIN || CONFIG_EKF2_OPTICAL_FLOW || CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_TERRAIN)
		(ParamExtInt<px4::params::EKF2_TERR_MASK>) _param_ekf2_terr_mask,
		(ParamExtFloat<px4::params::EKF2_TERR_NOISE>) _param_ekf2_terr_noise,
		(ParamExtFloat<px4::params::EKF2_TERR_GRAD>) _param_ekf2_terr_grad,
#endif // CONFIG_EKF2_TERRAIN
#if defined(CONFIG_EKF2_RANGE_FINDER)
		// range finder fusion
		(ParamExtInt<px4::params::EKF2_RNG_CTRL>) _param_ekf2_rng_ctrl,
		(ParamExtFloat<px4::params::EKF2_RNG_DELAY>) _param_ekf2_rng_delay,
		(ParamExtFloat<px4::params::EKF2_RNG_NOISE>) _param_ekf2_rng_noise,
		(ParamExtFloat<px4::params::EKF2_RNG_SFE>) _param_ekf2_rng_sfe,
		(ParamExtFloat<px4::params::EKF2_RNG_GATE>) _param_ekf2_rng_gate,
		(ParamExtFloat<px4::params::EKF2_RNG_PITCH>) _param_ekf2_rng_pitch,
		(ParamExtFloat<px4::params::EKF2_RNG_A_VMAX>) _param_ekf2_rng_a_vmax,
		(ParamExtFloat<px4::params::EKF2_RNG_A_HMAX>) _param_ekf2_rng_a_hmax,
		(ParamExtFloat<px4::params::EKF2_RNG_A_IGATE>) _param_ekf2_rng_a_igate,
		(ParamExtFloat<px4::params::EKF2_RNG_QLTY_T>) _param_ekf2_rng_qlty_t,
		(ParamExtFloat<px4::params::EKF2_RNG_K_GATE>) _param_ekf2_rng_k_gate,
		(ParamExtFloat<px4::params::EKF2_RNG_POS_X>) _param_ekf2_rng_pos_x,
		(ParamExtFloat<px4::params::EKF2_RNG_POS_Y>) _param_ekf2_rng_pos_y,
		(ParamExtFloat<px4::params::EKF2_RNG_POS_Z>) _param_ekf2_rng_pos_z,
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		// vision estimate fusion
		(ParamExtFloat<px4::params::EKF2_EV_DELAY>)
		_param_ekf2_ev_delay, ///< off-board vision measurement delay relative to the IMU (mSec)

		(ParamExtInt<px4::params::EKF2_EV_CTRL>) _param_ekf2_ev_ctrl,	 ///< external vision (EV) control selection
		(ParamInt<px4::params::EKF2_EV_NOISE_MD>) _param_ekf2_ev_noise_md, ///< determine source of vision observation noise
		(ParamExtInt<px4::params::EKF2_EV_QMIN>) _param_ekf2_ev_qmin,
		(ParamExtFloat<px4::params::EKF2_EVP_NOISE>)
		_param_ekf2_evp_noise, ///< default position observation noise for exernal vision measurements (m)
		(ParamExtFloat<px4::params::EKF2_EVV_NOISE>)
		_param_ekf2_evv_noise, ///< default velocity observation noise for exernal vision measurements (m/s)
		(ParamExtFloat<px4::params::EKF2_EVA_NOISE>)
		_param_ekf2_eva_noise, ///< default angular observation noise for exernal vision measurements (rad)
		(ParamExtFloat<px4::params::EKF2_EVV_GATE>)
		_param_ekf2_evv_gate, ///< external vision velocity innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_EVP_GATE>)
		_param_ekf2_evp_gate, ///< external vision position innovation consistency gate size (STD)

		(ParamExtFloat<px4::params::EKF2_EV_POS_X>)
		_param_ekf2_ev_pos_x, ///< X position of VI sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_EV_POS_Y>)
		_param_ekf2_ev_pos_y, ///< Y position of VI sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_EV_POS_Z>)
		_param_ekf2_ev_pos_z, ///< Z position of VI sensor focal point in body frame (m)
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
		// optical flow fusion
		(ParamExtInt<px4::params::EKF2_OF_CTRL>)
		_param_ekf2_of_ctrl, ///< optical flow fusion selection
		(ParamExtFloat<px4::params::EKF2_OF_DELAY>)
		_param_ekf2_of_delay, ///< optical flow measurement delay relative to the IMU (mSec) - this is to the middle of the optical flow integration interval
		(ParamExtFloat<px4::params::EKF2_OF_N_MIN>)
		_param_ekf2_of_n_min, ///< best quality observation noise for optical flow LOS rate measurements (rad/sec)
		(ParamExtFloat<px4::params::EKF2_OF_N_MAX>)
		_param_ekf2_of_n_max, ///< worst quality observation noise for optical flow LOS rate measurements (rad/sec)
		(ParamExtInt<px4::params::EKF2_OF_QMIN>)
		_param_ekf2_of_qmin, ///< minimum acceptable quality integer from  the flow sensor when in air
		(ParamExtInt<px4::params::EKF2_OF_QMIN_GND>)
		_param_ekf2_of_qmin_gnd, ///< minimum acceptable quality integer from  the flow sensor when on ground
		(ParamExtFloat<px4::params::EKF2_OF_GATE>)
		_param_ekf2_of_gate, ///< optical flow fusion innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_OF_POS_X>)
		_param_ekf2_of_pos_x, ///< X position of optical flow sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_OF_POS_Y>)
		_param_ekf2_of_pos_y, ///< Y position of optical flow sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_OF_POS_Z>)
		_param_ekf2_of_pos_z, ///< Z position of optical flow sensor focal point in body frame (m)
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_DRAG_FUSION)
		(ParamExtInt<px4::params::EKF2_DRAG_CTRL>) _param_ekf2_drag_ctrl,		///< drag fusion selection
		// Multi-rotor drag specific force fusion
		(ParamExtFloat<px4::params::EKF2_DRAG_NOISE>)
		_param_ekf2_drag_noise,	///< observation noise variance for drag specific force measurements (m/sec**2)**2
		(ParamExtFloat<px4::params::EKF2_BCOEF_X>) _param_ekf2_bcoef_x,		///< ballistic coefficient along the X-axis (kg/m**2)
		(ParamExtFloat<px4::params::EKF2_BCOEF_Y>) _param_ekf2_bcoef_y,		///< ballistic coefficient along the Y-axis (kg/m**2)
		(ParamExtFloat<px4::params::EKF2_MCOEF>) _param_ekf2_mcoef,		///< propeller momentum drag coefficient (1/s)
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
		(ParamExtFloat<px4::params::EKF2_GRAV_NOISE>) _param_ekf2_grav_noise,
#endif // CONFIG_EKF2_GRAVITY_FUSION

		// sensor positions in body frame
		(ParamExtFloat<px4::params::EKF2_IMU_POS_X>) _param_ekf2_imu_pos_x,		///< X position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_IMU_POS_Y>) _param_ekf2_imu_pos_y,		///< Y position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_IMU_POS_Z>) _param_ekf2_imu_pos_z,		///< Z position of IMU in body frame (m)

		// IMU switch on bias parameters
		(ParamExtFloat<px4::params::EKF2_GBIAS_INIT>)
		_param_ekf2_gbias_init,	///< 1-sigma gyro bias uncertainty at switch on (rad/sec)
		(ParamExtFloat<px4::params::EKF2_ABIAS_INIT>)
		_param_ekf2_abias_init,	///< 1-sigma accelerometer bias uncertainty at switch on (m/sec**2)
		(ParamExtFloat<px4::params::EKF2_ANGERR_INIT>)
		_param_ekf2_angerr_init,	///< 1-sigma tilt error after initial alignment using gravity vector (rad)

		// EKF accel bias learning control
		(ParamExtFloat<px4::params::EKF2_ABL_LIM>) _param_ekf2_abl_lim,	///< Accelerometer bias learning limit (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_ACCLIM>)
		_param_ekf2_abl_acclim,	///< Maximum IMU accel magnitude that allows IMU bias learning (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_GYRLIM>)
		_param_ekf2_abl_gyrlim,	///< Maximum IMU gyro angular rate magnitude that allows IMU bias learning (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_TAU>)
		_param_ekf2_abl_tau,	///< Time constant used to inhibit IMU delta velocity bias learning (sec)

		(ParamExtFloat<px4::params::EKF2_GYR_B_LIM>) _param_ekf2_gyr_b_lim,	///< Gyro bias learning limit (rad/s)

		// output predictor filter time constants
		(ParamFloat<px4::params::EKF2_TAU_VEL>) _param_ekf2_tau_vel,
		(ParamFloat<px4::params::EKF2_TAU_POS>) _param_ekf2_tau_pos
	)
};
#endif // !EKF2_HPP
