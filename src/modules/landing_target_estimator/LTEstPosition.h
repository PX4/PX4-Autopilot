/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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

/*
 * @file LTEstPosition.h
 * Landing target position estimator. Filter and publish the position of a landing target on the ground as observed by an onboard sensor.
 *
 * @author Jonas Perolini <jonas.perolini@epfl.ch>
 *
 */

#pragma once

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/workqueue.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/irlock_report.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/fiducial_marker_pos_report.h>
#include <uORB/topics/landing_target_gnss.h>
#include <uORB/topics/target_estimator_state.h>
#include <uORB/topics/uwb_distance.h>
#include <uORB/topics/uwb_grid.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/estimator_aid_source_3d.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <matrix/Matrix.hpp>
#include <lib/conversion/rotation.h>
#include <lib/geo/geo.h>
#include "KF_xyzb_decoupled_static.h"
#include "KF_xyzb_decoupled_moving.h"
#include "KF_xyzb_v_decoupled_moving.h"
#include "KF_xyzb_coupled_moving.h"
#include "KF_xyzb_v_coupled_moving.h"
#include "KF_xyzb_coupled_static.h"

using namespace time_literals;

namespace landing_target_estimator
{

class LTEstPosition: public ModuleParams
{
public:

	LTEstPosition();
	virtual ~LTEstPosition();

	/*
	 * Get new measurements and update the state estimate
	 */
	void update(const matrix::Vector3f &acc_ned);

	bool init();

	void resetFilter();

	void set_landpoint(const int lat, const int lon, const float alt);

	void set_range_sensor(const float dist, const bool valid);

	void set_local_position(const matrix::Vector3f &xyz, const bool valid);

private:
	struct accInput {

		bool acc_ned_valid;
		matrix::Vector3f vehicle_acc_ned;
	};

protected:

	/*
	 * Update parameters.
	 */
	void updateParams() override;


	/* timeout after which the target is not valid if no measurements are seen*/
	static constexpr uint32_t landing_target_valid_TIMEOUT_US = 2000000;

	/* timeout after which the measurement is not valid*/
	static constexpr uint32_t measurement_valid_TIMEOUT_US = 1000000;

	/* timeout after which the measurement is not considered updated*/
	static constexpr uint32_t measurement_updated_TIMEOUT_US = 100000;

	uORB::Publication<landing_target_pose_s> _targetPosePub{ORB_ID(landing_target_pose)};
	uORB::Publication<target_estimator_state_s> _targetEstimatorStatePub{ORB_ID(target_estimator_state)};

	// publish innovations target_estimator_gps_pos
	uORB::Publication<estimator_aid_source_3d_s> _ltest_aid_gps_pos_target_pub{ORB_ID(ltest_aid_gps_pos_target)};
	uORB::Publication<estimator_aid_source_3d_s> _ltest_aid_gps_pos_mission_pub{ORB_ID(ltest_aid_gps_pos_mission)};
	uORB::Publication<estimator_aid_source_3d_s> _ltest_aid_gps_vel_target_pub{ORB_ID(ltest_aid_gps_vel_target)};
	uORB::Publication<estimator_aid_source_3d_s> _ltest_aid_gps_vel_rel_pub{ORB_ID(ltest_aid_gps_vel_rel)};
	uORB::Publication<estimator_aid_source_3d_s> _ltest_aid_fiducial_marker_pub{ORB_ID(ltest_aid_fiducial_marker)};
	uORB::Publication<estimator_aid_source_3d_s> _ltest_aid_irlock_pub{ORB_ID(ltest_aid_irlock)};
	uORB::Publication<estimator_aid_source_3d_s> _ltest_aid_uwb_pub{ORB_ID(ltest_aid_uwb)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

private:

	enum class TargetMode {
		Stationary = 0,
		Moving = 1,
		MovingAugmented = 2,
		NotInit
	};

	enum class TargetModel {
		Decoupled = 0,
		Coupled = 1,
		NotInit
	};

	TargetMode _target_mode{TargetMode::NotInit};
	TargetModel _target_model{TargetModel::NotInit};

	enum ObservationType {
		target_gps_pos = 0,
		mission_gps_pos = 1,
		vel_rel_gps = 2,
		vel_target_gps = 3,
		fiducial_marker = 4,
		irlock = 5,
		uwb = 6,
	};

	struct targetObsPos {

		ObservationType type;
		hrt_abstime timestamp;

		matrix::Vector<bool, 3> updated_xyz; // Indicates if we have an observation in the x, y or z direction
		matrix::Vector3f meas_xyz;			// Measurements (meas_x, meas_y, meas_z)
		matrix::Vector3f meas_unc_xyz;		// Measurements' uncertainties
		matrix::Matrix<float, 3, 15>
		meas_h_xyz; // Observation matrix where the rows correspond to the x,y,z observations and the columns to the state = [r_xyz, v_drone_xyz, b_xyz, v_target_xyz, a_target_xyz]
	};

	enum Directions {
		x = 0,
		y = 1,
		z = 2,
		nb_directions = 3,
	};

	enum SensorFusionMask : uint16_t {
		// Bit locations for fusion_mode
		USE_TARGET_GPS_POS  = (1 << 0),    ///< set to true to use target GPS position data
		USE_GPS_REL_VEL     = (1 << 1),    ///< set to true to use drone GPS velocity data (and target GPS velocity data if the target is moving)
		USE_EXT_VIS_POS 	= (1 << 2),    ///< set to true to use target external vision-based relative position data
		USE_IRLOCK_POS 		= (1 << 3),    ///< set to true to use target relative position from irlock data
		USE_UWB_POS     	= (1 << 4),    ///< set to true to use target relative position from uwb data
		USE_MISSION_POS     = (1 << 5),    ///< set to true to use the PX4 mission landing position
	};

	bool selectTargetEstimator();
	bool initEstimator(matrix::Vector3f pos_init, matrix::Vector3f vel_rel_init, matrix::Vector3f acc_init,
			   matrix::Vector3f bias_init, matrix::Vector3f target_vel_init);
	bool update_step(matrix::Vector3f vehicle_acc_ned);
	void predictionStep(matrix::Vector3f acc);

	bool processObsIRlock(const irlock_report_s &irlock_report, targetObsPos &obs);
	bool processObsUWB(const uwb_distance_s &uwb_distance, targetObsPos &obs);
	bool processObsVision(const fiducial_marker_pos_report_s &fiducial_marker_pose, targetObsPos &obs);
	bool processObsGNSSPosTarget(const landing_target_gnss_s &target_GNSS_report,
				     const sensor_gps_s &vehicle_gps_position, targetObsPos &obs);
	bool processObsGNSSPosMission(const sensor_gps_s &vehicle_gps_position, targetObsPos &obs);
	bool processObsGNSSVelRel(const landing_target_gnss_s &target_GNSS_report, bool target_GNSS_report_valid,
				  const sensor_gps_s &vehicle_gps_position, bool vehicle_gps_vel_updated,
				  targetObsPos &obs);
	bool processObsGNSSVelTarget(const landing_target_gnss_s &target_GNSS_report, targetObsPos &obs);

	bool fuse_meas(const matrix::Vector3f vehicle_acc_ned, const targetObsPos &target_pos_obs);
	void publishTarget();

	uORB::Subscription _irlockReportSub{ORB_ID(irlock_report)};
	uORB::Subscription _uwbDistanceSub{ORB_ID(uwb_distance)};
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _fiducial_marker_report_sub{ORB_ID(fiducial_marker_pos_report)};
	uORB::Subscription _landing_target_gnss_sub{ORB_ID(landing_target_gnss)};

	perf_counter_t _ltest_predict_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": LTEST prediction")};
	perf_counter_t _ltest_update_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": LTEST update")};

	struct localPos {
		bool valid = false;
		matrix::Vector3f xyz;
		hrt_abstime last_update = 0;
	};

	struct rangeSensor {
		bool valid;
		float dist_bottom;
		hrt_abstime last_update = 0;
	};

	rangeSensor _range_sensor{};
	localPos _local_position{};

	struct globalPos {
		bool valid = false;
		int lat = 0; 		// Latitude in 1E-7 degrees
		int lon	= 0; 		// Longitude in 1E-7 degrees
		float alt = 0.f;	// Altitude in 1E-3 meters AMSL, (millimetres)
	};

	globalPos _landing_pos{};

	struct vecStamped {
		hrt_abstime timestamp;
		bool valid = false;
		matrix::Vector3f xyz;
	};

	vecStamped _uav_gps_vel{};
	vecStamped _target_gps_vel{};
	vecStamped _pos_rel_gnss{};
	bool _bias_set;

	uint64_t _new_pos_sensor_acquired_time{0};
	uint64_t _land_time{0};
	bool _estimator_initialized{false};

	matrix::Quaternionf _q_att; //Quaternion orientation of the body frame
	Base_KF_decoupled *_target_estimator[nb_directions] {nullptr, nullptr, nullptr};
	Base_KF_coupled *_target_estimator_coupled {nullptr};
	hrt_abstime _last_predict{0}; // timestamp of last filter prediction
	hrt_abstime _last_update{0}; // timestamp of last filter update (used to check timeout)

	void _check_params(const bool force);

	/* parameters */
	/* timeout after which filter is reset if target not seen */
	uint32_t _ltest_TIMEOUT_US = 3000000;
	int _ltest_aid_mask{0};
	float _target_acc_unc;
	float _bias_unc;
	float _meas_unc;
	float _drone_acc_unc;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::LTEST_AID_MASK>) _param_ltest_aid_mask,
		(ParamFloat<px4::params::LTEST_BTOUT>) _param_ltest_btout,
		(ParamFloat<px4::params::LTEST_ACC_D_UNC>) _param_ltest_acc_d_unc,
		(ParamFloat<px4::params::LTEST_ACC_T_UNC>) _param_ltest_acc_t_unc,
		(ParamFloat<px4::params::LTEST_BIAS_LIM>) _param_ltest_bias_lim,
		(ParamFloat<px4::params::LTEST_BIAS_UNC>) _param_ltest_bias_unc,
		(ParamFloat<px4::params::LTEST_MEAS_UNC>) _param_ltest_meas_unc,
		(ParamFloat<px4::params::LTEST_POS_UNC_IN>) _param_ltest_pos_unc_in,
		(ParamFloat<px4::params::LTEST_VEL_UNC_IN>) _param_ltest_vel_unc_in,
		(ParamFloat<px4::params::LTEST_BIA_UNC_IN>) _param_ltest_bias_unc_in,
		(ParamFloat<px4::params::LTEST_ACC_UNC_IN>) _param_ltest_acc_unc_in,
		(ParamInt<px4::params::LTEST_MODE>) _param_ltest_mode,
		(ParamInt<px4::params::LTEST_MODEL>) _param_ltest_model,
		(ParamFloat<px4::params::LTEST_SCALE_X>) _param_ltest_scale_x,
		(ParamFloat<px4::params::LTEST_SCALE_Y>) _param_ltest_scale_y,
		(ParamInt<px4::params::LTEST_SENS_ROT>) _param_ltest_sens_rot,
		(ParamFloat<px4::params::LTEST_SENS_POS_X>) _param_ltest_sens_pos_x,
		(ParamFloat<px4::params::LTEST_SENS_POS_Y>) _param_ltest_sens_pos_y,
		(ParamFloat<px4::params::LTEST_SENS_POS_Z>) _param_ltest_sens_pos_z
	)
};
} // namespace landing_target_estimator
