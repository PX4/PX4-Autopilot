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
 * @file LandingTargetEstimator.h
 * Landing target position estimator. Filter and publish the position of a landing target on the ground as observed by an onboard sensor.
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 * @author Mohammed Kabir <kabir@uasys.io>
 *
 */

#pragma once

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
#include <uORB/topics/target_estimator_state.h>
#include <uORB/topics/uwb_distance.h>
#include <uORB/topics/uwb_grid.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/estimator_aid_source_3d.h>
#include <uORB/topics/estimator_aid_source_1d.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <matrix/Matrix.hpp>
#include <lib/conversion/rotation.h>
#include <lib/geo/geo.h>
#include "KalmanFilter.h"
#include "KFxyzDecoupledStatic.h"
#include "KFxyzDecoupledMoving.h"
#include "KFxyzCoupledMoving.h"
#include "KFxyzCoupledStatic.h"

using namespace time_literals;

namespace landing_target_estimator
{

class LandingTargetEstimator: public ModuleParams
{
public:

	LandingTargetEstimator();
	virtual ~LandingTargetEstimator();

	/*
	 * Get new measurements and update the state estimate
	 */
	void update();

private:
	struct accInput {

		bool acc_ned_valid;
		matrix::Vector3f vehicle_acc_ned;
	};

protected:

	/*
	 * Update uORB topics.
	 */
	void _update_topics(accInput *input);

	/*
	 * Update parameters.
	 */
	void updateParams() override;

	// TODO: increase the timeout to at least 4 000 000
	/* timeout after which filter is reset if target not seen */
	static constexpr uint32_t landing_target_estimator_TIMEOUT_US = 2000000;

	/* timeout after which the target is not valid if no measurements are seen*/
	static constexpr uint32_t landing_target_valid_TIMEOUT_US = 2000000;

	/* timeout after which the measurement is not valid*/
	static constexpr uint32_t measurement_valid_TIMEOUT_US = 2000000;

	uORB::Publication<landing_target_pose_s> _targetPosePub{ORB_ID(landing_target_pose)};
	uORB::Publication<target_estimator_state_s> _targetEstimatorStatePub{ORB_ID(target_estimator_state)};

	// publish innovations target_estimator_gps_pos
	uORB::Publication<estimator_aid_source_3d_s> _target_estimator_aid_gps_pos_pub{ORB_ID(target_estimator_aid_gps_pos)};
	uORB::Publication<estimator_aid_source_3d_s> _target_estimator_aid_gps_vel_pub{ORB_ID(target_estimator_aid_gps_vel)};
	uORB::Publication<estimator_aid_source_3d_s> _target_estimator_aid_vision_pub{ORB_ID(target_estimator_aid_vision)};
	uORB::Publication<estimator_aid_source_3d_s> _target_estimator_aid_irlock_pub{ORB_ID(target_estimator_aid_irlock)};
	uORB::Publication<estimator_aid_source_3d_s> _target_estimator_aid_uwb_pub{ORB_ID(target_estimator_aid_uwb)};

	estimator_aid_source_3d_s _target_innovations_array[5] {};

	uORB::Publication<estimator_aid_source_1d_s> _target_estimator_aid_ev_yaw_pub{ORB_ID(target_estimator_aid_ev_yaw)};
	estimator_aid_source_1d_s _target_estimator_aid_ev_yaw;

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

private:

	bool _start_detection = false;
	uint8_t _nave_state = 0;

	enum class TargetMode {
		Moving = 0,
		Stationary,
		NotInit
	};

	enum class TargetModel {
		FullPoseDecoupled = 0,
		FullPoseCoupled,
		Horizontal,
		NotInit
	};

	struct targetObsOrientation {
		hrt_abstime timestamp;
		// Theta
		bool updated_theta = false;
		float meas_theta = 0.f;
		float meas_unc_theta = 0.f;
		float meas_h_theta = 0.f;
	};

	struct targetObsPos {
		hrt_abstime timestamp;

		//TODO: check that all vectors are initialized to zero
		// x,y,z
		bool any_xyz_updated;
		matrix::Vector<bool, 3> updated_xyz; // Indicates if we have an observation in the x, y or z direction
		matrix::Vector3f meas_xyz;			// Measurements (meas_x, meas_y, meas_z)
		matrix::Vector3f meas_unc_xyz;		// Measurements' uncertainties
		matrix::Matrix<float, 3, 12> meas_h_xyz; // Observation matrix where the rows correspond to the x,y,z directions.
	};

	enum ObservationType {
		target_gps_pos = 0,
		uav_gps_vel = 1,
		fiducial_marker = 2,
		irlock = 3,
		uwb = 4,
		nb_observations = 5
	};

	enum Directions {
		x = 0,
		y = 1,
		z = 2,
		nb_directions = 3,
	};

	/*Each component corresponds to a different measurements (VISION, IrLock, UWB, GPS, GPS velocity). For each measurement, we have observations in the x,y,z directions.*/
	targetObsPos _target_pos_obs[nb_observations] {};
	targetObsOrientation _target_orientation_obs{};

	TargetMode _target_mode{TargetMode::NotInit};
	TargetModel _target_model{TargetModel::NotInit};

	enum SensorFusionMask : uint16_t {
		// Bit locations for fusion_mode
		USE_TARGET_GPS_POS  = (1 << 0),    ///< set to true to use target GPS position data
		USE_UAV_GPS_VEL     = (1 << 1),    ///< set to true to use drone GPS velocity data
		USE_EXT_VIS_POS 	= (1 << 2),    ///< set to true to use target external vision-based relative position data
		USE_IRLOCK_POS 		= (1 << 3),    ///< set to true to use target relative position from irlock data
		USE_UWB_POS     	= (1 << 4),    ///< set to true to use target relative position from uwb data
		USE_MISSION_POS     = (1 << 5),    ///< set to true to use the PX4 mission landing position
	};

	int _ltest_aid_mask{0};
	int _nb_position_kf; // Number of kalman filter instances for the position estimate (no orientation)
	bool _estimate_orientation;

	void selectTargetEstimator();
	void initEstimator();
	void predictionStep(matrix::Vector3f acc);
	bool updateNED(matrix::Vector3f acc);
	bool updateOrientation();
	void publishTarget();
	void publishInnovations();

	uORB::Subscription _vehicleLocalPositionSub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _attitudeSub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _irlockReportSub{ORB_ID(irlock_report)};
	uORB::Subscription _uwbDistanceSub{ORB_ID(uwb_distance)};
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _fiducial_marker_report_sub{ORB_ID(fiducial_marker_report)};
	uORB::Subscription _target_GNSS_report_sub{ORB_ID(target_GNSS_report)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _pos_sp_triplet_sub{ORB_ID(position_setpoint_triplet)};

	struct localPos {
		bool valid = false;
		float x = 0.f;
		float y = 0.f;
		float z = 0.f;
	};

	localPos _local_pos{};

	struct globalPos {
		bool valid = false;
		int lat = 0; 		// Latitude in 1E-7 degrees
		int lon	= 0; 		// Longitude in 1E-7 degrees
		float alt = 0.f;	// Altitude in 1E-3 meters AMSL, (millimetres)
	};

	globalPos _landing_pos{};

	// keep track of which topics we have received
	bool _new_sensorReport{false};
	bool _estimator_initialized{false};

	matrix::Quaternion<float> _q_att; //Quaternion orientation of the body frame
	TargetEstimator *_target_estimator[nb_directions] {nullptr, nullptr, nullptr};
	TargetEstimator *_target_estimator_orientation {nullptr};
	TargetEstimatorCoupled *_target_estimator_coupled {nullptr};
	hrt_abstime _last_predict{0}; // timestamp of last filter prediction
	hrt_abstime _last_update{0}; // timestamp of last filter update (used to check timeout)

	void _check_params(const bool force);

	void _update_state();

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::LTEST_AID_MASK>) _param_ltest_aid_mask,
		(ParamFloat<px4::params::LTEST_GPS_T_UNC>) _param_ltest_gps_t_unc,
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
