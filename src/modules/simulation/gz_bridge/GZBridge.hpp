/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#pragma once

#include "GZMixingInterfaceESC.hpp"
#include "GZMixingInterfaceServo.hpp"
#include "GZMixingInterfaceWheel.hpp"
#include "GZGimbal.hpp"

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/device/Device.hpp>
#include <lib/geo/geo.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_optical_flow.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/wheel_encoders.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>

#include <gz/math.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <gz/msgs/imu.pb.h>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/air_speed.pb.h>
#include <gz/msgs/model.pb.h>
#include <gz/msgs/odometry_with_covariance.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/scene.pb.h>
// Custom PX4 proto
#include <opticalflow.pb.h>

using namespace time_literals;

class GZBridge : public ModuleBase<GZBridge>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	GZBridge(const std::string &world, const std::string &model_name);
	~GZBridge() override;

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	int init();

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	void Run() override;

	void clockCallback(const gz::msgs::Clock &msg);
	void airspeedCallback(const gz::msgs::AirSpeed &msg);
	void barometerCallback(const gz::msgs::FluidPressure &msg);
	void imuCallback(const gz::msgs::IMU &msg);
	void poseInfoCallback(const gz::msgs::Pose_V &msg);
	void odometryCallback(const gz::msgs::OdometryWithCovariance &msg);
	void navSatCallback(const gz::msgs::NavSat &msg);
	void laserScantoLidarSensorCallback(const gz::msgs::LaserScan &msg);
	void laserScanCallback(const gz::msgs::LaserScan &msg);
	void opticalFlowCallback(const px4::msgs::OpticalFlow &msg);
	void magnetometerCallback(const gz::msgs::Magnetometer &msg);

	static void rotateQuaternion(gz::math::Quaterniond &q_FRD_to_NED, const gz::math::Quaterniond q_FLU_to_ENU);

	void addGpsNoise(double &latitude, double &longitude, double &altitude,
			 float &vel_north, float &vel_east, float &vel_down);

	uORB::SubscriptionInterval                    _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<distance_sensor_s>          _distance_sensor_pub{ORB_ID(distance_sensor)};
	uORB::Publication<differential_pressure_s>    _differential_pressure_pub{ORB_ID(differential_pressure)};
	uORB::Publication<obstacle_distance_s>        _obstacle_distance_pub{ORB_ID(obstacle_distance)};
	uORB::Publication<vehicle_angular_velocity_s> _angular_velocity_ground_truth_pub{ORB_ID(vehicle_angular_velocity_groundtruth)};
	uORB::Publication<vehicle_attitude_s>         _attitude_ground_truth_pub{ORB_ID(vehicle_attitude_groundtruth)};
	uORB::Publication<vehicle_global_position_s>  _gpos_ground_truth_pub{ORB_ID(vehicle_global_position_groundtruth)};
	uORB::Publication<vehicle_local_position_s>   _lpos_ground_truth_pub{ORB_ID(vehicle_local_position_groundtruth)};
	uORB::PublicationMulti<sensor_gps_s>          _sensor_gps_pub{ORB_ID(sensor_gps)};
	uORB::PublicationMulti<sensor_baro_s>         _sensor_baro_pub{ORB_ID(sensor_baro)};
	uORB::PublicationMulti<sensor_accel_s>        _sensor_accel_pub{ORB_ID(sensor_accel)};
	uORB::PublicationMulti<sensor_gyro_s>         _sensor_gyro_pub{ORB_ID(sensor_gyro)};
	uORB::PublicationMulti<sensor_mag_s>          _sensor_mag_pub{ORB_ID(sensor_mag)};
	uORB::PublicationMulti<vehicle_odometry_s>    _visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};
	uORB::PublicationMulti<sensor_optical_flow_s> _optical_flow_pub{ORB_ID(sensor_optical_flow)};


	GZMixingInterfaceESC   _mixing_interface_esc{_node};
	GZMixingInterfaceServo _mixing_interface_servo{_node};
	GZMixingInterfaceWheel _mixing_interface_wheel{_node};

	GZGimbal _gimbal{_node};

	MapProjection _pos_ref{};
	double _alt_ref{};

	matrix::Vector3d _position_prev{};
	matrix::Vector3d _velocity_prev{};
	matrix::Vector3f _euler_prev{};
	hrt_abstime _timestamp_prev{};

	const std::string _world_name;
	const std::string _model_name;

	float _temperature{288.15};  // 15 degrees

	bool _realtime_clock_set{false};
	gz::transport::Node _node;

	// GPS noise model
	float _gps_pos_noise_n = 0.0f;
	float _gps_pos_noise_e = 0.0f;
	float _gps_pos_noise_d = 0.0f;
	float _gps_vel_noise_n = 0.0f;
	float _gps_vel_noise_e = 0.0f;
	float _gps_vel_noise_d = 0.0f;
	const float _pos_noise_amplitude = 0.8f;    // Position noise amplitude [m]
	const float _pos_random_walk = 0.01f;       // Position random walk coefficient
	const float _pos_markov_time = 0.95f;       // Position Markov process coefficient
	const float _vel_noise_amplitude = 0.05f;   // Velocity noise amplitude [m/s]
	const float _vel_noise_density = 0.2f;      // Velocity noise process density
	const float _vel_markov_time = 0.85f;       // Velocity Markov process coefficient

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SIM_GPS_USED>) _sim_gps_used
	)
};
