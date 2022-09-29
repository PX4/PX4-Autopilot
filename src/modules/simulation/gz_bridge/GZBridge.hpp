/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/geo/geo.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs/imu.pb.h>

using namespace time_literals;

class GZBridge : public ModuleBase<GZBridge>, public OutputModuleInterface
{
public:
	GZBridge(const char *world, const char *name, const char *model, const char *pose_str);
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

	uint64_t world_time_us() const { return _world_time_us.load(); }

private:

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	void Run() override;

	bool updateClock(const uint64_t tv_sec, const uint64_t tv_nsec);

	void clockCallback(const ignition::msgs::Clock &clock);
	void imuCallback(const ignition::msgs::IMU &imu);
	void poseInfoCallback(const ignition::msgs::Pose_V &pose);
	void motorSpeedCallback(const ignition::msgs::Actuators &actuators);

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<esc_status_s>               _esc_status_pub{ORB_ID(esc_status)};
	uORB::Publication<vehicle_angular_velocity_s> _angular_velocity_ground_truth_pub{ORB_ID(vehicle_angular_velocity_groundtruth)};
	uORB::Publication<vehicle_attitude_s>         _attitude_ground_truth_pub{ORB_ID(vehicle_attitude_groundtruth)};
	uORB::Publication<vehicle_global_position_s>  _gpos_ground_truth_pub{ORB_ID(vehicle_global_position_groundtruth)};
	uORB::Publication<vehicle_local_position_s>   _lpos_ground_truth_pub{ORB_ID(vehicle_local_position_groundtruth)};

	uORB::PublicationMulti<sensor_accel_s> _sensor_accel_pub{ORB_ID(sensor_accel)};
	uORB::PublicationMulti<sensor_gyro_s>  _sensor_gyro_pub{ORB_ID(sensor_gyro)};

	px4::atomic<uint64_t> _world_time_us{0};

	pthread_mutex_t _mutex;

	MapProjection _pos_ref{};

	matrix::Vector3d _position_prev{};
	matrix::Vector3d _velocity_prev{};
	matrix::Vector3f _euler_prev{};
	hrt_abstime _timestamp_prev{};

	const std::string _world_name;
	const std::string _model_name;
	const std::string _model_sim;
	const std::string _model_pose;

	MixingOutput _mixing_output{"SIM_GZ", 8, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	ignition::transport::Node _node;
	ignition::transport::Node::Publisher _actuators_pub;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SIM_GZ_HOME_LAT>) _param_sim_home_lat,
		(ParamFloat<px4::params::SIM_GZ_HOME_LON>) _param_sim_home_lon,
		(ParamFloat<px4::params::SIM_GZ_HOME_ALT>) _param_sim_home_alt
	)
};
