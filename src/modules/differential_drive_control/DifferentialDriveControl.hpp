/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

// PX4 includes
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// uORB includes
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/differential_drive_setpoint.h>

// Standard library includes
#include <math.h>

// Local includes
#include <DifferentialDriveKinematics.hpp>

namespace differential_drive_control
{

class DifferentialDriveControl : public ModuleBase<DifferentialDriveControl>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	DifferentialDriveControl();
	~DifferentialDriveControl() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

protected:
	void updateParams() override;

private:
	void Run() override;

	uORB::Subscription _differential_drive_setpoint_sub{ORB_ID(differential_drive_setpoint)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::PublicationMulti<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<differential_drive_setpoint_s> _differential_drive_setpoint_pub{ORB_ID(differential_drive_setpoint)};

	differential_drive_setpoint_s _differential_drive_setpoint{};
	DifferentialDriveKinematics _differential_drive_kinematics{};

	bool _armed = false;
	bool _manual_driving = false;
	float _max_speed{0.f};
	float _max_angular_velocity{0.f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RDD_SPEED_SCALE>) _param_rdd_speed_scale,
		(ParamFloat<px4::params::RDD_ANG_SCALE>) _param_rdd_ang_velocity_scale,
		(ParamFloat<px4::params::RDD_WHL_SPEED>) _param_rdd_max_wheel_speed,
		(ParamFloat<px4::params::RDD_WHEEL_BASE>) _param_rdd_wheel_base,
		(ParamFloat<px4::params::RDD_WHEEL_RADIUS>) _param_rdd_wheel_radius,
		(ParamInt<px4::params::CA_R_REV>) _param_r_rev
	)
};

} // namespace differential_drive_control
