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

// PX4 includes
#include <px4_platform_common/module_params.h>

// Library includes
#include <matrix/matrix/math.hpp>
#include <lib/rover_control/RoverControl.hpp>
#include <lib/rover_control/RoverControllerWorkItem.hpp>
#include <math.h>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/surface_vehicle_course_setpoint.h>
#include <uORB/topics/surface_vehicle_attitude_setpoint.h>
#include <uORB/topics/surface_vehicle_speed_setpoint.h>
#include <uORB/topics/surface_vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

/**
 * @brief Enum class for the different states of driving.
 */
enum class DrivingState {
	SPOT_TURNING, // The vehicle is currently turning on the spot.
	DRIVING      // The vehicle is currently driving.
};

/**
 * @brief Class for differential course control.
 *
 * Sits between the position controller and the attitude and speed controllers: turns a course over ground into a heading
 * setpoint (drift compensation forwards, pointing away from the course in reverse) and passes the speed on, reduced
 * for large heading errors and zero while turning on the spot. Without a speed setpoint it commands the heading for the thrust
 * commanded elsewhere and deactivates the speed controller (NaN speed setpoint). The last valid course setpoint is controlled until a NaN course is received.
 */
class DifferentialCourseControl : public ModuleParams, public RoverControllerWorkItem
{
public:
	/**
	 * @brief Constructor for DifferentialCourseControl.
	 * @param parent The parent ModuleParams object.
	 */
	DifferentialCourseControl(ModuleParams *parent);
	~DifferentialCourseControl() = default;

	/**
	 * @brief Generate and publish surfaceVehicleAttitudeSetpoint and surfaceVehicleSpeedSetpoint from the last surfaceVehicleCourseSetpoint.
	 */
	void updateCourseControl();

	/**
	 * @brief Check if the necessary parameters are set.
	 * @return True if all checks pass.
	 */
	bool runSanityChecks();

	/**
	 * @brief Reset course controller.
	 */
	void reset() { _course_setpoint = NAN; _speed_setpoint = NAN; _current_state = DrivingState::DRIVING; _drift_angle_estimator.reset(); }

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	void runController() override { updateCourseControl(); }

	/**
	 * @brief Update uORB subscriptions used in course controller.
	 */
	void updateSubscriptions();

	// uORB subscriptions
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _surface_vehicle_course_setpoint_sub{ORB_ID(surface_vehicle_course_setpoint)};
	uORB::Subscription _surface_vehicle_thrust_setpoint_sub{ORB_ID(surface_vehicle_thrust_setpoint)};

	// uORB publications
	uORB::Publication<surface_vehicle_attitude_setpoint_s> _surface_vehicle_attitude_setpoint_pub{ORB_ID(surface_vehicle_attitude_setpoint)};
	uORB::Publication<surface_vehicle_speed_setpoint_s> _surface_vehicle_speed_setpoint_pub{ORB_ID(surface_vehicle_speed_setpoint)};

	// Variables
	RoverControl::DriftAngleEstimator _drift_angle_estimator{};
	DrivingState _current_state{DrivingState::DRIVING};
	float _course_setpoint{NAN}; // Last received course setpoint, controlled until a NaN course is received
	float _speed_setpoint{NAN}; // Last received speed setpoint, passed on if finite
	float _vehicle_yaw{0.f};
	float _thrust_setpoint_x{0.f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RD_TRANS_TRN_DRV>) _param_rd_trans_trn_drv,
		(ParamFloat<px4::params::RD_TRANS_DRV_TRN>) _param_rd_trans_drv_trn,
		(ParamFloat<px4::params::RO_MAX_THR_SPEED>) _param_ro_max_thr_speed,
		(ParamFloat<px4::params::RO_SPEED_RED>)     _param_ro_speed_red
	)
};
