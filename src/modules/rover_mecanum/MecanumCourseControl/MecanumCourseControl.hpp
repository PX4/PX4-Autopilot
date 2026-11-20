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

using namespace matrix;

/**
 * @brief Class for mecanum course control.
 *
 * Sits between the position controller and the speed and attitude controllers. With a speed it turns the course over
 * ground into a body frame velocity setpoint, independent of the heading. Without a speed it turns the heading so that
 * the direction of the thrust commanded elsewhere (body x and y) points along the course, and deactivates the speed
 * controller (NaN speed setpoint). The last valid course setpoint is controlled until a
 * NaN course is received. With a NaN course the speed is passed on along the body x axis, with NaN course and speed
 * the course controller stops publishing.
 */
class MecanumCourseControl : public ModuleParams, public RoverControllerWorkItem
{
public:
	/**
	 * @brief Constructor for MecanumCourseControl.
	 * @param parent The parent ModuleParams object.
	 */
	MecanumCourseControl(ModuleParams *parent);
	~MecanumCourseControl() = default;

	/**
	 * @brief Generate and publish surfaceVehicleSpeedSetpoint and, without a speed, surfaceVehicleAttitudeSetpoint from the last surfaceVehicleCourseSetpoint.
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
	void reset() { _course_setpoint = NAN; _speed_setpoint = NAN; _thrust_direction = 0.f; }

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
	uORB::Subscription _surface_vehicle_course_setpoint_sub{ORB_ID(surface_vehicle_course_setpoint)};
	uORB::Subscription _surface_vehicle_thrust_setpoint_sub{ORB_ID(surface_vehicle_thrust_setpoint)};

	// uORB publications
	uORB::Publication<surface_vehicle_speed_setpoint_s> _surface_vehicle_speed_setpoint_pub{ORB_ID(surface_vehicle_speed_setpoint)};
	uORB::Publication<surface_vehicle_attitude_setpoint_s> _surface_vehicle_attitude_setpoint_pub{ORB_ID(surface_vehicle_attitude_setpoint)};

	// Variables
	Quatf _vehicle_attitude_quaternion{};
	float _course_setpoint{NAN}; // Last received course setpoint, controlled until a NaN course is received
	float _speed_setpoint{NAN}; // Last received speed setpoint
	float _thrust_direction{0.f}; // [rad] Direction of the last nonzero thrust setpoint in body frame, 0 forwards, pi/2 right
};
