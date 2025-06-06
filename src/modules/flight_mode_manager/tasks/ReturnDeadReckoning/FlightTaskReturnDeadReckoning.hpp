/****************************************************************************
 *
 *   Copyright (c) 2019-2025 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskReturnDeadReckoning.hpp
 */

#pragma once

#include "FlightTask.hpp"
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_global_position.h>
#include <lib/motion_planning/HeadingSmoothing.hpp>
#include <lib/motion_planning/PositionSmoothing.hpp>
#include <lib/slew_rate/SlewRate.hpp>

class FlightTaskReturnDeadReckoning : public FlightTask
{
public:
	FlightTaskReturnDeadReckoning() = default;
	virtual ~FlightTaskReturnDeadReckoning() = default;

	bool update() override;
	bool activate(const trajectory_setpoint_s &last_setpoint) override;

private:
	uORB::SubscriptionData<vehicle_global_position_s> _sub_vehicle_global_position{ORB_ID(vehicle_global_position)};

	void _updateSetpoints();

	/**
	 * Initialize home position
	 * @return true on success, false on error
	 */
	bool _readHomePosition(matrix::Vector3d &home_position);

	/**
	 * Reads current GNSS position
	 * @return true on success, false on error
	 */
	bool _readGlobalPosition(matrix::Vector3d &global_position);


	bool _updateBearingToHome();

	/**
	 * Compute bearing to home position
	 * @return true on success, false on error
	 */
	float _computeBearing(const matrix::Vector3d &_global_position_start,
				const matrix::Vector3d &_global_position_end);

	/**
	 * Computes return altitude velocity
	 * @return true on success, false on error
	 */
	bool _computeReturnParameters();

	/**
	 * Initialize the trajectory setpoint smoothers
	 * @return true on success, false on error
	 */
	bool _initializeSmoothers();

	void _updateSubscriptions();

	bool _isGlobalPositionValid() const
	{
		return _sub_vehicle_global_position.get().lat_lon_valid && _sub_vehicle_global_position.get().alt_valid;
	}

	bool _isAboveReturnAltitude() const;

	bool _isWithinHomePositionRadius();


	// Variable storing the home position
	matrix::Vector3d _home_position;			/**< Stores home position */
	matrix::Vector3d _start_vehicle_global_position;	/**< Stores vehicle last known GNSS position */
	float _bearing_to_home{0.0f};				/**< Stores bearing between home and last GNSS position */

	HeadingSmoothing _heading_smoothing;			/**< Smoother for heading */
	SlewRate<float> _slew_rate_acceleration_x{0.0f};	/**< Slew rate for x-acceleration setpoint */
	SlewRate<float> _slew_rate_acceleration_y{0.0f};	/**< Slew rate for y-acceleration setpoint */
	SlewRate<float> _slew_rate_velocity_z{0.0f};		/**< Smoother for vertical velocity */


	float _rtl_alt{0.0f};			/**< Return altitude */
	float _rtl_acc{0.0f};			/**< Return acceleration */

	enum class State {
		UNKNOWN = 0,
		INIT,
		ASCENT,
		RETURN,
		HOLD
	};
	State _state{State::UNKNOWN};		/**< State machine for flight task */

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
					(ParamFloat<px4::params::MPC_ACC_HOR_MAX>) _param_mpc_acc_hor_max,
					(ParamFloat<px4::params::RTL_RETURN_ALT>) _param_rtl_return_alt,	//< RTL altitude
					(ParamFloat<px4::params::MPC_YAWRAUTO_MAX>) _param_mpc_yawrauto_max,	//< max yaw rate in auto modes
					(ParamFloat<px4::params::MPC_YAWRAUTO_ACC>) _param_mpc_yawrauto_acc,	//< max yaw acceleration in auto modes
					(ParamFloat<px4::params::MPC_JERK_AUTO>) _param_mpc_jerk_auto,		//< maximum jerk in auto modes
					(ParamFloat<px4::params::MPC_Z_V_AUTO_UP>) _param_mpc_z_v_auto_up,	//< max vertical velocity up
					(ParamFloat<px4::params::MPC_ACC_UP_MAX>) _param_mpc_acc_up_max,	//< max vertical acceleration up
					(ParamFloat<px4::params::NAV_MC_ALT_RAD>) _param_nav_mc_alt_rad,	//< max vertical error
					(ParamFloat<px4::params::NAV_ACC_RAD>) _param_nav_acc_rad		//< max horizontal error
				       );
};
