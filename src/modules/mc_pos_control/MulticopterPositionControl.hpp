/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * Multicopter position controller.
 */

#pragma once

#include "PositionControl/PositionControl.hpp"
#include "Takeoff/Takeoff.hpp"
#include "GotoControl/GotoControl.hpp"

#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/mathlib/math/filter/NotchFilter.hpp>
#include <lib/mathlib/math/WelfordMean.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRateYaw.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

using namespace time_literals;

class MulticopterPositionControl : public ModuleBase, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	MulticopterPositionControl(bool vtol = false);
	~MulticopterPositionControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	friend class MulticopterPositionControlTestPeer;

	void Run() override;

	TakeoffHandling _takeoff; /**< state machine and ramp to bring the vehicle off the ground without jumps */

	orb_advert_t _mavlink_log_pub{nullptr};

	uORB::PublicationData<takeoff_status_s>              _takeoff_status_pub{ORB_ID(takeoff_status)};
	uORB::Publication<vehicle_attitude_setpoint_s>	     _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};	/**< vehicle local position setpoint publication */

	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};	/**< vehicle local position */

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_constraints_sub{ORB_ID(vehicle_constraints)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	hrt_abstime _time_stamp_last_loop{0};		/**< time stamp of last loop iteration */
	hrt_abstime _time_position_control_enabled{0};

	// Velocity limits decaying from a speed higher than maximum after a control mode change to the configured maximum
	float _vel_limit_xy{NAN};
	float _vel_limit_up{NAN};
	float _vel_limit_down{NAN};
	hrt_abstime _time_vel_limit_reset{0};

	trajectory_setpoint_s _setpoint{PositionControl::empty_trajectory_setpoint};
	trajectory_setpoint_s _last_valid_setpoint{PositionControl::empty_trajectory_setpoint};
	vehicle_control_mode_s _vehicle_control_mode{};

	vehicle_constraints_s _vehicle_constraints {
		.timestamp = 0,
		.speed_up = NAN,
		.speed_down = NAN,
		.want_takeoff = false,
	};

	vehicle_land_detected_s _vehicle_land_detected {
		.timestamp = 0,
		.freefall = false,
		.ground_contact = true,
		.maybe_landed = true,
		.landed = true,
	};

	PARAMETERS(
		// Position Control
		MPC_XY_P,
		MPC_Z_P,
		MPC_XY_VEL_P_ACC,
		MPC_XY_VEL_I_ACC,
		MPC_XY_VEL_D_ACC,
		MPC_Z_VEL_P_ACC,
		MPC_Z_VEL_I_ACC,
		MPC_Z_VEL_D_ACC,
		MPC_XY_VEL_MAX,
		MPC_Z_V_AUTO_UP,
		MPC_Z_VEL_MAX_UP,
		MPC_Z_V_AUTO_DN,
		MPC_Z_VEL_MAX_DN,
		MPC_TILTMAX_AIR,
		MPC_THR_HOVER,
		MPC_ACC_DECOUPLE,

		MPC_VEL_LP,
		MPC_VEL_NF_FRQ,
		MPC_VEL_NF_BW,
		MPC_VELD_LP,

		// Takeoff / Land
		COM_SPOOLUP_TIME, /**< time to let motors spool up after arming */
		COM_THROW_EN, /**< throw launch enabled  */
		MPC_TKO_RAMP_T,   /**< time constant for smooth takeoff ramp */
		MPC_TKO_SPEED,
		MPC_LAND_SPEED,

		MPC_VEL_MANUAL,
		MPC_VEL_MAN_BACK,
		MPC_VEL_MAN_SIDE,
		MPC_XY_CRUISE,
		MPC_LAND_ALT2,    /**< downwards speed limited below this altitude */
		MPC_ALT_MODE,
		MPC_TILTMAX_LND,  /**< maximum tilt for landing and smooth takeoff */
		MPC_THR_MIN,
		MPC_THR_MAX,
		MPC_THR_XY_MARG,

		SYS_VEHICLE_RESP,
		MPC_ACC_HOR,
		MPC_ACC_DOWN_MAX,
		MPC_ACC_UP_MAX,
		MPC_ACC_HOR_MAX,
		MPC_JERK_AUTO,
		MPC_JERK_MAX,
		MPC_MAN_Y_MAX,
		MPC_MAN_Y_TAU,

		MPC_XY_VEL_ALL,
		MPC_Z_VEL_ALL,

		MPC_XY_ERR_MAX,
		MPC_YAWRAUTO_MAX,
		MPC_YAWRAUTO_ACC
	);

	math::WelfordMean<float> _sample_interval_s{};

	AlphaFilter<matrix::Vector2f> _vel_xy_lp_filter{};
	AlphaFilter<float> _vel_z_lp_filter{};

	math::NotchFilter<matrix::Vector2f> _vel_xy_notch_filter{};
	math::NotchFilter<float> _vel_z_notch_filter{};

	AlphaFilter<matrix::Vector2f> _vel_deriv_xy_lp_filter{};
	AlphaFilter<float> _vel_deriv_z_lp_filter{};

	GotoControl _goto_control; ///< class for handling smooth goto position setpoints
	PositionControl _control; ///< class for core PID position control

	hrt_abstime _last_warn{0}; /**< timer when the last warn message was sent out */

	bool _hover_thrust_initialized{false};

	/** Timeout in us for trajectory data to get considered invalid */
	static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500_ms;

	/** During smooth-takeoff, below ALTITUDE_THRESHOLD the yaw-control is turned off and tilt is limited */
	static constexpr float ALTITUDE_THRESHOLD = 0.3f;

	static constexpr float MAX_SAFE_TILT_DEG = 89.f; // Numerical issues above this value due to tanf

	SlewRate<float> _tilt_limit_slew_rate;

	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _heading_reset_counter{0};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};

	/**
	 * Update our local parameter cache.
	 * Parameter update can be forced when argument is true.
	 * @param force forces parameter update.
	 */
	void parameters_update(bool force);

	/**
	 * Check for validity of positon/velocity states.
	 */
	PositionControlStates set_vehicle_states(const vehicle_local_position_s &local_pos, const float dt_s);

	/**
	 * Generate setpoint to bridge no executable setpoint being available.
	 * Used to handle transitions where no proper setpoint was generated yet and when the received setpoint is invalid.
	 * This should only happen briefly when transitioning and never during mode operation or by design.
	 */
	trajectory_setpoint_s generateFailsafeSetpoint(const hrt_abstime &now, const PositionControlStates &states, bool warn);

	/**
	 * @brief adjust existing (or older) setpoint with any EKF reset deltas and update the local counters
	 *
	 * @param[in] vehicle_local_position struct containing EKF reset deltas and counters
	 * @param[in,out] setpoint trajectory setpoint struct to be adjusted
	 * @param[in,out] fallback_setpoint stored last valid setpoint, adjusted here because this
	 *                call latches the reset counters
	 */
	void adjustSetpointForEKFResets(const vehicle_local_position_s &vehicle_local_position,
					trajectory_setpoint_s &setpoint, trajectory_setpoint_s &fallback_setpoint);
};
