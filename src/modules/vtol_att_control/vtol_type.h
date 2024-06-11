/****************************************************************************
 *
 *   Copyright (c) 2015-2022 PX4 Development Team. All rights reserved.
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
* @file vtol_type.h
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author Sander Smeets		<sander@droneslab.com>
* @author Andreas Antener	<andreas@uaventure.com>
*
*/

#ifndef VTOL_TYPE_H
#define VTOL_TYPE_H

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/module_params.h>


static constexpr float kFlapSlewRateVtol = 1.f; // minimum time from none to full flap deflection [s]
static constexpr float kSpoilerSlewRateVtol = 1.f; // minimum time from none to full spoiler deflection [s]


// Has to match 1:1 msg/vtol_vehicle_status.msg
enum class mode {
	TRANSITION_TO_FW = 1,
	TRANSITION_TO_MC = 2,
	ROTARY_WING = 3,
	FIXED_WING = 4
};

enum class vtol_type {
	TAILSITTER = 0,
	TILTROTOR,
	STANDARD
};

enum VtolForwardActuationMode {
	DISABLE = 0,
	ENABLE_WITHOUT_LAND,
	ENABLE_ABOVE_MPC_LAND_ALT1,
	ENABLE_ABOVE_MPC_LAND_ALT2,
	ENABLE_ALL_MODES,
	ENABLE_ABOVE_MPC_LAND_ALT1_WITHOUT_LAND,
	ENABLE_ABOVE_MPC_LAND_ALT2_WITHOUT_LAND
};

// enum for bitmask of VT_FW_DIFTHR_EN parameter options
enum class VtFwDifthrEnBits : int32_t {
	YAW_BIT = (1 << 0),
	ROLL_BIT = (1 << 1),
	PITCH_BIT = (1 << 2),
};

enum class QuadchuteReason {
	None = 0,
	TransitionTimeout,
	ExternalCommand,
	MinimumAltBreached,
	UncommandedDescent,
	TransitionAltitudeLoss,
	MaximumPitchExceeded,
	MaximumRollExceeded,
};

class VtolAttitudeControl;

class VtolType : public ModuleParams
{
public:

	VtolType(VtolAttitudeControl *att_controller);
	VtolType(const VtolType &) = delete;
	VtolType &operator=(const VtolType &) = delete;

	virtual ~VtolType() = default;

	/**
	 * Initialise.
	 */
	bool init();

	/**
	 * Update vtol state.
	 */
	virtual void update_vtol_state() = 0;

	/**
	 * Update transition state.
	 */
	virtual void update_transition_state() = 0;

	/**
	 * Update multicopter state.
	 */
	virtual void update_mc_state();

	/**
	 * Update fixed wing state.
	 */
	virtual void update_fw_state();

	/**
	 * Write control values to actuator output topics.
	 */
	virtual void fill_actuator_outputs() = 0;

	/**
	 * Special handling opportunity for the time right after transition to FW
	 * before TECS is running.
	 */
	virtual void waiting_on_tecs() {}

	/**
	 *  @brief Indicates if quadchute is enabled.
	 *
	 * @return     true if enabled
	 */
	bool isQuadchuteEnabled();

	/**
	 *  @brief Evaluates quadchute conditions and returns a reson for quadchute.
	 *
	 * @return     QuadchuteReason, can be None
	 */
	QuadchuteReason getQuadchuteReason();

	/**
	 *  @brief Indicates if the vehicle is lower than VT_FW_MIN_ALT above the local origin.
	 *
	 * @return     true if below threshold
	 */
	bool isMinAltBreached();

	/**
	 * @brief Indicates if conditions are met for uncommanded-descent quad-chute.
	 *
	 * @return true if integrated height rate error larger than threshold
	 */
	bool isUncommandedDescent();

	/**
	 * @brief Indicates if there is an altitude loss higher than specified threshold during a VTOL transition to FW
	 *
	 * @return true if error larger than threshold
	 */
	bool isFrontTransitionAltitudeLoss();

	/**
	 *  @brief Indicates if the absolute value of the vehicle pitch angle exceeds the threshold defined by VT_FW_QC_P
	 *
	 * @return     true if exeeded
	 */
	bool isPitchExceeded();

	/**
	 *  @brief Indicates if the absolute value of the vehicle roll angle exceeds the threshold defined by VT_FW_QC_R
	 *
	 * @return     true if exeeded
	 */
	bool isRollExceeded();

	/**
	 *  @brief Indicates if the front transition duration has exceeded the timeout definded by VT_TRANS_TIMEOUT
	 *
	 * @return     true if exeeded
	 */
	bool isFrontTransitionTimeout();

	/**
	 *  @brief Special handling of QuadchuteReason::ReasonExternal
	 */
	void handleSpecialExternalCommandQuadchute();

	/**
	 * Checks for fixed-wing failsafe condition and issues abort request if needed.
	 */
	void check_quadchute_condition();

	/**
	 * Returns true if we're allowed to do a mode transition on the ground.
	 */
	bool can_transition_on_ground();

	/**
	 * Pusher assist in hover (pusher/pull for standard VTOL, motor tilt for tiltrotor)
	 */
	float pusher_assist();

	virtual void blendThrottleAfterFrontTransition(float scale) {};

	mode get_mode() {return _common_vtol_mode;}

	/**
	 * @return Minimum front transition time scaled for air density (if available) [s]
	*/
	float getMinimumFrontTransitionTime() const;

	/**
	 * @return Front transition timeout scaled for air density (if available) [s]
	*/
	float getFrontTransitionTimeout() const;

	/**
	* @return Minimum open-loop front transition time scaled for air density (if available) [s]
	*/
	float getOpenLoopFrontTransitionTime() const;

	/**
	 *
	 * @return The calibrated blending airspeed [m/s]
	 */
	float getBlendAirspeed() const;

	/**
	 *
	 * @return The calibrated transition airspeed [m/s]
	 */
	float getTransitionAirspeed() const;

	virtual void parameters_update() = 0;


	/**
	 * @brief Resets the transition timer states.
	 *
	 */
	void resetTransitionStates();

	/**
	 * @brief Handle EKF position resets.
	 *
	 */
	void handleEkfResets();

protected:
	VtolAttitudeControl *_attc;
	mode _common_vtol_mode;

	static constexpr const int num_outputs_max = 8;

	struct vehicle_attitude_s		*_v_att;				//vehicle attitude
	struct vehicle_attitude_setpoint_s	*_v_att_sp;			//vehicle attitude setpoint
	struct vehicle_attitude_setpoint_s *_mc_virtual_att_sp;	// virtual mc attitude setpoint
	struct vehicle_attitude_setpoint_s *_fw_virtual_att_sp;	// virtual fw attitude setpoint
	struct vehicle_control_mode_s		*_v_control_mode;	//vehicle control mode
	struct vtol_vehicle_status_s 		*_vtol_vehicle_status;
	struct vehicle_torque_setpoint_s 		*_vehicle_torque_setpoint_virtual_mc;
	struct vehicle_torque_setpoint_s 		*_vehicle_torque_setpoint_virtual_fw;
	struct vehicle_thrust_setpoint_s 		*_vehicle_thrust_setpoint_virtual_mc;
	struct vehicle_thrust_setpoint_s 		*_vehicle_thrust_setpoint_virtual_fw;
	struct vehicle_local_position_s			*_local_pos;
	struct vehicle_local_position_setpoint_s	*_local_pos_sp;
	struct airspeed_validated_s 			*_airspeed_validated;					// airspeed
	struct tecs_status_s				*_tecs_status;
	struct vehicle_land_detected_s			*_land_detected;

	struct vehicle_torque_setpoint_s 		*_torque_setpoint_0;
	struct vehicle_torque_setpoint_s 		*_torque_setpoint_1;
	struct vehicle_thrust_setpoint_s 		*_thrust_setpoint_0;
	struct vehicle_thrust_setpoint_s 		*_thrust_setpoint_1;

	float _mc_roll_weight = 1.0f;	// weight for multicopter attitude controller roll output
	float _mc_pitch_weight = 1.0f;	// weight for multicopter attitude controller pitch output
	float _mc_yaw_weight = 1.0f;	// weight for multicopter attitude controller yaw output
	float _mc_throttle_weight = 1.0f;	// weight for multicopter throttle command. Used to avoid

	// motors spinning up or cutting too fast when doing transitions.
	float _thrust_transition = 0.0f;	// thrust value applied during a front transition (tailsitter & tiltrotor only)
	float _last_thr_in_fw_mode = 0.0f;
	float _last_thr_in_mc = 0.f;

	hrt_abstime _trans_finished_ts = 0;
	hrt_abstime _transition_start_timestamp{0};
	float _time_since_trans_start{0};

	bool _tecs_running = false;
	hrt_abstime _tecs_running_ts = 0;

	hrt_abstime _last_loop_ts = 0;
	float _transition_dt = 0;

	float _quadchute_ref_alt{NAN};	// altitude (AMSL) reference to compute quad-chute altitude loss condition

	float _accel_to_pitch_integ = 0;

	bool _quadchute_command_treated{false};

	float update_and_get_backtransition_pitch_sp();
	bool isFrontTransitionCompleted();
	virtual bool isFrontTransitionCompletedBase();

	float _local_position_z_start_of_transition{0.f}; // altitude at start of transition

	int _altitude_reset_counter{0};

	DEFINE_PARAMETERS_CUSTOM_PARENT(ModuleParams,
					(ParamBool<px4::params::VT_ELEV_MC_LOCK>) _param_vt_elev_mc_lock,
					(ParamFloat<px4::params::VT_FW_MIN_ALT>) _param_vt_fw_min_alt,
					(ParamFloat<px4::params::VT_QC_ALT_LOSS>) _param_vt_qc_alt_loss,
					(ParamInt<px4::params::VT_FW_QC_P>) _param_vt_fw_qc_p,
					(ParamInt<px4::params::VT_FW_QC_R>) _param_vt_fw_qc_r,
					(ParamFloat<px4::params::VT_QC_T_ALT_LOSS>) _param_vt_qc_t_alt_loss,
					(ParamInt<px4::params::VT_FW_QC_HMAX>) _param_quadchute_max_height,
					(ParamFloat<px4::params::VT_F_TR_OL_TM>) _param_vt_f_tr_ol_tm,
					(ParamFloat<px4::params::VT_TRANS_MIN_TM>) _param_vt_trans_min_tm,

					(ParamFloat<px4::params::VT_F_TRANS_DUR>) _param_vt_f_trans_dur,
					(ParamFloat<px4::params::VT_B_TRANS_DUR>) _param_vt_b_trans_dur,
					(ParamFloat<px4::params::VT_ARSP_TRANS>) _param_vt_arsp_trans,
					(ParamFloat<px4::params::VT_F_TRANS_THR>) _param_vt_f_trans_thr,
					(ParamFloat<px4::params::VT_ARSP_BLEND>) _param_vt_arsp_blend,
					(ParamBool<px4::params::FW_USE_AIRSPD>) _param_fw_use_airspd,
					(ParamFloat<px4::params::VT_TRANS_TIMEOUT>) _param_vt_trans_timeout,
					(ParamFloat<px4::params::MPC_XY_CRUISE>) _param_mpc_xy_cruise,
					(ParamInt<px4::params::VT_FW_DIFTHR_EN>) _param_vt_fw_difthr_en,
					(ParamFloat<px4::params::VT_FW_DIFTHR_S_Y>) _param_vt_fw_difthr_s_y,
					(ParamFloat<px4::params::VT_FW_DIFTHR_S_P>) _param_vt_fw_difthr_s_p,
					(ParamFloat<px4::params::VT_FW_DIFTHR_S_R>) _param_vt_fw_difthr_s_r,
					(ParamFloat<px4::params::VT_B_DEC_I>) _param_vt_b_dec_i,
					(ParamFloat<px4::params::VT_B_DEC_MSS>) _param_vt_b_dec_mss,

					(ParamFloat<px4::params::VT_PITCH_MIN>) _param_vt_pitch_min,
					(ParamFloat<px4::params::VT_FWD_THRUST_SC>) _param_vt_fwd_thrust_sc,
					(ParamInt<px4::params::VT_FWD_THRUST_EN>) _param_vt_fwd_thrust_en,
					(ParamFloat<px4::params::MPC_LAND_ALT1>) _param_mpc_land_alt1,
					(ParamFloat<px4::params::MPC_LAND_ALT2>) _param_mpc_land_alt2,
					(ParamFloat<px4::params::VT_LND_PITCH_MIN>) _param_vt_lnd_pitch_min,
					(ParamFloat<px4::params::WEIGHT_BASE>) _param_weight_base,
					(ParamFloat<px4::params::WEIGHT_GROSS>) _param_weight_gross

				       )

private:
	hrt_abstime _throttle_blend_start_ts{0};	// time at which we start blending between transition throttle and fixed wing throttle

	void resetAccelToPitchPitchIntegrator() { _accel_to_pitch_integ = 0.f; }
	bool shouldBlendThrottleAfterFrontTransition() { return _throttle_blend_start_ts != 0; };

	void stopBlendingThrottleAfterFrontTransition() { _throttle_blend_start_ts = 0; }

	/**
	 * @return Transition time scale factor for density.
	*/
	float getFrontTransitionTimeFactor() const;

};

#endif
