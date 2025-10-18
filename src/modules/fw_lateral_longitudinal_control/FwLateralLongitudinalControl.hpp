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


/**
 * @file FwLateralLongitudinalControl.hpp
 */

#ifndef PX4_FWLATERALLONGITUDINALCONTROL_HPP
#define PX4_FWLATERALLONGITUDINALCONTROL_HPP

#include <float.h>
#include <fw_performance_model/PerformanceModel.hpp>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/atmosphere/atmosphere.h>
#include <lib/npfg/CourseToAirspeedRefMapper.hpp>
#include <lib/npfg/AirspeedDirectionController.hpp>
#include <lib/tecs/TECS.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <slew_rate/SlewRate.hpp>
#include <uORB/uORB.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/fixed_wing_lateral_setpoint.h>
#include <uORB/topics/fixed_wing_lateral_status.h>
#include <uORB/topics/fixed_wing_longitudinal_setpoint.h>
#include <uORB/topics/normalized_unsigned_setpoint.h>
#include <uORB/topics/flight_phase_estimation.h>
#include <uORB/topics/lateral_control_configuration.h>
#include <uORB/topics/longitudinal_control_configuration.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>

static constexpr fixed_wing_lateral_setpoint_s empty_lateral_control_setpoint = {.timestamp = 0, .course = NAN, .airspeed_direction = NAN, .lateral_acceleration = NAN};
static constexpr fixed_wing_longitudinal_setpoint_s empty_longitudinal_control_setpoint = {.timestamp = 0, .altitude = NAN, .height_rate = NAN, .equivalent_airspeed = NAN, .pitch_direct = NAN, .throttle_direct = NAN};

class FwLateralLongitudinalControl final : public ModuleBase<FwLateralLongitudinalControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	FwLateralLongitudinalControl(bool is_vtol);

	~FwLateralLongitudinalControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _flaps_setpoint_sub{ORB_ID(flaps_setpoint)};
	uORB::Subscription _wind_sub{ORB_ID(wind)};
	uORB::SubscriptionData<vehicle_control_mode_s> _control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::SubscriptionData<vehicle_air_data_s> _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_landed_sub{ORB_ID(vehicle_land_detected)};
	uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _fw_lateral_ctrl_sub{ORB_ID(fixed_wing_lateral_setpoint)};
	uORB::Subscription _fw_longitudinal_ctrl_sub{ORB_ID(fixed_wing_longitudinal_setpoint)};
	uORB::Subscription _long_control_configuration_sub{ORB_ID(longitudinal_control_configuration)};
	uORB::Subscription _lateral_control_configuration_sub{ORB_ID(lateral_control_configuration)};

	vehicle_local_position_s _local_pos{};
	fixed_wing_longitudinal_setpoint_s _long_control_sp{empty_longitudinal_control_setpoint};
	longitudinal_control_configuration_s _long_configuration{};
	fixed_wing_lateral_setpoint_s _lat_control_sp{empty_lateral_control_setpoint};
	lateral_control_configuration_s _lateral_configuration{};

	float _flaps_setpoint{0.f};

	uORB::Publication <vehicle_attitude_setpoint_s> _attitude_sp_pub;
	uORB::Publication <tecs_status_s> _tecs_status_pub{ORB_ID(tecs_status)};
	uORB::PublicationData <flight_phase_estimation_s> _flight_phase_estimation_pub{ORB_ID(flight_phase_estimation)};
	uORB::Publication <fixed_wing_lateral_status_s> _fixed_wing_lateral_status_pub{ORB_ID(fixed_wing_lateral_status)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_PSP_OFF>) _param_fw_psp_off,
		(ParamBool<px4::params::FW_USE_AIRSPD>) _param_fw_use_airspd,
		(ParamFloat<px4::params::NAV_FW_ALT_RAD>) _param_nav_fw_alt_rad,
		(ParamFloat<px4::params::FW_R_LIM>) _param_fw_r_lim,
		(ParamFloat<px4::params::FW_P_LIM_MAX>) _param_fw_p_lim_max,
		(ParamFloat<px4::params::FW_P_LIM_MIN>) _param_fw_p_lim_min,
		(ParamFloat<px4::params::FW_PN_R_SLEW_MAX>) _param_fw_pn_r_slew_max,
		(ParamFloat<px4::params::FW_T_HRATE_FF>) _param_fw_t_hrate_ff,
		(ParamFloat<px4::params::FW_T_ALT_TC>) _param_fw_t_h_error_tc,
		(ParamFloat<px4::params::FW_T_F_ALT_ERR>) _param_fw_t_fast_alt_err,
		(ParamFloat<px4::params::FW_T_THR_INTEG>) _param_fw_t_thr_integ,
		(ParamFloat<px4::params::FW_T_I_GAIN_PIT>) _param_fw_t_I_gain_pit,
		(ParamFloat<px4::params::FW_T_PTCH_DAMP>) _param_fw_t_ptch_damp,
		(ParamFloat<px4::params::FW_T_RLL2THR>) _param_fw_t_rll2thr,
		(ParamFloat<px4::params::FW_T_SINK_MAX>) _param_fw_t_sink_max,
		(ParamFloat<px4::params::FW_T_TAS_TC>) _param_fw_t_tas_error_tc,
		(ParamFloat<px4::params::FW_T_THR_DAMPING>) _param_fw_t_thr_damping,
		(ParamFloat<px4::params::FW_T_VERT_ACC>) _param_fw_t_vert_acc,
		(ParamFloat<px4::params::FW_T_STE_R_TC>) _param_ste_rate_time_const,
		(ParamFloat<px4::params::FW_T_SEB_R_FF>) _param_seb_rate_ff,
		(ParamFloat<px4::params::FW_T_SPDWEIGHT>) _param_t_spdweight,
		(ParamFloat<px4::params::FW_T_SPD_STD>) _param_speed_standard_dev,
		(ParamFloat<px4::params::FW_T_SPD_DEV_STD>) _param_speed_rate_standard_dev,
		(ParamFloat<px4::params::FW_T_SPD_PRC_STD>) _param_process_noise_standard_dev,
		(ParamFloat<px4::params::FW_T_CLMB_R_SP>) _param_climbrate_target,
		(ParamFloat<px4::params::FW_T_SINK_R_SP>) _param_sinkrate_target,
		(ParamFloat<px4::params::FW_THR_MAX>) _param_fw_thr_max,
		(ParamFloat<px4::params::FW_THR_MIN>) _param_fw_thr_min,
		(ParamFloat<px4::params::FW_THR_SLEW_MAX>) _param_fw_thr_slew_max,
		(ParamFloat<px4::params::FW_LND_THRTC_SC>) _param_fw_thrtc_sc,
		(ParamFloat<px4::params::FW_T_THR_LOW_HGT>) _param_fw_t_thr_low_hgt,
		(ParamFloat<px4::params::FW_WIND_ARSP_SC>) _param_fw_wind_arsp_sc,
		(ParamFloat<px4::params::FW_GND_SPD_MIN>) _param_fw_gnd_spd_min
	)

	hrt_abstime _last_time_loop_ran{};
	uint8_t _z_reset_counter{UINT8_C(0)};
	uint64_t _time_airspeed_last_valid{UINT64_C(0)};
	float _air_density{atmosphere::kAirDensitySeaLevelStandardAtmos};
	// Smooths changes in the altitude tracking error time constant value
	SlewRate<float> _tecs_alt_time_const_slew_rate;
	struct longitudinal_control_state {
		float pitch_rad{0.f};
		float altitude_msl{0.f};
		float airspeed_eas{0.f};
		float eas2tas{1.f};
		float height_rate{0.f};
	} _long_control_state{};

	bool _wind_valid{false};
	hrt_abstime _time_wind_last_received{0};
	SlewRate<float> _roll_slew_rate;
	float _yaw{0.f};
	struct lateral_control_state {
		matrix::Vector2f ground_speed;
		matrix::Vector2f wind_speed;
	} _lateral_control_state{};
	bool _need_report_npfg_uncertain_condition{false}; ///< boolean if reporting of uncertain npfg output condition is needed
	hrt_abstime _time_since_first_reduced_roll{0U}; ///< absolute time since start when entering reduced roll angle for the first time
	hrt_abstime _time_since_last_npfg_call{0U}; 	///< absolute time since start when the npfg reduced roll angle calculations was last performed
	vehicle_attitude_setpoint_s _att_sp{};
	bool _landed{false};
	float _can_run_factor{0.f};
	SlewRate<float> _airspeed_slew_rate_controller;

	perf_counter_t _loop_perf; // loop performance counter

	PerformanceModel _performance_model;
	TECS _tecs;
	CourseToAirspeedRefMapper _course_to_airspeed;
	AirspeedDirectionController _airspeed_direction_control;

	float _min_airspeed_from_guidance{0.f}; // need to store it bc we only update after running longitudinal controller

	void parameters_update();
	void update_control_state();
	void tecs_update_pitch_throttle(const float control_interval, float alt_sp, float airspeed_sp,
					float pitch_min_rad, float pitch_max_rad, float throttle_min,
					float throttle_max, const float desired_max_sinkrate,
					const float desired_max_climbrate,
					bool disable_underspeed_detection, float hgt_rate_sp);

	void tecs_status_publish(float alt_sp, float equivalent_airspeed_sp, float true_airspeed_derivative_raw,
				 float throttle_trim);

	void updateAirspeed();

	void updateAttitude();

	void updateAltitudeAndHeightRate();

	float mapLateralAccelerationToRollAngle(float lateral_acceleration_sp) const;

	void updateWind();

	void updateTECSAltitudeTimeConstant(const bool is_low_height, const float dt);

	bool checkLowHeightConditions() const;

	float getGuidanceQualityFactor(const vehicle_local_position_s &local_pos, const bool is_wind_valid) const;

	float getCorrectedLateralAccelSetpoint(float lateral_accel_sp);

	void setDefaultLongitudinalControlConfiguration();

	void updateLongitudinalControlConfiguration(const longitudinal_control_configuration_s &configuration_in);

	void updateControllerConfiguration();

	float getLoadFactor() const;

	/**
	 * @brief Returns an adapted calibrated airspeed setpoint
	 *
	 * Adjusts the setpoint for wind, accelerated stall, and slew rates.
	 *
	 * @param control_interval Time since the last position control update [s]
	 * @param calibrated_airspeed_setpoint Calibrated airspeed septoint (generally from the position setpoint) [m/s]
	 * @param calibrated_min_airspeed_guidance Minimum airspeed required for lateral guidance [m/s]
	 * @return Adjusted calibrated airspeed setpoint [m/s]
	 */
	float adapt_airspeed_setpoint(const float control_interval, float calibrated_airspeed_setpoint,
				      float calibrated_min_airspeed_guidance, float wind_speed);
};

#endif //PX4_FWLATERALLONGITUDINALCONTROL_HPP
