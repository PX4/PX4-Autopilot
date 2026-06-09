#pragma once

// PX4 platform / module framework
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// uORB
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/estimator_aid_source1d.h>
#include <uORB/topics/estimator_aid_source2d.h>
#include <uORB/topics/estimator_aid_source3d.h>
#include <uORB/topics/formic_ev_state_machine.h>
#include <uORB/topics/formic_pos_req.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>



// Math / filters
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <mathlib/math/filter/AlphaFilter.hpp>

// #include <utility>

// Drivers
#include <drivers/drv_hrt.h>


enum class pipline_status : uint8_t {
	MANUAL   	= 0,
	WAIT_TO_DATA  	= 1,
	INIT_NOT_FUSED 	= 2,
	INIT_FUSED 	= 3,
	VALID_POS 	= 4,
	EV_ERROR 	= 5,	// 'ERROR' collides with the ERROR macro in px4_platform_common/defines.h
};


class FormicWatchdogEv : public ModuleBase<FormicWatchdogEv>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	FormicWatchdogEv();
	~FormicWatchdogEv() override = default;

	static int task_spawn(int argc, char *argv[]);
	static FormicWatchdogEv *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	void Run() override;

private:
	// --- Methods ---
	void parameters_update(bool force = false);
	void update_pipeline_status(); // single place that decides _formic_state.status each cycle
	void copy_odometry_msg(vehicle_odometry_s &odometry);
	bool check_EV_aid_src_heading(float vio_yaw, float estimator_yaw); // returns true if EV yaw aid data was fused this cycle; sets heading_alligned_with_ev
	void resetcounter(vehicle_odometry_s &odometry);
	void no_EvData();
	bool multi_sensor_z_velocity_check();
	float get_yaw_from_quat(const vehicle_odometry_s &odometry);
	void accumulate_quality(const vehicle_odometry_s &odometry); // sum one quality sample during the settle window
	float finalize_quality_average() const;                      // mean quality over the settle window
	void accumulate_3d_velocity(const vehicle_odometry_s &odometry); // sum one EV 3D speed sample during the settle window
	float finalize_3d_velocity_average() const;  
	void handle_pos_req_user_intention();
	bool handle_pos_reset(const float vio_pos[2], const float estimator_pos[2]); // returns true if EV pos aid data was fused this cycle; sets pos_alligned_with_ev

	// --- Subscriptions ---
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _odometry_sub_formic{ORB_ID(formic_odom)};
	uORB::Subscription _air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _estimator_aid_src_heading_sub{ORB_ID(estimator_aid_src_ev_yaw)};
	uORB::Subscription _estimator_aid_src_pos_sub{ORB_ID(estimator_aid_src_ev_pos)};
	uORB::Subscription _estimtor_odometry_sub{ORB_ID(estimator_odometry)};
	uORB::Subscription _formic_pos_req{ORB_ID(formic_pos_req)};  // pos req at the tick time 



	// --- Publications ---
	uORB::Publication<vehicle_odometry_s> _odometry_pub{ORB_ID(vehicle_visual_odometry)};
	uORB::Publication<formic_ev_state_machine_s> _formic_state_machine_pub{ORB_ID(formic_ev_state_machine)};

	// --- Baro state ---
	// Baro derivative low-pass filter (2nd order, ~20 Hz sample, 1 Hz cutoff)
	math::LowPassFilter2p<float> _baro_deriv_filter{20.0f, 0.2f};
	float _baro_filtered{0.0f};


	// --- EV velocity state ---
	bool _ev_vel_enabled{false};
	bool _ev_hpos_enabled{false};

	float _ev_vel_data_z{0.0f};
	hrt_abstime _last_ev_timestamp{0};
	hrt_abstime _first_ev_timestamp{0}; // time the EV stream first started arriving (settle window start)
	hrt_abstime _forwarding_start_time{0}; // time the first EV sample was forwarded this session (1 s VALID_POS hold-off start; reset on dropout)

	// --- EV position derivative (used when velocity not available) — 1st order filter ---
	AlphaFilter<float> _ev_pos_deriv_filter{};
	float _prev_ev_z{0.0f};
	hrt_abstime _prev_ev_pos_time{0};

	// --- Heading reset counter state ---
	hrt_abstime _last_reset_time{0};  // last increment time (for the 3 s throttle)
	bool at_reset_counter {true}; // true while in the RESET phase: keep resetting until the heading aligns with the EV, then flips false to start error checking (re-armed each session on EV dropout)

	// --- State machine output ---
	formic_ev_state_machine_s _formic_state{0};
	bool _pos_requested{false}; // latched formic_pos_req.pos_req: true while a position mode is requested

	
	bool  _init_check_done{false};                 // true once the init average has been computed for this session
	// quality init check
	int   _quality_count{0};   // number of samples accumulated during the settle window
	float _quality_sum{0.0f}; // summed quality accumulated during the settle window
	float _quality_average_init{0.0f};               // mean quality over the settle window (computed once at the transition)
	
	// velocity init check (EV z-velocity, mirrors the quality average above)
	int   _vel_count{0};        // number of EV z-velocity samples accumulated during the settle window
	float _vel_sum{0.0f};       // summed EV z-velocity accumulated during the settle window
	float _vel_average_init{0.0f}; // mean EV z-velocity over the settle window (computed once at the transition)
	
	// --- Parameters ---
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FORMIC_WDEV_EN>) _param_formic_wdev_en,
		(ParamInt<px4::params::FORMIC_WDEV_INIT>) _param_formic_wdev_init,
		(ParamInt<px4::params::EKF2_EV_CTRL>) _param_ekf2_ev_ctrl,
		(ParamInt<px4::params::EKF2_EV_QMIN>) _param_ekf2_ev_qmin,
		(ParamFloat<px4::params::FORMIC_WDEV_DYAW>) _param_formic_wdev_dyaw,
		(ParamFloat<px4::params::FORMIC_WDEV_DPOS>) _param_formic_wdev_dpos,
		(ParamFloat<px4::params::FORMIC_WDEV_VINI>) _param_formic_wdev_vini
	)
};
