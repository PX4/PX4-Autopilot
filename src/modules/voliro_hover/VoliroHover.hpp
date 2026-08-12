#pragma once

#include "VoliroHoverTrajectory.hpp"

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/trajectory_setpoint6dof.h>
#include <uORB/topics/unregister_ext_component.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/voliro_hover_status.h>
#include <uORB/topics/voliro_tilt_feedback.h>

class VoliroHover : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	VoliroHover();
	~VoliroHover() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	bool init();
	int print_status() override;

private:
	static constexpr uint64_t EXT_COMPONENT_REQUEST_ID = 0x56484f560001ULL;
	static constexpr uint8_t ALL_TILTS_MASK = 0x3f;
	static constexpr uint16_t SIM_TILT_FEEDBACK_ID = 4242;

	enum class RegistrationState : uint8_t {
		Unregistered = 0,
		Registered,
		Configured
	};

	enum Command : uint8_t {
		CommandNone = 0,
		CommandTakeoff = 1,
		CommandLand = 2
	};

	void Run() override;
	bool updateConfiguration();
	void updateInputs();
	void requestRegistration(hrt_abstime now);
	void publishModeConfiguration(hrt_abstime now);
	void updateRegistration(hrt_abstime now);
	void unregisterMode();
	void updateArmingCheck();
	void updateModeState(hrt_abstime now);
	void processCommands(hrt_abstime now);
	void advanceLaunch(hrt_abstime now);
	void publishTrajectory(hrt_abstime now);
	void publishStatus(hrt_abstime now);
	bool odometryValid(hrt_abstime now) const;
	uint8_t readinessFailure(hrt_abstime now, bool already_active) const;
	bool tiltCentered() const;
	bool gl40MotionEnabled() const;
	bool armed() const;
	const char *phaseName() const;
	const char *failureName(uint8_t failure) const;

	uORB::Subscription _register_reply_sub{ORB_ID(register_ext_component_reply)};
	uORB::Subscription _arming_check_request_sub{ORB_ID(arming_check_request)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_odometry_sub{ORB_ID(vehicle_odometry)};
	uORB::Subscription _land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _tilt_feedback_sub{ORB_ID(voliro_tilt_feedback)};
	uORB::Subscription _debug_array_sub{ORB_ID(debug_array)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1'000'000};

	uORB::Publication<register_ext_component_request_s> _register_request_pub{ORB_ID(register_ext_component_request)};
	uORB::Publication<unregister_ext_component_s> _unregister_pub{ORB_ID(unregister_ext_component)};
	uORB::Publication<vehicle_control_mode_s> _config_control_setpoints_pub{ORB_ID(config_control_setpoints)};
	uORB::Publication<arming_check_reply_s> _arming_check_reply_pub{ORB_ID(arming_check_reply)};
	uORB::Publication<trajectory_setpoint6dof_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint6dof)};
	uORB::Publication<voliro_hover_status_s> _status_pub{ORB_ID(voliro_hover_status)};

	vehicle_status_s _vehicle_status{};
	vehicle_odometry_s _vehicle_odometry{};
	vehicle_land_detected_s _land_detected{};
	voliro_tilt_feedback_s _tilt_feedback{};
	VoliroHoverTrajectory _trajectory;
	VoliroHoverTrajectory::Setpoint _last_setpoint{};
	px4::atomic<uint8_t> _pending_commands{CommandNone};

	RegistrationState _registration_state{RegistrationState::Unregistered};
	int8_t _mode_id{-1};
	int8_t _arming_check_id{-1};
	hrt_abstime _last_registration_request{0};
	hrt_abstime _launch_started{0};
	uint8_t _failure_reason{voliro_hover_status_s::FAILURE_NOT_REGISTERED};
	bool _mode_selected{false};
	bool _mode_selected_previous{false};
	bool _was_armed{false};
	bool _can_arm_and_run{false};
	bool _typed_feedback_seen{false};
	param_t _gl40_enable_handle{PARAM_INVALID};
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::VHOV_EN>) _param_enable,
		(ParamFloat<px4::params::VHOV_ALT>) _param_height,
		(ParamFloat<px4::params::VHOV_LCH_ACC>) _param_launch_acceleration,
		(ParamFloat<px4::params::VHOV_LCH_TMO>) _param_launch_timeout,
		(ParamFloat<px4::params::VHOV_TD_ACC>) _param_touchdown_acceleration,
		(ParamFloat<px4::params::VHOV_TKO_T>) _param_takeoff_duration,
		(ParamFloat<px4::params::VHOV_LND_T>) _param_land_duration,
		(ParamFloat<px4::params::VHOV_ODOM_TMO>) _param_odometry_timeout,
		(ParamFloat<px4::params::VHOV_FB_TMO>) _param_feedback_timeout,
		(ParamFloat<px4::params::VHOV_ZERO_TOL>) _param_zero_tolerance,
		(ParamBool<px4::params::VOLA_EN>) _param_allocator_enable,
		(ParamBool<px4::params::VCTRL_EN>) _param_controller_enable
	)
};
