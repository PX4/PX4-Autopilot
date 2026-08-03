#pragma once

#include "GL40IIProtocol.hpp"

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/voliro_tilt_feedback.h>
#include <uORB/topics/voliro_tilt_setpoint.h>

class GL40IIDriver : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	GL40IIDriver();
	~GL40IIDriver() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	enum class ControlState : uint8_t {
		Disabled = 0,
		Enabling,
		Active,
	};

	static constexpr hrt_abstime RUN_INTERVAL_US = 1'000;
	static constexpr hrt_abstime DISABLE_INTERVAL_US = 20'000;
	static constexpr hrt_abstime ENABLE_INTERVAL_US = 20'000;
	static constexpr hrt_abstime FEEDBACK_PUBLISH_INTERVAL_US = 20'000;

	void Run() override;
	void updateSubscriptions(hrt_abstime now);
	void drainRx(hrt_abstime now);
	void updateMasks(hrt_abstime now);
	void publishFeedback(hrt_abstime now, bool force = false);
	void enterDisabled(hrt_abstime now);

	bool sendFrame(uint8_t bus, const gl40ii::Frame &frame);
	bool sendSpecialAll(gl40ii::SpecialCommand command);
	bool sendMotorCommands();
	bool configurationValid() const;
	bool commandValid() const;
	bool feedbackFresh() const;
	bool enabledFeedback() const;
	uint8_t activeMask() const;
	float motorSign(uint8_t rotor) const;
	const char *stateName() const;

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1'000'000};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _tilt_setpoint_sub{ORB_ID(voliro_tilt_setpoint)};
	uORB::Publication<voliro_tilt_feedback_s> _tilt_feedback_pub{ORB_ID(voliro_tilt_feedback)};

	voliro_tilt_setpoint_s _tilt_setpoint{};
	gl40ii::Limits _limits{};

	float _joint_angle[gl40ii::NUM_MOTORS] {};
	float _joint_velocity[gl40ii::NUM_MOTORS] {};
	float _motor_torque[gl40ii::NUM_MOTORS] {};
	int8_t _driver_temperature[gl40ii::NUM_MOTORS] {};
	int8_t _motor_temperature[gl40ii::NUM_MOTORS] {};
	uint8_t _motor_state[gl40ii::NUM_MOTORS] {};
	uint32_t _rx_count[gl40ii::NUM_MOTORS] {};
	hrt_abstime _last_rx[gl40ii::NUM_MOTORS] {};

	hrt_abstime _last_setpoint_rx{0};
	hrt_abstime _last_tx{0};
	hrt_abstime _last_disable_tx{0};
	hrt_abstime _last_feedback_publish{0};
	uint32_t _tx_error_count{0};
	uint32_t _rx_error_count{0};
	uint32_t _invalid_rx_count{0};
	uint8_t _valid_mask{0};
	uint8_t _enabled_mask{0};
	uint8_t _fault_mask{0};
	uint8_t _stale_mask{0};
	bool _armed{false};
	bool _feedback_dirty{false};
	ControlState _control_state{ControlState::Disabled};
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::GL40_EN>) _param_enable,
		(ParamInt<px4::params::GL40_MASK>) _param_active_mask,
		(ParamInt<px4::params::GL40_RATE>) _param_command_rate,
		(ParamFloat<px4::params::GL40_PMAX>) _param_position_limit,
		(ParamFloat<px4::params::GL40_VMAX>) _param_velocity_limit,
		(ParamFloat<px4::params::GL40_TMAX>) _param_torque_limit,
		(ParamFloat<px4::params::GL40_RATIO>) _param_ratio,
		(ParamFloat<px4::params::GL40_KP>) _param_kp,
		(ParamFloat<px4::params::GL40_KD>) _param_kd,
		(ParamFloat<px4::params::GL40_SP_TMO>) _param_setpoint_timeout,
		(ParamFloat<px4::params::GL40_FB_TMO>) _param_feedback_timeout,
		(ParamInt<px4::params::GL40_REV>) _param_reverse_mask
	)
};
