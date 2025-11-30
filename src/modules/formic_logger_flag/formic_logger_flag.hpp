#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/debug_flag.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/rc_channels.h>


#define linear_mapping(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

enum class ClickState {
	rest = 0,
	onClick = 1,
  };
  

class FormicLoggerFlag : public ModuleBase<FormicLoggerFlag>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	FormicLoggerFlag();
	~FormicLoggerFlag() override = default;

	static int task_spawn(int argc, char *argv[]);
	static FormicLoggerFlag *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	ClickState _click_state = ClickState::rest;
	bool init();
	void Run() override;

private:
	float _threshold_value = 0.1f;
	float get_max_value();
	ClickState find_click_state(float current_click_value);
	void parameters_update(bool force = false);
	void run_once();
	void publish_debug_flag(ClickState click_state);

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _rc_channels_sub{ORB_ID(rc_channels)};
	uORB::Publication<debug_flag_s> _debug_flag_pub{ORB_ID(debug_flag)};
	float on_press_logger_flag();

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FORMIC_LG_CH>) _param_formic_logger_flag_ch,
		(ParamFloat<px4::params::FORMIC_LG_INVERT>) _param_formic_logger_flag_invert
	)

	int32_t _active_rc_channel{-1};
	bool _flag_activate_state{false};
};

	