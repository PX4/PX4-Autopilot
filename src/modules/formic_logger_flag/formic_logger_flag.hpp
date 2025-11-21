#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/debug_flag.h>
#include <uORB/topics/input_rc.h>

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

	bool init();
	void Run() override;

private:
	void parameters_update(bool force = false);
	void run_once();

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Publication<debug_flag_s> _debug_flag_pub{ORB_ID(debug_flag)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FORMIC_LGR_RC_CH>) _param_formic_logger_flag_rc_ch
	)

	int32_t _active_rc_channel{-1};
};

