#pragma once

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/unregister_ext_component.h>

class ModeRegister : public ModuleBase<ModeRegister>, public ModuleParams, public px4::WorkItem
{
public:
	ModeRegister();
	~ModeRegister() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool doRegister();

	bool init();

private:
	void Run() override;

	uORB::SubscriptionCallbackWorkItem _register_ext_component_reply_sub{this, ORB_ID(register_ext_component_reply)};

	uORB::Publication<register_ext_component_request_s>	_register_ext_component_request_pub{ORB_ID(register_ext_component_request)};
	uORB::Publication<unregister_ext_component_s>	_unregister_ext_component_pub{ORB_ID(unregister_ext_component)};

	bool _requested{false};
	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

};
