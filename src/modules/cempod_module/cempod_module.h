#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>


extern "C" __EXPORT int cempod_module_main(int argc, char *argv[]);


class CempodModule : public ModuleBase<CempodModule>, public ModuleParams
{
public:
	CempodModule(int refresh_param);

	virtual ~CempodModule() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static CempodModule *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;


};
