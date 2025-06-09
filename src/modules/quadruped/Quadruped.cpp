#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/quadruped_leg_command.h>
#include <mathlib/mathlib.h>
#include <cmath>

using namespace time_literals;

class Quadruped : public ModuleBase<Quadruped>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	Quadruped() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
	{
		_start_time = hrt_absolute_time();
	}

	~Quadruped() override { }

	bool init()
	{
		ScheduleOnInterval(100_ms); // 10 Hz
		return true;
	}

	void Run() override
	{
		if (should_exit()) {
			ScheduleClear();
			exit_and_cleanup();
			return;
		}

		if (_parameter_update_sub.updated()) {
			parameter_update_s p{};
			_parameter_update_sub.copy(&p);
			updateParams();
		}

		const float period_s = math::max(0.001f, static_cast<float>(_param_qdp_period_ms.get()) / 1000.f);
		const float amp = math::constrain(_param_qdp_step_amp.get(), 0.f, 1.f);
		const float rot_amp = math::constrain(_param_qdp_rotate_amp.get(), 0.f, 1.f);

		const float phase = fmodf((hrt_absolute_time() - _start_time) / 1e6f, period_s) / period_s;

		quadruped_leg_command_s cmd{};
		cmd.timestamp = hrt_absolute_time();

		for (int i = 0; i < 4; ++i) {
			float leg_phase = phase + ((i == 1 || i == 2) ? 0.5f : 0.f);

			if (leg_phase >= 1.f) { leg_phase -= 1.f; }

			cmd.wheel_setpoints[i] = (leg_phase < 0.5f) ? amp : -amp;
			cmd.rotate_setpoints[i] = rot_amp * sinf(leg_phase * M_PI_F * 2.f);
		}

		_cmd_pub.publish(cmd);
	}

private:
	uORB::Publication<quadruped_leg_command_s> _cmd_pub{ORB_ID(quadruped_leg_command)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	hrt_abstime _start_time{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::QDP_PERIOD_MS>) _param_qdp_period_ms,
		(ParamFloat<px4::params::QDP_STEP_AMP>) _param_qdp_step_amp,
		(ParamFloat<px4::params::QDP_ROTATE_AMP>) _param_qdp_rotate_amp
	)
};

int Quadruped_main(int argc, char *argv[])
{
	return Quadruped::main(argc, argv);
}
