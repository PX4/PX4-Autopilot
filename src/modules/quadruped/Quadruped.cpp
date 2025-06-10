#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/quadruped_leg_command.h>
#include <uORB/topics/rover_throttle_setpoint.h>
#include <uORB/topics/rover_steering_setpoint.h>
#include <uORB/topics/rover_velocity_setpoint.h>
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

               rover_throttle_setpoint_s thr{};
               if (_throttle_sub.update(&thr)) {
                       _throttle_body_x = thr.throttle_body_x;
               }

               rover_velocity_setpoint_s vel{};
               if (_velocity_sub.update(&vel)) {
                       _throttle_body_x = math::constrain(vel.speed / _param_qdp_max_speed.get(), -1.f, 1.f);
               }

               rover_steering_setpoint_s steer{};
               if (_steering_sub.update(&steer)) {
                       _steering_diff = steer.normalized_speed_diff;
               }

               const float speed_amp = math::constrain(_param_qdp_step_amp.get() * _throttle_body_x, -1.f, 1.f);
               const float rot_amp = math::constrain(_param_qdp_rotate_amp.get() * _steering_diff, -1.f, 1.f);

               const float phase = fmodf((hrt_absolute_time() - _start_time) / 1e6f, period_s) / period_s;

               quadruped_leg_command_s cmd{};
               cmd.timestamp = hrt_absolute_time();

               for (int i = 0; i < 4; ++i) {
                       float leg_phase = phase + ((i == 1 || i == 2) ? 0.5f : 0.f);

                       if (leg_phase >= 1.f) { leg_phase -= 1.f; }

                       cmd.wheel_setpoints[i] = (leg_phase < 0.5f) ? speed_amp : -speed_amp;
                       cmd.rotate_setpoints[i] = rot_amp * sinf(leg_phase * M_PI_F * 2.f);
               }

		_cmd_pub.publish(cmd);
	}

private:
        uORB::Publication<quadruped_leg_command_s> _cmd_pub{ORB_ID(quadruped_leg_command)};
        uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
       uORB::Subscription _throttle_sub{ORB_ID(rover_throttle_setpoint)};
       uORB::Subscription _steering_sub{ORB_ID(rover_steering_setpoint)};
       uORB::Subscription _velocity_sub{ORB_ID(rover_velocity_setpoint)};
        hrt_abstime _start_time{0};

        float _throttle_body_x{0.f};
        float _steering_diff{0.f};

        DEFINE_PARAMETERS(
                (ParamInt<px4::params::QDP_PERIOD_MS>) _param_qdp_period_ms,
                (ParamFloat<px4::params::QDP_STEP_AMP>) _param_qdp_step_amp,
                (ParamFloat<px4::params::QDP_ROTATE_AMP>) _param_qdp_rotate_amp,
                (ParamFloat<px4::params::QDP_MAX_SPEED>) _param_qdp_max_speed
        )
};

int Quadruped_main(int argc, char *argv[])
{
	return Quadruped::main(argc, argv);
}
