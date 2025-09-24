#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/stm32_setpoint.h>
#include <mathlib/math/Functions.hpp>

extern "C" __EXPORT int stm32_bridge_main(int argc, char *argv[]);

static int daemon_task = -1;
static volatile bool should_exit = false;

static int bridge_thread(int, char **)
{
	px4_sem_t wait_sem;
	px4_sem_init(&wait_sem, 0, 0);

	uORB::Subscription cmd_sub{ORB_ID(vehicle_command)};
	uORB::Publication<stm32_setpoint_s> sp_pub{ORB_ID(stm32_setpoint)};

	while (!should_exit) {
		// poll-like behavior: sleep a short time, then drain
		px4_usleep(5000);

		vehicle_command_s cmd{};
		while (cmd_sub.update(&cmd)) {
			if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_MOTOR_TEST) {
				// Expect percent mode: param2 == 0 (MOTOR_TEST_THROTTLE_PERCENT)
				if ((int)(cmd.param2 + 0.5f) != 0) {
					continue;
				}

				const int instance = (int)(cmd.param1 + 0.5f);
				if (instance <= 0) { continue; }

				stm32_setpoint_s sp{};
				sp.timestamp = hrt_absolute_time();
				sp.source_system = cmd.source_system;
				sp.source_component = cmd.source_component;
				sp.mode = 1; // open-loop duty
				sp.setpoint = cmd.param3 * 0.01f; // percent -> 0..1 duty
				sp.kp = NAN; sp.ki = NAN; sp.kd = NAN;
				sp.out_min = 0.f; sp.out_max = 1.f;
				sp.timeout_ms = (uint32_t)(math::constrain(cmd.param4, 0.f, 3.f) * 1000.f + 0.5f);

				sp_pub.publish(sp);
			}

			if (cmd.command == vehicle_command_s::VEHICLE_CMD_ACTUATOR_TEST) {
				stm32_setpoint_s sp{};
				sp.timestamp = hrt_absolute_time();
				sp.source_system = cmd.source_system;
				sp.source_component = cmd.source_component;
				sp.mode = 1; // open-loop duty
				sp.setpoint = math::constrain(cmd.param1, -1.f, 1.f);
				sp.kp = NAN; sp.ki = NAN; sp.kd = NAN;
				sp.out_min = 0.f; sp.out_max = 1.f;
				sp.timeout_ms = (uint32_t)(math::constrain(cmd.param2, 0.f, 3.f) * 1000.f + 0.5f);
				sp_pub.publish(sp);
			}
		}
	}

	px4_sem_destroy(&wait_sem);
	return 0;
}

int stm32_bridge_main(int argc, char *argv[])
{
	if (argc > 1 && !strcmp(argv[1], "start")) {
		if (daemon_task >= 0) { PX4_INFO("already running"); return 0; }
		should_exit = false;
		daemon_task = px4_task_spawn_cmd("stm32_bridge",
			SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT + 5, 1500,
			bridge_thread, nullptr);
		return daemon_task < 0 ? -1 : 0;
	}
	if (argc > 1 && !strcmp(argv[1], "stop")) {
		should_exit = true; return 0;
	}
	PX4_INFO("usage: stm32_bridge {start|stop}");
	return 0;
}


