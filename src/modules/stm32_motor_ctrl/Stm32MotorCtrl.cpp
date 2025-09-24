#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <mathlib/math/Functions.hpp>
#include <uORB/topics/stm32_setpoint.h>
#include <uORB/topics/actuator_motors.h>

extern "C" __EXPORT int stm32_motor_ctrl_main(int argc, char *argv[]);

static int daemon_task = -1;
static volatile bool should_exit = false;

static int ctrl_thread(int, char **)
{
	uORB::Subscription sp_sub{ORB_ID(stm32_setpoint)};
	uORB::Publication<actuator_motors_s> motors_pub{ORB_ID(actuator_motors)};

	hrt_abstime last_sp_ts = 0;
	stm32_setpoint_s sp{};
	float hold_duty = 0.f;
	uint32_t hold_timeout_ms = 0;

	while (!should_exit) {
		px4_usleep(1000); // ~1 kHz loop
		const hrt_abstime now = hrt_absolute_time();

		while (sp_sub.update(&sp)) {
			last_sp_ts = sp.timestamp;
			hold_timeout_ms = sp.timeout_ms;
			hold_duty = sp.setpoint; // open-loop duty
		}

		// timeout handling
		if (hold_timeout_ms > 0) {
			if ((now - last_sp_ts) / 1000ULL > hold_timeout_ms) {
				hold_duty = 0.f;
			}
		}

		actuator_motors_s out{};
		out.timestamp = now;
		// 简单示例：只驱动 Motor1（索引0），按需扩展映射
		out.control[0] = math::constrain(hold_duty, 0.f, 1.f);
		motors_pub.publish(out);
	}

	return 0;
}

int stm32_motor_ctrl_main(int argc, char *argv[])
{
	if (argc > 1 && !strcmp(argv[1], "start")) {
		if (daemon_task >= 0) { PX4_INFO("already running"); return 0; }
		should_exit = false;
		daemon_task = px4_task_spawn_cmd("stm32_motor_ctrl",
			SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT + 6, 1700,
			ctrl_thread, nullptr);
		return daemon_task < 0 ? -1 : 0;
	}
	if (argc > 1 && !strcmp(argv[1], "stop")) {
		should_exit = true; return 0;
	}
	PX4_INFO("usage: stm32_motor_ctrl {start|stop}");
	return 0;
}


