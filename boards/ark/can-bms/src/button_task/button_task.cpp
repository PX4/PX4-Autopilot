/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/power_button_state.h>

#include "../board_config.h"

#define BUTTON_DEBOUNCE_DURATION_US    1000
#define BUTTON_SHORT_PRESS_DURATION_US 300000
#define BUTTON_LONG_PRESS_DURATION_US  500000
#define BQ40Z80_SHUTDOWN_CURRENT_LIMIT_A 0.5f // Current charging or discharging must be less than this to allow turning off the FETs
static constexpr uint64_t BUTTON_SHUTDOWN_DURATION_US = 3000000ul;

using namespace time_literals;

static bool button_pressed = false;

class ButtonTask : public ModuleBase<ButtonTask>, public px4::ScheduledWorkItem
{
public:
	ButtonTask();
	~ButtonTask() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	static int isr_callback(int irq, FAR void *context, void *arg);

private:
	void Run() override;

	void toggleLEDs(bool toggle);
	void strobeLEDs(void);

	orb_advert_t _power_button_state_topic{nullptr};

	uORB::Subscription _battery_sub{ORB_ID(battery_status)};

	int _led_strobe_state = {};
};

ButtonTask::ButtonTask() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	uORB::SubscriptionData<battery_status_s> sensor_accel_sub{ORB_ID(battery_status)};
}

ButtonTask::~ButtonTask()
{
	ScheduleClear();
}

int ButtonTask::isr_callback(int irq, FAR void *context, void *arg)
{
	printf("button pressed: %d\n", irq);
	button_pressed = true;
	return OK;
}

static bool first_pass = true;
void
ButtonTask::Run()
{
	if (first_pass) {
		first_pass = false;

		for (unsigned i = 0; i < 2; i++) {
			toggleLEDs(true);
			usleep(100000);
			toggleLEDs(false);
			usleep(100000);
		}

		usleep(100000);
	}

	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	power_button_state_s new_report = {};

	battery_status_s data;
	_battery_sub.update(&data);

	if (!button_pressed) {
		return;

	} else {
		button_pressed = false;
	}

	bool button_held = !stm32_gpioread(GPIO_BTN_N);

	auto start_time = hrt_absolute_time();
	auto time_now = start_time;

	while (button_held) {
		time_now = hrt_absolute_time();

		uint64_t elapsed = time_now - start_time;
		PX4_INFO("button held: %d", (int)elapsed);

		strobeLEDs();

		if ((elapsed > BUTTON_SHUTDOWN_DURATION_US) && (data.current_a < BQ40Z80_SHUTDOWN_CURRENT_LIMIT_A)) {

			PX4_INFO("Time to shut down");

			px4_arch_gpiosetevent(GPIO_BTN_N, false, true, false, &ButtonTask::isr_callback, this);

			new_report.timestamp = time_now;
			new_report.event = power_button_state_s::PWR_BUTTON_STATE_REQUEST_SHUTDOWN;

			int instance = 0;
			orb_publish_auto(ORB_ID(power_button_state), &_power_button_state_topic, &new_report, &instance);

			stm32_gpiowrite(GPIO_PWR_EN, false);

			stm32_gpiowrite(GPIO_nLED_RED, true);
			stm32_gpiowrite(GPIO_nLED_BLUE, true);

			// TODO: Check if actually shut down. If not, need to keep running and assume we're in parallel with another battery or connected to a charger.
			while (1) {;};
		}

		usleep(100000); // 10hz
		button_held = !stm32_gpioread(GPIO_BTN_N);
	}

	_led_strobe_state = 0;
}

int
ButtonTask::task_spawn(int argc, char *argv[])
{
	ButtonTask *instance = new ButtonTask();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

bool
ButtonTask::init()
{
	ScheduleOnInterval(100_ms);

	px4_arch_configgpio(GPIO_BTN_N);
	px4_arch_gpiosetevent(GPIO_BTN_N, false, true, true, &ButtonTask::isr_callback, this);

	PX4_INFO("starting");

	return true;
}

int ButtonTask::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ButtonTask::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

When the button is held, this task will monitor it and shut the system down if held for more than 3 seconds.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("button_task", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int button_task_main(int argc, char *argv[])
{
	return ButtonTask::main(argc, argv);
}

void ButtonTask::toggleLEDs(bool toggle)
{
	stm32_gpiowrite(GPIO_nLED_RED, toggle);
	stm32_gpiowrite(GPIO_nLED_BLUE, toggle);
}

void ButtonTask::strobeLEDs(void)
{
	switch (_led_strobe_state)
	{
		case 0:
			stm32_gpiowrite(GPIO_nLED_RED, true);
			stm32_gpiowrite(GPIO_nLED_BLUE, true);
			break;

		case 1:
			stm32_gpiowrite(GPIO_nLED_RED, false);
			stm32_gpiowrite(GPIO_nLED_BLUE, true);
			break;

		case 2:
			stm32_gpiowrite(GPIO_nLED_RED, false);
			stm32_gpiowrite(GPIO_nLED_BLUE, false);
			break;
	}

	_led_strobe_state++;

	if (_led_strobe_state == 3) {
		_led_strobe_state = 0;
	}
}
