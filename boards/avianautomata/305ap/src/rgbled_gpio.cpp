/****************************************************************************
 *
 *   Copyright (c) 2026 Avian Automata LLC. All rights reserved.
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

/**
 * @file rgbled_gpio.cpp
 *
 * Driver for the 305AP RGB status LED (three discrete active-low GPIO LEDs)
 * controlled via the led_control uORB topic.
 *
 * LED pin mapping (from led.c g_ledmap):
 *   LED_BLUE  (0) → GPIO_nLED_BLUE  PE12
 *   LED_RED   (1) → GPIO_nLED_RED   PD9
 *   LED_GREEN (3) → GPIO_nLED_GREEN PB12
 *
 * LEDs are on/off only (no PWM dimming); any brightness > 0 is treated as on.
 */

#include <lib/led/led.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_board_led.h>

#define MODULE_NAME "rgbled_gpio"

__BEGIN_DECLS
extern void led_on(int led);
extern void led_off(int led);
__END_DECLS

namespace
{

class RGBLED_GPIO : public px4::ScheduledWorkItem
{
public:
	RGBLED_GPIO() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}

	~RGBLED_GPIO() override
	{
		_should_run = false;
		int counter = 0;

		while (_running && ++counter < 10) {
			px4_usleep(100000);
		}

		set_rgb(false, false, false);
	}

	int init()
	{
		set_rgb(false, false, false);
		_running = true;
		ScheduleNow();
		return PX4_OK;
	}

private:
	volatile bool _running{false};
	volatile bool _should_run{true};
	LedController _led_controller;

	void Run() override
	{
		if (!_should_run) {
			_running = false;
			return;
		}

		LedControlData data;

		if (_led_controller.update(data) == 1) {
			bool on = data.leds[0].brightness > 0;

			switch (data.leds[0].color) {
			case led_control_s::COLOR_RED:    set_rgb(on,    false, false); break;

			case led_control_s::COLOR_GREEN:  set_rgb(false, on,    false); break;

			case led_control_s::COLOR_BLUE:   set_rgb(false, false, on);   break;

			case led_control_s::COLOR_AMBER:
			case led_control_s::COLOR_YELLOW: set_rgb(on,    on,    false); break;

			case led_control_s::COLOR_PURPLE: set_rgb(on,    false, on);   break;

			case led_control_s::COLOR_CYAN:   set_rgb(false, on,    on);   break;

			case led_control_s::COLOR_WHITE:  set_rgb(on,    on,    on);   break;

			default:                          set_rgb(false, false, false); break;
			}
		}

		ScheduleDelayed(_led_controller.maximum_update_interval());
	}

	void set_rgb(bool r, bool g, bool b)
	{
		r ? led_on(LED_RED)   : led_off(LED_RED);
		g ? led_on(LED_GREEN) : led_off(LED_GREEN);
		b ? led_on(LED_BLUE)  : led_off(LED_BLUE);
	}
};

RGBLED_GPIO *g_dev{nullptr};

} // namespace

extern "C" __EXPORT int rgbled_gpio_main(int argc, char *argv[]);

int rgbled_gpio_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_INFO("usage: rgbled_gpio {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (g_dev) {
			PX4_WARN("already running");
			return 0;
		}

		g_dev = new RGBLED_GPIO();

		if (!g_dev || g_dev->init() != PX4_OK) {
			delete g_dev;
			g_dev = nullptr;
			PX4_ERR("start failed");
			return 1;
		}

		return 0;

	} else if (!strcmp(argv[1], "stop")) {
		if (!g_dev) {
			PX4_WARN("not running");
			return 0;
		}

		delete g_dev;
		g_dev = nullptr;
		return 0;

	} else if (!strcmp(argv[1], "status")) {
		PX4_INFO("%s", g_dev ? "running" : "not running");
		return 0;
	}

	PX4_INFO("usage: rgbled_gpio {start|stop|status}");
	return 1;
}
