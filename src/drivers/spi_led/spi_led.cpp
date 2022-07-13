/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "spi_led.hpp"

spi_led::spi_led(int bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode) :
	SPI(DRV_DEVTYPE_UNUSED, MODULE_NAME, bus, device, spi_mode, bus_frequency),
	ModuleParams(nullptr),
	ScheduledWorkItem("spi_led", px4::wq_configurations::test1)
{
}

spi_led::~spi_led()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
	delete _buf;
	delete _rbuf;
}

int spi_led::init()
{
	if(SPI::init() != OK) {
		printf("SPI::init() failed\n");
		DEVICE_DEBUG("SPI init failed");
		return false;
	}

	// execute Run() on every spi_led publication

	if (!_spi_led_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// alternatively, Run on fixed interval
	//ScheduleOnInterval(100_ms); // 2000 us interval, 200 Hz rate

	return true;
}

void spi_led::Run()
{
	//printf("made it into run()\n");

	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	if (_spi_led_sub.updated()) {
		struct spi_led_s led_struct;

		if (_spi_led_sub.copy(&led_struct)) {

			uint32_t size = led_struct.number_leds + 11;
			if(size != _size) {
				_size = size;
				delete _buf;
				delete _rbuf;
				_buf = new uint32_t[size];
				_rbuf = new uint32_t[size];
				for(uint32_t i = 0; i < size; i++) {
					_buf[i] = 0x00000000;
				}
				_buf[size-1] = 0xFFFFFFFF;
			}

			//printf("offset: %d\n", (led_struct.offset_group*10)+1);
			for(uint32_t i = 0; i < 10; i++) {
				if ((i + (led_struct.offset_group*10)) < led_struct.number_leds) {
					_buf[i + (1+led_struct.offset_group)*10] = led_struct.led_values[i];
				}
			}


			transfer((uint8_t *)_buf, (uint8_t *)_rbuf, _size*4);

		}
	}

	perf_end(_loop_perf);
}

int spi_led::task_spawn(int argc, char *argv[])
{
	spi_led *instance = new spi_led(1, 0, 4000000, SPIDEV_MODE0);

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

int spi_led::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int spi_led::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int spi_led::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
spi_led driver for leds like apa102c and sk9822.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("spi_led", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int spi_led_main(int argc, char *argv[])
{
	return spi_led::main(argc, argv);
}
