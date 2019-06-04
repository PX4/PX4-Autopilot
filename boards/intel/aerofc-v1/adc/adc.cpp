/****************************************************************************
 *
 *   Copyright (C) 2016-2019 Intel Corporation. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_module.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/adc_report.h>

using namespace time_literals;

#define SLAVE_ADDR 0x50
#define ADC_ENABLE_REG 0x00
#define ADC_CHANNEL_REG 0x05
#define MAX_CHANNEL 5

enum AEROFC_ADC_BUS {
	AEROFC_ADC_BUS_ALL = 0,
	AEROFC_ADC_BUS_I2C_INTERNAL,
	AEROFC_ADC_BUS_I2C_EXTERNAL
};

static constexpr struct aerofc_adc_bus_option {
	enum AEROFC_ADC_BUS busid;
	uint8_t busnum;
} bus_options[] = {
#ifdef PX4_I2C_BUS_EXPANSION
	{ AEROFC_ADC_BUS_I2C_EXTERNAL, PX4_I2C_BUS_EXPANSION },
#endif
#ifdef PX4_I2C_BUS_EXPANSION1
	{ AEROFC_ADC_BUS_I2C_EXTERNAL, PX4_I2C_BUS_EXPANSION1 },
#endif
#ifdef PX4_I2C_BUS_ONBOARD
	{ AEROFC_ADC_BUS_I2C_INTERNAL, PX4_I2C_BUS_ONBOARD },
#endif
};

class ADC : public ModuleBase<ADC>, public device::I2C, public px4::ScheduledWorkItem
{
public:
	ADC(uint8_t bus);
	~ADC();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	int		init();

	int		probe() override;

	int		start();
	int		stop();

private:
	void			Run() override;

	void			update_adc_report(const hrt_abstime &now);

	static constexpr uint32_t TICKRATE{10_ms};	/**< 100Hz base rate */

	perf_counter_t				_sample_perf;
	adc_report_s				_samples{};		/**< sample buffer */

	uORB::Publication<adc_report_s>		_adc_report_pub{ORB_ID(adc_report)};
};

ADC::ADC(uint8_t bus) :
	I2C("AEROFC_ADC", nullptr, bus, SLAVE_ADDR, 400000),
	ScheduledWorkItem(px4::device_bus_to_wq(get_device_id())),
	_sample_perf(perf_alloc(PC_ELAPSED, "adc_sample"))
{
	for (int i = 0; i < adc_report_s::MAX_CHANNELS; i++) {
		_samples.channel_id[i] = -1;
	}
}

ADC::~ADC()
{
	ScheduleClear();
	perf_free(_sample_perf);
}

int
ADC::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		return ret;
	}

	return start();
}

int
ADC::start()
{
	/* and schedule regular updates */
	ScheduleOnInterval(TICKRATE);
	return PX4_OK;
}

int
ADC::stop()
{
	ScheduleClear();
	return PX4_OK;
}

int
ADC::probe()
{
	uint8_t buffer[2] {};

	_retries = 3;

	/* Enable ADC */
	buffer[0] = ADC_ENABLE_REG;
	buffer[1] = 0x01;
	int ret = transfer(buffer, 2, NULL, 0);

	if (ret != PX4_OK) {
		return -EIO;
	}

	usleep(10000);

	/* Read ADC value */
	buffer[0] = ADC_CHANNEL_REG;
	ret = transfer(buffer, 1, buffer, 2);

	if (ret != PX4_OK) {
		return -EIO;
	}

	return PX4_OK;
}

void
ADC::Run()
{
	uint8_t buffer[2] {};

	buffer[0] = ADC_CHANNEL_REG;
	int ret = transfer(buffer, 1, buffer, sizeof(buffer));

	if (ret != PX4_OK) {
		PX4_ERR("Error reading sample");
		return;
	}

	_samples.channel_id[0] = ADC_BATTERY_VOLTAGE_CHANNEL;
	_samples.channel_value[0] = (buffer[0] | (buffer[1] << 8));

	_samples.timestamp = hrt_absolute_time();

	_adc_report_pub.publish(_samples);
}

int
ADC::print_status()
{
	PX4_INFO("Running");

	perf_print_counter(_sample_perf);

	return 0;
}

int
ADC::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
ADC::task_spawn(int argc, char *argv[])
{
	ADC *instance = new ADC(ADC_CHANNELS);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return instance->start();
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int
ADC::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
ADC driver.
)DESCR_STR");

	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_NAME("adc", "driver");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int adc_main(int argc, char *argv[]);

int adc_main(int argc, char *argv[])
{
	return ADC::main(argc, argv);
}
