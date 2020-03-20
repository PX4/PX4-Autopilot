/****************************************************************************
 *
 *   Copyright (C) 2012-2020 PX4 Development Team. All rights reserved.
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
 * @file ads1115_main.cpp
 * @author SalimTerryLi
 *
 * Driver for the ADS1115 connected via I2C.
 */

#include "ADS1115.h"
#include <drivers/device/i2c.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/adc_report.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_adc.h>
#include <px4_platform_common/getopt.h>

#define ADS1115_DEFAULT_PATH "/dev/ads1115"

using namespace time_literals;

class ADS1115_Drv : public device::I2C, public I2C_Interface, public ModuleBase<ADS1115_Drv>,
	public px4::ScheduledWorkItem
{
public:
	ADS1115_Drv(uint8_t bus, uint8_t address);
	~ADS1115_Drv() override;
	int init() override;

	void readReg(uint8_t addr, uint8_t *buf, size_t len) override;

	void writeReg(uint8_t addr, uint8_t *buf, size_t len) override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

protected:
	ADS1115 *_ads1115 = nullptr;

	adc_report_s _adc_report = {};

private:
	void Run() override;

	uORB::Publication<adc_report_s>		_to_adc_report{ORB_ID(adc_report)};

	static const hrt_abstime	SAMPLE_INTERVAL{50_ms};

	perf_counter_t			_cycle_perf;

	int     _channel_cycle_count = 0;

};

ADS1115_Drv::ADS1115_Drv(uint8_t bus, uint8_t address) :
	I2C("ADS1115", ADS1115_DEFAULT_PATH, bus, address, 400000),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(this->get_device_id())),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample"))
{
	_ads1115 = new ADS1115(this);

	// TODO fill a vaild dev type
	//this->_device_id.devid_s.devtype=0;

	_adc_report.device_id = this->get_device_id();
	_adc_report.resolution = 32768;
	_adc_report.v_ref = 6.144f;

	for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; ++i) {
		_adc_report.channel_id[i] = -1;
	}

	if (_ads1115 == nullptr) {
		PX4_ERR("ads1115 logical driver alloc failed");
	}
}

ADS1115_Drv::~ADS1115_Drv()
{
	ScheduleClear();

	if (_ads1115 != nullptr) {
		delete _ads1115;
	}

	perf_free(_cycle_perf);
}

int ADS1115_Drv::init()
{
	if (_ads1115 == nullptr) {
		return -ENOMEM;
	}

	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("I2C init failed");
		return ret;
	}

	ret = _ads1115->init();

	if (ret != PX4_OK) {
		PX4_ERR("ADS1115 init failed");
		return ret;
	}

	_ads1115->setChannel(ADS1115::A0);  // prepare for the first measure.

	ScheduleOnInterval(SAMPLE_INTERVAL / 4, SAMPLE_INTERVAL / 4);

	return PX4_OK;
}

void ADS1115_Drv::readReg(uint8_t addr, uint8_t *buf, size_t len)
{
	transfer(&addr, 1, buf, len);
}

void ADS1115_Drv::writeReg(uint8_t addr, uint8_t *buf, size_t len)
{
	uint8_t buffer[len + 1];
	buffer[0] = addr;
	memcpy(buffer + 1, buf, sizeof(uint8_t)*len);
	transfer(buffer, len + 1, nullptr, 0);
}

void ADS1115_Drv::Run()
{
	if (should_exit()) {
		PX4_INFO("ADS1115 stopping.");
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	lock();
	perf_begin(_cycle_perf);

	_adc_report.timestamp = hrt_absolute_time();

	if (_ads1115->isSampleReady()) { // whether ADS1115 is ready to be read or not
		int16_t buf;
		ADS1115::ChannelSelection ch = _ads1115->cycleMeasure(&buf);
		++_channel_cycle_count;

		switch (ch) {
		case ADS1115::A0:
			_adc_report.channel_id[0] = 0;
			_adc_report.raw_data[0] = buf;
			break;

		case ADS1115::A1:
			_adc_report.channel_id[1] = 1;
			_adc_report.raw_data[1] = buf;
			break;

		case ADS1115::A2:
			_adc_report.channel_id[2] = 2;
			_adc_report.raw_data[2] = buf;
			break;

		case ADS1115::A3:
			_adc_report.channel_id[3] = 3;
			_adc_report.raw_data[3] = buf;
			break;

		default:
			PX4_DEBUG("ADS1115: undefined behaviour");
			_ads1115->setChannel(ADS1115::A0);
			--_channel_cycle_count;
			break;
		}

		if (_channel_cycle_count == 4) { // ADS1115 has 4 channels
			_channel_cycle_count = 0;
			_to_adc_report.publish(_adc_report);
		}

	} else {
		PX4_WARN("ADS1115 not ready!");
	}

	perf_end(_cycle_perf);
	unlock();
}

int ADS1115_Drv::task_spawn(int argc, char **argv)
{
	ADS1115_Drv *instance;

	uint8_t i2c_bus = 1;
	uint8_t i2c_addr = 0x48;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "a:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

		case 'a':
			i2c_addr = atoi(myoptarg);
			break;

		default:
			print_usage("unknown parameters");
			return -EINVAL;
		}
	}

	instance = new ADS1115_Drv(i2c_bus, i2c_addr);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
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

int ADS1115_Drv::custom_command(int argc, char **argv)
{
	return PX4_OK;
}

int ADS1115_Drv::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
ADS1115 driver.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("ads1115", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_INT('a',72,0,255,"device address on this bus",true);
    PRINT_MODULE_USAGE_PARAM_INT('b',1,0,255,"bus that ADS1115 is connected to",true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" int ads1115_main(int argc, char *argv[])
{
	return ADS1115_Drv::main(argc,argv);
}