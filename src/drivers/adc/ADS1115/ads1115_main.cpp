/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/adc_report.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_sensor.h>

using namespace time_literals;

class ADS1115_Wrapper : public device::I2C, public I2C_Interface, public I2CSPIDriver<ADS1115_Wrapper>
{
public:
	ADS1115_Wrapper(I2CSPIBusOption bus_option, int bus, int addr, int bus_frequency);
	~ADS1115_Wrapper() override;
	int init() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);

	static void print_usage();

	void RunImpl();

	int readReg(uint8_t addr, uint8_t *buf, size_t len) override;

	int writeReg(uint8_t addr, uint8_t *buf, size_t len) override;

protected:
	ADS1115 *_ads1115 = nullptr;

	adc_report_s _adc_report = {};

	void print_status() override;

private:

	uORB::Publication<adc_report_s>		_to_adc_report{ORB_ID(adc_report)};

	static const hrt_abstime	SAMPLE_INTERVAL{50_ms};

	perf_counter_t			_cycle_perf;

	int     _channel_cycle_count = 0;

};

ADS1115_Wrapper::ADS1115_Wrapper(I2CSPIBusOption bus_option, int bus, int addr, int bus_frequency) :
	I2C(MODULE_NAME, nullptr, bus, addr, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample"))
{
	set_device_type(DRV_ADC_DEVTYPE_ADS1115);

	_ads1115 = new ADS1115(this);

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

ADS1115_Wrapper::~ADS1115_Wrapper()
{
	ScheduleClear();

	if (_ads1115 != nullptr) {
		delete _ads1115;
	}

	perf_free(_cycle_perf);
}

int ADS1115_Wrapper::init()
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

int ADS1115_Wrapper::readReg(uint8_t addr, uint8_t *buf, size_t len)
{
	return transfer(&addr, 1, buf, len);
}

int ADS1115_Wrapper::writeReg(uint8_t addr, uint8_t *buf, size_t len)
{
	uint8_t buffer[len + 1];
	buffer[0] = addr;
	memcpy(buffer + 1, buf, sizeof(uint8_t)*len);
	return transfer(buffer, len + 1, nullptr, 0);
}

void ADS1115_Wrapper::RunImpl()
{
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
}

I2CSPIDriverBase *ADS1115_Wrapper::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
		int runtime_instance)
{
	ADS1115_Wrapper *instance = new ADS1115_Wrapper(iterator.configuredBusOption(), iterator.bus(), cli.i2c_address,
			cli.bus_frequency);

	if (!instance) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (OK != instance->init()) {
		delete instance;
		return nullptr;
	}

	return instance;
}

void ADS1115_Wrapper::print_usage()
{
	PRINT_MODULE_USAGE_NAME("ads1115", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("adc");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAM_INT('A', 0x48, 0, 0xff, "Address", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

void ADS1115_Wrapper::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_cycle_perf);
}

extern "C" int ads1115_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = ADS1115_Wrapper;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = 0x48;

	while ((ch = cli.getopt(argc, argv, "A:")) != EOF) {
		switch (ch) {
		case 'A':
			cli.i2c_address = atoi(cli.optarg());
			break;
		}
	}

	const char *verb = cli.optarg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_ADC_DEVTYPE_ADS1115);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}