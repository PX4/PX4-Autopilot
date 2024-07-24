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

#include "ADS7953.h"

ADS7953::ADS7953(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample"))
{
	_ext_adc_report.device_id =  this->get_device_id();
	_ext_adc_report.resolution = 4095;
	_ext_adc_report.v_ref = 3.3f;

	for (unsigned i = 0; i < MAX_ADC_CHANNELS; i++) {
		_ext_adc_report.channel_id[i] = -1;
		_ext_adc_report.raw_data[i] = -1;
	}
}

ADS7953::~ADS7953()
{
	ScheduleClear();
	perf_free(_cycle_perf);
}


void ADS7953::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();	// nothing to do
}


/*collect and publish the sensor data
*/
void ADS7953::RunImpl()
{
	uint16_t adc_channel_data[12] = {0};
	uint16_t temp = 0;

	if (should_exit()) {
		PX4_INFO("stopping");
		return;	// stop and return immediately to avoid unexpected schedule from stopping procedure
	}

	perf_begin(_cycle_perf);

	_ext_adc_report.timestamp = hrt_absolute_time();

	ADC_Operate(adc_channel_data);	//operate the ADC

	for (int i = 0; i < MAX_ADC_CHANNELS; i++) {
		_ext_adc_report.channel_id[i] = adc_channel_data[i] >> 12;	//channel ID
		temp = adc_channel_data[i] & 0x0fff;	//12 bit data
		_ext_adc_report.raw_data[i] = (double)(2.5 * temp) / 4095.0;	//prepare raw data to get actual adc data
		temp = 0;
	}

	_to_external_adc.publish(_ext_adc_report);
	perf_end(_cycle_perf);
}


void ADS7953::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_cycle_perf);
}

int ADS7953::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		PX4_INFO("Error Initializing External ADC");
		return ret;
	}

	ScheduleOnInterval(1000_ms, 1000_ms);

	return PX4_OK;
}

int ADS7953::probe()
{
	return PX4_OK;
	// px4_arch_gpiowrite();
}

void ADS7953::ADC_Operate(uint16_t *adc_channels)
{
	uint8_t adc_data[2] = {0};
	manualSelect();
	Auto_2_Select_ADC(ADC_SELECT, adc_data);	//programming cycle
	px4_mdelay(1);
	ADC_AUTO_2_Program();
	px4_mdelay(1);

	for (int i = 0; i < MAX_ADC_CHANNELS; i++) {
		Auto_2_Select_ADC(ADC_READ, adc_data);		//data read cycle
		px4_mdelay(1);
		//data combine ...
		adc_channels[i] = (adc_data[0] << 8 | adc_data[1]);
	}
}

/*
to select manual mode for the External ADC
*/
int ADS7953::manualSelect()
{
	uint8_t buf[2] = {'\0'};
	buf[0] = MANUAL_MODE_1 & 0xff;
	buf[1] = MANUAL_MODE_2 & 0xff;
	int ret = writeReg(buf, nullptr, 2);
	px4_mdelay(1);
	return ret;
}

/*
to select the AUTO 2 mode for External ADC

ADC_SELECT
*/
void ADS7953::Auto_2_Select_ADC(operation_modes mode, uint8_t *adc_data)
{
	uint8_t command[2] = {0};

	switch (mode) {
	case ADC_SELECT:
		command[0] = AUTO_2_MODE_1 & 0xff;
		command[1] = AUTO_2_MODE_2 & 0xff;
		writeReg(command, nullptr, 2);
		px4_mdelay(1);
		break;

	case ADC_READ:
		command[0] = AUTO_2_MODE2_1 & 0xff;
		command[1] = AUTO_2_MODE2_2 & 0xff;
		writeReg(command, adc_data, 2);
		px4_mdelay(1);
		break;

	default:
		PX4_INFO("Invalid mode Selected..");
		break;
	}
}


/*
to program AUTO 2 mode for External ADC
*/
void ADS7953::ADC_AUTO_2_Program()
{
	uint8_t command[2] = {0};

	command[0] = ADC_AUTO_2_PROGRAM2_1;
	command[1] = ADC_AUTO_2_PROGRAM2_2;
	writeReg(command, nullptr, 2);
	px4_mdelay(10);
}

/* Parameters:
	cmd: command to be written to the chip
	retval: return value from the chip(if any)
	len: length of command to be sent
*/
int ADS7953::readReg(uint8_t *cmd, uint8_t *retval, size_t len)
{
	return transfer(cmd, retval, len);	//transmit-receive (SPI)
}


int ADS7953::writeReg(uint8_t *cmd, uint8_t *retval, size_t len)
{
	return transfer(cmd, retval, len);
}
