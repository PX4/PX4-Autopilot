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

#pragma once

#include <stdint.h>

#include <lib/drivers/device/spi.h>
#include <math.h>
#include <drivers/drv_hrt.h>

#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <lib/parameters/param.h>

#include <uORB/topics/adc_report.h>
#include <uORB/Publication.hpp>

#include <lib/drivers/device/Device.hpp>

using namespace device;
/*commands to set the modes of the ADC*/
#define 		MANUAL_MODE_1  			0x1A
#define			MANUAL_MODE_2			0xC0

#define 		AUTO_1_MODE_1 			0x2C
#define			AUTO_1_MODE_2			0x0F

#define 		AUTO_2_MODE_1 			0x3C
#define			AUTO_2_MODE_2			0x00

#define 		AUTO_2_MODE2_1 			0x38
#define			AUTO_2_MODE2_2			0x00

/*commands to program the modes of the ADC*/
#define			AUTO_1_PROGRAM_1		0x80			//this makes the ADC to go into programming AUTO-1 mode.
#define 		AUTO_1_PROGRAM_2		0x00
// #define 		AUTO_1_SEQUENCE			0x7FFF			//this gives the sequence of the channels to be sampled.
#define 		ADC_AUTO_2_PROGRAM_1		0x91
#define			ADC_AUTO_2_PROGRAM_2		0xC0

#define			ADC_AUTO_2_PROGRAM2_1		0x93
#define			ADC_AUTO_2_PROGRAM2_2		0xC0

#define 		MAX_ADC_CHANNELS		12

using namespace time_literals;

typedef enum _operation_modes {
	ADC_SELECT = 1,
	ADC_READ = 2
} operation_modes;



/*
 * This driver configure ADS1115 into 4 channels with gnd as baseline.
 * Start each sample cycle by setting sample channel.
 * PGA set to 6.144V
 * SPS set to 256
 * Valid output ranges from 0 to 32767 on each channel.
 */
class ADS7953 : public device::SPI, public I2CSPIDriver<ADS7953>
{
public:
	ADS7953(const I2CSPIDriverConfig &config);
	~ADS7953() override;

	int init() override;

	static void print_usage();

	void RunImpl();


	uint32_t get_device_id() { return device::SPI::get_device_id(); }

	int manualSelect();
	void Auto_2_Select_ADC(operation_modes mode, uint8_t *adc_data);
	void ADC_AUTO_2_Program();

	void ADC_Operate(uint16_t *adc_channels);

	int probe() override;

	int readWriteReg(uint8_t *cmd, uint8_t *retval, size_t len);

protected:

	void print_status() override;

	void exit_and_cleanup() override;

private:

	uORB::Publication<adc_report_s>		_to_external_adc{ORB_ID(external_adc)};

	static const hrt_abstime	SAMPLE_INTERVAL{2000_ms};

	adc_report_s _ext_adc_report{};

	perf_counter_t			_cycle_perf;

	int     _channel_cycle_count{0};

	bool	_collect_phase{false};	//to indicate data collection

	bool    _reported_ready_last_cycle{false};

	/* set multiplexer to specific channel */
	// int setChannel(ChannelSelection ch);
	/* return true if sample result is valid */
	// bool isSampleReady();
	/*
	 * get adc sample. return the channel being measured.
	 * Invalid indicates sample failure.
	 */
	// ChannelSelection getMeasurement(int16_t *value);
	/*
	 * get adc sample and automatically switch to next channel and start another measurement
	 */
	// ChannelSelection cycleMeasure(int16_t *value);
	int readReg(uint8_t *cmd, uint8_t *retval, size_t len);
	int writeReg(uint8_t *cmd, uint8_t *retval, size_t len);
};

