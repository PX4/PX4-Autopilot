/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

// NOTE: this part is functionality equivalent to the lis2mdl
// https://www.st.com/resource/en/design_tip/dt0131-digital-magnetometer-and-ecompass-efficient-design-tips--stmicroelectronics.pdf
// https://www.st.com/resource/en/datasheet/iis2mdc.pdf

#include "iis2mdc.h"

using namespace time_literals;

IIS2MDC::IIS2MDC(device::Device *interface, const I2CSPIDriverConfig &config) :
	I2CSPIDriver(config),
	_interface(interface),
	_px4_mag(interface->get_device_id(), config.rotation),
	_sample_count(perf_alloc(PC_COUNT, "iis2mdc_read")),
	_comms_errors(perf_alloc(PC_COUNT, "iis2mdc_comms_errors"))
{}

IIS2MDC::~IIS2MDC()
{
	perf_free(_sample_count);
	perf_free(_comms_errors);
	delete _interface;
}

int IIS2MDC::init()
{
	if (hrt_absolute_time() < 20_ms) {
		px4_usleep(20_ms); // ~10ms power-on time
	}

	write_register(IIS2MDC_ADDR_CFG_REG_A, MD_CONTINUOUS | ODR_100 | COMP_TEMP_EN);
	write_register(IIS2MDC_ADDR_CFG_REG_B, OFF_CANC);
	write_register(IIS2MDC_ADDR_CFG_REG_C, BDU);

	_px4_mag.set_scale(100.f / 65535.f); // +/- 50 Gauss, 16bit

	ScheduleDelayed(20_ms);

	return PX4_OK;
}

void IIS2MDC::RunImpl()
{
	uint8_t status = read_register(IIS2MDC_ADDR_STATUS_REG);

	if (status & IIS2MDC_STATUS_REG_READY) {
		SensorData data = {};

		if (read_register_block(&data) == PX4_OK) {
			int16_t x = int16_t((data.xout1 << 8) | data.xout0);
			int16_t y = int16_t((data.yout1 << 8) | data.yout0);
			int16_t z = -int16_t((data.zout1 << 8) | data.zout0);
			int16_t t = int16_t((data.tout1 << 8) | data.tout0);
			// 16 bits twos complement with a sensitivity of 8 LSB/°C. Typically, the output zero level corresponds to 25 °C.
			_px4_mag.set_temperature(float(t) / 8.f + 25.f);
			_px4_mag.update(hrt_absolute_time(), x, y, z);
			_px4_mag.set_error_count(perf_event_count(_comms_errors));
			perf_count(_sample_count);

		} else {
			PX4_DEBUG("read failed");
			perf_count(_comms_errors);
		}

	} else {
		PX4_DEBUG("not ready: %u", status);
		perf_count(_comms_errors);
	}

	ScheduleDelayed(10_ms);
}

uint8_t IIS2MDC::read_register_block(SensorData *data)
{
	uint8_t reg = IIS2MDC_ADDR_OUTX_L_REG;

	if (_interface->read(reg, data, sizeof(SensorData)) != PX4_OK) {
		perf_count(_comms_errors);

		return PX4_ERROR;
	}

	return PX4_OK;
}

uint8_t IIS2MDC::read_register(uint8_t reg)
{
	uint8_t value = 0;

	if (_interface->read(reg, &value, sizeof(value)) != PX4_OK) {
		perf_count(_comms_errors);
	}

	return value;
}

void IIS2MDC::write_register(uint8_t reg, uint8_t value)
{
	if (_interface->write(reg, &value, sizeof(value)) != PX4_OK) {
		perf_count(_comms_errors);
	}
}

void IIS2MDC::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_count);
	perf_print_counter(_comms_errors);
}
