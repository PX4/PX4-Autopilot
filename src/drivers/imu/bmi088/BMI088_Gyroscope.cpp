/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "BMI088_Gyroscope.hpp"

namespace Bosch_BMI088_Gyroscope
{

#define DIR_READ                0x80
#define DIR_WRITE               0x00

static constexpr uint32_t BMI088_BUS_SPEED = 10 * 1000 * 1000;

BMI088_Gyroscope::BMI088_Gyroscope(int bus, uint32_t device, enum Rotation rotation) :
	SPI("BMI088_GYRO", nullptr, bus, device, SPIDEV_MODE3, BMI088_BUS_SPEED),
	ScheduledWorkItem(px4::device_bus_to_wq(get_device_id())),
	_px4_gyro(get_device_id(), ORB_PRIO_DEFAULT, rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmi088: gyro read")),
	_measure_interval(perf_alloc(PC_INTERVAL, "bmi088: gyro measure interval")),
	_drdy_interval_perf(perf_alloc(PC_INTERVAL, "bmi088: gyro data ready interval")),
	_fifo_reset_perf(perf_alloc(PC_COUNT, "bmi088: gyro FIFO reset")),
	_bad_transfers(perf_alloc(PC_COUNT, "bmi088: gyro bad transfers")),
	_bad_registers(perf_alloc(PC_COUNT, "bmi088: gyro bad registers"))
{
	_px4_gyro.set_device_type(DRV_DEVTYPE_BMI088);
}

BMI088_Gyroscope::~BMI088_Gyroscope()
{
	// make sure we are truly inactive
	ScheduleClear();

	// delete the perf counter
	perf_free(_sample_perf);
	perf_free(_measure_interval);
	perf_free(_drdy_interval_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_bad_transfers);
	perf_free(_bad_registers);
}

int
BMI088_Gyroscope::init()
{
	// do SPI init (and probe) first
	int ret = SPI::init();

	// if probe/setup failed, bail now
	if (ret != OK) {
		DEVICE_DEBUG("SPI setup failed");
		return ret;
	}

	ret = reset();

	if (ret != PX4_OK) {
		return ret;
	}

	Start();

	return PX4_OK;
}

uint8_t
BMI088_Gyroscope::registerRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void
BMI088_Gyroscope::registerWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_WRITE;
	cmd[1] = value;
	transfer(cmd, cmd, sizeof(cmd));
}

bool
BMI088_Gyroscope::registerWriteVerified(Register reg, uint8_t value)
{
	// check if value is already set
	if (registerRead(reg) == value) {
		return true;
	}

	// accel is stubborn, writes get 20 attempts
	for (int i = 0; i < 20; i++) {
		uint8_t cmd[2] {};
		cmd[0] = static_cast<uint8_t>(reg) | DIR_WRITE;
		cmd[1] = value;
		transfer(cmd, cmd, sizeof(cmd));

		const uint8_t read_value = registerRead(reg);

		if (read_value == value) {
			//PX4_INFO("writing register: %X success!", (int)reg);
			return true;
		}
	}

	PX4_ERR("writing register: %X failed 0x%02hhx", (int)reg, value);

	return false;
}

bool
BMI088_Gyroscope::registerSetBits(Register reg, uint8_t setbits)
{
	uint8_t val = registerRead(reg);

	// only write if necessary
	if (!(val & setbits)) {
		val |= setbits;
		return registerWriteVerified(reg, val);
	}

	return true;
}

bool
BMI088_Gyroscope::registerClearBits(Register reg, uint8_t clearbits)
{
	uint8_t val = registerRead(reg);

	// only write if necessary
	if (val & clearbits) {
		val &= !clearbits;
		return registerWriteVerified(reg, val);
	}

	return true;
}

int
BMI088_Gyroscope::reset()
{
	// GYRO_SOFTRESET: Writing a value of 0xB6 to this register resets the sensor.
	// Following a delay of 30 ms, all configuration settings are overwritten with their reset value.
	registerWrite(Register::GYRO_SOFTRESET, 0xB6);
	px4_usleep(30000);


	// GYRO_BANDWIDTH: reset default 2000 Hz
	_px4_gyro.set_sample_rate(2000);
	_px4_gyro.set_integrator_reset_interval(2500); // 400 Hz (2.5 ms)

	// GYRO_RANGE: reset default ±2000 °/s
	_px4_gyro.set_scale(math::radians(2000.0f) / 32767);


	// INT3_INT4_IO_CONF: Push-pull and Active low
	if (!registerWriteVerified(Register::INT3_INT4_IO_CONF, 0)) {
		PX4_ERR("INT3_INT4_IO_CONF failed");
		return PX4_ERROR;
	}

	if (!registerSetBits(Register::INT3_INT4_IO_CONF, INT3_INT4_IO_CONF_BIT::Int4_lvl)) {
		PX4_ERR("INT3_INT4_IO_CONF Int4_lvl failed");
		return PX4_ERROR;
	}


	// FIFO_EXT_INT_S: enable external FIFO synchronization on pin INT4
	if (!registerSetBits(Register::FIFO_EXT_INT_S,
			     FIFO_EXT_INT_S_BIT::ext_fifo_s_en | FIFO_EXT_INT_S_BIT::ext_fifo_s_sel)) {
		PX4_ERR("FIFO_EXT_INT_S failed");
		return PX4_ERROR;
	}


	// GYRO_INT_CTRL: Enables the FIFO interrupt.
	if (!registerSetBits(Register::GYRO_INT_CTRL, GYRO_INT_CTRL_BIT::data_en | GYRO_INT_CTRL_BIT::fifo_en)) {
		PX4_ERR("GYRO_INT_CTRL failed");
		return PX4_ERROR;
	}


	// INT3_INT4_IO_MAP: data ready interrupt is mapped to INT3.
	if (!registerSetBits(Register::INT3_INT4_IO_MAP, INT3_INT4_IO_MAP_BIT::Int3_data)) {
		PX4_ERR("INT3_INT4_IO_MAP failed");
		return PX4_ERROR;
	}

	// FIFO_CONFIG_1: sampling continues when buffer is full (i.e. filled with 99 frames); old is discarded
	if (!registerWriteVerified(Register::FIFO_CONFIG_1, FIFO_CONFIG_1_BIT::STREAM_MODE)) {
		PX4_ERR("FIFO_CONFIG_1 failed");
		return PX4_ERROR;
	}


	return OK;
}

#ifdef GPIO_DRDY_BMI088_INT3_GYRO
static int data_ready_interrupt(int irq, void *context, void *arg)
{
	BMI088_Gyroscope *dev = reinterpret_cast<BMI088_Gyroscope *>(arg);
	dev->data_ready_perf();
	dev->ScheduleNow();

	return 0;
}
#endif // GPIO_DRDY_BMI088_INT3_GYRO

bool
BMI088_Gyroscope::Start()
{
	Stop();

#ifdef GPIO_DRDY_BMI088_INT3_GYRO
	// Setup data ready Interrupt on Falling edge for better noise immunity
	px4_arch_gpiosetevent(GPIO_DRDY_BMI088_INT3_GYRO, false, true, false, &data_ready_interrupt, this);
#else
	ScheduleOnInterval(500, 500);
#endif // GPIO_DRDY_BMI088_INT3_GYRO

	resetFIFO();

	return true;
}

bool
BMI088_Gyroscope::Stop()
{
#ifdef GPIO_DRDY_BMI088_INT3_GYRO
	// Disable data ready callback
	px4_arch_gpiosetevent(GPIO_DRDY_BMI088_INT3_GYRO, false, false, false, nullptr, nullptr);
#endif // GPIO_DRDY_BMI088_INT3_GYRO

	ScheduleClear();

	return true;
}

bool
BMI088_Gyroscope::resetFIFO()
{
	perf_count(_fifo_reset_perf);

	registerRead(Register::INT_STATUS_1);

	// FIFO overrun condition can only be cleared by writing to the FIFO configuration register FIFO_CONFIG_1
	registerWriteVerified(Register::FIFO_CONFIG_1, 0x00);
	registerWriteVerified(Register::FIFO_CONFIG_1, FIFO_CONFIG_1_BIT::STREAM_MODE);

	// Writing to water mark level trigger in register 0x3D clears the FIFO buffer.
	uint8_t watermark = 6; // 1 frame (ODR 2000 Hz)
	registerWriteVerified(Register::FIFO_CONFIG_0, 0x00);
	registerWriteVerified(Register::FIFO_CONFIG_0, watermark);

	return true;
}

int
BMI088_Gyroscope::probe()
{
	// look for device ID
	uint8_t whoami = registerRead(Register::GYRO_CHIP_ID);

	if (whoami != ID) {
		PX4_ERR("unexpected whoami 0x%02x", whoami);

		return -EIO;
	}

	return PX4_OK;
}

void
BMI088_Gyroscope::Run()
{
	// start measuring
	perf_begin(_sample_perf);
	perf_count(_measure_interval);

	const uint8_t fifo_status = registerRead(Register::FIFO_STATUS);

	if (Fifo_overrun & FIFO_STATUS_BIT::Fifo_frame_counter) {
		PX4_ERR("FIFO overrun condition has occurred");

		resetFIFO();

		perf_end(_sample_perf);
		return;
	}

	uint8_t fifo_frame_counter = fifo_status & FIFO_STATUS_BIT::Fifo_frame_counter;

	//PX4_INFO("fifo_frame_counter: %d", fifo_frame_counter);

	if (fifo_frame_counter == 0) {
		perf_end(_sample_perf);
		return;
	}

	// empty FIFO_DATA
	uint8_t fifo_buffer[sizeof(FIFO::DATA)*fifo_frame_counter + 1] {};
	fifo_buffer[0] = static_cast<uint8_t>(Register::FIFO_DATA) | DIR_READ;

	if (transfer(&fifo_buffer[0], &fifo_buffer[0], sizeof(fifo_buffer)) != PX4_OK) {
		perf_end(_sample_perf);
		_px4_gyro.set_error_count(perf_event_count(_bad_transfers) + perf_event_count(_bad_registers));
		return;
	}

	unsigned fifo_buffer_index = 1;
	int sensor_data_frame = 0;

	while (fifo_buffer_index < sizeof(fifo_buffer)) {

		FIFO::DATA *data = (FIFO::DATA *)&fifo_buffer[fifo_buffer_index];

		// check for interrupt tag
		// the least significant bit of the z-axis is used as tag-bit
		if (data->RATE_Z_LSB & Bit0) {
			//PX4_INFO("gyro tag bit set: %d", fifo_buffer_index);
		}

		int16_t x = (int16_t)(data->RATE_X_MSB << 8) + data->RATE_X_LSB;
		int16_t y = (int16_t)(data->RATE_Y_MSB << 8) + data->RATE_Y_LSB;
		int16_t z = (int16_t)(data->RATE_Z_MSB << 8) + data->RATE_Z_LSB;

		// 500 microseconds per sample (2000 Hz ODR)
		const hrt_abstime timestamp_sample = _timestamp_drdy + sensor_data_frame * 500;

		// Sensing axes orientation (see datasheet 8.2)
		//  flip y and z
		_px4_gyro.update(timestamp_sample, x, -y, -z);

		fifo_buffer_index += sizeof(FIFO::DATA);
		sensor_data_frame++;
	}

	perf_end(_sample_perf);
}

void
BMI088_Gyroscope::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_measure_interval);
	perf_print_counter(_drdy_interval_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_bad_registers);

	_px4_gyro.print_status();
}

void
BMI088_Gyroscope::print_registers()
{
	printf("BMI088 gyro registers\n");

	PX4_INFO("GYRO_CHIP_ID:\t0x%02hhx", registerRead(Register::GYRO_CHIP_ID));

	PX4_INFO("INT_STATUS_1:\t0x%02hhx", registerRead(Register::INT_STATUS_1));

	PX4_INFO("FIFO_STATUS:\t0x%02hhx", registerRead(Register::FIFO_STATUS));
	PX4_INFO("GYRO_RANGE:\t0x%02hhx", registerRead(Register::GYRO_RANGE));
	PX4_INFO("GYRO_BANDWIDTH:\t0x%02hhx", registerRead(Register::GYRO_BANDWIDTH));
	PX4_INFO("GYRO_LPM1:\t0x%02hhx", registerRead(Register::GYRO_LPM1));

	PX4_INFO("GYRO_SOFTRESET:\t0x%02hhx", registerRead(Register::GYRO_SOFTRESET));
	PX4_INFO("GYRO_INT_CTRL:\t0x%02hhx", registerRead(Register::GYRO_INT_CTRL));

	PX4_INFO("INT3_INT4_IO_CONF:\t0x%02hhx", registerRead(Register::INT3_INT4_IO_CONF));

	PX4_INFO("INT3_INT4_IO_MAP:\t0x%02hhx", registerRead(Register::INT3_INT4_IO_MAP));

	PX4_INFO("FIFO_WM_ENABLE:\t0x%02hhx", registerRead(Register::FIFO_WM_ENABLE));

	PX4_INFO("FIFO_EXT_INT_S:\t0x%02hhx", registerRead(Register::FIFO_EXT_INT_S));

	PX4_INFO("FIFO_CONFIG_0:\t0x%02hhx", registerRead(Register::FIFO_CONFIG_0));
	PX4_INFO("FIFO_CONFIG_1:\t0x%02hhx", registerRead(Register::FIFO_CONFIG_1));

	printf("\n");
}

} // namespace Bosch_BMI088_Gyroscope
