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

#include "BMI088_Accelerometer.hpp"

#include <ecl/geo/geo.h> // CONSTANTS_ONE_G

namespace Bosch_BMI088_Accelerometer
{

#define DIR_READ                0x80
#define DIR_WRITE               0x00

static constexpr uint32_t BMI088_BUS_SPEED = 10 * 1000 * 1000;

BMI088_Accelerometer::BMI088_Accelerometer(int bus, uint32_t device, enum Rotation rotation) :
	SPI("BMI088_ACCEL", nullptr, bus, device, SPIDEV_MODE3, BMI088_BUS_SPEED),
	ScheduledWorkItem(px4::device_bus_to_wq(get_device_id())),
	_px4_accel(get_device_id(), ORB_PRIO_DEFAULT, rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmi088: accel read")),
	_measure_interval(perf_alloc(PC_INTERVAL, "bmi088: accel measure interval")),
	_drdy_interval_perf(perf_alloc(PC_INTERVAL, "bmi088: accel data ready interval")),
	_fifo_reset_perf(perf_alloc(PC_COUNT, "bmi088: accel FIFO reset")),
	_bad_transfers(perf_alloc(PC_COUNT, "bmi088: accel bad transfers")),
	_bad_registers(perf_alloc(PC_COUNT, "bmi088: accel bad registers"))
{
	_px4_accel.set_device_type(DRV_DEVTYPE_BMI088);
}

BMI088_Accelerometer::~BMI088_Accelerometer()
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
BMI088_Accelerometer::init()
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
BMI088_Accelerometer::registerRead(Register reg)
{
	/* 6.1.2 SPI interface of accelerometer part
	 *
	 * In case of read operations of the accelerometer part, the requested data
	 * is not sent immediately, but instead first a dummy byte is sent, and
	 * after this dummy byte the actual requested register content is transmitted.
	 */

	uint8_t cmd[3] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	// cmd[1] dummy byte

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[2];
}

void
BMI088_Accelerometer::registerWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_WRITE;
	cmd[1] = value;
	transfer(cmd, cmd, sizeof(cmd));
}

bool
BMI088_Accelerometer::registerWriteVerified(Register reg, uint8_t value)
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
BMI088_Accelerometer::registerSetBits(Register reg, uint8_t setbits)
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
BMI088_Accelerometer::registerClearBits(Register reg, uint8_t clearbits)
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
BMI088_Accelerometer::reset()
{
	// ACC_SOFTRESET: Writing a value of 0xB6 to this register resets the sensor
	registerWrite(Register::ACC_SOFTRESET, 0xB6);
	px4_usleep(2000);


	// ACC_PWR_CONF: 0x00 Active mode
	if (!registerWriteVerified(Register::ACC_PWR_CONF, 0x00)) {
		PX4_ERR("ACC_PWR_CONF failed");
		return PX4_ERROR;
	}


	// ACC_PWR_CTRL: Enter normal mode by writing ‘4’ to ACC_PWR_CTRL and wait for 50 ms
	registerWrite(Register::ACC_PWR_CTRL, 0x04);
	px4_usleep(50000);

	if (!registerWriteVerified(Register::ACC_PWR_CTRL, 0x04)) {
		PX4_ERR("ACC_PWR_CTRL failed");
		return PX4_ERROR;
	}


	// ACC_RANGE: Set ±24g range by writing 0x03 to register ACC_RANGE (0x41)
	static constexpr uint8_t ACC_RANGE_VALUE = 0x03;

	if (!registerWriteVerified(Register::ACC_RANGE, ACC_RANGE_VALUE)) {
		PX4_ERR("ACC_RANGE failed");
		return PX4_ERROR;
	}

	// Accel_X_in_mg = Accel_X_int16 / 32768 * 1000 * 2^(<0x41> + 1) * 1.5
	_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE_VALUE + 1) * 1.5f) / 32768.0f);


	// ACC_CONF:
	uint8_t acc_bwp = 0x0A; // Normal
	uint8_t acc_odr = 0x0C; // ODR 1600 Hz

	if (!registerWriteVerified(Register::ACC_CONF, (acc_bwp << 4) | acc_odr)) {
		PX4_ERR("ACC_CONF failed");
		return PX4_ERROR;
	}

	_px4_accel.set_sample_rate(1600); // 1600 Hz ODR
	_px4_accel.set_integrator_reset_interval(2500); // 400 Hz (2.5 ms)

	// Wait for > 2 ms
	px4_usleep(2000);


	// FIFO_CONFIG_1: The FIFO for accelerometer sensor data is enabled by setting bit #6 (Acc_en) in register 0x49 (FIFO_CONFIG).
	if (!registerSetBits(Register::FIFO_CONFIG_1, FIFO_CONFIG_1_BIT::Acc_en | FIFO_CONFIG_1_BIT::Int2_en)) {
		PX4_ERR("FIFO_CONFIG_1 failed");
		return PX4_ERROR;
	}


	// FIFO_WTM: 13 bit FIFO watermark level value
	// 7 bytes per sensor reading * 4 (2.5 ms of data at 1600 Hz ODR)
	const uint16_t fifo_water_mark = sizeof(FIFO::DATA) * 5;
	registerWriteVerified(Register::FIFO_WTM_0, (fifo_water_mark & 0x00FF));	// fifo_water_mark[7:0]
	registerWriteVerified(Register::FIFO_WTM_1, (fifo_water_mark & 0x0700) >> 8);	// fifo_water_mark[12:8]


	// INT1_IO_CONF: Enable INT1 as output pin
	if (!registerWriteVerified(Register::INT1_IO_CONF, INT1_IO_CONF_BIT::int1_out)) {
		PX4_ERR("INT1_IO_CONF failed");
		return PX4_ERROR;
	}


	// INT2_IO_CONF: Enable INT2 as input pin
	if (!registerSetBits(Register::INT2_IO_CONF, INT2_IO_CONF_BIT::int2_io)) {
		PX4_ERR("INT2_IO_CONF failed");
		return PX4_ERROR;
	}


	// INT1_INT2_MAP_DATA: Map FIFO full interrupt and watermark to pin INT1
	if (!registerSetBits(Register::INT1_INT2_MAP_DATA, INT1_INT2_MAP_DATA_BIT::int1_fwm)) {
		PX4_ERR("INT1_INT2_MAP_DATA failed");
		return PX4_ERROR;
	}


	// ACC_ERR_REG
	uint8_t accel_error_register = registerRead(Register::ACC_ERR_REG);

	if (accel_error_register & Bit0) {
		PX4_ERR("Fatal Error, chip is not in operational state");
		return PX4_ERROR;

	} else if (accel_error_register & Bit2) {
		PX4_ERR("Accelerometer configuration (ACC_CONF) invalid");
		return PX4_ERROR;
	}

	return PX4_OK;
}

#ifdef GPIO_DRDY_BMI088_INT1_ACCEL
static int data_ready_interrupt(int irq, void *context, void *arg)
{
	BMI088_Accelerometer *dev = reinterpret_cast<BMI088_Accelerometer *>(arg);
	dev->data_ready_perf();
	dev->ScheduleNow();

	return 0;
}
#endif // GPIO_DRDY_BMI088_INT1_ACCEL

bool
BMI088_Accelerometer::Start()
{
	Stop();

#ifdef GPIO_DRDY_BMI088_INT1_ACCEL
	// Setup data ready Interrupt on Falling edge for better noise immunity
	px4_arch_gpiosetevent(GPIO_DRDY_BMI088_INT1_ACCEL, false, true, false, &data_ready_interrupt, this);
#else
	ScheduleOnInterval(2500, 2500);
#endif // GPIO_DRDY_BMI088_INT1_ACCEL

	resetFIFO();

	return true;
}

bool
BMI088_Accelerometer::Stop()
{
#ifdef GPIO_DRDY_BMI088_INT1_ACCEL
	// Disable data ready callback
	px4_arch_gpiosetevent(GPIO_DRDY_BMI088_INT1_ACCEL, false, false, false, nullptr, nullptr);
#endif // GPIO_DRDY_BMI088_INT1_ACCEL

	ScheduleClear();

	return true;
}

bool
BMI088_Accelerometer::resetFIFO()
{
	perf_count(_fifo_reset_perf);

	// trigger a FIFO reset by writing 0xB0 to ACC_SOFTRESET (register 0x7E).
	registerWrite(Register::ACC_SOFTRESET, 0xB0);

	return true;
}

int
BMI088_Accelerometer::probe()
{
	/* 6.1 Serial Peripheral Interface (SPI)
	 * ... the accelerometer part starts always in I2C mode
	 * (regardless of the level of the PS pin) and needs to be changed to SPI
	 *  mode actively by sending a rising edge on the CSB1 pin
	 *  (chip select of the accelerometer), on which the accelerometer part
	 *  switches to SPI mode and stays in this mode until the next power-up-reset.
	 *
	 *  To change the sensor to SPI mode in the initialization phase, the user
	 *  could perfom a dummy SPI read operation, e.g. of register ACC_CHIP_ID
	 *  (the obtained value will be invalid).In case of read operations,
	 */
	registerRead(Register::ACC_CHIP_ID);

	// look for device ID
	uint8_t whoami = registerRead(Register::ACC_CHIP_ID);

	if (whoami != ID) {
		PX4_ERR("unexpected whoami 0x%02x", whoami);

		return -EIO;
	}

	return PX4_OK;
}

void
BMI088_Accelerometer::Run()
{
	// start measuring
	perf_begin(_sample_perf);
	perf_count(_measure_interval);

	// FIFO length registers FIFO_LENGTH_1 and FIFO_LENGTH_0 contain the 14 bit FIFO byte
	uint8_t fifo_len_buf[4] {};
	fifo_len_buf[0] = static_cast<uint8_t>(Register::FIFO_LENGTH_0) | DIR_READ;
	// fifo_len_buf[1] dummy byte

	if (transfer(&fifo_len_buf[0], &fifo_len_buf[0], sizeof(fifo_len_buf)) != PX4_OK) {
		perf_end(_sample_perf);
		return;
	}

	const uint8_t FIFO_LENGTH_0 = fifo_len_buf[2];		// fifo_byte_counter[7:0]
	const uint8_t FIFO_LENGTH_1 = fifo_len_buf[3] & 0xFD;	// fifo_byte_counter[13:8]

	const uint16_t fifo_byte_counter = FIFO_LENGTH_0 + (FIFO_LENGTH_1 << 8);

	//PX4_INFO("fifo_byte_counter: %d", fifo_byte_counter);

	// An empty FIFO corresponds to 0x8000
	if ((fifo_byte_counter == 0) || (fifo_byte_counter == 0x8000)) {
		perf_end(_sample_perf);
		return;
	}

	if (fifo_byte_counter > FIFO::SIZE) {
		PX4_ERR("fifo size: %d", fifo_byte_counter);
		resetFIFO();
		perf_end(_sample_perf);
		return;
	}

	// empty FIFO_DATA
	uint8_t fifo_buffer[fifo_byte_counter + 2] {};
	fifo_buffer[0] = static_cast<uint8_t>(Register::FIFO_DATA) | DIR_READ;
	// fifo_buffer[1] dummy byte

	if (transfer(&fifo_buffer[0], &fifo_buffer[0], fifo_byte_counter + 2) != PX4_OK) {
		perf_end(_sample_perf);
		_px4_accel.set_error_count(perf_event_count(_bad_transfers) + perf_event_count(_bad_registers));
		return;
	}


	// first find all sensor data frames in the buffer
	unsigned fifo_buffer_index = 2;

	// array to store the index to every sensor data frame
	int sensor_data_frame_count = 0;
	uint8_t sensor_data_frame_index_array[fifo_byte_counter / 7] {};

	while (fifo_buffer_index < sizeof(fifo_buffer)) {
		// header set by first 6 bits
		switch (fifo_buffer[fifo_buffer_index] & 0xFC) {
		case FIFO::header::sensor_data_frame: {
				// Acceleration sensor data frame
				// Frame length: 7 bytes (1 byte header + 6 bytes payload)
				PX4_DEBUG("Acceleration sensor data frame");

				// check for [INT2 tag]
				if (fifo_buffer[fifo_buffer_index] & Bit1) {
					PX4_INFO("INT2 tag");
				}

				sensor_data_frame_index_array[sensor_data_frame_count] = fifo_buffer_index;
				sensor_data_frame_count++;

				fifo_buffer_index += 7;
			}
			break;

		case FIFO::header::skip_frame: {
				// Skip Frame
				// Frame length: 2 bytes (1 byte header + 1 byte payload)
				PX4_DEBUG("Skip Frame");
				fifo_buffer_index += 2;
			}
			break;

		case FIFO::header::sensor_time_frame: {
				// Sensortime Frame
				// Frame length: 4 bytes (1 byte header + 3 bytes payload)
				PX4_DEBUG("Sensortime Frame");
				fifo_buffer_index += 4;
			}
			break;

		case FIFO::header::FIFO_input_config_frame: {
				// FIFO input config Frame
				// Frame length: 2 bytes (1 byte header + 1 byte payload)
				PX4_DEBUG("FIFO input config Frame");
				fifo_buffer_index += 2;
			}
			break;

		case FIFO::header::sample_drop_frame: {
				// Sample drop Frame
				// Frame length: 2 bytes (1 byte header + 1 byte payload)
				PX4_DEBUG("Sample drop Frame");
				fifo_buffer_index += 2;
			}
			break;

		default:
			fifo_buffer_index++;
			break;
		}
	}

	// assuming DRDY timestamp corresponds with last sample (1600 Hz ODR = 625 us between samples)
	const hrt_abstime timestamp_first = _timestamp_drdy - (sensor_data_frame_count * 625);

	for (int i = 0; i < sensor_data_frame_count; i++) {
		const int fifo_index = sensor_data_frame_index_array[i];

		if (fifo_index > 0) {
			FIFO::DATA *data = (FIFO::DATA *)&fifo_buffer[fifo_index];

			int16_t x = (int16_t)(data->ACC_X_MSB << 8) + data->ACC_X_LSB;
			int16_t y = (int16_t)(data->ACC_Y_MSB << 8) + data->ACC_Y_LSB;
			int16_t z = (int16_t)(data->ACC_Z_MSB << 8) + data->ACC_Z_LSB;

			// 625 microseconds per sample (1600 Hz ODR)
			const hrt_abstime timestamp_sample = timestamp_first + i * 625;

			// Sensing axes orientation (see datasheet 8.2)
			//  flip y and z
			_px4_accel.update(timestamp_sample, x, -y, -z);
		}
	}


	// Temperature: sensor data is updated every 1.28 s.
	if (hrt_elapsed_time(&_last_temperature_update) > 1280000) {
		// stored in an 11-bit value in 2’s complement format
		uint8_t temperature_buf[4] {};
		temperature_buf[0] = static_cast<uint8_t>(Register::TEMP_MSB) | DIR_READ;
		// temperature_buf[1] dummy byte

		if (transfer(&temperature_buf[0], &temperature_buf[0], sizeof(temperature_buf)) != PX4_OK) {
			perf_end(_sample_perf);
			return;
		}

		_last_temperature_update = hrt_absolute_time();

		const uint8_t TEMP_MSB = temperature_buf[2];
		const uint8_t TEMP_LSB = temperature_buf[3];

		// Datasheet 5.3.7: Register 0x22 – 0x23: Temperature sensor data
		uint16_t Temp_uint11 = (TEMP_MSB * 8) + (TEMP_LSB / 32);
		int16_t Temp_int11 = 0;

		if (Temp_uint11 > 1023) {
			Temp_int11 = Temp_uint11 - 2048;

		} else {
			Temp_int11 = Temp_uint11;
		}

		float temperature = (Temp_int11 * 0.125f) + 23.0f;	// Temp_int11 * 0.125°C/LSB + 23°C

		_px4_accel.set_temperature(temperature);
	}

	perf_end(_sample_perf);
}

void
BMI088_Accelerometer::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_measure_interval);
	perf_print_counter(_drdy_interval_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_bad_registers);

	_px4_accel.print_status();
}

void
BMI088_Accelerometer::print_registers()
{
	printf("BMI088 accel registers\n");

	PX4_INFO("ACC_CHIP_ID:\t0x%02hhx", registerRead(Register::ACC_CHIP_ID));

	PX4_INFO("FIFO_LENGTH_0:\t0x%02hhx", registerRead(Register::FIFO_LENGTH_0));
	PX4_INFO("FIFO_LENGTH_1:\t0x%02hhx", registerRead(Register::FIFO_LENGTH_1));

	PX4_INFO("ACC_CONF:\t0x%02hhx", registerRead(Register::ACC_CONF));
	PX4_INFO("ACC_RANGE:\t0x%02hhx", registerRead(Register::ACC_RANGE));

	PX4_INFO("FIFO_WTM_0:\t0x%02hhx", registerRead(Register::FIFO_WTM_0));
	PX4_INFO("FIFO_WTM_1:\t0x%02hhx", registerRead(Register::FIFO_WTM_1));

	PX4_INFO("FIFO_CONFIG_0:\t0x%02hhx", registerRead(Register::FIFO_CONFIG_0));
	PX4_INFO("FIFO_CONFIG_1:\t0x%02hhx", registerRead(Register::FIFO_CONFIG_1));

	PX4_INFO("INT1_IO_CONF:\t0x%02hhx", registerRead(Register::INT1_IO_CONF));
	PX4_INFO("INT2_IO_CONF:\t0x%02hhx", registerRead(Register::INT2_IO_CONF));

	PX4_INFO("INT1_INT2_MAP_DATA:\t0x%02hhx", registerRead(Register::INT1_INT2_MAP_DATA));

	PX4_INFO("ACC_PWR_CONF:\t0x%02hhx", registerRead(Register::ACC_PWR_CONF));
	PX4_INFO("ACC_PWR_CTRL:\t0x%02hhx", registerRead(Register::ACC_PWR_CTRL));
	PX4_INFO("ACC_SOFTRESET:\t0x%02hhx", registerRead(Register::ACC_SOFTRESET));

	printf("\n");
}

} // namespace Bosch_BMI088_Accelerometer
