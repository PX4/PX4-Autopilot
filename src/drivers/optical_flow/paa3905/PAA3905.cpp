/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "PAA3905.hpp"

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

PAA3905::PAA3905(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio)
{
	float yaw_rotation_degrees = (float)config.custom1;

	if (yaw_rotation_degrees >= 0.f) {
		PX4_INFO("using yaw rotation %.3f degrees (%.3f radians)",
			 (double)yaw_rotation_degrees, (double)math::radians(yaw_rotation_degrees));

		_rotation = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, math::radians(yaw_rotation_degrees)}};

	} else {
		// otherwise use the parameter SENS_FLOW_ROT
		param_t rot = param_find("SENS_FLOW_ROT");
		int32_t val = 0;

		if (param_get(rot, &val) == PX4_OK) {
			_rotation = get_rot_matrix((enum Rotation)val);

		} else {
			_rotation.identity();
		}
	}
}

PAA3905::~PAA3905()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_interval_perf);
	perf_free(_comms_errors);
	perf_free(_false_motion_perf);
	perf_free(_register_write_fail_perf);
}

int PAA3905::init()
{
	/* do SPI init (and probe) first */
	if (SPI::init() != OK) {
		return PX4_ERROR;
	}

	Configure();

	_previous_collect_timestamp = hrt_absolute_time();

	return PX4_OK;
}

int PAA3905::probe()
{
	const uint8_t Product_ID = RegisterRead(Register::Product_ID);

	if (Product_ID != PRODUCT_ID) {
		PX4_ERR("Product_ID: %X", Product_ID);
		return PX4_ERROR;
	}

	const uint8_t Revision_ID = RegisterRead(Register::Revision_ID);

	if (Revision_ID != REVISION_ID) {
		PX4_ERR("Revision_ID: %X", Revision_ID);
		return PX4_ERROR;
	}

	const uint8_t Inverse_Product_ID = RegisterRead(Register::Inverse_Product_ID);

	if (Inverse_Product_ID != PRODUCT_ID_INVERSE) {
		PX4_ERR("Inverse_Product_ID: %X", Inverse_Product_ID);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int PAA3905::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<PAA3905 *>(arg)->DataReady();
	return 0;
}

void PAA3905::DataReady()
{
	ScheduleNow();
}

bool PAA3905::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	if (px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0) {
		_data_ready_interrupt_enabled = true;
		return true;
	}

	_data_ready_interrupt_enabled = false;
	return false;
}

bool PAA3905::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	_data_ready_interrupt_enabled = false;

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

void PAA3905::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void PAA3905::Configure()
{
	DataReadyInterruptDisable();
	ScheduleClear();

	// Issue a soft reset
	RegisterWrite(Register::Power_Up_Reset, 0x5A);
	px4_usleep(1000);
	_last_reset = hrt_absolute_time();

	StandardDetectionSetting();
	ModeAuto012();

	CheckMode();

	switch (_mode) {
	case Mode::Bright:
		_scheduled_interval_us = SAMPLE_INTERVAL_MODE_0;
		break;

	case Mode::LowLight:
		_scheduled_interval_us = SAMPLE_INTERVAL_MODE_1;
		break;

	case Mode::SuperLowLight:
		_scheduled_interval_us = SAMPLE_INTERVAL_MODE_2;
		break;
	}

	EnableLed();

	_discard_reading = 3;
	_valid_count = 0;

	if (DataReadyInterruptConfigure()) {
		// backup schedule as a watchdog timeout
		ScheduleDelayed(_scheduled_interval_us * 2);

	} else {
		ScheduleOnInterval(_scheduled_interval_us);
	}
}

void PAA3905::CheckMode()
{
	// Read Register 0x15. Check Bit [7:6] for AMS mode and store it into a variable.
	const uint8_t Observation = RegisterRead(Register::Observation);

	// Bit [7:6] AMS mode
	const uint8_t ams_mode = (Observation & (Bit7 | Bit6)) >> 5;

	if (ams_mode == 0x0) {
		// Mode 0
		_mode = Mode::SuperLowLight;

	} else if (ams_mode == 0x1) {
		// Mode 1
		_mode = Mode::LowLight;

	} else if (ams_mode == 0x2) {
		// Mode 2
		_mode = Mode::SuperLowLight;
	}
}

void PAA3905::StandardDetectionSetting()
{
	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x51, 0xFF);
	RegisterWriteVerified(0x4E, 0x2A);
	RegisterWriteVerified(0x66, 0x3E);
	RegisterWriteVerified(0x7F, 0x14);
	RegisterWriteVerified(0x7E, 0x71);
	RegisterWriteVerified(0x55, 0x00);
	RegisterWriteVerified(0x59, 0x00);
	RegisterWriteVerified(0x6F, 0x2C);
	RegisterWriteVerified(0x7F, 0x05);
	RegisterWriteVerified(0x4D, 0xAC);
	RegisterWriteVerified(0x4E, 0x32);
	RegisterWriteVerified(0x7F, 0x09);
	RegisterWriteVerified(0x5C, 0xAF);
	RegisterWriteVerified(0x5F, 0xAF);
	RegisterWriteVerified(0x70, 0x08);
	RegisterWriteVerified(0x71, 0x04);
	RegisterWriteVerified(0x72, 0x06);
	RegisterWriteVerified(0x74, 0x3C);
	RegisterWriteVerified(0x75, 0x28);
	RegisterWriteVerified(0x76, 0x20);
	RegisterWriteVerified(0x4E, 0xBF);
	RegisterWriteVerified(0x7F, 0x03);
	RegisterWriteVerified(0x64, 0x14);
	RegisterWriteVerified(0x65, 0x0A);
	RegisterWriteVerified(0x66, 0x10);
	RegisterWriteVerified(0x55, 0x3C);
	RegisterWriteVerified(0x56, 0x28);
	RegisterWriteVerified(0x57, 0x20);
	RegisterWriteVerified(0x4A, 0x2D);

	RegisterWriteVerified(0x4B, 0x2D);
	RegisterWriteVerified(0x4E, 0x4B);
	RegisterWriteVerified(0x69, 0xFA);
	RegisterWriteVerified(0x7F, 0x05);
	RegisterWriteVerified(0x69, 0x1F);
	RegisterWriteVerified(0x47, 0x1F);
	RegisterWriteVerified(0x48, 0x0C);
	RegisterWriteVerified(0x5A, 0x20);
	RegisterWriteVerified(0x75, 0x0F);
	RegisterWriteVerified(0x4A, 0x0F);
	RegisterWriteVerified(0x42, 0x02);
	RegisterWriteVerified(0x45, 0x03);
	RegisterWriteVerified(0x65, 0x00);
	RegisterWriteVerified(0x67, 0x76);
	RegisterWriteVerified(0x68, 0x76);
	RegisterWriteVerified(0x6A, 0xC5);
	RegisterWriteVerified(0x43, 0x00);
	RegisterWriteVerified(0x7F, 0x06);
	RegisterWriteVerified(0x4A, 0x18);
	RegisterWriteVerified(0x4B, 0x0C);
	RegisterWriteVerified(0x4C, 0x0C);
	RegisterWriteVerified(0x4D, 0x0C);
	RegisterWriteVerified(0x46, 0x0A);
	RegisterWriteVerified(0x59, 0xCD);
	RegisterWriteVerified(0x7F, 0x0A);
	RegisterWriteVerified(0x4A, 0x2A);
	RegisterWriteVerified(0x48, 0x96);
	RegisterWriteVerified(0x52, 0xB4);
	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x5B, 0xA0);
}

void PAA3905::EnhancedDetectionMode()
{
	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x51, 0xFF);
	RegisterWriteVerified(0x4E, 0x2A);
	RegisterWriteVerified(0x66, 0x26);
	RegisterWriteVerified(0x7F, 0x14);
	RegisterWriteVerified(0x7E, 0x71);
	RegisterWriteVerified(0x55, 0x00);
	RegisterWriteVerified(0x59, 0x00);
	RegisterWriteVerified(0x6F, 0x2C);
	RegisterWriteVerified(0x7F, 0x05);
	RegisterWriteVerified(0x4D, 0xAC);
	RegisterWriteVerified(0x4E, 0x65);
	RegisterWriteVerified(0x7F, 0x09);
	RegisterWriteVerified(0x5C, 0xAF);
	RegisterWriteVerified(0x5F, 0xAF);
	RegisterWriteVerified(0x70, 0x00);
	RegisterWriteVerified(0x71, 0x00);
	RegisterWriteVerified(0x72, 0x00);
	RegisterWriteVerified(0x74, 0x14);
	RegisterWriteVerified(0x75, 0x14);
	RegisterWriteVerified(0x76, 0x06);
	RegisterWriteVerified(0x4E, 0x8F);
	RegisterWriteVerified(0x7F, 0x03);
	RegisterWriteVerified(0x64, 0x00);
	RegisterWriteVerified(0x65, 0x00);
	RegisterWriteVerified(0x66, 0x00);
	RegisterWriteVerified(0x55, 0x14);
	RegisterWriteVerified(0x56, 0x14);
	RegisterWriteVerified(0x57, 0x06);
	RegisterWriteVerified(0x4A, 0x20);

	RegisterWriteVerified(0x4B, 0x20);
	RegisterWriteVerified(0x4E, 0x32);
	RegisterWriteVerified(0x69, 0xFE);
	RegisterWriteVerified(0x7F, 0x05);
	RegisterWriteVerified(0x69, 0x14);
	RegisterWriteVerified(0x47, 0x14);
	RegisterWriteVerified(0x48, 0x1C);
	RegisterWriteVerified(0x5A, 0x20);
	RegisterWriteVerified(0x75, 0xE5);
	RegisterWriteVerified(0x4A, 0x05);
	RegisterWriteVerified(0x42, 0x04);
	RegisterWriteVerified(0x45, 0x03);
	RegisterWriteVerified(0x65, 0x00);
	RegisterWriteVerified(0x67, 0x50);
	RegisterWriteVerified(0x68, 0x50);
	RegisterWriteVerified(0x6A, 0xC5);
	RegisterWriteVerified(0x43, 0x00);
	RegisterWriteVerified(0x7F, 0x06);
	RegisterWriteVerified(0x4A, 0x1E);
	RegisterWriteVerified(0x4B, 0x1E);
	RegisterWriteVerified(0x4C, 0x34);
	RegisterWriteVerified(0x4D, 0x34);
	RegisterWriteVerified(0x46, 0x32);
	RegisterWriteVerified(0x59, 0x0D);
	RegisterWriteVerified(0x7F, 0x0A);
	RegisterWriteVerified(0x4A, 0x2A);
	RegisterWriteVerified(0x48, 0x96);
	RegisterWriteVerified(0x52, 0xB4);
	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x5B, 0xA0);
}

void PAA3905::ModeAuto012()
{
	// Automatic switching between Mode 0, 1 and 2:
	RegisterWriteVerified(0x7F, 0x08);
	RegisterWriteVerified(0x68, 0x02);
	RegisterWriteVerified(0x7F, 0x00);
}

void PAA3905::EnableLed()
{
	// Enable LED_N controls
	RegisterWriteVerified(0x7F, 0x14);
	RegisterWriteVerified(0x6F, 0x0C);
	RegisterWriteVerified(0x7F, 0x00);
}

uint8_t PAA3905::RegisterRead(uint8_t reg, int retries)
{
	for (int i = 0; i < retries; i++) {
		px4_udelay(TIME_us_TSRAD);
		uint8_t cmd[2] {reg, 0};

		if (transfer(&cmd[0], &cmd[0], sizeof(cmd)) == 0) {
			return cmd[1];
		}
	}

	perf_count(_comms_errors);
	return 0;
}

void PAA3905::RegisterWrite(uint8_t reg, uint8_t data)
{
	uint8_t cmd[2];
	cmd[0] = DIR_WRITE(reg);
	cmd[1] = data;

	if (transfer(&cmd[0], nullptr, sizeof(cmd)) != 0) {
		perf_count(_comms_errors);
	}
}

bool PAA3905::RegisterWriteVerified(uint8_t reg, uint8_t data, int retries)
{
	for (int i = 0; i < retries; i++) {
		uint8_t cmd[2];
		cmd[0] = DIR_WRITE(reg);
		cmd[1] = data;
		transfer(&cmd[0], nullptr, sizeof(cmd));
		px4_udelay(TIME_us_TSWW);

		// read back to verify
		uint8_t data_read = RegisterRead(reg);

		if (data_read == data) {
			return true;
		}

		PX4_DEBUG("Register write failed 0x%02hhX: 0x%02hhX (actual value 0x%02hhX)", reg, data, data_read);
	}

	perf_count(_register_write_fail_perf);

	return false;
}

void PAA3905::RunImpl()
{
	// backup schedule
	if (_data_ready_interrupt_enabled) {
		ScheduleDelayed(_scheduled_interval_us * 2);
	}

	// force reset if there hasn't been valid data for an extended period (sensor could be in a bad state)
	static constexpr hrt_abstime RESET_TIMEOUT_US = 5_s;

	if ((hrt_elapsed_time(&_last_good_publish) > RESET_TIMEOUT_US) && (hrt_elapsed_time(&_last_reset) > RESET_TIMEOUT_US)) {
		Configure();
		return;
	}

	perf_begin(_sample_perf);
	perf_count(_interval_perf);

	struct TransferBuffer {
		uint8_t cmd = Register::Motion_Burst;
		BURST_TRANSFER data{};
	} buf{};
	static_assert(sizeof(buf) == (14 + 1));

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (transfer((uint8_t *)&buf, (uint8_t *)&buf, sizeof(buf)) != PX4_OK) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return;
	}

	perf_end(_sample_perf);

	const uint64_t dt_flow = timestamp_sample - _previous_collect_timestamp;

	// update for next iteration
	_previous_collect_timestamp = timestamp_sample;

	if (_discard_reading > 0) {
		_discard_reading--;
		ResetAccumulatedData();
		_valid_count = 0;
		return;
	}

	CheckMode(); // update _mode variable

	// check SQUAL & Shutter values
	// To suppress false motion reports, discard Delta X and Delta Y values if the SQUAL and Shutter values meet the condition
	// Bright Mode,          SQUAL < 0x19, Shutter ≥ 0x00FF80
	// Low Light Mode,       SQUAL < 0x46, Shutter ≥ 0x00FF80
	// Super Low Light Mode, SQUAL < 0x55, Shutter ≥ 0x025998
	const uint32_t shutter = (buf.data.Shutter_Upper << 16) | (buf.data.Shutter_Middle << 8) | buf.data.Shutter_Lower;

	bool data_valid = (buf.data.SQUAL > 0);

	switch (_mode) {
	case Mode::Bright:

		// quality < 25 (0x19) and shutter >= 0x00FF80
		if ((buf.data.SQUAL < 0x19) && (shutter >= 0x00FF80)) {
			// false motion report, discarding
			perf_count(_false_motion_perf);
			data_valid = false;
		}

		break;

	case Mode::LowLight:

		// quality < 70 (0x46) and shutter >= 0x00FF80
		if ((buf.data.SQUAL < 0x46) && (shutter >= 0x00FF80)) {
			// false motion report, discarding
			perf_count(_false_motion_perf);
			data_valid = false;
		}

		break;

	case Mode::SuperLowLight:

		// quality < 85 (0x55) and shutter >= 0x025998
		if ((buf.data.SQUAL < 0x55) && (shutter >= 0x025998)) {
			// false motion report, discarding
			perf_count(_false_motion_perf);
			data_valid = false;
		}

		break;
	}

	if (data_valid) {
		const int16_t delta_x_raw = combine(buf.data.Delta_X_H, buf.data.Delta_X_L);
		const int16_t delta_y_raw = combine(buf.data.Delta_Y_H, buf.data.Delta_Y_L);

		_flow_dt_sum_usec += dt_flow;
		_flow_sum_x += delta_x_raw;
		_flow_sum_y += delta_y_raw;
		_flow_sample_counter++;
		_flow_quality_sum += buf.data.SQUAL;

		_valid_count++;

	} else {
		_valid_count = 0;
		ResetAccumulatedData();
		return;
	}

	// returns if the collect time has not been reached
	if (_flow_dt_sum_usec < COLLECT_TIME) {
		return;
	}

	optical_flow_s report{};
	report.timestamp = timestamp_sample;
	//report.device_id = get_device_id();

	float pixel_flow_x_integral = (float)_flow_sum_x / 500.0f;	// proportional factor + convert from pixels to radians
	float pixel_flow_y_integral = (float)_flow_sum_y / 500.0f;	// proportional factor + convert from pixels to radians

	// rotate measurements in yaw from sensor frame to body frame
	const matrix::Vector3f pixel_flow_rotated = _rotation * matrix::Vector3f{pixel_flow_x_integral, pixel_flow_y_integral, 0.f};
	report.pixel_flow_x_integral = pixel_flow_rotated(0);
	report.pixel_flow_y_integral = pixel_flow_rotated(1);

	report.frame_count_since_last_readout = _flow_sample_counter; // number of frames
	report.integration_timespan = _flow_dt_sum_usec;              // microseconds

	report.quality = _flow_quality_sum / _flow_sample_counter;

	// No gyro on this board
	report.gyro_x_rate_integral = NAN;
	report.gyro_y_rate_integral = NAN;
	report.gyro_z_rate_integral = NAN;

	// set (conservative) specs according to datasheet
	report.max_flow_rate = 7.4f;        // Datasheet: 7.4 rad/s
	report.min_ground_distance = 0.08f; // Datasheet: 80mm
	report.max_ground_distance = 30.0f; // Datasheet: infinity


	switch (_mode) {
	case Mode::Bright:
		report.mode = optical_flow_s::MODE_BRIGHT;
		break;

	case Mode::LowLight:
		report.mode = optical_flow_s::MODE_LOWLIGHT;
		break;

	case Mode::SuperLowLight:
		report.mode = optical_flow_s::MODE_SUPER_LOWLIGHT;
		break;
	}

	report.timestamp = hrt_absolute_time();
	_optical_flow_pub.publish(report);

	if (report.quality > 10) {
		_last_good_publish = report.timestamp;
	}

	ResetAccumulatedData();
}

void PAA3905::ResetAccumulatedData()
{
	// reset
	_flow_dt_sum_usec = 0;
	_flow_sum_x = 0;
	_flow_sum_y = 0;
	_flow_sample_counter = 0;
	_flow_quality_sum = 0;
}

void PAA3905::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_sample_perf);
	perf_print_counter(_interval_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_false_motion_perf);
	perf_print_counter(_register_write_fail_perf);
}
