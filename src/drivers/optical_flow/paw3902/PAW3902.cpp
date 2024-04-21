/****************************************************************************
 *
 *   Copyright (c) 2019-2024 PX4 Development Team. All rights reserved.
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

#include "PAW3902.hpp"

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

PAW3902::PAW3902(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio)
{
	if (_drdy_gpio != 0) {
		_no_motion_interrupt_perf = perf_alloc(PC_COUNT, MODULE_NAME": no motion interrupt");
	}

	float yaw_rotation_degrees = (float)config.custom1;

	if (yaw_rotation_degrees >= 0.f) {
		PX4_INFO("using yaw rotation %.3f degrees (%.3f radians)",
			 (double)yaw_rotation_degrees, (double)math::radians(yaw_rotation_degrees));

		_rotation = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, math::radians(yaw_rotation_degrees)}};

	} else {
		_rotation.identity();
	}
}

PAW3902::~PAW3902()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_reset_perf);
	perf_free(_false_motion_perf);
	perf_free(_mode_change_bright_perf);
	perf_free(_mode_change_low_light_perf);
	perf_free(_mode_change_super_low_light_perf);
	perf_free(_no_motion_interrupt_perf);
}

int PAW3902::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool PAW3902::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	_drdy_timestamp_sample.store(0);
	ScheduleClear();
	ScheduleNow();
	return true;
}

void PAW3902::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void PAW3902::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_reset_perf);
	perf_print_counter(_false_motion_perf);
	perf_print_counter(_mode_change_bright_perf);
	perf_print_counter(_mode_change_low_light_perf);
	perf_print_counter(_mode_change_super_low_light_perf);
	perf_print_counter(_no_motion_interrupt_perf);
}

int PAW3902::probe()
{
	for (int retry = 0; retry < 3; retry++) {
		const uint8_t Product_ID = RegisterRead(Register::Product_ID);
		const uint8_t Revision_ID = RegisterRead(Register::Revision_ID);
		const uint8_t Inverse_Product_ID = RegisterRead(Register::Inverse_Product_ID);

		if (Product_ID != PRODUCT_ID) {
			DEVICE_DEBUG("unexpected Product_ID 0x%02x", Product_ID);
			break;
		}

		if (Revision_ID != REVISION_ID) {
			DEVICE_DEBUG("unexpected Revision_ID 0x%02x", Revision_ID);
			break;
		}

		if (Inverse_Product_ID != PRODUCT_ID_INVERSE) {
			DEVICE_DEBUG("unexpected Inverse_Product_ID 0x%02x", Inverse_Product_ID);
			break;
		}

		return PX4_OK;
	}

	return PX4_ERROR;
}

void PAW3902::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// Issue a soft reset
		RegisterWrite(Register::Power_Up_Reset, 0x5A);

		_bright_to_low_counter = 0;
		_low_to_superlow_counter = 0;
		_low_to_bright_counter = 0;
		_superlow_to_low_counter = 0;

		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		perf_count(_reset_perf);
		ScheduleDelayed(1_ms);
		break;

	case STATE::WAIT_FOR_RESET:
		if (RegisterRead(Register::Product_ID) == PRODUCT_ID) {
			// Read from registers 0x02, 0x03, 0x04, 0x05 and 0x06 one time regardless of the motion pin state.
			RegisterRead(0x02);
			RegisterRead(0x03);
			RegisterRead(0x04);
			RegisterRead(0x05);
			RegisterRead(0x06);

			_discard_reading = 3;

			// if reset succeeded then configure
			_state = STATE::CONFIGURE;
			ScheduleNow();

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 100 ms");
				ScheduleDelayed(100_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start measurement cycle
			_state = STATE::READ;

			if (DataReadyInterruptConfigure()) {
				_motion_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
				ScheduleDelayed(1_s);

			} else {
				_motion_interrupt_enabled = false;
				ScheduleOnInterval(_scheduled_interval_us, _scheduled_interval_us);
			}

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::READ: {
			hrt_abstime timestamp_sample = now;

			if (_motion_interrupt_enabled) {
				// scheduled from interrupt if _drdy_timestamp_sample was set as expected
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if (now < drdy_timestamp_sample + _scheduled_interval_us) {
					timestamp_sample = drdy_timestamp_sample;

				} else {
					perf_count(_no_motion_interrupt_perf);
				}

				// push backup schedule back
				ScheduleDelayed(kBackupScheduleIntervalUs);
			}

			struct TransferBuffer {
				uint8_t cmd = Register::Motion_Burst;
				BURST_TRANSFER data{};
			} buffer{};
			static_assert(sizeof(buffer) == (12 + 1));

			bool success = false;

			if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, sizeof(buffer)) == 0) {

				hrt_store_absolute_time(&_last_read_time);

				if (_discard_reading > 0) {
					_discard_reading--;
				}

				if (buffer.data.RawData_Sum > 0x98) {
					perf_count(_bad_register_perf);
					PX4_ERR("invalid RawData_Sum > 0x98");
				}

				// publish sensor_optical_flow
				sensor_optical_flow_s sensor_optical_flow{};
				sensor_optical_flow.timestamp_sample = timestamp_sample;
				sensor_optical_flow.device_id = get_device_id();

				sensor_optical_flow.error_count = perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf);

				// set specs according to datasheet
				sensor_optical_flow.max_flow_rate = 7.4f;           // Datasheet: 7.4 rad/s
				sensor_optical_flow.min_ground_distance = 0.08f;    // Datasheet: 80mm
				sensor_optical_flow.max_ground_distance = INFINITY; // Datasheet: infinity

				// check SQUAL & Shutter values
				// To suppress false motion reports, discard Delta X and Delta Y values if the SQUAL and Shutter values meet the condition
				// Bright Mode,          SQUAL < 0x19, Shutter ≥ 0x1FF0
				// Low Light Mode,       SQUAL < 0x46, Shutter ≥ 0x1FF0
				// Super Low Light Mode, SQUAL < 0x55, Shutter ≥ 0x0BC0

				// 13-bit Shutter register
				const uint8_t shutter_upper = buffer.data.Shutter_Upper & (Bit4 | Bit3 | Bit2 | Bit1 | Bit0);
				const uint8_t shutter_lower = buffer.data.Shutter_Lower;

				const uint16_t shutter = (shutter_upper << 8) | shutter_lower;

				// Number of Features = SQUAL * 4
				// RawData_Sum maximum register value is 0x98
				bool data_valid = (buffer.data.SQUAL > 0)
						  && (buffer.data.RawData_Sum > 0)
						  && (buffer.data.RawData_Sum <= 0x98)
						  && (shutter > 0)
						  && (_discard_reading == 0);

				switch (_mode) {
				case Mode::Bright:
					sensor_optical_flow.integration_timespan_us = SAMPLE_INTERVAL_MODE_0;
					sensor_optical_flow.mode = sensor_optical_flow_s::MODE_BRIGHT;

					// quality < 25 (0x19) and shutter >= 8176 (0x1FF0)
					if ((buffer.data.SQUAL < 0x19) && (shutter >= 0x1FF0)) {
						// false motion report, discarding
						data_valid = false;
						perf_count(_false_motion_perf);
					}

					// shutter >= 8190 (0x1FFE), raw data sum < 60 (0x3C)
					if ((shutter >= 0x1FFE) && (buffer.data.RawData_Sum < 0x3C)) {
						// Bright -> LowLight
						_bright_to_low_counter++;

						if (_bright_to_low_counter >= 10) {
							_mode = Mode::LowLight;
							Reset();
						}

					} else {
						_bright_to_low_counter = 0;
					}

					break;

				case Mode::LowLight:
					sensor_optical_flow.integration_timespan_us = SAMPLE_INTERVAL_MODE_1;
					sensor_optical_flow.mode = sensor_optical_flow_s::MODE_LOWLIGHT;

					// quality < 70 (0x46) and shutter >= 8176 (0x1FF0)
					if ((buffer.data.SQUAL < 0x46) && (shutter >= 0x1FF0)) {
						// false motion report, discarding
						data_valid = false;
						perf_count(_false_motion_perf);
					}

					// shutter >= 8190 (0x1FFE) and raw data sum < 90 (0x5A)
					if ((shutter >= 0x1FFE) && (buffer.data.RawData_Sum < 0x5A)) {
						// LowLight -> SuperLowLight
						_low_to_bright_counter = 0;
						_low_to_superlow_counter++;

						if (_low_to_superlow_counter >= 10) {
							_mode = Mode::SuperLowLight;
							Reset();
						}

					} else if (shutter < 0x0BB8) {
						// LowLight -> Bright
						//  shutter < 0x0BB8 (3000)
						_low_to_bright_counter++;
						_low_to_superlow_counter = 0;

						if (_low_to_bright_counter >= 10) {
							_mode = Mode::Bright;
							Reset();
						}

					} else {
						_low_to_bright_counter = 0;
						_low_to_superlow_counter = 0;
					}

					break;

				case Mode::SuperLowLight:
					sensor_optical_flow.integration_timespan_us = SAMPLE_INTERVAL_MODE_2;
					sensor_optical_flow.mode = sensor_optical_flow_s::MODE_SUPER_LOWLIGHT;

					// quality < 85 (0x55) and shutter >= 3008 (0x0BC0)
					if ((buffer.data.SQUAL < 0x55) && (shutter >= 0x0BC0)) {
						// false motion report, discarding
						data_valid = false;
						perf_count(_false_motion_perf);
					}

					// shutter < 500 (0x01F4)
					if (shutter < 0x01F4) {
						// should not operate with Shutter < 0x01F4 in Mode 2
						_superlow_to_low_counter++;
						data_valid = false;

					} else if (shutter < 0x03E8) {
						// SuperLowLight -> LowLight
						//  shutter < 1000 (0x03E8)
						_superlow_to_low_counter++;
					}

					if (_superlow_to_low_counter >= 10) {
						_mode = Mode::LowLight;
						Reset();
					}

					break;
				}

				// motion in burst transfer
				const bool motion_reported = (buffer.data.Motion & Motion_Bit::MOT);

				const int16_t delta_x_raw = combine(buffer.data.Delta_X_H, buffer.data.Delta_X_L);
				const int16_t delta_y_raw = combine(buffer.data.Delta_Y_H, buffer.data.Delta_Y_L);

				if (data_valid) {

					const bool zero_flow = (delta_x_raw == 0) && (delta_y_raw == 0);
					const bool little_to_no_flow = (abs(delta_x_raw) <= 1) && (abs(delta_y_raw) <= 1);

					bool publish = false;

					if (motion_reported) {
						// rotate measurements in yaw from sensor frame to body frame
						const matrix::Vector3f pixel_flow_rotated = _rotation * matrix::Vector3f{(float)delta_x_raw, (float)delta_y_raw, 0.f};

						// datasheet provides 11.914 CPI (count per inch) scaling per meter of height
						static constexpr float PIXART_RESOLUTION = 11.914f; // counts per inch (CPI) per meter (from surface)
						static constexpr float INCHES_PER_METER = 39.3701f;

						// CPI/m -> radians
						static constexpr float SCALE = 1.f / (PIXART_RESOLUTION * INCHES_PER_METER);

						sensor_optical_flow.pixel_flow[0] = pixel_flow_rotated(0) * SCALE;
						sensor_optical_flow.pixel_flow[1] = pixel_flow_rotated(1) * SCALE;

						sensor_optical_flow.quality = buffer.data.SQUAL;

						publish = true;

						_last_motion = timestamp_sample;

					} else if (zero_flow && (timestamp_sample > _last_motion)) {
						// no motion, but burst read looks valid and we should have seen new data by now if there was any motion
						const bool burst_read_changed = (delta_x_raw != _delta_x_raw_prev) || (delta_y_raw != _delta_y_raw_prev)
										|| (shutter != _shutter_prev)
										|| (buffer.data.RawData_Sum != _raw_data_sum_prev)
										|| (buffer.data.SQUAL != _quality_prev);

						if (burst_read_changed) {

							sensor_optical_flow.pixel_flow[0] = 0;
							sensor_optical_flow.pixel_flow[1] = 0;

							sensor_optical_flow.quality = buffer.data.SQUAL;

							publish = true;
						}
					}

					// only publish when there's valid data or on timeout
					if (publish || (hrt_elapsed_time(&_last_publish) >= kBackupScheduleIntervalUs)) {

						sensor_optical_flow.timestamp = hrt_absolute_time();
						_sensor_optical_flow_pub.publish(sensor_optical_flow);

						_last_publish = sensor_optical_flow.timestamp_sample;
					}

					// backup schedule if we're reliant on the motion interrupt and there's very little flow
					if (_motion_interrupt_enabled && little_to_no_flow) {
						switch (_mode) {
						case Mode::Bright:
							ScheduleDelayed(SAMPLE_INTERVAL_MODE_0);
							break;

						case Mode::LowLight:
							ScheduleDelayed(SAMPLE_INTERVAL_MODE_1);
							break;

						case Mode::SuperLowLight:
							ScheduleDelayed(SAMPLE_INTERVAL_MODE_2);
							break;
						}
					}

					success = true;

					if (_failure_count > 0) {
						_failure_count--;
					}
				}

				_delta_x_raw_prev = delta_x_raw;
				_delta_y_raw_prev = delta_y_raw;
				_shutter_prev = shutter;
				_raw_data_sum_prev = buffer.data.RawData_Sum;
				_quality_prev = buffer.data.SQUAL;

			} else {
				perf_count(_bad_transfer_perf);
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					Reset();
				}
			}
		}

		break;
	}
}

bool PAW3902::Configure()
{
	switch (_mode) {
	case Mode::Bright:
		ConfigureModeBright();
		_scheduled_interval_us = SAMPLE_INTERVAL_MODE_0 / 2;
		perf_count(_mode_change_bright_perf);
		break;

	case Mode::LowLight:
		ConfigureModeLowLight();
		_scheduled_interval_us = SAMPLE_INTERVAL_MODE_1 / 2;
		perf_count(_mode_change_low_light_perf);
		break;

	case Mode::SuperLowLight:
		ConfigureModeSuperLowLight();
		_scheduled_interval_us = SAMPLE_INTERVAL_MODE_2 / 2;
		perf_count(_mode_change_super_low_light_perf);
		break;
	}

	EnableLed();

	return true;
}

void PAW3902::ConfigureModeBright()
{
	// Mode 0: Bright (126 fps) 60 Lux typical

	// set performance optimization registers
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x55, 0x01);
	RegisterWrite(0x50, 0x07);
	RegisterWrite(0x7f, 0x0e);
	RegisterWrite(0x43, 0x10);

	RegisterWrite(0x48, 0x02);
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x51, 0x7b);
	RegisterWrite(0x50, 0x00);
	RegisterWrite(0x55, 0x00);

	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x61, 0xAD);
	RegisterWrite(0x7F, 0x03);
	RegisterWrite(0x40, 0x00);
	RegisterWrite(0x7F, 0x05);
	RegisterWrite(0x41, 0xB3);
	RegisterWrite(0x43, 0xF1);
	RegisterWrite(0x45, 0x14);
	RegisterWrite(0x5F, 0x34);
	RegisterWrite(0x7B, 0x08);
	RegisterWrite(0x5e, 0x34);
	RegisterWrite(0x5b, 0x32);
	RegisterWrite(0x6d, 0x32);
	RegisterWrite(0x45, 0x17);
	RegisterWrite(0x70, 0xe5);
	RegisterWrite(0x71, 0xe5);
	RegisterWrite(0x7F, 0x06);
	RegisterWrite(0x44, 0x1B);
	RegisterWrite(0x40, 0xBF);
	RegisterWrite(0x4E, 0x3F);
	RegisterWrite(0x7F, 0x08);
	RegisterWrite(0x66, 0x44);
	RegisterWrite(0x65, 0x20);
	RegisterWrite(0x6a, 0x3a);
	RegisterWrite(0x61, 0x05);
	RegisterWrite(0x62, 0x05);
	RegisterWrite(0x7F, 0x09);
	RegisterWrite(0x4F, 0xAF);
	RegisterWrite(0x48, 0x80);
	RegisterWrite(0x49, 0x80);
	RegisterWrite(0x57, 0x77);
	RegisterWrite(0x5F, 0x40);
	RegisterWrite(0x60, 0x78);
	RegisterWrite(0x61, 0x78);
	RegisterWrite(0x62, 0x08);
	RegisterWrite(0x63, 0x50);
	RegisterWrite(0x7F, 0x0A);
	RegisterWrite(0x45, 0x60);
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x4D, 0x11);
	RegisterWrite(0x55, 0x80);
	RegisterWrite(0x74, 0x21);
	RegisterWrite(0x75, 0x1F);
	RegisterWrite(0x4A, 0x78);
	RegisterWrite(0x4B, 0x78);
	RegisterWrite(0x44, 0x08);
	RegisterWrite(0x45, 0x50);
	RegisterWrite(0x64, 0xFE);
	RegisterWrite(0x65, 0x1F);
	RegisterWrite(0x72, 0x0A);
	RegisterWrite(0x73, 0x00);
	RegisterWrite(0x7F, 0x14);
	RegisterWrite(0x44, 0x84);
	RegisterWrite(0x65, 0x47);
	RegisterWrite(0x66, 0x18);
	RegisterWrite(0x63, 0x70);
	RegisterWrite(0x6f, 0x2c);
	RegisterWrite(0x7F, 0x15);
	RegisterWrite(0x48, 0x48);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x41, 0x0D);
	RegisterWrite(0x43, 0x14);
	RegisterWrite(0x4B, 0x0E);
	RegisterWrite(0x45, 0x0F);
	RegisterWrite(0x44, 0x42);
	RegisterWrite(0x4C, 0x80);
	RegisterWrite(0x7F, 0x10);
	RegisterWrite(0x5B, 0x03);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x40, 0x41);

	px4_usleep(10'000); // delay 10ms

	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x32, 0x00);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x40, 0x40);
	RegisterWrite(0x7F, 0x06);
	RegisterWrite(0x68, 0x70);
	RegisterWrite(0x69, 0x01);
	RegisterWrite(0x7F, 0x0D);
	RegisterWrite(0x48, 0xC0);
	RegisterWrite(0x6F, 0xD5);
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x5B, 0xA0);
	RegisterWrite(0x4E, 0xA8);
	RegisterWrite(0x5A, 0x50);
	RegisterWrite(0x40, 0x80);
	RegisterWrite(0x73, 0x1f);

	px4_usleep(10'000); // delay 10ms

	RegisterWrite(0x73, 0x00);
}

void PAW3902::ConfigureModeLowLight()
{
	// Mode 1: Low Light (126 fps) 30 Lux typical
	// low light and low speed motion tracking

	// set performance optimization registers
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x55, 0x01);
	RegisterWrite(0x50, 0x07);
	RegisterWrite(0x7f, 0x0e);
	RegisterWrite(0x43, 0x10);

	RegisterWrite(0x48, 0x02);
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x51, 0x7b);
	RegisterWrite(0x50, 0x00);
	RegisterWrite(0x55, 0x00);

	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x61, 0xAD);
	RegisterWrite(0x7F, 0x03);
	RegisterWrite(0x40, 0x00);
	RegisterWrite(0x7F, 0x05);
	RegisterWrite(0x41, 0xB3);
	RegisterWrite(0x43, 0xF1);
	RegisterWrite(0x45, 0x14);
	RegisterWrite(0x5F, 0x34);
	RegisterWrite(0x7B, 0x08);
	RegisterWrite(0x5e, 0x34);
	RegisterWrite(0x5b, 0x65);
	RegisterWrite(0x6d, 0x65);
	RegisterWrite(0x45, 0x17);
	RegisterWrite(0x70, 0xe5);
	RegisterWrite(0x71, 0xe5);
	RegisterWrite(0x7F, 0x06);
	RegisterWrite(0x44, 0x1B);
	RegisterWrite(0x40, 0xBF);
	RegisterWrite(0x4E, 0x3F);
	RegisterWrite(0x7F, 0x08);
	RegisterWrite(0x66, 0x44);
	RegisterWrite(0x65, 0x20);
	RegisterWrite(0x6a, 0x3a);
	RegisterWrite(0x61, 0x05);
	RegisterWrite(0x62, 0x05);
	RegisterWrite(0x7F, 0x09);
	RegisterWrite(0x4F, 0xAF);
	RegisterWrite(0x48, 0x80);
	RegisterWrite(0x49, 0x80);
	RegisterWrite(0x57, 0x77);
	RegisterWrite(0x5F, 0x40);
	RegisterWrite(0x60, 0x78);
	RegisterWrite(0x61, 0x78);
	RegisterWrite(0x62, 0x08);
	RegisterWrite(0x63, 0x50);
	RegisterWrite(0x7F, 0x0A);
	RegisterWrite(0x45, 0x60);
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x4D, 0x11);
	RegisterWrite(0x55, 0x80);
	RegisterWrite(0x74, 0x21);
	RegisterWrite(0x75, 0x1F);
	RegisterWrite(0x4A, 0x78);
	RegisterWrite(0x4B, 0x78);
	RegisterWrite(0x44, 0x08);
	RegisterWrite(0x45, 0x50);
	RegisterWrite(0x64, 0xFE);
	RegisterWrite(0x65, 0x1F);
	RegisterWrite(0x72, 0x0A);
	RegisterWrite(0x73, 0x00);
	RegisterWrite(0x7F, 0x14);
	RegisterWrite(0x44, 0x84);
	RegisterWrite(0x65, 0x67);
	RegisterWrite(0x66, 0x18);
	RegisterWrite(0x63, 0x70);
	RegisterWrite(0x6f, 0x2c);
	RegisterWrite(0x7F, 0x15);
	RegisterWrite(0x48, 0x48);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x41, 0x0D);
	RegisterWrite(0x43, 0x14);
	RegisterWrite(0x4B, 0x0E);
	RegisterWrite(0x45, 0x0F);
	RegisterWrite(0x44, 0x42);
	RegisterWrite(0x4C, 0x80);
	RegisterWrite(0x7F, 0x10);
	RegisterWrite(0x5B, 0x03);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x40, 0x41);

	px4_usleep(10'000); // delay 10ms

	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x32, 0x00);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x40, 0x40);
	RegisterWrite(0x7F, 0x06);
	RegisterWrite(0x68, 0x70);
	RegisterWrite(0x69, 0x01);
	RegisterWrite(0x7F, 0x0D);
	RegisterWrite(0x48, 0xC0);
	RegisterWrite(0x6F, 0xD5);
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x5B, 0xA0);
	RegisterWrite(0x4E, 0xA8);
	RegisterWrite(0x5A, 0x50);
	RegisterWrite(0x40, 0x80);
	RegisterWrite(0x73, 0x1f);

	px4_usleep(10'000); // delay 10ms

	RegisterWrite(0x73, 0x00);
}

void PAW3902::ConfigureModeSuperLowLight()
{
	// Mode 2: Super Low Light (50 fps) 9 Lux typical
	// super low light and low speed motion tracking

	// set performance optimization registers
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x55, 0x01);
	RegisterWrite(0x50, 0x07);
	RegisterWrite(0x7f, 0x0e);
	RegisterWrite(0x43, 0x10);

	RegisterWrite(0x48, 0x04);
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x51, 0x7b);
	RegisterWrite(0x50, 0x00);
	RegisterWrite(0x55, 0x00);

	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x61, 0xAD);
	RegisterWrite(0x7F, 0x03);
	RegisterWrite(0x40, 0x00);
	RegisterWrite(0x7F, 0x05);
	RegisterWrite(0x41, 0xB3);
	RegisterWrite(0x43, 0xF1);
	RegisterWrite(0x45, 0x14);
	RegisterWrite(0x5F, 0x34);
	RegisterWrite(0x7B, 0x08);
	RegisterWrite(0x5E, 0x34);
	RegisterWrite(0x5B, 0x32);
	RegisterWrite(0x6D, 0x32);
	RegisterWrite(0x45, 0x17);
	RegisterWrite(0x70, 0xE5);
	RegisterWrite(0x71, 0xE5);
	RegisterWrite(0x7F, 0x06);
	RegisterWrite(0x44, 0x1B);
	RegisterWrite(0x40, 0xBF);
	RegisterWrite(0x4E, 0x3F);
	RegisterWrite(0x7F, 0x08);
	RegisterWrite(0x66, 0x44);
	RegisterWrite(0x65, 0x20);
	RegisterWrite(0x6A, 0x3a);
	RegisterWrite(0x61, 0x05);
	RegisterWrite(0x62, 0x05);
	RegisterWrite(0x7F, 0x09);
	RegisterWrite(0x4F, 0xAF);
	RegisterWrite(0x48, 0x80);
	RegisterWrite(0x49, 0x80);
	RegisterWrite(0x57, 0x77);
	RegisterWrite(0x5F, 0x40);
	RegisterWrite(0x60, 0x78);
	RegisterWrite(0x61, 0x78);
	RegisterWrite(0x62, 0x08);
	RegisterWrite(0x63, 0x50);
	RegisterWrite(0x7F, 0x0A);
	RegisterWrite(0x45, 0x60);
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x4D, 0x11);
	RegisterWrite(0x55, 0x80);
	RegisterWrite(0x74, 0x21);
	RegisterWrite(0x75, 0x1F);
	RegisterWrite(0x4A, 0x78);
	RegisterWrite(0x4B, 0x78);
	RegisterWrite(0x44, 0x08);
	RegisterWrite(0x45, 0x50);
	RegisterWrite(0x64, 0xCE);
	RegisterWrite(0x65, 0x0B);
	RegisterWrite(0x72, 0x0A);
	RegisterWrite(0x73, 0x00);
	RegisterWrite(0x7F, 0x14);
	RegisterWrite(0x44, 0x84);
	RegisterWrite(0x65, 0x67);
	RegisterWrite(0x66, 0x18);
	RegisterWrite(0x63, 0x70);
	RegisterWrite(0x6f, 0x2c);
	RegisterWrite(0x7F, 0x15);
	RegisterWrite(0x48, 0x48);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x41, 0x0D);
	RegisterWrite(0x43, 0x14);
	RegisterWrite(0x4B, 0x0E);
	RegisterWrite(0x45, 0x0F);
	RegisterWrite(0x44, 0x42);
	RegisterWrite(0x4C, 0x80);
	RegisterWrite(0x7F, 0x10);
	RegisterWrite(0x5B, 0x02);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x40, 0x41);

	px4_usleep(25'000); // delay 25ms

	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x32, 0x44);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x40, 0x40);
	RegisterWrite(0x7F, 0x06);
	RegisterWrite(0x68, 0x40);
	RegisterWrite(0x69, 0x02);
	RegisterWrite(0x7F, 0x0D);
	RegisterWrite(0x48, 0xC0);
	RegisterWrite(0x6F, 0xD5);
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x5B, 0xA0);
	RegisterWrite(0x4E, 0xA8);
	RegisterWrite(0x5A, 0x50);
	RegisterWrite(0x40, 0x80);
	RegisterWrite(0x73, 0x0B);

	px4_usleep(25'000); // delay 25ms

	RegisterWrite(0x73, 0x00);
}

void PAW3902::EnableLed()
{
	// Enable LED_N controls
	RegisterWrite(0x7F, 0x14);
	RegisterWrite(0x6F, 0x1c);
	RegisterWrite(0x7F, 0x00);
}

int PAW3902::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<PAW3902 *>(arg)->DataReady();
	return 0;
}

void PAW3902::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool PAW3902::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return (px4_arch_gpiosetevent(_drdy_gpio, false, true, false, &DataReadyInterruptCallback, this) == 0);
}

bool PAW3902::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return (px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0);
}

uint8_t PAW3902::RegisterRead(uint8_t reg)
{
	// tSWR SPI Time Between Write And Read Commands
	const hrt_abstime elapsed_last_write = hrt_elapsed_time(&_last_write_time);

	if (elapsed_last_write < TIME_TSWR_us) {
		px4_udelay(TIME_TSWR_us - elapsed_last_write);
	}

	// tSRW/tSRR SPI Time Between Read And Subsequent Commands
	const hrt_abstime elapsed_last_read = hrt_elapsed_time(&_last_read_time);

	if (elapsed_last_read < TIME_TSRW_TSRR_us) {
		px4_udelay(TIME_TSRW_TSRR_us - elapsed_last_read);
	}

	uint8_t cmd[2];
	cmd[0] = DIR_READ(reg);
	cmd[1] = 0;
	transfer(&cmd[0], &cmd[0], sizeof(cmd));
	hrt_store_absolute_time(&_last_read_time);

	return cmd[1];
}

void PAW3902::RegisterWrite(uint8_t reg, uint8_t data)
{
	// tSWW SPI Time Between Write Commands
	const hrt_abstime elapsed_last_write = hrt_elapsed_time(&_last_write_time);

	if (elapsed_last_write < TIME_TSWW_us) {
		px4_udelay(TIME_TSWW_us - elapsed_last_write);
	}

	uint8_t cmd[2];
	cmd[0] = DIR_WRITE(reg);
	cmd[1] = data;
	transfer(&cmd[0], nullptr, sizeof(cmd));
	hrt_store_absolute_time(&_last_write_time);
}
