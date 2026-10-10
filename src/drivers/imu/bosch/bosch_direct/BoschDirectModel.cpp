/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "BoschDirectModel.hpp"
#include "BoschDirectRegisters.hpp"

namespace bosch_direct_model
{
using namespace bosch_direct_registers;

uint8_t loadRegisters(
	const Profile &profile,
	uint16_t watermark,
	RegisterConfig(&out)[kMaxRegisterConfigs])
{
	uint8_t count = 0;
	bool    valid = true;
	const auto add = [&](uint8_t reg, uint8_t set, uint8_t clear) {
		if (count >= kMaxRegisterConfigs || (set & clear)) {
			valid = false;

			return;
		}

		out[count++] = {reg, set, clear};
	};
	const auto value = [&](uint8_t reg, uint8_t bits, uint8_t mask = 0xff) {
		if (bits & ~mask) {
			valid = false;
		}

		add(reg, bits, mask & ~bits);
	};

	switch (profile.variant) {
#if defined(CONFIG_BOSCH_DIRECT_BMI055)

	case Variant::kBmi055Accel: {
			value(fixed::Range,        0x0c, 0x0f); // 16 g, original 12-bit mode.
			add(fixed::HighBandwidth,  0x80, 0);    // Unfiltered data.
			add(fixed::AccelIntEnable, 0x40, 0);
			add(fixed::AccelIntMap,    0x02, 0);    // INT1 watermark.
			add(fixed::AccelIntIo,     0,    0x03); // Active-low push-pull.
			break;
		}

	case Variant::kBmi055Gyro: {
			value(fixed::Range,         0,    0x07); // 2000 dps.
			add(fixed::HighBandwidth,   0x80, 0);    // Retain BMI055's unfiltered path.
			add(fixed::GyroIntEnable,   0x40, 0);
			add(fixed::GyroIntIo,       0,    0x03);
			add(fixed::GyroIntMap,      0x04, 0);
			add(fixed::WatermarkEnable, 0x80, 0);
			break;
		}

#endif // CONFIG_BOSCH_DIRECT_BMI055

#if defined(CONFIG_BOSCH_DIRECT_BMI085) \
	|| defined(CONFIG_BOSCH_DIRECT_BMI088)

	case Variant::kBmi08xAccel: {
			add(tagged::PowerConfig,   0,    0x03); // Disable power save.
			add(tagged::PowerControl,  0x04, 0);    // Enable accel.
			value(tagged::AccelConfig, 0xac, 0xff); // 1600 Hz, normal bandwidth.
			value(tagged::AccelRange,  0x03, 0x03); // Per-model full scale.
			add(tagged::FifoConfig0,   0x03, 0);    // Stop on full, reserved bit 1 required.
			add(tagged::FifoConfig1, tagged::FifoHeader | tagged::FifoAccel, 0);
#if defined(CONFIG_BOSCH_DIRECT_ACCELEROMETER_INT1)
			add(tagged::Int1Io, 0x08, 0);
			add(tagged::IntMap, 0x02, 0);
#elif defined(CONFIG_BOSCH_DIRECT_ACCELEROMETER_INT2)
			add(tagged::Int2Io, 0x08, 0);
			add(tagged::IntMap, 0x20, 0);
#endif // BOSCH_DIRECT accelerometer interrupt selection
			break;
		}

	case Variant::kBmi08xGyro: {
			value(fixed::Range,         0,    0x07); // 2000 dps.
			value(fixed::Bandwidth,     0,    0x07); // Original 2000 Hz / 532 Hz filter.
			add(fixed::GyroIntEnable,   0x40, 0);
			add(fixed::WatermarkEnable, 0x88, 0);    // Bit 3 is required on BMI08x.
#if defined(CONFIG_BOSCH_DIRECT_GYROSCOPE_INT3)
			add(fixed::GyroIntIo,  0,    0x03);
			add(fixed::GyroIntMap, 0x04, 0);
#elif defined(CONFIG_BOSCH_DIRECT_GYROSCOPE_INT4)
			add(fixed::GyroIntIo,  0,    0x0c);
			add(fixed::GyroIntMap, 0x20, 0);
#endif // BOSCH_DIRECT gyroscope interrupt selection
			break;
		}

#endif // CONFIG_BOSCH_DIRECT_BMI085 || CONFIG_BOSCH_DIRECT_BMI088

#if defined(CONFIG_BOSCH_DIRECT_BMI270)

	case Variant::kBmi270: {
			add(tagged::PowerConfig,   0,    0x03);
			add(tagged::PowerControl,  0x0e, 0);    // Accel, gyro, temperature.
			value(tagged::AccelConfig, 0xac);       // 1600 Hz, original normal bandwidth.
			value(tagged::GyroConfig,  0xec);       // 1600 Hz, original performance settings.
			value(tagged::AccelRange,  0x03, 0x03); // 16 g.
			value(tagged::GyroRange,   0,    0x07); // Verify the 2000 dps scale explicitly.
			add(tagged::FifoConfig0,   0x03, 0);
			add(tagged::FifoConfig1, tagged::FifoHeader | tagged::FifoAccel | tagged::FifoGyro, 0);
			value(tagged::Int1Io, 0x08, 0x1e); // INT1 output, active-low push-pull.
			add(tagged::IntMap,   0x02, 0);    // INT1 watermark.
			break;
		}

#endif // CONFIG_BOSCH_DIRECT_BMI270

	default: {
			return 0;
		}
	}

	if (bosch_direct_fifo::tagged(profile.format)) {
		const uint8_t high_mask = profile.variant == Variant::kBmi270 ? 0x1f : 0x03;

		if (watermark == 0 || watermark >= profile.fifo_capacity_bytes) {
			return 0;
		}

		value(tagged::WatermarkLow,  watermark & 0xff);
		value(tagged::WatermarkHigh, watermark >> 8,   high_mask);

	} else {
		if (watermark == 0 || watermark > 32) {
			return 0;
		}

		value(profile.watermark_reg, watermark);
		add(fixed::FifoConfig, fixed::FifoMode, 0);
	}

	return valid ? count : 0;
}

} // namespace bosch_direct_model
