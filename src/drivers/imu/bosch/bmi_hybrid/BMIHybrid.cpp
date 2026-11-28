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

/**
 * @file BMIHybrid.cpp
 * @brief Implementation of the BMIHybrid Bosch SPI IMU family driver.
 */

#include "BMIHybrid.hpp"
#include "BMIHybridRegisters.hpp"

#if defined(CONFIG_BMI_HYBRID_BMI270)
#include "BMI270Config.hpp"
#endif // CONFIG_BMI_HYBRID_BMI270

#include <lib/mathlib/mathlib.h>

#include <cmath>
#include <cstring>
#if !defined(__PX4_NUTTX)
#include <new>
#endif // !__PX4_NUTTX

using namespace time_literals;
using namespace bmi_hybrid_registers;

namespace
{
template<bool condition, typename TrueType, typename FalseType>
struct Conditional {
	using Type = TrueType;
};

template<typename TrueType, typename FalseType>
struct Conditional<false, TrueType, FalseType> {
	using Type = FalseType;
};

template<bool condition, typename TrueType, typename FalseType>
using ConditionalType = typename Conditional<condition, TrueType, FalseType>::Type;

template<typename Type, typename... Args>
Type *allocateObject(const Args &... args)
{
#if defined(__PX4_NUTTX)
	return new Type(args...);
#else
	return new (std::nothrow) Type(args...);
#endif // __PX4_NUTTX
}

} // namespace

BMIHybrid::BMIHybrid(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_endpoint_guard(config),
	_profile(*static_cast<const Profile *>(config.custom_data)),
	_spi_mode(config.spi_mode),
	_register_frequency(imu::spiConfigFrequency(_profile.device, config.custom2)),
	_data_frequency(imu::spiDataFrequency(_profile.device, config.custom2)),
	_drdy_gpio(config.drdy_gpio),
	_sample_dt_us(1e6f / _profile.rate_hz),
	_px4_accel(_profile.device.component != 'G'
		   ? allocateObject<PX4Accelerometer>(get_device_id(), config.rotation, config.external)
		   : nullptr),
	_px4_gyro(_profile.device.component != 'A'
		  ? allocateObject<PX4Gyroscope>(get_device_id(), config.rotation, config.external)
		  : nullptr)
{
	static_assert(kMaxSamples == sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0]),
		      "FIFO publisher capacities must match");

	// A split device owns only its registered channel; an integrated device owns both publishers.
	if (_px4_accel) {
		_px4_accel->set_range(_profile.accel_range);
		_px4_accel->set_scale(_profile.accel_scale);
	}

	if (_px4_gyro) {
		_px4_gyro->set_range(_profile.gyro_range);
		_px4_gyro->set_scale(_profile.gyro_scale);
	}

	configureSampleRate();
}

BMIHybrid::~BMIHybrid()
{
	disableInterrupt();
	delete _px4_gyro;
	delete _px4_accel;
}

int BMIHybrid::init()
{
	if (!_endpoint_guard.acquire()) {
		return PX4_ERROR;
	}

	if (!_dma_buffer
	    || (_profile.device.component != 'G' && !_px4_accel)
	    || (_profile.device.component != 'A' && !_px4_gyro)
	    || _register_count == 0
	    || _interval_us == 0) {
		return PX4_ERROR;
	}

	const int result = SPI::init();

	if (result != PX4_OK) {
		return result;
	}

	reset();

	return PX4_OK;
}

int BMIHybrid::probe()
{
	for (unsigned attempt = 0; attempt < 3; ++attempt) {
		_transfer_failed = false;

		if (identity()) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

bool BMIHybrid::identity()
{
	// These endpoints enter SPI mode after the first CS edge. The dummy read's
	// content is discarded, but an actual bus transfer failure is not ignored.
	if (_profile.device.register_dummy_bytes) {
		readRegister(ChipId);

		if (_profile.variant == Variant::kBmi270) {
			// BMI270 DS section 6.3: SPI selection takes 200 us after the first CS rising edge.
			// This startup delay is distinct from the ordinary 2 us inter-transfer idle time.
			px4_udelay(200_us);
		}
	}

	const uint8_t whoami = readRegister(ChipId);

	if (_profile.variant == Variant::kBmi08xAccel && !_transfer_failed) {
		// BMI085/088 require 5 ms normal-mode startup; BMI090L shares the BMI088 endpoint with 50 ms startup.
		constexpr uint8_t kBmi090LChipId { 0x1a };

		_accel_startup_us = whoami == kBmi090LChipId ? 50_ms : 5_ms;
	}

	return !_transfer_failed && (whoami == _profile.whoami || whoami == _profile.alternate_whoami);
}

void BMIHybrid::configureSampleRate()
{
	const int requested_rate = _px4_gyro ? _px4_gyro->get_max_rate_hz() : _px4_accel ? _px4_accel->get_max_rate_hz() : 0;

	if (requested_rate <= 0) {
		return;
	}

	_watermark_samples = static_cast<uint8_t>(math::constrain(roundf((1e6f / requested_rate) / _sample_dt_us),
			     1.f, static_cast<float>(kMaxSamples)));
	_interval_us = static_cast<uint32_t>(roundf(_watermark_samples * _sample_dt_us));

	// Fixed FIFOs count samples; tagged FIFOs count bytes, including each frame's header.
	const uint16_t watermark = bmi_hybrid_fifo::tagged(_profile.format)
				   ? _watermark_samples * bmi_hybrid_fifo::frameBytes(_profile.format) : _watermark_samples;
	_register_count = bmi_hybrid_model::loadRegisters(_profile, watermark, _registers);
}

void BMIHybrid::reset()
{
	disableInterrupt();
	_config_check_failure_count = 0;
	_drdy_backlog_drain         = false;
	ScheduleClear();
	_state = State::kReset;
	ScheduleNow();
}

void BMIHybrid::exit_and_cleanup()
{
	disableInterrupt();
	I2CSPIDriverBase::exit_and_cleanup();
}

void BMIHybrid::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("model: %s, endpoint: %s",
		 _profile.device.name,
		 _profile.device.component == 'A' ? "accel" : _profile.device.component == 'G' ? "gyro" : "IMU");

	imu::printSpiStatus(_profile.device,
			    _spi_mode,
			    _register_frequency,
			    _data_frequency,
			    get_frequency());

	PX4_INFO("FIFO interval: %u us, watermark: %u samples, ODR: %u Hz",
		 unsigned(_interval_us),
		 unsigned(_watermark_samples),
		 unsigned(_profile.rate_hz));

	_transfer_perf.print();
	_fifo_perf.print();
	_drdy_missed_perf.print();
}

bool BMIHybrid::transferChecked(uint8_t *tx, uint8_t *rx, size_t size)
{
	if (_transfer_failed) {
		return false;
	}

	if (!tx || size == 0) {
		_transfer_failed = true;
		_transfer_perf.bad_transfer.count();

		return false;
	}

	// Save the command before an in-place read overwrites the TX bytes.
	const bool is_write        = !(tx[0] & Read);
	const bool suspended_write = is_write && (_state == State::kReset || _state == State::kWaitReset);

	if (_next_transfer != 0) {
		const hrt_abstime now = hrt_absolute_time();

		if (now < _next_transfer) {
			px4_udelay(static_cast<unsigned>(math::min(_next_transfer - now, 1_ms)));
		}

		_next_transfer = 0;
	}

	const int result = device::SPI::transfer(tx, rx, size);

	if (_profile.variant == Variant::kBmi270) {
		// BMI270 DS section 6.4: 2 us after reads/normal writes, 450 us after
		// writes before advanced power save is disabled. Scheduled startup
		// waits normally satisfy the long gap without busy-waiting the queue.
		_next_transfer = hrt_absolute_time() + (suspended_write ? 450_us : 2_us);

	} else if (is_write && (_profile.variant == Variant::kBmi08xAccel || _profile.variant == Variant::kBmi08xGyro)) {
		// BMI08x serial-interface synchronization: 2 us after normal writes;
		// use the stricter BMI088 1000 us requirement during suspend/startup.
		// FIFO reads do not add a delay or a second timestamp read to the hot path.
		_next_transfer = hrt_absolute_time() + (suspended_write ? 1_ms : 2_us);

	} else if (is_write) {
		// BMI055 DS Table 30 / section 9.2.1: >= 2 us after normal-mode writes, >= 450 us in suspend.
		_next_transfer = hrt_absolute_time() + (suspended_write ? 450_us : 2_us);
	}

	if (result != PX4_OK) {
		_transfer_failed = true;
		_transfer_perf.bad_transfer.count();

		return false;
	}

	return true;
}

bool BMIHybrid::readRegisters(uint8_t reg, uint8_t *data, size_t count)
{
	uint8_t     *buffer = _dma_buffer.data();
	const size_t prefix = 1 + _profile.device.register_dummy_bytes;

	if (!data
	    || count == 0
	    || prefix + count > _dma_buffer.capacity()) {
		return false;
	}

	memset(buffer, 0, prefix + count);
	buffer[0] = reg | Read;
	set_frequency(_register_frequency);

	if (!transferChecked(buffer, buffer, prefix + count)) {
		return false;
	}

	memcpy(data, buffer + prefix, count);

	return true;
}

uint8_t BMIHybrid::readRegister(uint8_t reg)
{
	uint8_t value = 0;

	readRegisters(reg, &value, 1);

	return value;
}

bool BMIHybrid::writeRegister(uint8_t reg, uint8_t value)
{
	uint8_t *buffer = _dma_buffer.data();
	buffer[0] = reg;
	buffer[1] = value;

	set_frequency(_register_frequency);

	return transferChecked(buffer, nullptr, 2);
}

bool BMIHybrid::modifyRegister(const RegisterConfig &config)
{
	const uint8_t original = readRegister(config.reg);

	if (_transfer_failed) {
		return false;
	}

	const uint8_t value = (original & ~config.clear_bits) | config.set_bits;

	return value == original || writeRegister(config.reg, value);
}

bool BMIHybrid::checkRegister(const RegisterConfig &config)
{
	const uint8_t value = readRegister(config.reg);

	return !_transfer_failed && (value & config.set_bits) == config.set_bits && (value & config.clear_bits) == 0;
}

bool BMIHybrid::configure()
{
	for (unsigned i = 0; i < _register_count; ++i) {
		if (_profile.variant == Variant::kBmi08xAccel
		    && (_registers[i].reg == tagged::PowerConfig || _registers[i].reg == tagged::PowerControl)) {
			// These writes and their no-access delays belong to the startup states.
			// Fail before further writes if the power transition was not accepted;
			// never silently re-enable a suspended sensor here.
			if (!checkRegister(_registers[i])) {
				return false;
			}

			continue;
		}

		if (!modifyRegister(_registers[i])) {
			return false;
		}
	}

	// Verify the complete register set before allowing acquisition to start.
	for (unsigned i = 0; i < _register_count; ++i) {
		if (!checkRegister(_registers[i])) {
			if (!_transfer_failed) {
				_transfer_perf.bad_register.count();
			}

			return false;
		}
	}

	return true;
}

void BMIHybrid::RunImpl()
{
	_transfer_failed = false;

	// Fetch the ISR timestamp before sampling hrt_absolute_time() so preemption cannot invert timestamp <= now.
	const hrt_abstime drdy_timestamp = (_state == State::kRead && _data_ready_interrupt_enabled)
					   ? _drdy_timestamp_sample.fetch_and(0)
					   : 0;
	const hrt_abstime now = hrt_absolute_time();

	// Steady-state sampling bypasses initialization dispatch; recovery stays in the same work-queue pass.
	if (__builtin_expect(_state == State::kRead, 1)) {
		hrt_abstime interrupt_time = 0;
		const bool  backlog_drain  = _drdy_backlog_drain;
		_drdy_backlog_drain        = false;

		if (_data_ready_interrupt_enabled) {
			if (drdy_timestamp != 0
			    && drdy_timestamp <= now
			    && now - drdy_timestamp < _interval_us) {
				interrupt_time = drdy_timestamp;

			} else if (!backlog_drain) {
				_drdy_missed_perf.count();
			}

			// The delayed read is a watchdog when interrupts stop arriving.
			ScheduleDelayed(2 * _interval_us);
		}

		const bool success = readFifo(now, interrupt_time);

		// A tagged configuration-change frame can request recovery from inside the decoder path.
		if (_state != State::kRead) {
			return;
		}

		if (success) {
			if (_failure_count) {
				--_failure_count;
			}

		} else if (++_failure_count > 10 || _transfer_failed) {
			reset();

			return;
		}

		// Spread periodic readback across passes instead of checking the entire table on the hot path.
		if (!success || now - _last_config_check_timestamp > 100_ms) {
			_transfer_failed = false;

#if defined(CONFIG_BMI_HYBRID_BMI270)

			if (_profile.variant == Variant::kBmi270) {
				const uint8_t err = readRegister(tagged::Error);

				if (!_transfer_failed) {
					if (err & tagged::FatalOrInternalError) {
						_transfer_perf.bad_register.count();
						reset();

						return;
					}

					if (err & tagged::FifoError) {
						_fifo_perf.overflow.count();

						if (!fifoReset()) {
							reset();

							return;
						}
					}
				}
			}

#endif // CONFIG_BMI_HYBRID_BMI270

			if (!checkRegister(_registers[_checked_register])) {
				if (!_transfer_failed) {
					_transfer_perf.bad_register.count();
					reset();

				} else if (++_config_check_failure_count > 10) {
					reset();
				}

				return;
			}

			_config_check_failure_count  = 0;
			_checked_register            = (_checked_register + 1) % _register_count;
			_last_config_check_timestamp = now;
		}

		if (!_transfer_failed && now - _temperature_update_timestamp >= 1_s) {
			updateTemperature();

			if (!_transfer_failed) {
				_temperature_update_timestamp = now;
			}
		}

		return;
	}

	switch (_state) {
	case State::kReset: {
			disableInterrupt();
			_reset_timestamp            = now;
			_failure_count              = 0;
			_config_check_failure_count = 0;
			_checked_register           = 0;

			if (!writeRegister(bmi_hybrid_model::resetReg(_profile.format), SoftReset)) {
				ScheduleDelayed(100_ms);
				break;
			}

			_state = State::kWaitReset;
			ScheduleDelayed(_profile.reset_wait_us);
			break;
		}

	case State::kWaitReset: {
			if (identity()) {
				if (bmi_hybrid_fifo::tagged(_profile.format) && !writeRegister(tagged::PowerConfig, 0)) {
					reset();
					break;
				}

				if (_profile.variant == Variant::kBmi270) {
					_state = State::kLoadConfig;

				} else if (_profile.variant == Variant::kBmi08xAccel) {
					_state = State::kEnableAccel;

				} else {
					_state = State::kConfigure;
				}

				ScheduleDelayed(_profile.configure_wait_us);

			} else if (now - _reset_timestamp > 1_s) {
				reset();

			} else {
				ScheduleDelayed(10_ms);
			}

			break;
		}

	case State::kEnableAccel: {
			constexpr uint8_t kAccelEnable { 1u << 2 };

			if (writeRegister(tagged::PowerControl, kAccelEnable)) {
				// BMI085/088 need 5 ms to settle into normal mode; BMI090L requires 50 ms.
				_state = State::kConfigure;
				ScheduleDelayed(_accel_startup_us);

			} else {
				reset();
			}

			break;
		}

	case State::kLoadConfig: {
#if defined(CONFIG_BMI_HYBRID_BMI270)

			// Stage the Flash config image in cache-aligned DMA memory before transmitting.
			if (bmi_hybrid_model::kBmi270ConfigSize <= _dma_buffer.capacity()
			    && writeRegister(tagged::InitControl, 0)) {
				uint8_t *config_tx = _dma_buffer.data();
				memcpy(config_tx, bmi_hybrid_model::bmi270ConfigData(), bmi_hybrid_model::kBmi270ConfigSize);

				if (transferChecked(config_tx, nullptr, bmi_hybrid_model::kBmi270ConfigSize)
				    && writeRegister(tagged::InitControl, 1)) {
					_state = State::kWaitConfig;
					ScheduleDelayed(150_ms);

				} else {
					reset();
				}

			} else {
				reset();
			}

#else
			reset();
#endif // CONFIG_BMI_HYBRID_BMI270
			break;
		}

	case State::kWaitConfig: {
#if defined(CONFIG_BMI_HYBRID_BMI270)

			if ((readRegister(tagged::InternalStatus) & tagged::InternalStatusMask) == tagged::ConfigLoaded
			    && !_transfer_failed) {
				_state = State::kConfigure;
				ScheduleNow();

			} else {
				if (!_transfer_failed) {
					_transfer_perf.bad_register.count();
				}

				reset();
			}

#else
			reset();
#endif // CONFIG_BMI_HYBRID_BMI270
			break;
		}

	case State::kConfigure: {
			if (configure()) {
				_state = State::kFifoReset;
				// BMI270 gyroscope normal-mode startup takes ~45 ms before FIFO flush and acquisition.
				ScheduleDelayed(_profile.variant == Variant::kBmi270 ? 50_ms : 10_ms);

			} else if (now - _reset_timestamp > 1_s) {
				reset();

			} else {
				ScheduleDelayed(100_ms);
			}

			break;
		}

	case State::kFifoReset: {
			if (!fifoReset()) {
				reset();
				break;
			}

			_state                        = State::kRead;
			_last_config_check_timestamp  = now;
			_temperature_update_timestamp = 0;

			// Arm the initial watchdog before enabling GPIO IRQ so a fast edge's ScheduleNow() is preserved.
			if (_drdy_gpio != 0 && _profile.interrupt_enabled) {
				ScheduleDelayed(100_ms);
			}

			_data_ready_interrupt_enabled = enableInterrupt();

			if (!_data_ready_interrupt_enabled) {
				ScheduleOnInterval(_interval_us, _interval_us);
			}

			break;
		}

	case State::kRead: {
			break; // Handled by the acquisition fast path above.
		}
	}
}

bool BMIHybrid::readFifo(hrt_abstime now, hrt_abstime interrupt_time)
{
	return bmi_hybrid_fifo::tagged(_profile.format) ? readTagged(now, interrupt_time) : readFixed(now, interrupt_time);
}

hrt_abstime BMIHybrid::sampleTime(hrt_abstime now, hrt_abstime interrupt_time, uint8_t samples) const
{
	if (interrupt_time == 0) {
		return now;
	}

	// The watermark IRQ timestamps its triggering sample; account for samples accumulated before the read.
	if (samples > _watermark_samples) {
		return math::min(now, interrupt_time + hrt_abstime((samples - _watermark_samples) * _sample_dt_us));
	}

	return interrupt_time;
}

bool BMIHybrid::readFixed(hrt_abstime now, hrt_abstime interrupt_time)
{
	const uint8_t status = readRegister(bmi_hybrid_model::countReg(_profile.format));

	if (_transfer_failed) {
		return false;
	}

	uint8_t samples = status & fixed::CountMask;

	if ((status & fixed::Overrun)
	    || samples > kMaxSamples
	    || samples * bmi_hybrid_fifo::kAxisBytes > _profile.fifo_capacity_bytes) {
		_fifo_perf.overflow.count();
		fifoReset();

		return false;
	}

	if (samples == 0) {
		_fifo_perf.empty.count();

		return false;
	}

	// Preserve the original fixed-FIFO one-sample jitter tolerance.
	bool trimmed_jitter_sample = false;

	if (samples == _watermark_samples + 1) {
		--samples;
		trimmed_jitter_sample = true;
	}

	const size_t bytes = samples * bmi_hybrid_fifo::kAxisBytes;
	const size_t size  = _profile.device.data_prefix_bytes + bytes;

	if (size > _dma_buffer.capacity() || size > _profile.device.max_transfer_bytes) {
		return false;
	}

	uint8_t *buffer = _dma_buffer.data();
	memset(buffer, 0, size);
	buffer[0] = bmi_hybrid_model::fifoReg(_profile.format) | Read;
	set_frequency(_data_frequency);

	if (!transferChecked(buffer, buffer, size)) {
		return false;
	}

	hrt_abstime post_now = math::max(now, hrt_absolute_time());

	if (trimmed_jitter_sample && interrupt_time == 0) {
		const hrt_abstime dt = static_cast<hrt_abstime>(_sample_dt_us);

		if (post_now <= dt) {
			return false;
		}

		post_now -= dt;
	}

	const uint8_t    *data      = buffer + _profile.device.data_prefix_bytes;
	const hrt_abstime timestamp = sampleTime(post_now, interrupt_time, samples);

	if (_profile.format == Format::kFixed12Accel) {
		sensor_accel_fifo_s batch {};

		if (!bmi_hybrid_fifo::decodeFixed<4>(data, bytes, false, batch)) {
			_transfer_perf.bad_transfer.count();
			fifoReset();

			return false;
		}

		batch.timestamp_sample = timestamp;
		batch.dt               = _sample_dt_us;
		_px4_accel->set_error_count(errorCount());
		_px4_accel->updateFIFO(batch);

		return true;
	}

	sensor_gyro_fifo_s batch {};

	if (!bmi_hybrid_fifo::decodeFixed<0>(data, bytes, _profile.reject_all_minimum, batch)) {
		_transfer_perf.bad_transfer.count();
		fifoReset();

		return false;
	}

	if (batch.samples == 0) {
		_fifo_perf.empty.count();

		return false;
	}

	batch.timestamp_sample = timestamp;
	batch.dt               = _sample_dt_us;
	_px4_gyro->set_error_count(errorCount());
	_px4_gyro->updateFIFO(batch);

	return true;
}

bool BMIHybrid::readTagged(hrt_abstime now, hrt_abstime interrupt_time)
{
	uint8_t count_bytes[2] {};

	if (!readRegisters(bmi_hybrid_model::countReg(_profile.format), count_bytes, sizeof(count_bytes))) {
		return false;
	}

	const uint16_t bytes       = uint16_t(count_bytes[0]) | (uint16_t(count_bytes[1] & 0x3f) << 8);
	const uint8_t  frame_bytes = bmi_hybrid_fifo::frameBytes(_profile.format);

	if (bytes == 0) {
		_fifo_perf.empty.count();

		return false;
	}

	if (bytes + frame_bytes > _profile.fifo_capacity_bytes) {
		_fifo_perf.overflow.count();
		fifoReset();

		return false;
	}

	const uint16_t max_payload_bytes = static_cast<uint16_t>(kMaxSamples) * frame_bytes;
	const bool     clamped           = bytes > max_payload_bytes;
	const uint16_t read_bytes        = clamped ? max_payload_bytes : bytes;
	const size_t   size              = _profile.device.data_prefix_bytes + read_bytes;

	if (size > _dma_buffer.capacity() || size > _profile.device.max_transfer_bytes) {
		_fifo_perf.overflow.count();
		fifoReset();

		return false;
	}

	uint8_t *buffer = _dma_buffer.data();
	memset(buffer, 0, size);
	buffer[0] = bmi_hybrid_model::fifoReg(_profile.format) | Read;
	set_frequency(_data_frequency);

	if (!transferChecked(buffer, buffer, size)) {
		return false;
	}

	const hrt_abstime post_now = math::max(now, hrt_absolute_time());
	uint16_t total_bytes       = bytes;

	// BMI08x accel transfers include a fresh count after the SPI command and dummy byte.
	if (_profile.device.data_prefix_bytes == 4) {
		const uint16_t embedded_count = uint16_t(buffer[2]) | (uint16_t(buffer[3] & 0x3f) << 8);

		if (embedded_count < bytes || embedded_count + frame_bytes > _profile.fifo_capacity_bytes) {
			_fifo_perf.overflow.count();
			fifoReset();

			return false;
		}

		total_bytes = embedded_count;
	}

	const uint8_t *data = buffer + _profile.device.data_prefix_bytes;

#if defined(CONFIG_BMI_HYBRID_BMI270)

	if (_profile.format == Format::kTaggedImu) {
		return processTagged<true>(data, read_bytes, total_bytes, clamped, post_now, interrupt_time);
	}

#endif // CONFIG_BMI_HYBRID_BMI270

	return processTagged<false>(data, read_bytes, total_bytes, clamped, post_now, interrupt_time);
}

template<bool integrated>
bool BMIHybrid::processTagged(
	const uint8_t *data,
	size_t size,
	uint16_t total_bytes,
	bool clamped,
	hrt_abstime post_now,
	hrt_abstime interrupt_time)
{
	sensor_accel_fifo_s accel {};
	ConditionalType<integrated, sensor_gyro_fifo_s, imu::SampleBatch<1>> gyro {};

	bmi_hybrid_fifo::Sample sample {};
	const auto decode_frame = [clamped](imu::ByteCursor & cursor, bmi_hybrid_fifo::Sample & decoded) {
		return bmi_hybrid_fifo::readTagged<integrated>(cursor, decoded, clamped);
	};
	constexpr imu::FifoChannels channels = integrated ? imu::FifoChannels::kBoth : imu::FifoChannels::kAccel;
	const imu::FifoDecodeResult result = imu::decodeFifoFrames<imu::FifoAxisMapping::kFlipYZ, channels>(
			data, size, accel, gyro, sample, decode_frame);

	if (result.status == imu::FifoDecodeStatus::kReconfigure) {
		_transfer_perf.bad_register.count();
		reset();

		return false;
	}

	if (result.status == imu::FifoDecodeStatus::kOverflow) {
		_fifo_perf.overflow.count();
		fifoReset();

		return false;
	}

	if (result.status == imu::FifoDecodeStatus::kInvalid) {
		_transfer_perf.bad_transfer.count();
		fifoReset();

		return false;
	}

	if (result.status == imu::FifoDecodeStatus::kEmpty) {
		_fifo_perf.empty.count();

		return false;
	}

	// Control frames consume FIFO watermark bytes without representing samples; fall back to post-SPI time.
	// Clamped reads retransmit any partial trailing frame on the next FIFO read, so measure unread bytes from consumed_bytes.
	const hrt_abstime irq           = result.has_metadata ? 0 : interrupt_time;
	const uint16_t    unread_bytes  = total_bytes > result.consumed_bytes
					  ? static_cast<uint16_t>(total_bytes - result.consumed_bytes)
					  : 0;
	const uint16_t    unread_frames = unread_bytes / bmi_hybrid_fifo::frameBytes(_profile.format);
	const uint64_t    errors        = errorCount();

	const auto resolve_timestamp = [&](uint8_t batch_samples) {
		if (irq != 0) {
			return sampleTime(post_now, irq, batch_samples);
		}

		return imu::adjustFifoBacklogTimestamp(
			       post_now,
			       false,
			       static_cast<uint16_t>(batch_samples + unread_frames),
			       batch_samples,
			       _sample_dt_us);
	};

	_px4_accel->set_error_count(errors);

	if constexpr(integrated) {
		_px4_gyro->set_error_count(errors);

		if (gyro.samples) {
			gyro.timestamp_sample = resolve_timestamp(gyro.samples);
			gyro.dt               = _sample_dt_us;
			_px4_gyro->updateFIFO(gyro);
		}
	}

	if (accel.samples) {
		accel.timestamp_sample = resolve_timestamp(accel.samples);
		accel.dt               = _sample_dt_us;
		_px4_accel->updateFIFO(accel);
	}

	if (unread_frames >= _watermark_samples) {
		_drdy_timestamp_sample.store(0);
		_drdy_backlog_drain = true;
		ScheduleNow();
	}

	return true;
}

bool BMIHybrid::fifoReset()
{
	_fifo_perf.reset.count();
	_drdy_backlog_drain = false;

	if (bmi_hybrid_fifo::tagged(_profile.format)) {
		const bool ok = writeRegister(bmi_hybrid_model::resetReg(_profile.format), FifoFlush);
		_drdy_timestamp_sample.store(0);

		return ok;
	}

	const uint8_t wm_reg = bmi_hybrid_model::watermarkReg(_profile.format);

	// Fixed FIFOs are cleared through bypass mode, then recover their configured watermark and mode.
	if (!writeRegister(wm_reg, 0) || !writeRegister(fixed::FifoConfig, 0)) {
		_drdy_timestamp_sample.store(0);

		return false;
	}

	for (unsigned i = 0; i < _register_count; ++i) {
		const auto &config = _registers[i];

		if ((config.reg == wm_reg || config.reg == fixed::FifoConfig) && !modifyRegister(config)) {
			_drdy_timestamp_sample.store(0);

			return false;
		}
	}

	_drdy_timestamp_sample.store(0);

	return true;
}

void BMIHybrid::updateTemperature()
{
	using Temperature = bmi_hybrid_model::Temperature;

	if (_transfer_failed || _profile.temperature == Temperature::kNone) {
		return;
	}

	uint8_t data[2] {};
	const size_t size = _profile.temperature == Temperature::kSigned8 ? 1 : 2;

	if (!readRegisters(bmi_hybrid_model::temperatureReg(_profile.temperature), data, size)) {
		return;
	}

	float temperature = NAN;

	switch (_profile.temperature) {
	case Temperature::kSigned8: {
			temperature = static_cast<int8_t>(data[0]) * 0.5f + 23.f;
			break;
		}

	case Temperature::kBmi08x11: {
			if (!bmi_hybrid_fifo::temperature08x(data, temperature)) {
				temperature = NAN;
			}

			break;
		}

#if defined(CONFIG_BMI_HYBRID_BMI270)

	case Temperature::kBmi27016: {
			if (!bmi_hybrid_fifo::temperature270(data, temperature)) {
				temperature = NAN;
			}

			break;
		}

#endif // CONFIG_BMI_HYBRID_BMI270

	default: {
			return;
		}
	}

	if (_px4_accel) {
		_px4_accel->set_temperature(temperature);
	}

	if (_px4_gyro) {
		_px4_gyro->set_temperature(temperature);
	}
}

uint64_t BMIHybrid::errorCount() const
{
	return _transfer_perf.bad_register.eventCount() + _transfer_perf.bad_transfer.eventCount()
	       + _fifo_perf.empty.eventCount() + _fifo_perf.overflow.eventCount();
}

int BMIHybrid::dataReadyCallback(int irq, void *context, void *arg)
{
	auto *driver = static_cast<BMIHybrid *>(arg);
	driver->_drdy_timestamp_sample.store(hrt_absolute_time());
	driver->ScheduleNow();

	return 0;
}

bool BMIHybrid::enableInterrupt()
{
	return _drdy_gpio != 0 && _profile.interrupt_enabled
	       && px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &dataReadyCallback, this) == 0;
}

void BMIHybrid::disableInterrupt()
{
	if (_data_ready_interrupt_enabled) {
		px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr);
	}

	_data_ready_interrupt_enabled = false;
	_drdy_timestamp_sample.store(0);
}
