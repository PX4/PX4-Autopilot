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

#include "BoschDirect.hpp"
#include "BoschDirectRegisters.hpp"

#if defined(CONFIG_BOSCH_DIRECT_BMI270)
#include "Bmi270Config.hpp"
#endif // CONFIG_BOSCH_DIRECT_BMI270

#include <lib/mathlib/mathlib.h>
#include <cstring>

using namespace time_literals;
using namespace bosch_direct_registers;

BoschDirect::BoschDirect(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_profile(*static_cast<const Profile *>(config.custom_data)),
	_register_frequency(imu::spiConfigFrequency(_profile.device, config.custom2)),
	_data_frequency(imu::spiDataFrequency(_profile.device, config.custom2)),
	_drdy_gpio(config.drdy_gpio),
	_sample_dt(1e6f / _profile.rate_hz),
	_accel(_profile.device.component != 'G'
	       ? new PX4Accelerometer(get_device_id(),
				      config.rotation,
				      config.external)
	       : nullptr),
	_gyro(_profile.device.component != 'A'
	      ? new PX4Gyroscope(get_device_id(),
				 config.rotation,
				 config.external)
	      : nullptr),
	_buffer(new uint8_t[_profile.device.max_transfer_bytes] {})
{
	static_assert(kMaxSamples == sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0]),
		      "FIFO publisher capacities must match");

	// A split device owns only its registered channel; an integrated device owns both publishers.
	if (_accel) {
		_accel->set_range(_profile.accel_range);
		_accel->set_scale(_profile.accel_scale);
	}

	if (_gyro) {
		_gyro->set_range(_profile.gyro_range);
		_gyro->set_scale(_profile.gyro_scale);
	}

	configureSampleRate();
}

BoschDirect::~BoschDirect()
{
	disableInterrupt();
	delete[] _buffer;
	delete _gyro;
	delete _accel;
}

int BoschDirect::init()
{
	if (!_buffer
	    || (_profile.device.component != 'G' && !_accel)
	    || (_profile.device.component != 'A' && !_gyro)
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

int BoschDirect::probe()
{
	for (unsigned attempt = 0; attempt < 3; ++attempt) {
		_transfer_failed = false;

		if (identity()) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

bool BoschDirect::identity()
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
		// BMI090L shares the BMI088 board endpoint, but its startup is longer.
		// Keep the registered device type unchanged; retain only the detected timing.
		constexpr uint8_t kBmi090LChipId { 0x1a };

		_accel_startup_us = whoami == kBmi090LChipId ? 50_ms : 1_ms;
	}

	return !_transfer_failed && (whoami == _profile.whoami || whoami == _profile.alternate_whoami);
}

void BoschDirect::configureSampleRate()
{
	const int requested_rate = _gyro ? _gyro->get_max_rate_hz() : _accel ? _accel->get_max_rate_hz() : 0;

	if (requested_rate <= 0) {
		return;
	}

	_watermark_samples = static_cast<uint8_t>(math::constrain(roundf((1e6f / requested_rate) / _sample_dt),
			     1.f, static_cast<float>(kMaxSamples)));
	_interval_us = static_cast<uint32_t>(roundf(_watermark_samples * _sample_dt));

	// Fixed FIFOs count samples; tagged FIFOs count bytes, including each frame's header.
	const uint16_t watermark = bosch_direct_fifo::tagged(_profile.format)
				   ? _watermark_samples * bosch_direct_fifo::frameBytes(_profile.format) : _watermark_samples;
	_register_count = bosch_direct_model::loadRegisters(_profile, watermark, _registers);
}

void BoschDirect::reset()
{
	disableInterrupt();
	ScheduleClear();
	_state = State::kReset;
	ScheduleNow();
}

void BoschDirect::exit_and_cleanup()
{
	disableInterrupt();
	I2CSPIDriverBase::exit_and_cleanup();
}

void BoschDirect::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("model: %s, endpoint: %s",
		 _profile.device.name,
		 _profile.device.component == 'A' ? "accel" : _profile.device.component == 'G' ? "gyro" : "IMU");

	imu::printSpiStatus(_profile.device,
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

bool BoschDirect::transferChecked(const uint8_t *tx, uint8_t *rx, size_t size)
{
	if (_transfer_failed) {
		return false;
	}

	if (!tx || size == 0) {
		_transfer_failed = true;
		_transfer_perf.bad_transfer.count();

		return false;
	}

#if defined(CONFIG_BOSCH_DIRECT_BMI270) \
	|| defined(CONFIG_BOSCH_DIRECT_BMI085) \
	|| defined(CONFIG_BOSCH_DIRECT_BMI088)

	// Save the command before an in-place read overwrites the TX bytes.
	const bool write           = !(tx[0] & Read);
	const bool suspended_write = write && (_state == State::kReset || _state == State::kWaitReset);

	if (_next_transfer != 0) {
		const hrt_abstime now = hrt_absolute_time();

		if (now < _next_transfer) {
			px4_udelay(static_cast<unsigned>(math::min(_next_transfer - now, 1_ms)));
		}

		_next_transfer = 0;
	}

#endif // BMI08x or BMI270 register-write timing

	// The legacy SPI signature is mutable; the backend consumes TX as const.
	const int result = device::SPI::transfer(const_cast<uint8_t *>(tx), rx, size);

#if defined(CONFIG_BOSCH_DIRECT_BMI270) \
	|| defined(CONFIG_BOSCH_DIRECT_BMI085) \
	|| defined(CONFIG_BOSCH_DIRECT_BMI088)

	if (_profile.variant == Variant::kBmi270) {
		// BMI270 DS section 6.4: 2 us after reads/normal writes, 450 us after
		// writes before advanced power save is disabled. Scheduled startup
		// waits normally satisfy the long gap without busy-waiting the queue.
		_next_transfer = hrt_absolute_time() + (suspended_write ? 450_us : 2_us);

	} else if (write && (_profile.variant == Variant::kBmi08xAccel || _profile.variant == Variant::kBmi08xGyro)) {
		// BMI08x serial-interface synchronization: 2 us after normal writes;
		// use the stricter BMI088 1000 us requirement during suspend/startup.
		// FIFO reads do not add a delay or a second timestamp read to the hot path.
		_next_transfer = hrt_absolute_time() + (suspended_write ? 1_ms : 2_us);
	}

#endif // BMI08x or BMI270 register-write timing

	if (result != PX4_OK) {
		_transfer_failed = true;
		_transfer_perf.bad_transfer.count();

		return false;
	}

	return true;
}

bool BoschDirect::readRegisters(uint8_t reg, uint8_t *data, size_t count)
{
	uint8_t buffer[4] {};
	const size_t prefix = 1 + _profile.device.register_dummy_bytes;

	if (!data
	    || count == 0
	    || prefix + count > sizeof(buffer)) {
		return false;
	}

	buffer[0] = reg | Read;
	set_frequency(_register_frequency);

	if (!transferChecked(buffer, buffer, prefix + count)) {
		return false;
	}

	memcpy(data, buffer + prefix, count);

	return true;
}

uint8_t BoschDirect::readRegister(uint8_t reg)
{
	uint8_t value = 0;

	readRegisters(reg, &value, 1);

	return value;
}

bool BoschDirect::writeRegister(uint8_t reg, uint8_t value)
{
	uint8_t buffer[2] { reg, value };

	set_frequency(_register_frequency);

	return transferChecked(buffer, nullptr, sizeof(buffer));
}

bool BoschDirect::modifyRegister(const RegisterConfig &cfg)
{
	const uint8_t original = readRegister(cfg.reg);

	if (_transfer_failed) {
		return false;
	}

	const uint8_t value = (original & ~cfg.clear_bits) | cfg.set_bits;

	return value == original || writeRegister(cfg.reg, value);
}

bool BoschDirect::checkRegister(const RegisterConfig &cfg)
{
	const uint8_t value = readRegister(cfg.reg);

	return !_transfer_failed && (value & cfg.set_bits) == cfg.set_bits && (value & cfg.clear_bits) == 0;
}

bool BoschDirect::configure()
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

void BoschDirect::RunImpl()
{
	_transfer_failed = false;

	const hrt_abstime now = hrt_absolute_time();

	// Steady-state sampling bypasses initialization dispatch; recovery stays in the same work-queue pass.
	if (__builtin_expect(_state == State::kRead, 1)) {
		hrt_abstime interrupt_time = 0;

		if (_interrupt_enabled) {
			const hrt_abstime timestamp = _drdy_timestamp.fetch_and(0);

			if (timestamp != 0
			    && timestamp <= now
			    && now - timestamp < _interval_us) {
				interrupt_time = timestamp;

			} else {
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
		if (!success || now - _last_config_check > 100_ms) {
			if (!checkRegister(_registers[_checked_register])) {
				if (!_transfer_failed) {
					_transfer_perf.bad_register.count();
				}

				reset();

				return;
			}

			_checked_register  = (_checked_register + 1) % _register_count;
			_last_config_check = now;
		}

		if (now - _last_temperature >= 1_s) {
			updateTemperature();
			_last_temperature = now;
		}

		return;
	}

	switch (_state) {
	case State::kReset: {
			disableInterrupt();
			_reset_timestamp  = now;
			_failure_count    = 0;
			_checked_register = 0;

			if (!writeRegister(_profile.reset_reg, SoftReset)) {
				ScheduleDelayed(100_ms);
				break;
			}

			_state = State::kWaitReset;
			ScheduleDelayed(_profile.reset_wait_us);
			break;
		}

	case State::kWaitReset: {
			if (identity()) {
				if (bosch_direct_fifo::tagged(_profile.format) && !writeRegister(tagged::PowerConfig, 0)) {
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
				// BMI085/088 need at least 450 us after ACC_PWR_CTRL. BMI090L's
				// quick-start sequence requires 50 ms. Do not access SPI in this gap.
				_state = State::kConfigure;
				ScheduleDelayed(_accel_startup_us);

			} else {
				reset();
			}

			break;
		}

	case State::kLoadConfig: {
#if defined(CONFIG_BOSCH_DIRECT_BMI270)

			// Keep the existing maximum-FIFO image and its required enable/status handshake.
			if (writeRegister(tagged::InitControl, 0)
			    && transferChecked(bosch_direct_model::kBmi270Config, nullptr, bosch_direct_model::kBmi270ConfigSize)
			    && writeRegister(tagged::InitControl, 1)) {
				_state = State::kWaitConfig;
				ScheduleDelayed(150_ms);

			} else {
				reset();
			}

#else
			reset();
#endif // CONFIG_BOSCH_DIRECT_BMI270
			break;
		}

	case State::kWaitConfig: {
			if (readRegister(tagged::InternalStatus) == tagged::ConfigLoaded && !_transfer_failed) {
				_state = State::kConfigure;
				ScheduleNow();

			} else {
				if (!_transfer_failed) {
					_transfer_perf.bad_register.count();
				}

				reset();
			}

			break;
		}

	case State::kConfigure: {
			if (configure()) {
				_state = State::kFifoReset;
				ScheduleDelayed(10_ms);

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

			_state             = State::kRead;
			_last_config_check = now;
			_last_temperature  = 0;
			_interrupt_enabled = enableInterrupt();

			if (_interrupt_enabled) {
				ScheduleDelayed(2 * _interval_us);

			} else {
				ScheduleOnInterval(_interval_us, _interval_us);
			}

			break;
		}

	case State::kRead: {
			break; // Handled by the acquisition fast path above.
		}
	}
}

bool BoschDirect::readFifo(hrt_abstime now, hrt_abstime interrupt_time)
{
	return bosch_direct_fifo::tagged(_profile.format) ? readTagged(now, interrupt_time) : readFixed(now, interrupt_time);
}

hrt_abstime BoschDirect::sampleTime(hrt_abstime now, hrt_abstime interrupt_time, uint8_t samples) const
{
	if (interrupt_time == 0) {
		return now;
	}

	// The watermark IRQ timestamps its triggering sample; account for samples accumulated before the read.
	if (samples > _watermark_samples) {
		return math::min(now, interrupt_time + hrt_abstime((samples - _watermark_samples) * _sample_dt));
	}

	return interrupt_time;
}

bool BoschDirect::readFixed(hrt_abstime now, hrt_abstime interrupt_time)
{
	const uint8_t status = readRegister(_profile.count_reg);

	if (_transfer_failed) {
		return false;
	}

	uint8_t samples = status & fixed::CountMask;

	if ((status & fixed::Overrun)
	    || samples > kMaxSamples
	    || samples * bosch_direct_fifo::kAxisBytes > _profile.fifo_capacity_bytes) {
		_fifo_perf.overflow.count();
		fifoReset();

		return false;
	}

	if (samples == 0) {
		_fifo_perf.empty.count();

		return false;
	}

	// Preserve the original fixed-FIFO one-sample jitter tolerance.
	if (samples == _watermark_samples + 1) {
		--samples;

		if (interrupt_time == 0) {
			const hrt_abstime dt = static_cast<hrt_abstime>(_sample_dt);

			if (now <= dt) {
				return false;
			}

			now -= dt;
		}
	}

	const size_t bytes = samples * bosch_direct_fifo::kAxisBytes;
	const size_t size  = _profile.device.data_prefix_bytes + bytes;

	if (size > _profile.device.max_transfer_bytes) {
		return false;
	}

	_buffer[0] = _profile.fifo_reg | Read;
	set_frequency(_data_frequency);

	if (!transferChecked(_buffer, _buffer, size)) {
		return false;
	}

	const uint8_t     *data     = _buffer + _profile.device.data_prefix_bytes;
	const hrt_abstime timestamp = sampleTime(now, interrupt_time, samples);

#if defined(CONFIG_BOSCH_DIRECT_BMI055)

	if (_profile.format == Format::kFixed12Accel) {
		sensor_accel_fifo_s batch {};

		if (!bosch_direct_fifo::decodeFixed<4>(data, bytes, false, batch)) {
			_transfer_perf.bad_transfer.count();
			fifoReset();

			return false;
		}

		batch.timestamp_sample = timestamp;
		batch.dt               = _sample_dt;
		_accel->set_error_count(errorCount());
		_accel->updateFIFO(batch);

		return true;
	}

#endif // CONFIG_BOSCH_DIRECT_BMI055

	sensor_gyro_fifo_s batch {};

	if (!bosch_direct_fifo::decodeFixed<0>(data, bytes, _profile.reject_all_minimum, batch)) {
		_transfer_perf.bad_transfer.count();
		fifoReset();

		return false;
	}

	batch.timestamp_sample = timestamp;
	batch.dt               = _sample_dt;
	_gyro->set_error_count(errorCount());
	_gyro->updateFIFO(batch);

	return true;
}

bool BoschDirect::readTagged(hrt_abstime now, hrt_abstime interrupt_time)
{
#if defined(CONFIG_BOSCH_DIRECT_BMI270)

	if (_profile.variant == Variant::kBmi270 && (readRegister(tagged::Error) & tagged::FifoError)) {
		_fifo_perf.overflow.count();
		fifoReset();

		return false;
	}

#endif // CONFIG_BOSCH_DIRECT_BMI270

	uint8_t count[2];

	if (!readRegisters(_profile.count_reg, count, sizeof(count))) {
		return false;
	}

	const uint16_t bytes = uint16_t(count[0]) | uint16_t(count[1] & 0x3f) << 8;
	const size_t   size  = _profile.device.data_prefix_bytes + bytes;

	if (bytes == 0) {
		_fifo_perf.empty.count();

		return false;
	}

	if (bytes >= _profile.fifo_capacity_bytes || size > _profile.device.max_transfer_bytes) {
		_fifo_perf.overflow.count();
		fifoReset();

		return false;
	}

	_buffer[0] = _profile.fifo_reg | Read;
	set_frequency(_data_frequency);

	if (!transferChecked(_buffer, _buffer, size)) {
		return false;
	}

	// BMI08x accel transfers include a fresh count after the SPI command and dummy byte.
	if (_profile.device.data_prefix_bytes == 4) {
		const uint16_t embedded_count = uint16_t(_buffer[2]) | uint16_t(_buffer[3] & 0x3f) << 8;

		if (embedded_count < bytes || embedded_count >= _profile.fifo_capacity_bytes) {
			_fifo_perf.overflow.count();
			fifoReset();

			return false;
		}
	}

	const uint8_t *data = _buffer + _profile.device.data_prefix_bytes;

#if defined(CONFIG_BOSCH_DIRECT_BMI270)

	if (_profile.format == Format::kTaggedImu) {
		return processTagged<true>(data, bytes, now, interrupt_time);
	}

#endif // CONFIG_BOSCH_DIRECT_BMI270

	return processTagged<false>(data, bytes, now, interrupt_time);
}

template<bool integrated>
bool BoschDirect::processTagged(
	const uint8_t *data,
	size_t size,
	hrt_abstime now,
	hrt_abstime interrupt_time)
{
	sensor_accel_fifo_s accel {};
	sensor_gyro_fifo_s gyro;

	gyro.samples = 0;

	// Keep symmetric storage without clearing unused axes in an accel-only endpoint.
	// Only an integrated endpoint may publish this buffer.
	if constexpr(integrated) {
		gyro = {};
	}

	bosch_direct_fifo::Sample sample;
	const auto decode = [](imu::ByteCursor & cursor, bosch_direct_fifo::Sample & decoded) {
		return bosch_direct_fifo::readTagged<integrated>(cursor, decoded);
	};
	constexpr imu::FifoChannels channels = integrated ? imu::FifoChannels::kBoth : imu::FifoChannels::kAccel;
	const imu::FifoDecodeResult result = imu::decodeFifoFrames<imu::FifoAxisMapping::kFlipYZ, channels>(
			data, size, accel, gyro, sample, decode);

	if (result.status == imu::FifoDecodeStatus::kReconfigure) {
		_transfer_perf.bad_register.count();
		reset();

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

	const uint8_t samples = math::max(accel.samples, gyro.samples);

	// Byte watermarks do not identify a sample tick when control frames or
	// unpaired channels are present. Fall back to the count-snapshot time.
	const hrt_abstime irq       = result.has_metadata || (integrated && !result.all_combined) ? 0 : interrupt_time;
	const hrt_abstime timestamp = sampleTime(now, irq, samples);
	const uint64_t    errors    = errorCount();

	_accel->set_error_count(errors);

	if constexpr(integrated) {
		_gyro->set_error_count(errors);

		if (gyro.samples) {
			gyro.timestamp_sample = timestamp;
			gyro.dt               = _sample_dt;
			_gyro->updateFIFO(gyro);
		}
	}

	if (accel.samples) {
		accel.timestamp_sample = timestamp;
		accel.dt               = _sample_dt;
		_accel->updateFIFO(accel);
	}

	return true;
}

bool BoschDirect::fifoReset()
{
	_fifo_perf.reset.count();
	_drdy_timestamp.store(0);

	if (bosch_direct_fifo::tagged(_profile.format)) {
		return writeRegister(_profile.reset_reg, FifoFlush);
	}

	// Fixed FIFOs are cleared through bypass mode, then recover their configured watermark and mode.
	if (!writeRegister(_profile.watermark_reg, 0) || !writeRegister(fixed::FifoConfig, 0)) {
		return false;
	}

	for (unsigned i = 0; i < _register_count; ++i) {
		const auto &cfg = _registers[i];

		if ((cfg.reg == _profile.watermark_reg || cfg.reg == fixed::FifoConfig) && !modifyRegister(cfg)) {
			return false;
		}
	}

	return true;
}

void BoschDirect::updateTemperature()
{
	using Temperature = bosch_direct_model::Temperature;

	if (_profile.temperature == Temperature::kNone) {
		return;
	}

	uint8_t data[2];
	const size_t size = _profile.temperature == Temperature::kSigned8 ? 1 : 2;

	if (!readRegisters(_profile.temperature_reg, data, size)) {
		return;
	}

	float temperature = 0.f;
	bool  valid       = true;

	switch (_profile.temperature) {
	case Temperature::kSigned8: {
			temperature = static_cast<int8_t>(data[0]) * 0.5f + 23.f;
			break;
		}

	case Temperature::kBmi08x11: {
			valid = bosch_direct_fifo::temperature08x(data, temperature);
			break;
		}

	case Temperature::kBmi27016: {
			valid = bosch_direct_fifo::temperature270(data, temperature);
			break;
		}

	default: {
			return;
		}
	}

	if (valid) {
		if (_accel) {
			_accel->set_temperature(temperature);
		}

		if (_gyro) {
			_gyro->set_temperature(temperature);
		}
	}
}

uint64_t BoschDirect::errorCount() const
{
	return _transfer_perf.bad_register.eventCount() + _transfer_perf.bad_transfer.eventCount()
	       + _fifo_perf.empty.eventCount() + _fifo_perf.overflow.eventCount();
}

int BoschDirect::dataReadyCallback(int irq, void *context, void *arg)
{
	auto *driver = static_cast<BoschDirect *>(arg);
	driver->_drdy_timestamp.store(hrt_absolute_time());
	driver->ScheduleNow();

	return 0;
}

bool BoschDirect::enableInterrupt()
{
	return _drdy_gpio != 0 && _profile.interrupt_enabled
	       && px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &dataReadyCallback, this) == 0;
}

void BoschDirect::disableInterrupt()
{
	if (_interrupt_enabled) {
		px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr);
	}

	_interrupt_enabled = false;
	_drdy_timestamp.store(0);
}
