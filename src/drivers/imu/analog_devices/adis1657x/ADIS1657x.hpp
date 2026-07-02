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

#pragma once

#include "Analog_Devices_ADIS1657x_registers.hpp"

#include <drivers/drv_hrt.h>
#include <drivers/device/spi.h>
#include <lib/conversion/rotation.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace Analog_Devices_ADIS1657x;

// 32-bit burst frame: cmd(2) + 17 words(34) = 36 bytes
struct __attribute__((packed)) adis1657x_burst {
	uint8_t  cmd[2];
	uint16_t diag_stat;
	uint16_t x_gyro_lsb;
	uint16_t x_gyro_msb;
	uint16_t y_gyro_lsb;
	uint16_t y_gyro_msb;
	uint16_t z_gyro_lsb;
	uint16_t z_gyro_msb;
	uint16_t x_accel_lsb;
	uint16_t x_accel_msb;
	uint16_t y_accel_lsb;
	uint16_t y_accel_msb;
	uint16_t z_accel_lsb;
	uint16_t z_accel_msb;
	uint16_t temp;
	uint16_t data_cntr;
	uint16_t timestamp_upr;
	uint16_t spi_chksum;
};

static_assert(sizeof(adis1657x_burst) == 36, "burst frame size mismatch");

class ADIS1657x : public device::SPI, public I2CSPIDriver<ADIS1657x>
{
public:
	ADIS1657x(const I2CSPIDriverConfig &config);
	~ADIS1657x() override;

	static void print_usage();

	static int selected_dec_rate;
	static int selected_filt_size;

	int		init() override;
	void		print_status() override;
	void		RunImpl();

protected:
	int		probe() override;
	void		exit_and_cleanup() override;

private:
	PX4Accelerometer	_px4_accel;
	PX4Gyroscope		_px4_gyro;

	perf_counter_t		_sample_perf;
	perf_counter_t		_bad_transfers;
	perf_counter_t		_reset_perf;
	perf_counter_t		_drdy_missed_perf;

	const spi_drdy_gpio_t	_drdy_gpio;

	int _dec_rate;
	int _filt_size;

	float _gyro_scale{0.0f};
	float _accel_scale{0.0f};
	float _sample_rate_hz{1000.0f};
	uint16_t _prod_id{0};
	uint16_t _last_data_cntr{0};
	px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};
	int32_t _drdy_count{0};
	bool _data_ready_interrupt_enabled{false};

	int _failure_count{0};
	int _configure_retries{0};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		READ,
	} _state{STATE::RESET};

	// SPI register access
	uint16_t	read_reg16(Register reg);
	bool		write_reg16(Register reg, uint16_t val);
	bool		write_reg16_verified(Register reg, uint16_t val, int retries = 3);

	// Burst read
	bool		read_burst32(adis1657x_burst &burst);
	bool		validate_checksum(const adis1657x_burst &burst);

	void		print_error_flags(uint16_t diag_stat);
	bool		self_test();
	void		Reset();
	bool		Configure();
	void		start();
	int		measure(hrt_abstime timestamp_sample);

	// DRDY interrupt handling
	void		DataReady();
	bool		DataReadyInterruptConfigure();
	bool		DataReadyInterruptDisable();
	static int	DataReadyInterruptCallback(int irq, void *context, void *arg);
};
