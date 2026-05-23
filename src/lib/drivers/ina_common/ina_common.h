/****************************************************************************
 *
 *   Copyright (c) 2019-2026 PX4 Development Team. All rights reserved.
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
 * @file ina_common.h
 *
 * Shared register definitions and utility class for INA226/INA228/INA231
 * power monitor drivers. These devices share the same register map.
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <lib/perf/perf_counter.h>
#include <battery/battery.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

/* INA Common Register Addresses */
#define INA_COMMON_REG_CONFIGURATION             (0x00)
#define INA_COMMON_REG_SHUNTVOLTAGE              (0x01)
#define INA_COMMON_REG_BUSVOLTAGE                (0x02)
#define INA_COMMON_REG_POWER                     (0x03)
#define INA_COMMON_REG_CURRENT                   (0x04)
#define INA_COMMON_REG_CALIBRATION               (0x05)
#define INA_COMMON_REG_MASKENABLE                (0x06)
#define INA_COMMON_REG_ALERTLIMIT                (0x07)

/* INA Common Configuration Register */
#define INA_COMMON_MODE_SHIFTS                   (0)
#define INA_COMMON_MODE_MASK                     (7 << INA_COMMON_MODE_SHIFTS)
#define INA_COMMON_MODE_SHUTDOWN                 (0 << INA_COMMON_MODE_SHIFTS)
#define INA_COMMON_MODE_SHUNT_TRIG               (1 << INA_COMMON_MODE_SHIFTS)
#define INA_COMMON_MODE_BUS_TRIG                 (2 << INA_COMMON_MODE_SHIFTS)
#define INA_COMMON_MODE_SHUNT_BUS_TRIG           (3 << INA_COMMON_MODE_SHIFTS)
#define INA_COMMON_MODE_ADC_OFF                  (4 << INA_COMMON_MODE_SHIFTS)
#define INA_COMMON_MODE_SHUNT_CONT               (5 << INA_COMMON_MODE_SHIFTS)
#define INA_COMMON_MODE_BUS_CONT                 (6 << INA_COMMON_MODE_SHIFTS)
#define INA_COMMON_MODE_SHUNT_BUS_CONT           (7 << INA_COMMON_MODE_SHIFTS)

#define INA_COMMON_VSHCT_SHIFTS                  (3)
#define INA_COMMON_VSHCT_MASK                    (7 << INA_COMMON_VSHCT_SHIFTS)
#define INA_COMMON_VSHCT_140US                   (0 << INA_COMMON_VSHCT_SHIFTS)
#define INA_COMMON_VSHCT_204US                   (1 << INA_COMMON_VSHCT_SHIFTS)
#define INA_COMMON_VSHCT_332US                   (2 << INA_COMMON_VSHCT_SHIFTS)
#define INA_COMMON_VSHCT_588US                   (3 << INA_COMMON_VSHCT_SHIFTS)
#define INA_COMMON_VSHCT_1100US                  (4 << INA_COMMON_VSHCT_SHIFTS)
#define INA_COMMON_VSHCT_2116US                  (5 << INA_COMMON_VSHCT_SHIFTS)
#define INA_COMMON_VSHCT_4156US                  (6 << INA_COMMON_VSHCT_SHIFTS)
#define INA_COMMON_VSHCT_8244US                  (7 << INA_COMMON_VSHCT_SHIFTS)

#define INA_COMMON_VBUSCT_SHIFTS                 (6)
#define INA_COMMON_VBUSCT_MASK                   (7 << INA_COMMON_VBUSCT_SHIFTS)
#define INA_COMMON_VBUSCT_140US                  (0 << INA_COMMON_VBUSCT_SHIFTS)
#define INA_COMMON_VBUSCT_204US                  (1 << INA_COMMON_VBUSCT_SHIFTS)
#define INA_COMMON_VBUSCT_332US                  (2 << INA_COMMON_VBUSCT_SHIFTS)
#define INA_COMMON_VBUSCT_588US                  (3 << INA_COMMON_VBUSCT_SHIFTS)
#define INA_COMMON_VBUSCT_1100US                 (4 << INA_COMMON_VBUSCT_SHIFTS)
#define INA_COMMON_VBUSCT_2116US                 (5 << INA_COMMON_VBUSCT_SHIFTS)
#define INA_COMMON_VBUSCT_4156US                 (6 << INA_COMMON_VBUSCT_SHIFTS)
#define INA_COMMON_VBUSCT_8244US                 (7 << INA_COMMON_VBUSCT_SHIFTS)

#define INA_COMMON_AVERAGES_SHIFTS               (9)
#define INA_COMMON_AVERAGES_MASK                 (7 << INA_COMMON_AVERAGES_SHIFTS)
#define INA_COMMON_AVERAGES_1                    (0 << INA_COMMON_AVERAGES_SHIFTS)
#define INA_COMMON_AVERAGES_4                    (1 << INA_COMMON_AVERAGES_SHIFTS)
#define INA_COMMON_AVERAGES_16                   (2 << INA_COMMON_AVERAGES_SHIFTS)
#define INA_COMMON_AVERAGES_64                   (3 << INA_COMMON_AVERAGES_SHIFTS)
#define INA_COMMON_AVERAGES_128                  (4 << INA_COMMON_AVERAGES_SHIFTS)
#define INA_COMMON_AVERAGES_256                  (5 << INA_COMMON_AVERAGES_SHIFTS)
#define INA_COMMON_AVERAGES_512                  (6 << INA_COMMON_AVERAGES_SHIFTS)
#define INA_COMMON_AVERAGES_1024                 (7 << INA_COMMON_AVERAGES_SHIFTS)

#define INA_COMMON_RST                           (1 << 15)

/* INA Common Enable / Mask Register */
#define INA_COMMON_LEN                           (1 << 0)
#define INA_COMMON_APOL                          (1 << 1)
#define INA_COMMON_OVF                           (1 << 2)
#define INA_COMMON_CVRF                          (1 << 3)
#define INA_COMMON_AFF                           (1 << 4)

#define INA_COMMON_CNVR                          (1 << 10)
#define INA_COMMON_POL                           (1 << 11)
#define INA_COMMON_BUL                           (1 << 12)
#define INA_COMMON_BOL                           (1 << 13)
#define INA_COMMON_SUL                           (1 << 14)
#define INA_COMMON_SOL                           (1 << 15)

/* Shared constants */
#define INA_COMMON_SAMPLE_FREQUENCY_HZ           10
#define INA_COMMON_SAMPLE_INTERVAL_US            (1_s / INA_COMMON_SAMPLE_FREQUENCY_HZ)
#define INA_COMMON_CONVERSION_INTERVAL           (INA_COMMON_SAMPLE_INTERVAL_US - 7)
#define INA_COMMON_INIT_RETRY_INTERVAL_US        500000
#define INA_COMMON_DN_MAX                        32768.0f  /* 2^15 */
#define INA_COMMON_CONST                         0.00512f  /* internal fixed value for scaling */
#define INA_COMMON_VSCALE                        0.00125f  /* LSB of voltage is 1.25 mV */

#define ina_common_swap16(w)                     __builtin_bswap16((w))

class INACommon
{
public:
	typedef int (*transfer_func_t)(void *context, const uint8_t *send, unsigned send_len,
				       uint8_t *recv, unsigned recv_len);

	INACommon(transfer_func_t transfer_func, void *transfer_context, Battery &battery,
		  perf_counter_t sample_perf, perf_counter_t comms_errors);

	/**
	 * Load device parameters (max current, shunt resistance, config register)
	 * from the PX4 parameter system and compute current/power LSBs.
	 */
	void loadParams(const char *current_param, const char *shunt_param, const char *config_param,
			float default_max_current, float default_shunt, uint16_t default_config);

	/**
	 * Read a 16-bit register via I2C with retry.
	 */
	int read(uint8_t address, int16_t &data);

	/**
	 * Write a 16-bit register via I2C.
	 */
	int write(uint8_t address, uint16_t data);

	/**
	 * Initialize the INA device: reset, write calibration, optionally write config.
	 * Call this after I2C::init() succeeds.
	 *
	 * @return PX4_OK on success.
	 */
	int init();

	/**
	 * Write config register when in triggered mode.
	 */
	int measure();

	/**
	 * Read bus voltage and current from the device, update battery status.
	 */
	int collect();

	/**
	 * Manage the connected state with debounce filtering.
	 */
	bool setConnected(bool state);

	bool     _mode_triggered{false};
	bool     _sensor_ok{false};
	bool     _initialized{false};
	float    _current_lsb{0};
	float    _power_lsb{0};
	float    _max_current{0};
	float    _rshunt{0};
	uint16_t _config{0};
	int16_t  _cal{0};

private:
	transfer_func_t  _transfer_func;
	void            *_transfer_context;
	Battery         &_battery;
	perf_counter_t   _sample_perf;
	perf_counter_t   _comms_errors;

	int16_t  _bus_voltage{0};
	int16_t  _current{0};
	uint8_t  _connected{0};
};
