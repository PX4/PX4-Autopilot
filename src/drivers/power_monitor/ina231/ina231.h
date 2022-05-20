/****************************************************************************
 *
 *   Copyright (C) 2022 PX4 Development Team. All rights reserved.
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
 * @file ina231.h
 *
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <battery/battery.h>
#include <drivers/drv_hrt.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace time_literals;

/* Configuration Constants */
#define INA231_BASEADDR 	                    0x40 /* 7-bit address. 8-bit address is 0x44 */
// If initialization is forced (with the -f flag on the command line), but it fails, the drive will try again to
// connect to the INA231 every this many microseconds
#define INA231_INIT_RETRY_INTERVAL_US			500000

/* INA231 Registers addresses */
#define INA231_REG_CONFIGURATION             (0x00)
#define INA231_REG_SHUNTVOLTAGE              (0x01)
#define INA231_REG_BUSVOLTAGE                (0x02)
#define INA231_REG_POWER                     (0x03)
#define INA231_REG_CURRENT                   (0x04)
#define INA231_REG_CALIBRATION               (0x05)
#define INA231_REG_MASKENABLE                (0x06)
#define INA231_REG_ALERTLIMIT                (0x07)
#define INA231_MFG_ID                        (0xfe)
#define INA231_MFG_DIEID                     (0xff)

/* INA231 Configuration Register */
#define INA231_MODE_SHIFTS                   (0)
#define INA231_MODE_MASK                     (7 << INA231_MODE_SHIFTS)
#define INA231_MODE_SHUTDOWN                 (0 << INA231_MODE_SHIFTS)
#define INA231_MODE_SHUNT_TRIG               (1 << INA231_MODE_SHIFTS)
#define INA231_MODE_BUS_TRIG                 (2 << INA231_MODE_SHIFTS)
#define INA231_MODE_SHUNT_BUS_TRIG           (3 << INA231_MODE_SHIFTS)
#define INA231_MODE_ADC_OFF                  (4 << INA231_MODE_SHIFTS)
#define INA231_MODE_SHUNT_CONT               (5 << INA231_MODE_SHIFTS)
#define INA231_MODE_BUS_CONT                 (6 << INA231_MODE_SHIFTS)
#define INA231_MODE_SHUNT_BUS_CONT           (7 << INA231_MODE_SHIFTS)

#define INA231_VSHCT_SHIFTS                  (3)
#define INA231_VSHCT_MASK                    (7 << INA231_VSHCT_SHIFTS)
#define INA231_VSHCT_140US                   (0 << INA231_VSHCT_SHIFTS)
#define INA231_VSHCT_204US                   (1 << INA231_VSHCT_SHIFTS)
#define INA231_VSHCT_332US                   (2 << INA231_VSHCT_SHIFTS)
#define INA231_VSHCT_588US                   (3 << INA231_VSHCT_SHIFTS)
#define INA231_VSHCT_1100US                  (4 << INA231_VSHCT_SHIFTS)
#define INA231_VSHCT_2116US                  (5 << INA231_VSHCT_SHIFTS)
#define INA231_VSHCT_4156US                  (6 << INA231_VSHCT_SHIFTS)
#define INA231_VSHCT_8244US                  (7 << INA231_VSHCT_SHIFTS)

#define INA231_VBUSCT_SHIFTS                 (6)
#define INA231_VBUSCT_MASK                   (7 << INA231_VBUSCT_SHIFTS)
#define INA231_VBUSCT_140US                  (0 << INA231_VBUSCT_SHIFTS)
#define INA231_VBUSCT_204US                  (1 << INA231_VBUSCT_SHIFTS)
#define INA231_VBUSCT_332US                  (2 << INA231_VBUSCT_SHIFTS)
#define INA231_VBUSCT_588US                  (3 << INA231_VBUSCT_SHIFTS)
#define INA231_VBUSCT_1100US                 (4 << INA231_VBUSCT_SHIFTS)
#define INA231_VBUSCT_2116US                 (5 << INA231_VBUSCT_SHIFTS)
#define INA231_VBUSCT_4156US                 (6 << INA231_VBUSCT_SHIFTS)
#define INA231_VBUSCT_8244US                 (7 << INA231_VBUSCT_SHIFTS)

#define INA231_AVERAGES_SHIFTS                (9)
#define INA231_AVERAGES_MASK                  (7 << INA231_AVERAGES_SHIFTS)
#define INA231_AVERAGES_1                     (0 << INA231_AVERAGES_SHIFTS)
#define INA231_AVERAGES_4                     (1 << INA231_AVERAGES_SHIFTS)
#define INA231_AVERAGES_16                    (2 << INA231_AVERAGES_SHIFTS)
#define INA231_AVERAGES_64                    (3 << INA231_AVERAGES_SHIFTS)
#define INA231_AVERAGES_128                   (4 << INA231_AVERAGES_SHIFTS)
#define INA231_AVERAGES_256                   (5 << INA231_AVERAGES_SHIFTS)
#define INA231_AVERAGES_512                   (6 << INA231_AVERAGES_SHIFTS)
#define INA231_AVERAGES_1024                  (7 << INA231_AVERAGES_SHIFTS)

#define INA231_CONFIG (INA231_MODE_SHUNT_BUS_CONT | INA231_VSHCT_588US | INA231_VBUSCT_588US | INA231_AVERAGES_64)

#define INA231_RST                            (1 << 15)

/* INA231 Enable / Mask Register */

#define INA231_LEN                           (1 << 0)
#define INA231_APOL                          (1 << 1)
#define INA231_OVF                           (1 << 2)
#define INA231_CVRF                          (1 << 3)
#define INA231_AFF                           (1 << 4)

#define INA231_CNVR                          (1 << 10)
#define INA231_POL                           (1 << 11)
#define INA231_BUL                           (1 << 12)
#define INA231_BOL                           (1 << 13)
#define INA231_SUL                           (1 << 14)
#define INA231_SOL                           (1 << 15)

#define INA231_SAMPLE_FREQUENCY_HZ            10
#define INA231_SAMPLE_INTERVAL_US             (1_s / INA231_SAMPLE_FREQUENCY_HZ)
#define INA231_CONVERSION_INTERVAL            (INA231_SAMPLE_INTERVAL_US - 7)
#define MAX_CURRENT                           164.0f    /* 164 Amps */
#define DN_MAX                                32768.0f  /* 2^15 */
#define INA231_CONST                          0.00512f  /* is an internal fixed value used to ensure scaling is maintained properly  */
#define INA231_SHUNT                          0.0005f   /* Shunt is 500 uOhm */
#define INA231_VSCALE                         0.00125f  /* LSB of voltage is 1.25 mV  */

#define swap16(w)                       __builtin_bswap16((w))

class INA231 : public device::I2C, public ModuleParams, public I2CSPIDriver<INA231>
{
public:
	INA231(const I2CSPIDriverConfig &config, int battery_index);
	virtual ~INA231();

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	void	RunImpl();

	int 		  init() override;

	/**
	 * Tries to call the init() function. If it fails, then it will schedule to retry again in
	 * INA231_INIT_RETRY_INTERVAL_US microseconds. It will keep retrying at this interval until initialization succeeds.
	 *
	 * @return PX4_OK if initialization succeeded on the first try. Negative value otherwise.
	 */
	int force_init();

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				      print_status() override;

protected:
	int	  		probe() override;

private:
	bool			        _sensor_ok{false};
	unsigned                        _measure_interval{0};
	bool			        _collect_phase{false};
	bool 					_initialized{false};

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t 		_collection_errors;
	perf_counter_t 		_measure_errors;

	int16_t           _bus_voltage{0};
	int16_t           _power{0};
	int16_t           _current{0};
	int16_t           _shunt{0};
	int16_t           _cal{0};
	bool              _mode_triggered{false};

	float             _max_current{MAX_CURRENT};
	float             _rshunt{INA231_SHUNT};
	uint16_t          _config{INA231_CONFIG};
	float             _current_lsb{_max_current / DN_MAX};
	float             _power_lsb{25.0f * _current_lsb};

	Battery 		  _battery;
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	int read(uint8_t address, int16_t &data);
	int write(uint8_t address, uint16_t data);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				      start();

	int					     measure();
	int					     collect();

};
