/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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


#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <battery/battery.h>
#include <drivers/drv_hrt.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include "ina238_registers.hpp"

using namespace time_literals;
using namespace ina238;

/* Configuration Constants */
#define INA238_BASEADDR 	            0x45 /* 7-bit address. 8-bit address is 0x45 */
// If initialization is forced (with the -f flag on the command line), but it fails, the drive will try again to
// connect to the INA238 every this many microseconds
#define INA238_INIT_RETRY_INTERVAL_US       500000


#define INA238_MFG_ID_TI                    (0x5449) // TI
#define INA238_MFG_DIE                      (0x238) // INA237, INA238

#define INA238_ADCRANGE_SHIFTS              (4)
#define INA238_ADCRANGE_LOW                 (1 << INA238_ADCRANGE_SHIFTS) // ± 40.96 mV
#define INA238_ADCRANGE_HIGH                (0 << INA238_ADCRANGE_SHIFTS) // ±163.84 mV

#define INA238_DEVICE_ID_SHIFTS              (4)
#define INA238_DEVICE_ID_MASK                (0xfff << INA238_DEVICE_ID_SHIFTS)
#define INA238_DEVICEID(v)                   (((v) & INA238_DEVICE_ID_MASK) >> INA238_DEVICE_ID_SHIFTS)

#define INA238_SAMPLE_FREQUENCY_HZ           10
#define INA238_SAMPLE_INTERVAL_US            (1_s / INA238_SAMPLE_FREQUENCY_HZ)
#define INA238_CONVERSION_INTERVAL           (INA238_SAMPLE_INTERVAL_US - 7)
#define INA238_DN_MAX                        32768.0f   /* 2^15 */
#define INA238_CONST                         819.2e6f  /* is an internal fixed value used to ensure scaling is maintained properly  */
#define INA238_VSCALE                        3.125e-03f  /* LSB of voltage is 3.1255 mV/LSB */

#define DEFAULT_MAX_CURRENT                  327.68f    /* Amps */
#define DEFAULT_SHUNT                        0.0003f   /* Shunt is 300 uOhm */

#define swap16(w)                            __builtin_bswap16((w))
#define swap32(d)                            __builtin_bswap32((d))
#define swap64(q)                            __builtin_bswap64((q))

class INA238 : public device::I2C, public ModuleParams, public I2CSPIDriver<INA238>
{
public:
	INA238(const I2CSPIDriverConfig &config, int battery_index);
	virtual ~INA238();

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	void RunImpl();

	int init() override;

	/**
	 * Tries to call the init() function. If it fails, then it will schedule to retry again in
	 * INA238_INIT_RETRY_INTERVAL_US microseconds. It will keep retrying at this interval until initialization succeeds.
	 *
	 * @return PX4_OK if initialization succeeded on the first try. Negative value otherwise.
	 */
	int force_init();

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void print_status() override;

protected:
	int probe() override;

private:
	// Sensor Configuration
	struct register_config_t {
		Register reg;
		uint16_t set_bits{0};
		uint16_t clear_bits{0};
	};
	bool RegisterCheck(const register_config_t &reg_cfg);
	int RegisterWrite(Register reg, uint16_t value);
	int RegisterRead(Register reg, uint16_t &value);
	int Reset();

	unsigned int _measure_interval{0};
	bool _collect_phase{false};
	bool _initialized{false};

	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;
	perf_counter_t _collection_errors;
	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};

	// Configuration state, computed from params
	float _max_current;
	float _rshunt;
	float _current_lsb;
	int16_t _range;
	uint16_t _shunt_calibration{0};


	hrt_abstime _last_config_check_timestamp{0};
	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{3};
	register_config_t _register_cfg[size_register_cfg] {
		// Register | Set bits, Clear bits
		{ Register::CONFIG, 0, 0}, // will be set dynamically
		{ Register::ADCCONFIG, MODE_TEMP_SHUNT_BUS_CONT |  VBUSCT_540US |  VSHCT_540US | VTCT_540US | AVERAGES_64},
		{ Register::SHUNT_CAL, 0, 0}	// will be set dynamically
	};

	Battery _battery;
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	int read(uint8_t address, uint16_t &data);
	int write(uint8_t address, uint16_t data);

	int read(uint8_t address, int16_t &data)
	{
		return read(address, (uint16_t &)data);
	}

	int write(uint8_t address, int16_t data)
	{
		return write(address, (uint16_t)data);
	}

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void start();

	int collect();

};
