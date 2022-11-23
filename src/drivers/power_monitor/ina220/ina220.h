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
 * @file ina220.h
 *
 */

#pragma once


#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <battery/battery.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/power_monitor.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace time_literals;

/* Configuration Constants */
#define INA220_BASEADDR 	                    0x41 /* 7-bit address. 8-bit address is 0x41 */
// If initialization is forced (with the -f flag on the command line), but it fails, the drive will try again to
// connect to the INA220 every this many microseconds
#define INA220_INIT_RETRY_INTERVAL_US			500000

/* INA220 Registers addresses */
#define INA220_REG_CONFIGURATION             (0x00)
#define INA220_REG_SHUNTVOLTAGE              (0x01)
#define INA220_REG_BUSVOLTAGE                (0x02)
#define INA220_REG_POWER                     (0x03)
#define INA220_REG_CURRENT                   (0x04)
#define INA220_REG_CALIBRATION               (0x05)

#define INA220_DEFAULT_CONFIG                (0x399F)

/* INA220 Configuration Register */
#define INA220_MODE_SHIFTS                   (0)
#define INA220_MODE_MASK                     (7 << INA220_MODE_SHIFTS)
#define INA220_MODE_SHUTDOWN                 (0 << INA220_MODE_SHIFTS)
#define INA220_MODE_SHUNT_TRIG               (1 << INA220_MODE_SHIFTS)
#define INA220_MODE_BUS_TRIG                 (2 << INA220_MODE_SHIFTS)
#define INA220_MODE_SHUNT_BUS_TRIG           (3 << INA220_MODE_SHIFTS)
#define INA220_MODE_ADC_OFF                  (4 << INA220_MODE_SHIFTS)
#define INA220_MODE_SHUNT_CONT               (5 << INA220_MODE_SHIFTS)
#define INA220_MODE_BUS_CONT                 (6 << INA220_MODE_SHIFTS)
#define INA220_MODE_SHUNT_BUS_CONT           (7 << INA220_MODE_SHIFTS)

#define INA220_SADC_SHIFTS                   (3)
#define INA220_SADC_MASK                     (15 << INA220_SADC_SHIFTS)
#define INA220_SADC_9BIT                     (0 << INA220_SADC_SHIFTS)
#define INA220_SADC_10BIT                    (1 << INA220_SADC_SHIFTS)
#define INA220_SADC_11BIT                    (2 << INA220_SADC_SHIFTS)
#define INA220_SADC_12BIT                    (3 << INA220_SADC_SHIFTS)
#define INA220_SADC_SAMPLES_2                (9 << INA220_SADC_SHIFTS)
#define INA220_SADC_SAMPLES_4                (10 << INA220_SADC_SHIFTS)
#define INA220_SADC_SAMPLES_8                (11 << INA220_SADC_SHIFTS)
#define INA220_SADC_SAMPLES_16               (12 << INA220_SADC_SHIFTS)
#define INA220_SADC_SAMPLES_32               (13 << INA220_SADC_SHIFTS)
#define INA220_SADC_SAMPLES_64               (14 << INA220_SADC_SHIFTS)
#define INA220_SADC_SAMPLES_128              (15 << INA220_SADC_SHIFTS)

#define INA220_BADC_SHIFTS                   (7)
#define INA220_BADC_MASK                     (15 << INA220_BADC_SHIFTS)
#define INA220_BADC_9BIT                     (0 << INA220_BADC_SHIFTS)
#define INA220_BADC_10BIT                    (1 << INA220_BADC_SHIFTS)
#define INA220_BADC_11BIT                    (2 << INA220_BADC_SHIFTS)
#define INA220_BADC_12BIT                    (3 << INA220_BADC_SHIFTS)
#define INA220_BADC_SAMPLES_2                (9 << INA220_BADC_SHIFTS)
#define INA220_BADC_SAMPLES_4                (10 << INA220_BADC_SHIFTS)
#define INA220_BADC_SAMPLES_8                (11 << INA220_BADC_SHIFTS)
#define INA220_BADC_SAMPLES_16               (12 << INA220_BADC_SHIFTS)
#define INA220_BADC_SAMPLES_32               (13 << INA220_BADC_SHIFTS)
#define INA220_BADC_SAMPLES_64               (14 << INA220_BADC_SHIFTS)
#define INA220_BADC_SAMPLES_128              (15 << INA220_BADC_SHIFTS)

#define INA220_PG_SHIFTS                     (11)
#define INA220_PG_MASK                       (2 << INA220_PG_SHIFTS)
#define INA220_PG_40mV                       (1 << INA220_PG_SHIFTS)
#define INA220_PG_80mV                       (2 << INA220_PG_SHIFTS)
#define INA220_PG_160mV                      (3 << INA220_PG_SHIFTS)
#define INA220_PG_320mV                      (4 << INA220_PG_SHIFTS)

#define INA220_BRNG_SHIFTS                   (13)
#define INA220_BRNG_MASK                     (1 << INA220_BRNG_SHIFTS)
#define INA220_BRNG_16V                      (0 << INA220_BRNG_SHIFTS)
#define INA220_BRNG_32V                      (1 << INA220_BRNG_SHIFTS)


#define INA220_CONFIG (INA220_BRNG_32V | INA220_PG_320mV | INA220_BADC_12BIT | INA220_SADC_12BIT | INA220_MODE_SHUNT_BUS_CONT)

#define INA220_RST                            (1 << 15)

#define INA220_SAMPLE_FREQUENCY_HZ            10
#define INA220_SAMPLE_INTERVAL_US             (1_s / INA220_SAMPLE_FREQUENCY_HZ)
#define INA220_CONVERSION_INTERVAL            (INA220_SAMPLE_INTERVAL_US - 7)
#define MAX_CURRENT                           400.0f    /* 400 Amps */
#define DN_MAX                                32768.0f  /* 2^15 */
#define INA220_CONST                          0.04096f  /* is an internal fixed value used to ensure scaling is maintained properly  */
#define INA220_SHUNT                          0.0005f   /* Shunt is 500 uOhm */
#define INA220_VSCALE                         0.004f    /* LSB of voltage is 4 mV  */
#define INA220_VSHUNTSCALE		      0.01f  /* LSB of shunt voltage is 10 uV  */
#define swap16(w)                       __builtin_bswap16((w))

enum PM_CH_TYPE {
	PM_CH_TYPE_VBATT = 0,
	PM_CH_TYPE_VREG
};

class INA220 : public device::I2C, public ModuleParams, public I2CSPIDriver<INA220>
{
public:
	INA220(const I2CSPIDriverConfig &config, int battery_index);
	virtual ~INA220();

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	void	RunImpl();

	int 		  init() override;

	/**
	 * Tries to call the init() function. If it fails, then it will schedule to retry again in
	 * INA220_INIT_RETRY_INTERVAL_US microseconds. It will keep retrying at this interval until initialization succeeds.
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
	bool 				_initialized{false};

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t 		_collection_errors;
	perf_counter_t 		_measure_errors;

	int16_t           _bus_voltage{0};
	int16_t           _bus_power{0};
	int16_t           _bus_current{0};
	int16_t           _shunt{0};
	uint16_t           _cal{0};
	bool              _mode_triggered{false};

	const PM_CH_TYPE  _ch_type;

	float             _max_current{MAX_CURRENT};
	float             _rshunt{INA220_SHUNT};
	uint16_t          _config{INA220_CONFIG};
	float             _current_lsb{_max_current / DN_MAX};
	float             _power_lsb{25.0f * _current_lsb};
	float	   	  _voltage{0};
	float 		  _current{0};
	float	          _vshunt{0};


	Battery 		  _battery;
	uORB::PublicationMulti<power_monitor_s>		_pm_pub_topic{ORB_ID(power_monitor)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	power_monitor_s 	_pm_status{};

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
