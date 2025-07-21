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

/**
 * @file ina228.h
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
#define INA228_BASEADDR 	                    0x45 /* 7-bit address. 8-bit address is 0x45 */
// If initialization is forced (with the -f flag on the command line), but it fails, the drive will try again to
// connect to the INA228 every this many microseconds
#define INA228_INIT_RETRY_INTERVAL_US			500000

/* INA228 Registers addresses */
#define INA228_REG_CONFIG                    (0x00)
#define INA228_REG_ADCCONFIG                 (0x01)
#define INA228_REG_SHUNTCAL                  (0x02)
#define INA228_REG_SHUNTTEMPCO               (0x03)
#define INA228_REG_VSHUNT                    (0x04)
#define INA228_REG_VSBUS                     (0x05)
#define INA228_REG_DIETEMP                   (0x06)
#define INA228_REG_CURRENT                   (0x07)
#define INA228_REG_POWER                     (0x08)
#define INA228_REG_ENERGY                    (0x09)
#define INA228_REG_CHARGE                    (0x0a)
#define INA228_REG_DIAG_ALRT                 (0x0b)
#define INA228_REG_SOVL                      (0x0c)
#define INA228_REG_SUVL                      (0x0d)
#define INA228_REG_BOVL                      (0x0e)
#define INA228_REG_BUVL                      (0x0f)
#define INA228_REG_TEMP_LIMIT                (0x10)
#define INA228_REG_TPWR_LIMIT                (0x11)
#define INA228_MANUFACTURER_ID               (0x3e)
#define INA228_DEVICE_ID                     (0x3f)

#define INA228_MFG_ID_TI                     (0x5449) // TI
#define INA228_MFG_DIE                       (0x228) // INA228

/* INA228 Configuration (CONFIG) 16-bit Register (Address = 0h) [reset = 0h] */
#define INA228_ADCRANGE_SHIFTS               (4)
#define INA228_ADCRANGE_MASK                 (1 << INA228_ADCRANGE_SHIFTS)
#  define INA228_ADCRANGE_LOW                (1 << INA228_ADCRANGE_SHIFTS) // ± 40.96 mV
#  define INA228_ADCRANGE_HIGH               (0 << INA228_ADCRANGE_SHIFTS) // ±163.84 mV
#define INA228_TEMPCOMP_SHIFTS               (5)
#define INA228_TEMPCOMP_MASK                 (1 << INA228_TEMPCOMP_SHIFTS)
#  define INA228_TEMPCOMP_ENABLE             (1 << INA228_TEMPCOMP_SHIFTS)
#  define INA228_TEMPCOMP_DISABLE            (0 << INA228_TEMPCOMP_SHIFTS)

#define INA228_CONVDLY_SHIFTS                (6)
#define INA228_CONVDLY_MASK                  (0xff << INA228_CONVDLY_SHIFTS)
#  define INA228_CONVDLY2MS(n)               ((n)  << INA228_CONVDLY_SHIFTS)

#define INA228_RSTACC_SHIFTS                 (14)
#define INA228_RSTACC_MASK                   (1 << INA228_RSTACC_SHIFTS)
#  define INA228_RSTACC_CLEAR                (1 << INA228_RSTACC_SHIFTS)
#  define INA228_RSTACC_NORMAL               (0 << INA228_RSTACC_SHIFTS)

#define INA228_RST_SHIFTS                    (15)
#define INA228_RST_MASK                      (1 << INA228_RST_SHIFTS)
#  define INA228_RST_RESET                   (1 << INA228_RST_SHIFTS)
#  define INA228_RST_NORMAL                  (0 << INA228_RST_SHIFTS)

/* INA228 ADC Configuration (ADC_CONFIG) 16-bit  Register (Address = 1h) [reset = FB68h] */

#define INA228_MODE_SHIFTS                   (12)
#define INA228_MODE_MASK                     (0xf << INA228_MODE_SHIFTS)
#define INA228_MODE_SHUTDOWN_TRIG            (0 << INA228_MODE_SHIFTS)
#define INA228_MODE_BUS_TRIG                 (1 << INA228_MODE_SHIFTS)
#define INA228_MODE_SHUNT_TRIG               (2 << INA228_MODE_SHIFTS)
#define INA228_MODE_SHUNT_BUS_TRIG           (3 << INA228_MODE_SHIFTS)
#define INA228_MODE_TEMP_TRIG                (4 << INA228_MODE_SHIFTS)
#define INA228_MODE_TEMP_BUS_TRIG            (5 << INA228_MODE_SHIFTS)
#define INA228_MODE_TEMP_SHUNT_TRIG          (6 << INA228_MODE_SHIFTS)
#define INA228_MODE_TEMP_SHUNT_BUS_TRIG      (7 << INA228_MODE_SHIFTS)

#define INA228_MODE_SHUTDOWN_CONT            (8 << INA228_MODE_SHIFTS)
#define INA228_MODE_BUS_CONT                 (9 << INA228_MODE_SHIFTS)
#define INA228_MODE_SHUNT_CONT               (10 << INA228_MODE_SHIFTS)
#define INA228_MODE_SHUNT_BUS_CONT           (11 << INA228_MODE_SHIFTS)
#define INA228_MODE_TEMP_CONT                (12 << INA228_MODE_SHIFTS)
#define INA228_MODE_TEMP_BUS_CONT            (13 << INA228_MODE_SHIFTS)
#define INA228_MODE_TEMP_SHUNT_CONT          (14 << INA228_MODE_SHIFTS)
#define INA228_MODE_TEMP_SHUNT_BUS_CONT      (15 << INA228_MODE_SHIFTS)

#define INA228_VBUSCT_SHIFTS                 (9)
#define INA228_VBUSCT_MASK                   (7 << INA228_VBUSCT_SHIFTS)
#define INA228_VBUSCT_50US                   (0 << INA228_VBUSCT_SHIFTS)
#define INA228_VBUSCT_84US                   (1 << INA228_VBUSCT_SHIFTS)
#define INA228_VBUSCT_150US                  (2 << INA228_VBUSCT_SHIFTS)
#define INA228_VBUSCT_280US                  (3 << INA228_VBUSCT_SHIFTS)
#define INA228_VBUSCT_540US                  (4 << INA228_VBUSCT_SHIFTS)
#define INA228_VBUSCT_1052US                 (5 << INA228_VBUSCT_SHIFTS)
#define INA228_VBUSCT_2074US                 (6 << INA228_VBUSCT_SHIFTS)
#define INA228_VBUSCT_4170US                 (7 << INA228_VBUSCT_SHIFTS)

#define INA228_VSHCT_SHIFTS                  (6)
#define INA228_VSHCT_MASK                    (7 << INA228_VSHCT_SHIFTS)
#define INA228_VSHCT_50US                    (0 << INA228_VSHCT_SHIFTS)
#define INA228_VSHCT_84US                    (1 << INA228_VSHCT_SHIFTS)
#define INA228_VSHCT_150US                   (2 << INA228_VSHCT_SHIFTS)
#define INA228_VSHCT_280US                   (3 << INA228_VSHCT_SHIFTS)
#define INA228_VSHCT_540US                   (4 << INA228_VSHCT_SHIFTS)
#define INA228_VSHCT_1052US                  (5 << INA228_VSHCT_SHIFTS)
#define INA228_VSHCT_2074US                  (6 << INA228_VSHCT_SHIFTS)
#define INA228_VSHCT_4170US                  (7 << INA228_VSHCT_SHIFTS)

#define INA228_VTCT_SHIFTS                   (3)
#define INA228_VTCT_MASK                     (7 << INA228_VTCT_SHIFTS)
#define INA228_VTCT_50US                     (0 << INA228_VTCT_SHIFTS)
#define INA228_VTCT_84US                     (1 << INA228_VTCT_SHIFTS)
#define INA228_VTCT_150US                    (2 << INA228_VTCT_SHIFTS)
#define INA228_VTCT_280US                    (3 << INA228_VTCT_SHIFTS)
#define INA228_VTCT_540US                    (4 << INA228_VTCT_SHIFTS)
#define INA228_VTCT_1052US                   (5 << INA228_VTCT_SHIFTS)
#define INA228_VTCT_2074US                   (6 << INA228_VTCT_SHIFTS)
#define INA228_VTCT_4170US                   (7 << INA228_VTCT_SHIFTS)

#define INA228_AVERAGES_SHIFTS               (0)
#define INA228_AVERAGES_MASK                 (7 << INA228_AVERAGES_SHIFTS)
#define INA228_AVERAGES_1                    (0 << INA228_AVERAGES_SHIFTS)
#define INA228_AVERAGES_4                    (1 << INA228_AVERAGES_SHIFTS)
#define INA228_AVERAGES_16                   (2 << INA228_AVERAGES_SHIFTS)
#define INA228_AVERAGES_64                   (3 << INA228_AVERAGES_SHIFTS)
#define INA228_AVERAGES_128                  (4 << INA228_AVERAGES_SHIFTS)
#define INA228_AVERAGES_256                  (5 << INA228_AVERAGES_SHIFTS)
#define INA228_AVERAGES_512                  (6 << INA228_AVERAGES_SHIFTS)
#define INA228_AVERAGES_1024                 (7 << INA228_AVERAGES_SHIFTS)

#define INA228_ADCCONFIG (INA228_MODE_TEMP_SHUNT_BUS_CONT | INA228_VBUSCT_540US | INA228_VSHCT_540US | INA228_VTCT_540US |INA228_AVERAGES_64)

/* INA228 Shunt Calibration (SHUNT_CAL) 16-bit Register (Address = 2h) [reset = 1000h] */

#define INA228_CURRLSB_SHIFTS                (0)
#define INA228_CURRLSB_MASK                  (0x7fff << INA228_CURRLSB_SHIFTS)

/* INA228 Shunt Temperature Coefficient (SHUNT_TEMPCO) 16-bit Register (Address = 3h) [reset = 0h] */

#define INA228_TEMPCO_SHIFTS                 (0)
#define INA228_TEMPCO_MASK                   (0x1fff << INA228_TEMPCO_SHIFTS)

/* INA228 Shunt Voltage Measurement (VSHUNT) 24-bit Register (Address = 4h) [reset = 0h] */

#define INA228_VSHUNT_SHIFTS                 (4)
#define INA228_VSHUNT_MASK                   (UINT32_C(0xffffff) << INA228_VSHUNT_SHIFTS)

/* INA228 Bus Voltage Measurement (VBUS) 24-bit Register (Address = 5h) [reset = 0h] */

#define INA228_VBUS_SHIFTS                   (4)
#define INA228_VBUS_MASK                     (UINT32_C(0xffffff) << INA228_VBUS_SHIFTS)

/* INA228 Temperature Measurement (DIETEMP) 16-bit Register (Address = 6h) [reset = 0h] */

#define INA228_DIETEMP_SHIFTS                (0)
#define INA228_DIETEMP_MASK                  (0xffff << INA228_DIETEMP_SHIFTS)

/* INA228 Current Result (CURRENT) 24-bit Register (Address = 7h) [reset = 0h] */

#define INA228_CURRENT_SHIFTS                (4)
#define INA228_CURRENT_MASK                  (UINT32_C(0xffffff) << INA228_CURRENT_SHIFTS)

/* INA228 Power Result (POWER) 24-bit Register (Address = 8h) [reset = 0h] */

#define INA228_POWER_SHIFTS                  (0)
#define INA228_POWER_MASK                    (UINT32_C(0xffffff) << INA228_POWER_SHIFTS)

/* INA228 Energy Result (ENERGY) 40-bit Register (Address = 9h) [reset = 0h] */

#define INA228_ENERGY_SHIFTS                  (0)
#define INA228_ENERGY_MASK                    (UINT64_C(0xffffffffff) << INA228_ENERGY_SHIFTS)

/* INA228 Charge Result (CHARGE) 40-bit Register (Address = Ah) [reset = 0h] */

#define INA228_CHARGE_SHIFTS                 (0)
#define INA228_CHARGE_MASK                   (UINT64_C(0xffffffffff) << INA228_CHARGE_SHIFTS)


/* INA228 Diagnostic Flags and Alert (DIAG_ALRT) 16-bit Register (Address = Bh) [reset = 0001h] */

#define INA228_MEMSTAT                       (1 << 0)  // This bit is set to 0 if a checksum error is detected in the device trim memory space
#define INA228_CNVRF                         (1 << 1)  // This bit is set to 1 if the conversion is completed. When ALATCH =1 this bit is cleared by reading the register or starting a new triggered conversion.
#define INA228_POL                           (1 << 2)  // This bit is set to 1 if the power measurement exceeds the threshold limit in the power limit register.
#define INA228_BUSUL                         (1 << 3)  // This bit is set to 1 if the bus voltage measurement falls below the threshold limit in the bus under-limit register.
#define INA228_BUSOL                         (1 << 4)  // This bit is set to 1 if the bus voltage measurement exceeds the threshold limit in the bus over-limit register.
#define INA228_SHNTUL                        (1 << 5)  // This bit is set to 1 if the shunt voltage measurement falls below the threshold limit in the shunt under-limit register
#define INA228_SHNTOL                        (1 << 6)  // This bit is set to 1 if the shunt voltage measurement exceeds the threshold limit in the shunt over-limit register.
#define INA228_TMPOL                         (1 << 7)  // This bit is set to 1 if the temperature measurement exceeds the threshold limit in the temperature over-limit register.
#define INA228_MATHOF                        (1 << 9)  // This bit is set to 1 if an arithmetic operation resulted in an overflow error.
#define INA228_CHARGEOF                      (1 << 10) // This bit indicates the health of the CHARGE register. If the 40 bit CHARGE register has overflowed this bit is set to 1.
#define INA228_ENERGYOF                      (1 << 11) // This bit indicates the health of the ENERGY register. If the 40 bit ENERGY register has overflowed this bit is set to 1.
#define INA228_APOL                          (1 << 12) // Alert Polarity bit sets the Alert pin polarity.
#define INA228_SLOWALER                      (1 << 13) // ALERT function is asserted on the completed averaged value.  This gives the flexibility to delay the ALERT after the averaged value.
#define INA228_CNVR                          (1 << 14) // Setting this bit high configures the Alert pin to be asserted when the Conversion Ready Flag (bit 1) is asserted, indicating that a conversion cycle has completed
#define INA228_ALATCH                        (1 << 15) // When the Alert Latch Enable bit is set to Transparent mode, the Alert pin and Flag bit reset to the idle state when the fault has been
// cleared. When the Alert Latch Enable bit is set to Latch mode, the Alert pin and Alert Flag bit remain active following a fault until
// the DIAG_ALRT Register has been read.

/* Shunt Overvoltage Threshold (SOVL) 16-bit Register (Address = Ch) [reset = 7FFFh] */

#define INA228_SOVL_SHIFTS                   (0)
#define INA228_SOVL_MASK                     (0xffff << INA228_SOVL_SHIFTS)

/* Shunt Undervoltage Threshold (SUVL) 16-bit Register (Address = Dh) [reset = 8000h] */

#define INA228_SUVL_SHIFTS                   (0)
#define INA228_SUVL_MASK                     (0xffff << INA228_SUVL_SHIFTS)

/* Bus Overvoltage Threshold (BOVL) 16-bit Register (Address = Eh) [reset = 7FFFh] */

#define INA228_BOVL_SHIFTS                   (0)
#define INA228_BOVL_MASK                     (0xffff << INA228_BOVL_SHIFTS)

/* Bus Undervoltage Threshold (BUVL) 16-bit Register (Address = Fh) [reset = 0h] */

#define INA228_BUVL_SHIFTS                   (0)
#define INA228_BUVL_MASK                     (0xffff << INA228_BUVL_SHIFTS)

/* Temperature Over-Limit Threshold (TEMP_LIMIT) 16-bit Register (Address = 10h) [reset = 7FFFh */

#define INA228_TEMP_LIMIT_SHIFTS             (0)
#define INA228_TEMP_LIMIT_MASK               (0xffff << INA228_TEMP_LIMIT_SHIFTS)

/* Power Over-Limit Threshold (PWR_LIMIT) 16-bit Register (Address = 11h) [reset = FFFFh] */

#define INA228_POWER_LIMIT_SHIFTS            (0)
#define INA228_POWER_LIMIT_MASK              (0xffff << INA228_POWER_LIMIT_SHIFTS)

/* Manufacturer ID (MANUFACTURER_ID) 16-bit Register (Address = 3Eh) [reset = 5449h] */

/* Device ID (DEVICE_ID) 16-bit Register (Address = 3Fh) [reset = 2280h] */

#define INA228_DEVICE_REV_ID_SHIFTS          (0)
#define INA228_DEVICE_REV_ID_MASK            (0xf << INA228_DEVICE_REV_ID_SHIFTS)
#define INA228_DEVICEREV_ID(v)               (((v) & INA228_DEVICE_REV_ID_MASK) >> INA228_DEVICE_REV_ID_SHIFTS)
#define INA228_DEVICE_ID_SHIFTS              (4)
#define INA228_DEVICE_ID_MASK                (0xfff << INA228_DEVICE_ID_SHIFTS)
#define INA228_DEVICEID(v)                   (((v) & INA228_DEVICE_ID_MASK) >> INA228_DEVICE_ID_SHIFTS)



#define INA228_SAMPLE_FREQUENCY_HZ           10
#define INA228_SAMPLE_INTERVAL_US            (1_s / INA228_SAMPLE_FREQUENCY_HZ)
#define INA228_CONVERSION_INTERVAL           (INA228_SAMPLE_INTERVAL_US - 7)
#define MAX_CURRENT                          327.68f    /* Amps */
#define DN_MAX                               524288.0f  /* 2^19 */
#define INA228_CONST                         13107.2e6f  /* is an internal fixed value used to ensure scaling is maintained properly  */
#define INA228_SHUNT                         0.0005f   /* Shunt is 500 uOhm */
#define INA228_VSCALE                        1.95e-04f  /* LSB of voltage is 195.3125 uV/LSB */
#define INA228_TSCALE                        7.8125e-03f /* LSB of temperature is 7.8125 mDegC/LSB */

#define INA228_ADCRANGE_LOW_V_SENSE          0.04096f // ± 40.96 mV

#define swap16(w)                            __builtin_bswap16((w))
#define swap32(d)                            __builtin_bswap32((d))
#define swap64(q)                            __builtin_bswap64((q))

class INA228 : public device::I2C, public ModuleParams, public I2CSPIDriver<INA228>
{
public:
	INA228(const I2CSPIDriverConfig &config, int battery_index);
	virtual ~INA228();

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	void	RunImpl();

	int 		  init() override;

	/**
	 * Tries to call the init() function. If it fails, then it will schedule to retry again in
	 * INA228_INIT_RETRY_INTERVAL_US microseconds. It will keep retrying at this interval until initialization succeeds.
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

	int32_t           _bus_voltage{0};
	int64_t           _power{0};
	int32_t           _current{0};
	int16_t           _temperature{0};
	int32_t           _shunt{0};
	int16_t           _cal{0};
	int16_t           _range{INA228_ADCRANGE_HIGH};
	bool              _mode_triggered{false};

	float             _max_current{MAX_CURRENT};
	float             _rshunt{INA228_SHUNT};
	uint16_t          _config{INA228_ADCCONFIG};
	float             _current_lsb{_max_current / DN_MAX};
	float             _power_lsb{25.0f * _current_lsb};

	Battery 		  _battery;
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	int read(uint8_t address, int16_t &data);
	int write(uint8_t address, int16_t data);

	int read(uint8_t address, uint16_t &data);
	int write(uint8_t address, uint16_t data);

	int read(uint8_t address, int32_t &data);
	int write(uint8_t address, int32_t data);

	int read(uint8_t address, int64_t &data);
	int write(uint8_t address, int64_t data);

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
