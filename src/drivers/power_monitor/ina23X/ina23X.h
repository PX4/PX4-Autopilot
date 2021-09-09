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
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace time_literals;

/* Configuration Constants */
#define INA23X_BASEADDR 	            0x45 /* 7-bit address. 8-bit address is 0x45 */
// If initialization is forced (with the -f flag on the command line), but it fails, the drive will try again to
// connect to the INA23X every this many microseconds
#define INA23X_INIT_RETRY_INTERVAL_US       500000

/* INA23X Registers addresses */
#define INA23X_REG_CONFIG                   (0x00)
#define INA23X_REG_ADCCONFIG                (0x01)
#define INA23X_REG_SHUNTCAL                 (0x02)
#define INA23X_REG_SHUNTTEMPCO              (0x03)
#define INA23X_REG_VSHUNT                   (0x04)
#define INA23X_REG_VSBUS                    (0x05)
#define INA23X_REG_DIETEMP                  (0x06)
#define INA23X_REG_CURRENT                  (0x07)
#define INA23X_REG_POWER                    (0x08)
#define INA23X_REG_ENERGY                   (0x09)
#define INA23X_REG_CHARGE                   (0x0a)
#define INA23X_REG_DIAG_ALRT                (0x0b)
#define INA23X_REG_SOVL                     (0x0c)
#define INA23X_REG_SUVL                     (0x0d)
#define INA23X_REG_BOVL                     (0x0e)
#define INA23X_REG_BUVL                     (0x0f)
#define INA23X_REG_TEMP_LIMIT               (0x10)
#define INA23X_REG_TPWR_LIMIT               (0x11)
#define INA23X_MANUFACTURER_ID              (0x3e)
#define INA23X_DEVICE_ID                    (0x3f)

#define INA23X_MFG_ID_TI                    (0x5449) // TI
#define INA238_MFG_DIE                      (0x238) // INA237, INA238
#define INA239_MFG_DIE                      (0x239) // INA239

/* INA23X Configuration (CONFIG) 16-bit Register (Address = 0h) [reset = 0h] */
#define INA23X_ADCRANGE_SHIFTS              (4)
#define INA23X_ADCRANGE_MASK                (1 << INA23X_ADCRANGE_SHIFTS)
#define INA23X_ADCRANGE_LOW                 (1 << INA23X_ADCRANGE_SHIFTS) // ± 40.96 mV
#define INA23X_ADCRANGE_HIGH                (0 << INA23X_ADCRANGE_SHIFTS) // ±163.84 mV
#define INA23X_TEMPCOMP_SHIFTS              (5)
#define INA23X_TEMPCOMP_MASK                (1 << INA23X_TEMPCOMP_SHIFTS)
#define INA23X_TEMPCOMP_ENABLE              (1 << INA23X_TEMPCOMP_SHIFTS)
#define INA23X_TEMPCOMP_DISABLE             (0 << INA23X_TEMPCOMP_SHIFTS)

#define INA23X_CONVDLY_SHIFTS               (6)
#define INA23X_CONVDLY_MASK                 (0xff << INA23X_CONVDLY_SHIFTS)
#define INA23X_CONVDLY2MS(n)                ((n)  << INA23X_CONVDLY_SHIFTS)

#define INA23X_RSTACC_SHIFTS                (14)
#define INA23X_RSTACC_MASK                  (1 << INA23X_RSTACC_SHIFTS)
#define INA23X_RSTACC_CLEAR                 (1 << INA23X_RSTACC_SHIFTS)
#define INA23X_RSTACC_NORMAL                (0 << INA23X_RSTACC_SHIFTS)

#define INA23X_RST_SHIFTS                   (15)
#define INA23X_RST_MASK                     (1 << INA23X_RST_SHIFTS)
#define INA23X_RST_RESET                    (1 << INA23X_RST_SHIFTS)
#define INA23X_RST_NORMAL                   (0 << INA23X_RST_SHIFTS)

/* INA23X ADC Configuration (ADC_CONFIG) 16-bit  Register (Address = 1h) [reset = FB68h] */
#define INA23X_MODE_SHIFTS                  (12)
#define INA23X_MODE_MASK                    (0xf << INA23X_MODE_SHIFTS)
#define INA23X_MODE_SHUTDOWN_TRIG           (0 << INA23X_MODE_SHIFTS)
#define INA23X_MODE_BUS_TRIG                (1 << INA23X_MODE_SHIFTS)
#define INA23X_MODE_SHUNT_TRIG              (2 << INA23X_MODE_SHIFTS)
#define INA23X_MODE_SHUNT_BUS_TRIG          (3 << INA23X_MODE_SHIFTS)
#define INA23X_MODE_TEMP_TRIG               (4 << INA23X_MODE_SHIFTS)
#define INA23X_MODE_TEMP_BUS_TRIG           (5 << INA23X_MODE_SHIFTS)
#define INA23X_MODE_TEMP_SHUNT_TRIG         (6 << INA23X_MODE_SHIFTS)
#define INA23X_MODE_TEMP_SHUNT_BUS_TRIG     (7 << INA23X_MODE_SHIFTS)

#define INA23X_MODE_SHUTDOWN_CONT           (8 << INA23X_MODE_SHIFTS)
#define INA23X_MODE_BUS_CONT                (9 << INA23X_MODE_SHIFTS)
#define INA23X_MODE_SHUNT_CONT              (10 << INA23X_MODE_SHIFTS)
#define INA23X_MODE_SHUNT_BUS_CONT          (11 << INA23X_MODE_SHIFTS)
#define INA23X_MODE_TEMP_CONT               (12 << INA23X_MODE_SHIFTS)
#define INA23X_MODE_TEMP_BUS_CONT           (13 << INA23X_MODE_SHIFTS)
#define INA23X_MODE_TEMP_SHUNT_CONT         (14 << INA23X_MODE_SHIFTS)
#define INA23X_MODE_TEMP_SHUNT_BUS_CONT     (15 << INA23X_MODE_SHIFTS)

#define INA23X_VBUSCT_SHIFTS                (9)
#define INA23X_VBUSCT_MASK                  (7 << INA23X_VBUSCT_SHIFTS)
#define INA23X_VBUSCT_50US                  (0 << INA23X_VBUSCT_SHIFTS)
#define INA23X_VBUSCT_84US                  (1 << INA23X_VBUSCT_SHIFTS)
#define INA23X_VBUSCT_150US                 (2 << INA23X_VBUSCT_SHIFTS)
#define INA23X_VBUSCT_280US                 (3 << INA23X_VBUSCT_SHIFTS)
#define INA23X_VBUSCT_540US                 (4 << INA23X_VBUSCT_SHIFTS)
#define INA23X_VBUSCT_1052US                (5 << INA23X_VBUSCT_SHIFTS)
#define INA23X_VBUSCT_2074US                (6 << INA23X_VBUSCT_SHIFTS)
#define INA23X_VBUSCT_4170US                (7 << INA23X_VBUSCT_SHIFTS)

#define INA23X_VSHCT_SHIFTS                 (6)
#define INA23X_VSHCT_MASK                   (7 << INA23X_VSHCT_SHIFTS)
#define INA23X_VSHCT_50US                   (0 << INA23X_VSHCT_SHIFTS)
#define INA23X_VSHCT_84US                   (1 << INA23X_VSHCT_SHIFTS)
#define INA23X_VSHCT_150US                  (2 << INA23X_VSHCT_SHIFTS)
#define INA23X_VSHCT_280US                  (3 << INA23X_VSHCT_SHIFTS)
#define INA23X_VSHCT_540US                  (4 << INA23X_VSHCT_SHIFTS)
#define INA23X_VSHCT_1052US                 (5 << INA23X_VSHCT_SHIFTS)
#define INA23X_VSHCT_2074US                 (6 << INA23X_VSHCT_SHIFTS)
#define INA23X_VSHCT_4170US                 (7 << INA23X_VSHCT_SHIFTS)

#define INA23X_VTCT_SHIFTS                  (3)
#define INA23X_VTCT_MASK                    (7 << INA23X_VTCT_SHIFTS)
#define INA23X_VTCT_50US                    (0 << INA23X_VTCT_SHIFTS)
#define INA23X_VTCT_84US                    (1 << INA23X_VTCT_SHIFTS)
#define INA23X_VTCT_150US                   (2 << INA23X_VTCT_SHIFTS)
#define INA23X_VTCT_280US                   (3 << INA23X_VTCT_SHIFTS)
#define INA23X_VTCT_540US                   (4 << INA23X_VTCT_SHIFTS)
#define INA23X_VTCT_1052US                  (5 << INA23X_VTCT_SHIFTS)
#define INA23X_VTCT_2074US                  (6 << INA23X_VTCT_SHIFTS)
#define INA23X_VTCT_4170US                  (7 << INA23X_VTCT_SHIFTS)

#define INA23X_AVERAGES_SHIFTS              (0)
#define INA23X_AVERAGES_MASK                (7 << INA23X_AVERAGES_SHIFTS)
#define INA23X_AVERAGES_1                   (0 << INA23X_AVERAGES_SHIFTS)
#define INA23X_AVERAGES_4                   (1 << INA23X_AVERAGES_SHIFTS)
#define INA23X_AVERAGES_16                  (2 << INA23X_AVERAGES_SHIFTS)
#define INA23X_AVERAGES_64                  (3 << INA23X_AVERAGES_SHIFTS)
#define INA23X_AVERAGES_128                 (4 << INA23X_AVERAGES_SHIFTS)
#define INA23X_AVERAGES_256                 (5 << INA23X_AVERAGES_SHIFTS)
#define INA23X_AVERAGES_512                 (6 << INA23X_AVERAGES_SHIFTS)
#define INA23X_AVERAGES_1024                (7 << INA23X_AVERAGES_SHIFTS)

#define INA23X_ADCCONFIG (INA23X_MODE_TEMP_SHUNT_BUS_CONT | INA23X_VBUSCT_540US | INA23X_VSHCT_540US | INA23X_VTCT_540US |INA23X_AVERAGES_64)

/* INA23X Shunt Calibration (SHUNT_CAL) 16-bit Register (Address = 2h) [reset = 1000h] */

#define INA23X_CURRLSB_SHIFTS               (0)
#define INA23X_CURRLSB_MASK                 (0x7fff << INA23X_CURRLSB_SHIFTS)

/* INA23X Shunt Temperature Coefficient (SHUNT_TEMPCO) 16-bit Register (Address = 3h) [reset = 0h] */

#define INA23X_TEMPCO_SHIFTS                (0)
#define INA23X_TEMPCO_MASK                  (0x1fff << INA23X_TEMPCO_SHIFTS)

/* INA23X Shunt Voltage Measurement (VSHUNT) 24-bit Register (Address = 4h) [reset = 0h] */

#define INA23X_VSHUNT_SHIFTS                (4)
#define INA23X_VSHUNT_MASK                  (UINT32_C(0xffffff) << INA23X_VSHUNT_SHIFTS)

/* INA23X Bus Voltage Measurement (VBUS) 24-bit Register (Address = 5h) [reset = 0h] */

#define INA23X_VBUS_SHIFTS                  (4)
#define INA23X_VBUS_MASK                    (UINT32_C(0xffffff) << INA23X_VBUS_SHIFTS)

/* INA23X Temperature Measurement (DIETEMP) 16-bit Register (Address = 6h) [reset = 0h] */

#define INA23X_DIETEMP_SHIFTS               (0)
#define INA23X_DIETEMP_MASK                 (0xffff << INA23X_DIETEMP_SHIFTS)

/* INA23X Current Result (CURRENT) 24-bit Register (Address = 7h) [reset = 0h] */

#define INA23X_CURRENT_SHIFTS               (4)
#define INA23X_CURRENT_MASK                 (UINT32_C(0xffffff) << INA23X_CURRENT_SHIFTS)

/* INA23X Power Result (POWER) 24-bit Register (Address = 8h) [reset = 0h] */

#define INA23X_POWER_SHIFTS                 (0)
#define INA23X_POWER_MASK                   (UINT32_C(0xffffff) << INA23X_POWER_SHIFTS)

/* INA23X Energy Result (ENERGY) 40-bit Register (Address = 9h) [reset = 0h] */

#define INA23X_ENERGY_SHIFTS                 (0)
#define INA23X_ENERGY_MASK                   (UINT64_C(0xffffffffff) << INA23X_ENERGY_SHIFTS)

/* INA23X Energy Result (CHARGE) 40-bit Register (Address = Ah) [reset = 0h] */

#define INA23X_CHARGE_SHIFTS                (0)
#define INA23X_CHARGE_MASK                  (UINT64_C(0xffffffffff) << INA23X_CHARGE_SHIFTS)


/* INA23X Diagnostic Flags and Alert (DIAG_ALRT) 16-bit Register (Address = Bh) [reset = 0001h] */

#define INA23X_MEMSTAT                      (1 << 0)  // This bit is set to 0 if a checksum error is detected in the device trim memory space
#define INA23X_CNVRF                        (1 << 1)  // This bit is set to 1 if the conversion is completed. When ALATCH =1 this bit is cleared by reading the register or starting a new triggered conversion.
#define INA23X_POL                          (1 << 2)  // This bit is set to 1 if the power measurement exceeds the threshold limit in the power limit register.
#define INA23X_BUSUL                        (1 << 3)  // This bit is set to 1 if the bus voltage measurement falls below the threshold limit in the bus under-limit register.
#define INA23X_BUSOL                        (1 << 4)  // This bit is set to 1 if the bus voltage measurement exceeds the threshold limit in the bus over-limit register.
#define INA23X_SHNTUL                       (1 << 5)  // This bit is set to 1 if the shunt voltage measurement falls below the threshold limit in the shunt under-limit register
#define INA23X_SHNTOL                       (1 << 6)  // This bit is set to 1 if the shunt voltage measurement exceeds the threshold limit in the shunt over-limit register.
#define INA23X_TMPOL                        (1 << 7)  // This bit is set to 1 if the temperature measurement exceeds the threshold limit in the temperature over-limit register.
#define INA23X_MATHOF                       (1 << 9)  // This bit is set to 1 if an arithmetic operation resulted in an overflow error.
#define INA23X_CHARGEOF                     (1 << 10) // This bit indicates the health of the CHARGE register. If the 40 bit CHARGE register has overflowed this bit is set to 1.
#define INA23X_ENERGYOF                     (1 << 11) // This bit indicates the health of the ENERGY register. If the 40 bit ENERGY register has overflowed this bit is set to 1.
#define INA23X_APOL                         (1 << 12) // Alert Polarity bit sets the Alert pin polarity.
#define INA23X_SLOWALER                     (1 << 13) // ALERT function is asserted on the completed averaged value.  This gives the flexibility to delay the ALERT after the averaged value.
#define INA23X_CNVR                         (1 << 14) // Setting this bit high configures the Alert pin to be asserted when the Conversion Ready Flag (bit 1) is asserted, indicating that a conversion cycle has completed
#define INA23X_ALATCH                       (1 << 15) // When the Alert Latch Enable bit is set to Transparent mode, the Alert pin and Flag bit reset to the idle state when the fault has been
// cleared. When the Alert Latch Enable bit is set to Latch mode, the Alert pin and Alert Flag bit remain active following a fault until
// the DIAG_ALRT Register has been read.

/* Shunt Overvoltage Threshold (SOVL) 16-bit Register (Address = Ch) [reset = 7FFFh] */

#define INA23X_SOVL_SHIFTS                   (0)
#define INA23X_SOVL_MASK                     (0xffff << INA23X_SOVL_SHIFTS)

/* Shunt Undervoltage Threshold (SUVL) 16-bit Register (Address = Dh) [reset = 8000h] */

#define INA23X_SUVL_SHIFTS                   (0)
#define INA23X_SUVL_MASK                     (0xffff << INA23X_SUVL_SHIFTS)

/* Bus Overvoltage Threshold (BOVL) 16-bit Register (Address = Eh) [reset = 7FFFh] */

#define INA23X_BOVL_SHIFTS                   (0)
#define INA23X_BOVL_MASK                     (0xffff << INA23X_BOVL_SHIFTS)

/* Bus Undervoltage Threshold (BUVL) 16-bit Register (Address = Fh) [reset = 0h] */

#define INA23X_BUVL_SHIFTS                   (0)
#define INA23X_BUVL_MASK                     (0xffff << INA23X_BUVL_SHIFTS)

/* Temperature Over-Limit Threshold (TEMP_LIMIT) 16-bit Register (Address = 10h) [reset = 7FFFh */

#define INA23X_TEMP_LIMIT_SHIFTS             (0)
#define INA23X_TEMP_LIMIT_MASK               (0xffff << INA23X_TEMP_LIMIT_SHIFTS)

/* Power Over-Limit Threshold (PWR_LIMIT) 16-bit Register (Address = 11h) [reset = FFFFh] */

#define INA23X_POWER_LIMIT_SHIFTS            (0)
#define INA23X_POWER_LIMIT_MASK              (0xffff << INA23X_POWER_LIMIT_SHIFTS)

/* Manufacturer ID (MANUFACTURER_ID) 16-bit Register (Address = 3Eh) [reset = 5449h] */

/* Device ID (DEVICE_ID) 16-bit Register (Address = 3Fh) [reset = 23X0h] */

#define INA23X_DEVICE_REV_ID_SHIFTS          (0)
#define INA23X_DEVICE_REV_ID_MASK            (0xf << INA23X_DEVICE_REV_ID_SHIFTS)
#define INA23X_DEVICEREV_ID(v)               (((v) & INA23X_DEVICE_REV_ID_MASK) >> INA23X_DEVICE_REV_ID_SHIFTS)
#define INA23X_DEVICE_ID_SHIFTS              (4)
#define INA23X_DEVICE_ID_MASK                (0xfff << INA23X_DEVICE_ID_SHIFTS)
#define INA23X_DEVICEID(v)                   (((v) & INA23X_DEVICE_ID_MASK) >> INA23X_DEVICE_ID_SHIFTS)

#define INA23X_SAMPLE_FREQUENCY_HZ           10
#define INA23X_SAMPLE_INTERVAL_US            (1_s / INA23X_SAMPLE_FREQUENCY_HZ)
#define INA23X_CONVERSION_INTERVAL           (INA23X_SAMPLE_INTERVAL_US - 7)
#define INA23X_DN_MAX                        32768.0f   /* 2^15 */
#define INA23X_CONST                         819.2e6f  /* is an internal fixed value used to ensure scaling is maintained properly  */
#define INA23X_VSCALE                        3.125e-03f  /* LSB of voltage is 3.1255 mV/LSB */


#define DEFAULT_MAX_CURRENT                  327.68f    /* Amps */
#define DEFAULT_SHUNT                        0.0003f   /* Shunt is 300 uOhm */

#define swap16(w)                            __builtin_bswap16((w))
#define swap32(d)                            __builtin_bswap32((d))
#define swap64(q)                            __builtin_bswap64((q))

class INA23X : public device::I2C, public ModuleParams, public I2CSPIDriver<INA23X>
{
public:
	INA23X(const I2CSPIDriverConfig &config, int battery_index);
	virtual ~INA23X();

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	void RunImpl();

	int init() override;

	/**
	 * Tries to call the init() function. If it fails, then it will schedule to retry again in
	 * INA23X_INIT_RETRY_INTERVAL_US microseconds. It will keep retrying at this interval until initialization succeeds.
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
	bool _sensor_ok{false};
	unsigned int _measure_interval{0};
	bool _collect_phase{false};
	bool _initialized{false};

	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;
	perf_counter_t _collection_errors;
	perf_counter_t _measure_errors;

	// Configuration state, computed from params
	float _max_current;
	float _rshunt;
	float _current_lsb;
	uint16_t _config;
	int16_t _range;
	bool _mode_triggered;

	actuator_controls_s _actuator_controls{};

	Battery _battery;
	uORB::Subscription _actuators_sub{ORB_ID(actuator_controls_0)};
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

	int measure();
	int collect();

};
