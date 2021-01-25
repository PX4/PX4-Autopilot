/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file batt_smbus.h
 *
 * Header for a battery monitor connected via SMBus (I2C).
 * Designed for BQ40Z50-R1/R2
 *
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 */

#pragma once

#include <ecl/geo/geo.h>
#include <lib/drivers/smbus/SMBus.hpp>
#include <mathlib/mathlib.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/topics/battery_status.h>

#include <board_config.h>

#define MAC_DATA_BUFFER_SIZE                            32

#define BATT_CELL_VOLTAGE_THRESHOLD_RTL                 0.5f            ///< Threshold in volts to RTL if cells are imbalanced
#define BATT_CELL_VOLTAGE_THRESHOLD_FAILED              1.5f            ///< Threshold in volts to Land if cells are imbalanced

#define BATT_CURRENT_UNDERVOLTAGE_THRESHOLD             5.0f            ///< Threshold in amps to disable undervoltage protection
#define BATT_VOLTAGE_UNDERVOLTAGE_THRESHOLD             3.4f            ///< Threshold in volts to re-enable undervoltage protection

#define BATT_SMBUS_ADDR                                 0x0B            ///< Default 7 bit address I2C address. 8 bit = 0x16

#define BATT_SMBUS_CURRENT                              0x0A            ///< current register
#define BATT_SMBUS_AVERAGE_CURRENT                      0x0B            ///< average current register
#define BATT_SMBUS_MAX_ERROR				0x0C		///< max error
#define BATT_SMBUS_RELATIVE_SOC				0x0D		///< Relative State Of Charge
#define BATT_SMBUS_TEMP                                 0x08            ///< temperature register
#define BATT_SMBUS_VOLTAGE                              0x09            ///< voltage register
#define BATT_SMBUS_FULL_CHARGE_CAPACITY                 0x10            ///< capacity when fully charged
#define BATT_SMBUS_RUN_TIME_TO_EMPTY                    0x11            ///< predicted remaining battery capacity based on the present rate of discharge in min
#define BATT_SMBUS_AVERAGE_TIME_TO_EMPTY                0x12            ///< predicted remaining battery capacity based on the present rate of discharge in min
#define BATT_SMBUS_REMAINING_CAPACITY                   0x0F            ///< predicted remaining battery capacity as mAh
#define BATT_SMBUS_RELATIVE_SOC				0x0D		///< predicted remaining battery capacity as a percentage
#define BATT_SMBUS_CYCLE_COUNT                          0x17            ///< number of cycles the battery has experienced
#define BATT_SMBUS_DESIGN_CAPACITY                      0x18            ///< design capacity register
#define BATT_SMBUS_DESIGN_VOLTAGE                       0x19            ///< design voltage register
#define BATT_SMBUS_MANUFACTURER_NAME                    0x20            ///< manufacturer name
#define BATT_SMBUS_MANUFACTURE_DATE                     0x1B            ///< manufacture date register
#define BATT_SMBUS_SERIAL_NUMBER                        0x1C            ///< serial number register
#define BATT_SMBUS_MEASUREMENT_INTERVAL_US              100000          ///< time in microseconds, measure at 10Hz
#define BATT_SMBUS_MANUFACTURER_ACCESS                  0x00
#define BATT_SMBUS_MANUFACTURER_DATA                    0x23
#define BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS            0x44
#define BATT_SMBUS_STATE_OF_HEALTH			0x4F		///< State of Health. The SOH information of the battery in percentage of Design Capacity
#define BATT_SMBUS_SECURITY_KEYS                        0x0035
#define BATT_SMBUS_CELL_1_VOLTAGE                       0x3F
#define BATT_SMBUS_CELL_2_VOLTAGE                       0x3E
#define BATT_SMBUS_CELL_3_VOLTAGE                       0x3D
#define BATT_SMBUS_CELL_4_VOLTAGE                       0x3C
#define BATT_SMBUS_CELL_5_VOLTAGE                       0x3B
#define BATT_SMBUS_CELL_6_VOLTAGE                       0x3A
#define BATT_SMBUS_CELL_7_VOLTAGE                       0x39
#define BATT_SMBUS_CELL_8_VOLTAGE                       0x38
#define BATT_SMBUS_CELL_9_VOLTAGE                       0x37
#define BATT_SMBUS_CELL_10_VOLTAGE                      0x36

#define BATT_SMBUS_CELL_COUNT                           0x40            // < This is not a default register in the BQ40Z50 chip, but one that is really needed
#define BATT_SMBUS_SAFETY_ALERT                         0x50            ///32 alert bits, threshold exceeded (used for burst current check)
#define BATT_SMBUS_SAFETY_STATUS                        0x51            ///32 status bits, threshold exceeded for certain duration
#define BATT_SMBUS_PF_ALERT                             0x52            ///32 permanent fail bits, issue warranting permanent shutoff occurred (used for cell voltage imbalance check)

#define SMART_BATTERY_ROTOYE_BATMON 1
#define SMART_BATTERY_BQ40Zx50 2
#define BATT_SMBUS_LIFETIME_FLUSH                       0x002E
#define BATT_SMBUS_LIFETIME_BLOCK_ONE                   0x0060
#define BATT_SMBUS_ENABLED_PROTECTIONS_A_ADDRESS        0x4938
#define BATT_SMBUS_SEAL                                 0x0030

#define BATT_SMBUS_ENABLED_PROTECTIONS_A_DEFAULT        0xcf
#define BATT_SMBUS_ENABLED_PROTECTIONS_A_CUV_DISABLED   0xce

#define MAX_CELL_COUNT 10

class BATT_SMBUS : public I2CSPIDriver<BATT_SMBUS>
{

struct BATT_SMBUS_Safety_Status
{
	uint8_t len = 4;
	union
	{
		struct
		{
			uint8_t _rsvd_31:1;
			uint8_t _rsvd_30:1;
			uint8_t FLAG_DISCHARGE_OVERCURRENT:1;
			uint8_t FLAG_CELL_OVERVOLTAGE_LATCH:1;
			uint8_t FLAG_DISCHARGE_UNDERTEMP:1;
			uint8_t FLAG_CHARGE_UNDERTEMP:1;
			uint8_t FLAG_OVERPRECHARGE_CURRENT:1;
			uint8_t FLAG_OVERCHARGE_VOLTAGE:1;
			uint8_t FLAG_OVERCHARGE_CURRENT:1;
			uint8_t FLAG_OVERCHARGE:1;
			uint8_t _rsvd_21:1;
			uint8_t FLAG_CHARGE_TIMEOUT:1;
			uint8_t _rsvd_19:1;
			uint8_t FLAG_PRECHARGE_TIMEOUT:1;
			uint8_t _rsvd_17:1;
			uint8_t FLAG_FET_OVERTEMP:1;
			uint8_t _rsvd_15:1;
			uint8_t FLAG_CELL_UNDERVOLTAGE_COMPENSATED:1;
			uint8_t FLAG_DISCHARGE_OVERTEMP:1;
			uint8_t FLAG_CHARGE_OVERTEMP:1;
			uint8_t FLAG_DISHCARGE_LATCH_SHORT_CIRCUIT:1;
			uint8_t FLAG_DISCHARGE_SHORT_CIRCUIT:1;
			uint8_t FLAG_CHARGE_LATCH_SHORT_CIRCUIT:1;
			uint8_t FLAG_CHARGE_SHORT_CIRCUIT:1;
			uint8_t FLAG_DISCHARGE_LATCH_OVERLOAD:1;
			uint8_t FLAG_DISCHARGE_OVERLOAD:1;
			uint8_t FLAG_DISCHARGE_OVERCURRENT_2:1;
			uint8_t FLAG_DISCHARGE_OVERCURRENT_1:1;
			uint8_t FLAG_CHARGE_OVERCURRENT_2:1;
			uint8_t FLAG_CHARGE_OVERCURRENT_1:1;
			uint8_t FLAG_CELL_OVERVOLTAGE:1;
			uint8_t FLAG_CELL_UNDERVOLTAGE:1;
		}flags;
		uint32_t data;
	}flag;
	uint8_t crc;
};

/*
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

enum BATT_SMBUS_BUS {
	BATT_SMBUS_BUS_ALL = 0,
	BATT_SMBUS_BUS_I2C_INTERNAL,
	BATT_SMBUS_BUS_I2C_EXTERNAL,
	BATT_SMBUS_BUS_I2C_EXTERNAL1,
	BATT_SMBUS_BUS_I2C_EXTERNAL2
};

struct batt_smbus_bus_option {
	enum BATT_SMBUS_BUS busid;
	const char *devpath;
	uint8_t busnum;
} bus_options[] = {
	{ BATT_SMBUS_BUS_I2C_EXTERNAL, "/dev/batt_smbus_ext", PX4_I2C_BUS_EXPANSION},
#ifdef PX4_I2C_BUS_EXPANSION1
	{ BATT_SMBUS_BUS_I2C_EXTERNAL1, "/dev/batt_smbus_ext1", PX4_I2C_BUS_EXPANSION1},
#endif
#ifdef PX4_I2C_BUS_EXPANSION2
	{ BATT_SMBUS_BUS_I2C_EXTERNAL2, "/dev/batt_smbus_ext2", PX4_I2C_BUS_EXPANSION2},
#endif
#ifdef PX4_I2C_BUS_ONBOARD
	{ BATT_SMBUS_BUS_I2C_INTERNAL, "/dev/batt_smbus_int", PX4_I2C_BUS_ONBOARD},
#endif
};*/

public:
	BATT_SMBUS(I2CSPIBusOption bus_option, const int bus, SMBus *interface);

	~BATT_SMBUS();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	friend SMBus;

	virtual void RunImpl() = 0;

	/**
	 * @brief Returns the SBS serial number of the battery device.
	 * @return Returns the SBS serial number of the battery device.
	 */
	uint16_t get_serial_number();

	/**
	* @brief Read info from battery on startup.
	* @return Returns PX4_OK on success, PX4_ERROR on failure.
	*/
	virtual int get_startup_info() = 0;

	/**
	 * @brief Gets the SBS manufacture date of the battery.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int manufacture_date();

	/**
	 * @brief Gets the SBS manufacturer name of the battery device.
	 * @param manufacturer_name Pointer to a buffer into which the manufacturer name is to be written.
	 * @param max_length The maximum number of bytes to attempt to read from the manufacturer name register,
	 *                   including the null character that is appended to the end.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int manufacturer_name(uint8_t *manufacturer_name, const uint8_t length);

	/**
	 * @brief Reads the cell voltages.
	 * @return Returns PX4_OK on success or associated read error code on failure.
	 */
	int get_cell_voltages();

	/**
	 * @brief Enables or disables the cell under voltage protection emergency shut off.
	 */

	void suspend();

	void resume();

protected:

	SMBus *_interface;

	perf_counter_t _cycle{perf_alloc(PC_ELAPSED, "batt_smbus_cycle")};

	float _cell_voltages[MAX_CELL_COUNT] = {};

	float _max_cell_voltage_delta{0};

	float _min_cell_voltage{0};

	/** @param _last_report Last published report, used for test(). */
	battery_status_s _last_report{};

	/** @param _batt_topic uORB battery topic. */
	orb_advert_t _batt_topic{nullptr};

	/** @param _cell_count Number of series cell. */
	uint16_t _cell_count;

	/** @param _batt_capacity Battery design capacity in mAh (0 means unknown). */
	uint16_t _batt_capacity{0};

	/** @param _batt_startup_capacity Battery remaining capacity in mAh on startup. */
	uint16_t _batt_startup_capacity{0};

	/** @param _cycle_count The number of cycles the battery has experienced. */
	uint16_t _cycle_count{0};

	/** @param _serial_number Serial number register. */
	uint16_t _serial_number{0};

	/** @param _crit_thr Critical battery threshold param. */
	float _crit_thr{0.f};

	/** @param _emergency_thr Emergency battery threshold param. */
	float _emergency_thr{0.f};

	/** @param _low_thr Low battery threshold param. */
	float _low_thr{0.f};

	/** @parama _c_mult Capacity/current multiplier param  */
	float _c_mult{0.f};

	/** @param _manufacturer_name Name of the battery manufacturer. */
	char *_manufacturer_name{nullptr};

	uint8_t _smart_battery_type;
	/** @param _lifetime_max_delta_cell_voltage Max lifetime delta of the battery cells */
	float _lifetime_max_delta_cell_voltage{0.f};

	/** @param _cell_undervoltage_protection_status 0 if protection disabled, 1 if enabled */
	uint8_t _cell_undervoltage_protection_status{1};

	//BATT_SMBUS(const BATT_SMBUS &) = delete;
	//BATT_SMBUS operator=(const BATT_SMBUS &) = delete;
};
