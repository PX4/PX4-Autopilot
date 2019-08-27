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
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/topics/battery_status.h>

#include "board_config.h"

#define DATA_BUFFER_SIZE                                32

#define BATT_CELL_VOLTAGE_THRESHOLD_RTL                 0.5f            ///< Threshold in volts to RTL if cells are imbalanced
#define BATT_CELL_VOLTAGE_THRESHOLD_FAILED              1.5f            ///< Threshold in volts to Land if cells are imbalanced

#define BATT_CURRENT_UNDERVOLTAGE_THRESHOLD             5.0f            ///< Threshold in amps to disable undervoltage protection
#define BATT_VOLTAGE_UNDERVOLTAGE_THRESHOLD             3.4f            ///< Threshold in volts to re-enable undervoltage protection

#define BATT_SMBUS_CURRENT                              0x0A            ///< current register
#define BATT_SMBUS_AVERAGE_CURRENT                      0x0B            ///< current register
#define BATT_SMBUS_ADDR                                 0x0B            ///< Default 7 bit address I2C address. 8 bit = 0x16
#define BATT_SMBUS_ADDR_MIN                             0x00            ///< lowest possible address
#define BATT_SMBUS_ADDR_MAX                             0xFF            ///< highest possible address
#define BATT_SMBUS_PEC_POLYNOMIAL                       0x07            ///< Polynomial for calculating PEC
#define BATT_SMBUS_TEMP                                 0x08            ///< temperature register
#define BATT_SMBUS_VOLTAGE                              0x09            ///< voltage register
#define BATT_SMBUS_FULL_CHARGE_CAPACITY                 0x10            ///< capacity when fully charged
#define BATT_SMBUS_RUN_TIME_TO_EMPTY                    0x11            ///< predicted remaining battery capacity based on the present rate of discharge in min
#define BATT_SMBUS_AVERAGE_TIME_TO_EMPTY                0x12            ///< predicted remaining battery capacity based on the present rate of discharge in min
#define BATT_SMBUS_REMAINING_CAPACITY                   0x0F            ///< predicted remaining battery capacity as a percentage
#define BATT_SMBUS_CYCLE_COUNT                          0x17            ///< number of cycles the battery has experienced
#define BATT_SMBUS_DESIGN_CAPACITY                      0x18            ///< design capacity register
#define BATT_SMBUS_DESIGN_VOLTAGE                       0x19            ///< design voltage register
#define BATT_SMBUS_MANUFACTURER_NAME                    0x20            ///< manufacturer name
#define BATT_SMBUS_MANUFACTURE_DATE                     0x1B            ///< manufacture date register
#define BATT_SMBUS_SERIAL_NUMBER                        0x1C            ///< serial number register
#define BATT_SMBUS_MEASUREMENT_INTERVAL_US              100000          ///< time in microseconds, measure at 10Hz
#define BATT_SMBUS_TIMEOUT_US                           1000000         ///< timeout looking for battery 10seconds after startup
#define BATT_SMBUS_MANUFACTURER_ACCESS                  0x00
#define BATT_SMBUS_MANUFACTURER_DATA                    0x23
#define BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS            0x44
#define BATT_SMBUS_SECURITY_KEYS                        0x0035
#define BATT_SMBUS_CELL_1_VOLTAGE                       0x3F
#define BATT_SMBUS_CELL_2_VOLTAGE                       0x3E
#define BATT_SMBUS_CELL_3_VOLTAGE                       0x3D
#define BATT_SMBUS_CELL_4_VOLTAGE                       0x3C
#define BATT_SMBUS_LIFETIME_FLUSH                       0x002E
#define BATT_SMBUS_LIFETIME_BLOCK_ONE                   0x0060
#define BATT_SMBUS_ENABLED_PROTECTIONS_A_ADDRESS        0x4938
#define BATT_SMBUS_SEAL                                 0x0030

#define BATT_SMBUS_ENABLED_PROTECTIONS_A_DEFAULT        0xcf
#define BATT_SMBUS_ENABLED_PROTECTIONS_A_CUV_DISABLED   0xce

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
};

/**
 * @brief Nuttshell accessible method to return the battery manufacture date.
 * @return Returns PX4_OK on success, PX4_ERROR on failure.
 */
int manufacture_date();

/**
 * @brief Nuttshell accessible method to return the battery manufacturer name.
 * @return Returns PX4_OK on success, PX4_ERROR on failure.
 */
int manufacturer_name();

/**
 * @brief Nuttshell accessible method to return the battery serial number.
 * @return Returns PX4_OK on success, PX4_ERROR on failure.
 */
int serial_number();



class BATT_SMBUS : public ModuleBase<BATT_SMBUS>, public px4::ScheduledWorkItem
{
public:

	BATT_SMBUS(SMBus *interface, const char *path);

	~BATT_SMBUS();

	friend SMBus;

	/** @see ModuleBase */
	static int print_usage();

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/**
	 * @brief Reads data from flash.
	 * @param address The address to start the read from.
	 * @param data The returned data.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int dataflash_read(uint16_t &address, void *data);

	/**
	 * @brief Writes data to flash.
	 * @param address The start address of the write.
	 * @param data The data to be written.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int dataflash_write(uint16_t &address, void *data, const unsigned length);

	/**
	 * @brief Returns the SBS serial number of the battery device.
	 * @return Returns the SBS serial number of the battery device.
	 */
	uint16_t get_serial_number();

	/**
	* @brief Read info from battery on startup.
	* @return Returns PX4_OK on success, PX4_ERROR on failure.
	*/
	int get_startup_info();

	/**
	 * @brief Prints the latest report.
	 */
	void print_report();


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
	 * @brief Performs a ManufacturerBlockAccess() read command.
	 * @param cmd_code The command code.
	 * @param data The returned data.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int manufacturer_read(const uint16_t cmd_code, void *data, const unsigned length);

	/**
	 * @brief Performs a ManufacturerBlockAccess() write command.
	 * @param cmd_code The command code.
	 * @param data The sent data.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int manufacturer_write(const uint16_t cmd_code, void *data, const unsigned length);

	/**
	 * @brief Search all possible slave addresses for a smart battery.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int search_addresses();

	/**
	 * @brief Unseals the battery to allow writing to restricted flash.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int unseal();

	/**
	 * @brief Seals the battery to disallow writing to restricted flash.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int seal();

	/**
	 * @brief This command flushes the RAM Lifetime Data to data flash to help streamline evaluation testing.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int lifetime_data_flush();

	/**
	 * @brief Reads the lifetime data from block 1.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int lifetime_read_block_one();

	/**
	 * @brief Reads the cell voltages.
	 * @return Returns PX4_OK on success or associated read error code on failure.
	 */
	int get_cell_voltages();

	/**
	 * @brief Enables or disables the cell under voltage protection emergency shut off.
	 */
	void set_undervoltage_protection(float average_current);

	SMBus *_interface;

	void suspend();

	void resume();

private:

	void Run() override;

	perf_counter_t _cycle;

	float _cell_voltages[4] = {};

	float _max_cell_voltage_delta{0};

	float _min_cell_voltage{0};

	bool _should_suspend{false};

	/** @param _last_report Last published report, used for test(). */
	battery_status_s _last_report = battery_status_s{};

	/** @param _batt_topic uORB battery topic. */
	orb_advert_t _batt_topic;

	/** @param _cell_count Number of series cell. */
	uint8_t _cell_count;

	/** @param _batt_capacity Battery design capacity in mAh (0 means unknown). */
	uint16_t _batt_capacity;

	/** @param _batt_startup_capacity Battery remaining capacity in mAh on startup. */
	uint16_t _batt_startup_capacity;

	/** @param _cycle_count The number of cycles the battery has experienced. */
	uint16_t _cycle_count;

	/** @param _serial_number Serial number register. */
	uint16_t _serial_number;

	/** @param _crit_thr Critical battery threshold param. */
	float _crit_thr;

	/** @param _emergency_thr Emergency battery threshold param. */
	float _emergency_thr;

	/** @param _low_thr Low battery threshold param. */
	float _low_thr;

	/** @param _manufacturer_name Name of the battery manufacturer. */
	char *_manufacturer_name;

	/** @param _lifetime_max_delta_cell_voltage Max lifetime delta of the battery cells */
	float _lifetime_max_delta_cell_voltage;

	/** @param _cell_undervoltage_protection_status 0 if protection disabled, 1 if enabled */
	uint8_t _cell_undervoltage_protection_status;

	/** Do not allow copy construction or move assignment of this class. */
	BATT_SMBUS(const BATT_SMBUS &);
	BATT_SMBUS operator=(const BATT_SMBUS &);
};
