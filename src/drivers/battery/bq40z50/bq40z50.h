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
 * @file bq40z50.h
 *
 * Header for a battery monitor connected via SMBus (I2C).
 * Designed for BQ40Z50-R1/R2
 *
 * @author Randy Mackay <rmackay9@yahoo.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Mark Sauder <mcsauder@gmail.com>
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 */


#include <px4_config.h>
#include <px4_workqueue.h>

#include <stdio.h>

#include <string.h>
#include <ecl/geo/geo.h>

#include <drivers/device/CDev.hpp>
#include <drivers/device/Device.hpp>
#include <drivers/device/i2c.h>
#include <drivers/drv_device.h>

#include <drivers/drv_hrt.h>

#include <uORB/topics/battery_status.h>

#define DATA_BUFFER_SIZE				                32

#define BATT_CELL_VOLTAGE_THRESHOLD_RTL                 0.5f                    ///< Threshold in volts to RTL if cells are imbalanced
#define BATT_CELL_VOLTAGE_THRESHOLD_FAILED              1.5f                    ///< Threshold in volts to Land if cells are imbalanced

#define BATT_CURRENT_UNDERVOLTAGE_THRESHOLD             5.0f                    ///< Threshold in amps to disable undervoltage protection
#define BATT_VOLTAGE_UNDERVOLTAGE_THRESHOLD             3.4f                    ///< Threshold in volts to re-enable undervoltage protection

#define BQ40Z50_I2C_BUS                              PX4_I2C_BUS_EXPANSION
#define BQ40Z50_CURRENT                              0x0A                    ///< current register
#define BQ40Z50_AVERAGE_CURRENT                      0x0B                    ///< current register
#define BQ40Z50_ADDR                                 0x0B                    ///< Default 7 bit address I2C address. 8 bit = 0x16
#define BQ40Z50_ADDR_MIN                             0x00                    ///< lowest possible address
#define BQ40Z50_ADDR_MAX                             0xFF                    ///< highest possible address
#define BQ40Z50_PEC_POLYNOMIAL                       0x07                    ///< Polynomial for calculating PEC
#define BQ40Z50_TEMP                                 0x08                    ///< temperature register
#define BQ40Z50_VOLTAGE                              0x09                    ///< voltage register
#define BQ40Z50_FULL_CHARGE_CAPACITY                 0x10                    ///< capacity when fully charged
#define BQ40Z50_RUN_TIME_TO_EMPTY                    0x11                    ///< predicted remaining battery capacity based on the present rate of discharge in min
#define BQ40Z50_AVERAGE_TIME_TO_EMPTY                0x12                    ///< predicted remaining battery capacity based on the present rate of discharge in min
#define BQ40Z50_REMAINING_CAPACITY                   0x0F                    ///< predicted remaining battery capacity as a percentage
#define BQ40Z50_CYCLE_COUNT                          0x17                    ///< number of cycles the battery has experienced
#define BQ40Z50_DESIGN_CAPACITY                      0x18                    ///< design capacity register
#define BQ40Z50_DESIGN_VOLTAGE                       0x19                    ///< design voltage register
#define BQ40Z50_MANUFACTURER_NAME                    0x20                    ///< manufacturer name
#define BQ40Z50_MANUFACTURE_DATE                     0x1B                    ///< manufacture date register
#define BQ40Z50_SERIAL_NUMBER                        0x1C                    ///< serial number register
#define BQ40Z50_MEASUREMENT_INTERVAL_US              100000                  ///< time in microseconds, measure at 10Hz
#define BQ40Z50_TIMEOUT_US                           1000000                 ///< timeout looking for battery 10seconds after startup
#define BQ40Z50_MANUFACTURER_ACCESS                  0x00
#define BQ40Z50_MANUFACTURER_DATA			        0x23
#define BQ40Z50_MANUFACTURER_BLOCK_ACCESS            0x44
#define BQ40Z50_SECURITY_KEYS			            0x0035
#define BQ40Z50_CELL_1_VOLTAGE                       0x3F
#define BQ40Z50_CELL_2_VOLTAGE                       0x3E
#define BQ40Z50_CELL_3_VOLTAGE                       0x3D
#define BQ40Z50_CELL_4_VOLTAGE                       0x3C
#define BQ40Z50_LIFETIME_FLUSH                       0x002E
#define BQ40Z50_LIFETIME_BLOCK_ONE                   0x0060
#define BQ40Z50_ENABLED_PROTECTIONS_A_ADDRESS        0x4938

#define BQ40Z50_ENABLED_PROTECTIONS_A_DEFAULT		0xcf
#define BQ40Z50_ENABLED_PROTECTIONS_A_CUV_DISABLED	0xce

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

/**
 * @brief Nuttshell accessible method to return the driver usage arguments.
 */
void bq40z50_usage();

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

extern device::Device *BQ40Z50_I2C_interface(int bus);
typedef device::Device *(*BQ40Z50_constructor)(int);

class BQ40Z50 : public device::CDev
{
public:

	/**
	 * @brief Default Constructor.
	 * @param interface The device communication interface (i2c)
	 * @param path The device i2c address
	 */
	BQ40Z50(device::Device *interface, const char *path);

	/**
	 * @brief Default Destructor.
	 */
	virtual ~BQ40Z50();

	/**
	 * @brief Sends a block read command.
	 * @param cmd_code The command code.
	 * @param data The returned data.
	 * @param length The number of bytes being read
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int block_read(const uint8_t cmd_code, void *data, const unsigned length);

	/**
	 * @brief Sends a block write command.
	 * @param cmd_code The command code.
	 * @param data The data to be written.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int block_write(const uint8_t cmd_code, void *data, const unsigned length);

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
	 * @brief Calculates the PEC from the data.
	 * @param buff The buffer that stores the data.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	uint8_t get_pec(uint8_t *buffer, const uint8_t length);

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
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int info();

	/**
	 * @brief Initializes the smart battery device. Calls probe() to check for device on bus.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	virtual int init();

	/**
	 * @brief Gets the SBS manufacture date of the battery.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int manufacture_date(void *man_date);

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
	 * @brief Sends a read-word command with cmd_code as the command.
	 * @param cmd_code The command code.
	 * @param data The returned data.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int read_word(const uint8_t cmd_code, void *data);

	/**
	 * @brief Search all possible slave addresses for a smart battery.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int search_addresses();

	/**
	 * @brief Starts periodic reads from the battery.
	 */
	void start();

	/**
	 * @brief Stops periodic reads from the battery.
	 */
	void stop();

	/**
	 * @brief Unseals the battery to allow writing to restricted flash.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int unseal();

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
	 * @brief Reads the lifetime data from block 1.
	 * @param address Address of the register to write
	 * @param tx_buf The sent data.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int write_flash(uint16_t address, uint8_t *tx_buf, const unsigned length);

protected:
	Device *_interface;

private:

	/**
	 * @brief Static function that is called by worker queue.
	 */
	static void cycle_trampoline(void *arg);

	/**
	 * @brief The loop that continually generates new reports.
	 */
	void cycle();

	/** @struct _work Work queue for scheduling reads. */
	work_s _work = work_s{};

	/** @param _last_report Last published report, used for test(). */
	battery_status_s _last_report = battery_status_s{};

	/** @param _batt_topic uORB battery topic. */
	orb_advert_t _batt_topic;

	/** @param _batt_orb_id uORB battery topic ID. */
	orb_id_t _batt_orb_id;

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

	/* Do not allow copy construction or move assignment of this class. */
	BQ40Z50(const BQ40Z50 &);
	BQ40Z50 operator=(const BQ40Z50 &);
};