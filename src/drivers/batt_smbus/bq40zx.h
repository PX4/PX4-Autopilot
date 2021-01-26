/**
 * @file bq40zx.h
 *
 * Header for a battery monitor connected via SMBus (I2C).
 * Designed for BQ40Z50/80
 *
 * @author Nick Belanger <nbelanger@mail.skymul.com>
 * @author Eohan George <eg@.skymul.com>
 */

#pragma once

#include "batt_smbus.h"


#define BATT_SMBUS_BQ40Z50_CELL_4_VOLTAGE               0x3C
#define BATT_SMBUS_BQ40Z50_CELL_3_VOLTAGE               0x3D
#define BATT_SMBUS_BQ40Z50_CELL_2_VOLTAGE               0x3E
#define BATT_SMBUS_BQ40Z50_CELL_1_VOLTAGE               0x3F

#define BATT_SMBUS_BQ40Z80_CELL_7_VOLTAGE               0x3C
#define BATT_SMBUS_BQ40Z80_CELL_6_VOLTAGE               0x3D
#define BATT_SMBUS_BQ40Z80_CELL_5_VOLTAGE               0x3E
#define BATT_SMBUS_BQ40Z80_CELL_4_VOLTAGE               0x3F

#define BATT_SMBUS_STATE_OF_HEALTH                      0x4F            ///< State of Health. The SOH information of the battery in percentage of Design Capacity

#define BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS            0x44

#define BATT_SMBUS_SECURITY_KEYS                        0x0035
#define BATT_SMBUS_LIFETIME_FLUSH                       0x002E
#define BATT_SMBUS_LIFETIME_BLOCK_ONE                   0x0060
#define BATT_SMBUS_ENABLED_PROTECTIONS_A_ADDRESS        0x4938
#define BATT_SMBUS_SEAL                                 0x0030
#define BATT_SMBUS_DASTATUS1                            0x0071
#define BATT_SMBUS_DASTATUS2                            0x0072
#define BATT_SMBUS_DASTATUS3                            0x007B

#define BATT_SMBUS_ENABLED_PROTECTIONS_A_DEFAULT        0xcf
#define BATT_SMBUS_ENABLED_PROTECTIONS_A_CUV_DISABLED   0xce

class BQ40ZX : public BATT_SMBUS
{

	using BATT_SMBUS::BATT_SMBUS;

	void RunImpl() override;

	/**
	 * @brief Reads data from flash.
	 * @param address The address to start the read from.
	 * @param data The returned data.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int dataflash_read(uint16_t &address, void *data, const unsigned length);

	/**
	 * @brief Writes data to flash.
	 * @param address The start address of the write.
	 * @param data The data to be written.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int dataflash_write(uint16_t address, void *data, const unsigned length);

	int get_startup_info() override;

	/**
	 * @brief Reads the cell voltages.
	 * @return Returns PX4_OK on success or associated read error code on failure.
	 */
	int get_cell_voltages();

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

	void set_undervoltage_protection(float average_current);

	void custom_method(const BusCLIArguments &cli) override;

};

