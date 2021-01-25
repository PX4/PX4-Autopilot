/**
 * @file bq40zx.h
 *
 * Header for a battery monitor connected via SMBus (I2C).
 * Designed for BQ40Z50-R1/R2
 *
 * @author Nick Belanger <nbelanger@mail.skymul.com>
 */

#pragma once

#include "batt_smbus.h"

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

