/**
 * @file rotoye_batmon.h
 *
 * Header for a battery monitor connected via SMBus (I2C).
 * Designed for Rotoye Batmon
 *
 * @author Nick Belanger <nbelanger@mail.skymul.com>
 * @author Eohan George <eg@.skymul.com>
 */

#pragma once

#include "../batt_smbus.h"


class Rotoye_Batmon : public BATT_SMBUS
{

	using BATT_SMBUS::BATT_SMBUS;

	/**
	 * @brief Reads the cell voltages.
	 * @return Returns PX4_OK on success or associated read error code on failure.
	 */
	int get_cell_voltages();

	void RunImpl() override;

	void custom_method(const BusCLIArguments &cli) override;

};

