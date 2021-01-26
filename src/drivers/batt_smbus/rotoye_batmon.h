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

#include "batt_smbus.h"

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

#define BATT_SMBUS_REMAINING_CAPACITY                   0x0F            ///< predicted remaining battery capacity as mAh
#define BATT_SMBUS_RELATIVE_SOC				0x0D		///< predicted remaining battery capacity as a percentage

#define BATT_SMBUS_CELL_COUNT                           0x40            // < This is not a default register in the BQ40Z50 chip, but one that is really needed
#define BATT_SMBUS_SAFETY_ALERT                         0x50            ///32 alert bits, threshold exceeded (used for burst current check)
#define BATT_SMBUS_SAFETY_STATUS                        0x51            ///32 status bits, threshold exceeded for certain duration
#define BATT_SMBUS_PF_ALERT                             0x52            ///32 permanent fail bits, issue warranting permanent shutoff occurred (used for cell voltage imbalance check)


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

