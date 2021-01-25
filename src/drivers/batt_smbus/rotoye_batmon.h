/**
 * @file rotoye_batmon.h
 *
 * Header for a battery monitor connected via SMBus (I2C).
 * Designed for Rotoye Batmon BMS
 *
 * @author Nick Belanger <nbelanger@mail.skymul.com>
 */

#pragma once

#include "batt_smbus.h"


class Rotoye_Batmon : public BATT_SMBUS
{

	using BATT_SMBUS::BATT_SMBUS;

	void RunImpl() override;

	int get_startup_info() override;

	void custom_method(const BusCLIArguments &cli) override;

};

