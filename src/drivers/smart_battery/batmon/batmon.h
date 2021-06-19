/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file batmon.h
 *
 * BatMon module for Smart Battery utilizing SBS 1.1 specifications
 * Setup/usage information: https://rotoye.com/batmon-tutorial/
 *
 * @author Eohan George <eohan@rotoye.com>
 * @author Nick Belanger <nbelanger@mail.skymul.com>
 */

#pragma once

#include <lib/drivers/smbus_sbs/SBS.hpp>
#include <px4_platform_common/module.h>

#define MAX_CELL_COUNT 16

#define BATMON_DEFAULT_SMBUS_ADDR                       0x0B            ///< Default 7 bit address I2C address. 8 bit = 0x16

class Batmon : public SMBUS_SBS_BaseClass<Batmon>
{

public:
	Batmon(I2CSPIBusOption bus_option, const int bus, SMBus *interface);
	~Batmon();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);

	static void print_usage();

	void RunImpl();
	enum {
		BATT_SMBUS_TEMP_EXTERNAL_1		=	0x48,
		BATT_SMBUS_TEMP_EXTERNAL_2		=	0x49,
		BATT_SMBUS_CELL_1_VOLTAGE               =       0x3F,
		BATT_SMBUS_CELL_2_VOLTAGE               =       0x3E,
		BATT_SMBUS_CELL_3_VOLTAGE               =       0x3D,
		BATT_SMBUS_CELL_4_VOLTAGE               =       0x3C,
		BATT_SMBUS_CELL_5_VOLTAGE               =       0x3B,
		BATT_SMBUS_CELL_6_VOLTAGE               =       0x3A,
		BATT_SMBUS_CELL_7_VOLTAGE               =       0x39,
		BATT_SMBUS_CELL_8_VOLTAGE               =       0x38,
		BATT_SMBUS_CELL_9_VOLTAGE               =       0x37,
		BATT_SMBUS_CELL_10_VOLTAGE              =       0x36,
		BATT_SMBUS_CELL_11_VOLTAGE              =       0x35,
		BATT_SMBUS_CELL_12_VOLTAGE              =       0x34,
		BATT_SMBUS_CELL_13_VOLTAGE              =       0x33,
		BATT_SMBUS_CELL_14_VOLTAGE              =       0x32,
		BATT_SMBUS_CELL_15_VOLTAGE              =       0x31,
		BATT_SMBUS_CELL_16_VOLTAGE              =       0x30,
		BATT_SMBUS_CELL_COUNT                   =       0x40
	} BATMON_REGISTERS;

private:

	float _cell_voltages[MAX_CELL_COUNT] = {};

	float _max_cell_voltage_delta{0};

	float _min_cell_voltage{0};

	/** @param _last_report Last published report, used finding v deltas */
	battery_status_s _last_report{};

	void custom_method(const BusCLIArguments &cli) override;

	int get_cell_voltages();

	int get_batmon_startup_info();

	/** @param _crit_thr Critical battery threshold param. */
	float _crit_thr{0.f};

	/** @param _emergency_thr Emergency battery threshold param. */
	float _emergency_thr{0.f};

	/** @param _low_thr Low battery threshold param. */
	float _low_thr{0.f};

};
