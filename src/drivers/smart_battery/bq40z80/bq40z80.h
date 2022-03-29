/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
 * @file bq40z80.cpp
 *
 * Driver for TI BQ40Z80 connected via SMBus (I2C). Loosely based on legacy
 * batt_smbus driver by Jacob Dahl <dahl.jakejacob@gmail.com>, Alex Klimaj
 * <alexklimaj@gmail.com> and Bazooka Joe <BazookaJoe1900@gmail.com>
 *
 * @author Alex Mikhalev <alex@corvus-robotics.com>
 * @author Mohammed Kabir <kabir@corvus-robotics.com>
 *
 */

#pragma once

#include <lib/drivers/smbus_sbs/SBS.hpp>
#include <mathlib/mathlib.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <hysteresis/hysteresis.h>

#include <board_config.h>

using namespace time_literals;

class BQ40Z80 : public SBSBattery<BQ40Z80>
{
public:
	using BaseClass = SBSBattery<BQ40Z80>;

	static constexpr const char *MOD_NAME = 		"bq40z80";
	static constexpr uint8_t BATT_ADDR = 			0x0B;		///< Default 7 bit address I2C address. 8 bit = 0x16

	static constexpr size_t MAC_DATA_BUFFER_SIZE =		32;

	static const uint8_t MAX_NUM_CELLS = 7;

	enum BQ40Z80_FLASH {
		BQ40Z80_FLASH_CELL_CONFIGURATION =                     	0x4D05,
	};

	enum BQ40Z80_REG {
		BQ40Z80_REG_BATTERY_STATUS =				0x16,
		BQ40Z80_REG_STATE_OF_HEALTH =                    	0x4F,
		BQ40Z80_REG_MANUFACTURER_BLOCK_ACCESS =			0x44,
	};

	static constexpr uint32_t BQ40Z80_BATTERY_STATUS_FC	      = 1 << 5;  ///< Fully charged
	static constexpr uint32_t BQ40Z80_BATTERY_STATUS_DSG	      = 1 << 6;  ///< Discharging or Relax (Charging not detected)
	static constexpr uint32_t BQ40Z80_BATTERY_STATUS_TCA	      = 1 << 15; ///< Terminate Charge Alarm

	enum BQ40Z80_MAN_DATA {
		BQ40Z80_MAN_DEVICE_TYPE =                         	0x0001,
		BQ40Z80_MAN_SECURITY_KEYS =                         	0x0035,
		BQ40Z80_MAN_LIFETIME_FLUSH =                        	0x002E,
		BQ40Z80_MAN_LIFETIME_BLOCK_ONE =                    	0x0060,
		BQ40Z80_MAN_ENABLED_PROTECTIONS_A_ADDRESS =         	0x4BBE,
		BQ40Z80_MAN_ENABLED_PROTECTIONS_B_ADDRESS =         	0x4BBF,
		BQ40Z80_MAN_LAST_VALID_CHARGE_TERM =         		0x459E,
		BQ40Z80_MAN_SEAL =                                  	0x0030,
		BQ40Z80_MAN_OPERATION_STATUS =                          0x0054,
		BQ40Z80_MAN_CHARGING_STATUS =                           0x0055,
		BQ40Z80_MAN_DASTATUS1 =                             	0x0071,
		BQ40Z80_MAN_DASTATUS2 =                             	0x0072,
		BQ40Z80_MAN_DASTATUS3 =                             	0x007B,
		BQ40Z80_MAN_MFC_SHUTDOWN_ENABLE_A =			0x270C,
		BQ40Z80_MAN_MFC_SHUTDOWN_ENABLE_B =			0x043D,
		BQ40Z80_MAN_MFC_SHUTDOWN_DISABLE =			0x23A7,
	};

	static constexpr uint16_t BQ40Z80_EXPECTED_DEVICE_TYPE = 	0x4800;

	// AOLDL, RSVD_1, ~OCD2, ~OCD1, OCC2, OCC1, COV, CUV
	static constexpr uint8_t BQ40Z80_ENABLED_PROTECTIONS_A_DEFAULT = 0xCF;
	static constexpr uint8_t BQ40Z80_ENABLED_PROTECTIONS_A_CUV     = 1 << 0;

	// ~RSVD, ~CUVC, ~OTD, OTC, ~ASCDL, RSVD, ~ASCCL, RSVD
	static constexpr uint8_t BQ40Z80_ENABLED_PROTECTIONS_B_DEFAULT = 0x15;
	static constexpr uint8_t BQ40Z80_ENABLED_PROTECTIONS_B_CUVC    = 1 << 6;

	static constexpr uint32_t BQ40Z80_OPERATION_STATUS_XDSG        = 1 << 13; ///< Discharging Disabled (by protection)
	static constexpr uint32_t BQ40Z80_OPERATION_STATUS_XCHG        = 1 << 14; ///< Charging Disabled (by protection)
	static constexpr uint32_t BQ40Z80_OPERATION_STATUS_CB          = 1 << 28; ///< Cell Balancing

	static constexpr uint32_t BQ40Z80_CHARGING_STATUS_VCT          = 1 << 15; ///< Valid Charge Termination

	BQ40Z80(const I2CSPIDriverConfig &config, SMBus *interface);

	~BQ40Z80();

	enum BQ40Z80_CCMD {
		BQ40Z80_CCMD_READ_MAN_BLOCK
	};

	void custom_method(const BusCLIArguments &cli) override;

	static int handle_command(const char *verb, BusCLIArguments &cli, BusInstanceIterator &iterator,
				  int argc, char *argv[]);

	static void print_module_description();
	static void print_usage_commands();

	int populate_startup_data() override;

	int populate_runtime_data(battery_status_s &msg) override;


private:
	/**
	 * @brief Reads data from flash.
	 * @param address The address to start the read from.
	 * @param data The returned data.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int dataflash_read(const uint16_t address, void *data, const unsigned length);

	/**
	 * @brief Writes data to flash.
	 * @param address The start address of the write.
	 * @param data The data to be written.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int dataflash_write(const uint16_t address, void *data, const unsigned length);

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
	 * @brief This command flushes lifetime data from RAM to data flash, where it can be read out from
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int lifetime_flush();

	/**
	 * @brief Reads the lifetime data from block 1.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int lifetime_read();

	/**
	 * @brief Reads the cell voltages.
	 * @return Returns PX4_OK on success or associated read error code on failure.
	 */
	int populate_cell_voltages(battery_status_s &data);

	/**
	 * @brief Enables or disables in-air protection mode, which disables certain BMS protection
	 * functions while armed and flying.
	 */
	int set_in_air_protection_mode(bool enable);

	/**
	 * @brief Enables or disables the manual FET shutdown mode, which shuts off the charge and
	 * discharge FETs on the BMS.
	 */
	int set_mfc_shutdown_mode(bool enable);

	/** @param _state_of_health state of health as read on connection  */
	float _state_of_health{0.f};

	systemlib::Hysteresis _in_air_protection_hyst{true};

	/** @param _in_air_protection_state true if protections are disabled, false if enabled.
	 * When in-air protection is _enabled_, we _disable_ certain BMS protections to avoid
	 * them potentially falsely triggering in flight and causing in-flight powerloss
	 */
	bool _in_air_protection_state{true};

	uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};
};
