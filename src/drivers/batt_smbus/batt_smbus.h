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
 * @author Randy Mackay <rmackay9@yahoo.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 */

#include <float.h>
#include <stdio.h>
#include <string.h>
#include <ecl/geo/geo.h>

#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>
#include <px4_config.h>
#include <px4_workqueue.h>
#include <perf/perf_counter.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/uORB.h>

#define BATT_SMBUS_ADDR_MIN						0x00				///< lowest possible address
#define BATT_SMBUS_ADDR_MAX						0xFF				///< highest possible address

#define BATT_SMBUS_I2C_BUS						PX4_I2C_BUS_EXPANSION
#define BATT_SMBUS_ADDR							0x0B				///< Default 7 bit address I2C address. 8 bit = 0x16
#define BATT_SMBUS_TEMP							0x08				///< temperature register
#define BATT_SMBUS_VOLTAGE						0x09				///< voltage register
#define BATT_SMBUS_REMAINING_CAPACITY			0x0F				///< predicted remaining battery capacity as a percentage
#define BATT_SMBUS_FULL_CHARGE_CAPACITY			0x10				///< capacity when fully charged
#define BATT_SMBUS_DESIGN_CAPACITY				0x18				///< design capacity register
#define BATT_SMBUS_DESIGN_VOLTAGE				0x19				///< design voltage register
#define BATT_SMBUS_MANUFACTURE_DATE				0x1B				///< manufacture date register
#define BATT_SMBUS_SERIAL_NUMBER				0x1C				///< serial number register
#define BATT_SMBUS_MANUFACTURER_NAME			0x20				///< manufacturer name
#define BATT_SMBUS_CURRENT						0x0A				///< current register
#define BATT_SMBUS_AVERAGE_CURRENT				0x0B				///< current register
#define BATT_SMBUS_MEASUREMENT_INTERVAL_US		(1000000 / 10)		///< time in microseconds, measure at 10Hz
#define BATT_SMBUS_TIMEOUT_US					10000000			///< timeout looking for battery 10seconds after startup
#define BATT_SMBUS_CYCLE_COUNT					0x17				///< number of cycles the battery has experienced
#define BATT_SMBUS_RUN_TIME_TO_EMPTY			0x11				///< predicted remaining battery capacity based on the present rate of discharge in min
#define BATT_SMBUS_AVERAGE_TIME_TO_EMPTY		0x12				///< predicted remaining battery capacity based on the present rate of discharge in min

#define BATT_SMBUS_MANUFACTURER_ACCESS			0x00
#define BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS	0x44

#define BATT_SMBUS_PEC_POLYNOMIAL				0x07				///< Polynomial for calculating PEC

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif


class BATT_SMBUS : public device::I2C
{
public:
	BATT_SMBUS(int bus = PX4_I2C_BUS_EXPANSION, uint16_t batt_smbus_addr = BATT_SMBUS_ADDR);
	virtual ~BATT_SMBUS();

	/**
	 * Initialize device
	 *
	 * Calls probe() to check for device on bus.
	 *
	 * @return 0 on success, error code on failure
	 */
	virtual int		init();

	/**
	 * Test device
	 *
	 * @return 0 on success, error code on failure
	 */
	virtual int		test();

	/**
	 * Search all possible slave addresses for a smart battery
	 */
	int				search();

	/**
	 * Get the SBS manufacturer name of the battery device
	 *
	 * @param manufacturer_name pointer a buffer into which the manufacturer name is to be written
	* @param max_length the maximum number of bytes to attempt to read from the manufacturer name register, including the null character that is appended to the end
	 *
	 * @return the number of bytes read
	 */
	uint8_t			manufacturer_name(uint8_t *man_name, uint8_t max_length);

	/**
	 * Return the SBS manufacture date of the battery device
	 *
	 * @return the date in the following format:
	*  see Smart Battery Data Specification, Revision  1.1
	*  http://sbs-forum.org/specs/sbdat110.pdf for more details
	 *  Date as uint16_t = (year-1980) * 512 + month * 32 + day
	 *  | Field | Bits | Format             | Allowable Values                           |
	 *  | ----- | ---- | ------------------ | ------------------------------------------ |
	 *  | Day     0-4    5-bit binary value   1-31 (corresponds to day)                  |
	 *  | Month   5-8    4-bit binary value   1-12 (corresponds to month number)         |
	 *  | Year    9-15   7-bit binary value   0-127 (corresponds to year biased by 1980) |
	 *  otherwise, return 0 on failure
	 */
	uint16_t		manufacture_date();

	/**
	 * Return the SBS serial number of the battery device
	 */
	uint16_t		serial_number();

protected:
	/**
	 * Check if the device can be contacted
	 */
	virtual int		probe();

private:

	/**
	 * Start periodic reads from the battery
	 */
	void			start();

	/**
	 * Stop periodic reads from the battery
	 */
	void			stop();

	/**
	 * static function that is called by worker queue
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * perform a read from the battery
	 */
	void			cycle();

	/**
	 * Read a word from specified register
	 */
	int				read_reg(uint8_t reg, uint16_t &val);

	/**
	 * Write a word to specified register
	 */
	int				write_reg(uint8_t reg, uint16_t val);

	/**
	 * Convert from 2's compliment to decimal
	 * @return the absolute value of the input in decimal
	 */
	uint16_t		convert_twos_comp(uint16_t val);

	/**
	 * Read block from bus
	 * @return returns number of characters read if successful, zero if unsuccessful
	 */
	uint8_t			read_block(uint8_t reg, uint8_t *data, uint8_t max_len, bool append_zero);

	/**
	 * Write block to the bus
	 * @return the number of characters sent if successful, zero if unsuccessful
	 */
	uint8_t			write_block(uint8_t reg, uint8_t *data, uint8_t len);

	/**
	 * Calculate PEC for a read or write from the battery
	 * @param buff is the data that was read or will be written
	 */
	uint8_t			get_PEC(uint8_t cmd, bool reading, const uint8_t buff[], uint8_t len);

	/**
	 * Write a word to Manufacturer Access register (0x00)
	 * @param cmd the word to be written to Manufacturer Access
	 */
	uint8_t			ManufacturerAccess(uint16_t cmd);

	/**
	 * Read info from battery on startup
	 * @return OK if everything was read successfully
	 */
	uint8_t			GetStartupInfo();

	// internal variables
	bool			_enabled;	///< true if we have successfully connected to battery
	work_s			_work{};		///< work queue for scheduling reads

	battery_status_s _last_report{};	///< last published report, used for test()

	orb_advert_t	_batt_topic;	///< uORB battery topic
	orb_id_t		_batt_orb_id;	///< uORB battery topic ID

	uint64_t		_start_time;	///< system time we first attempt to communicate with battery
	uint16_t		_batt_capacity;	///< battery's design capacity in mAh (0 means unknown)
	uint16_t		_batt_startup_capacity;	///< battery's remaining capacity on startup
	char			*_manufacturer_name;  ///< The name of the battery manufacturer
	uint16_t		_cycle_count;	///< number of cycles the battery has experienced
	uint16_t		_serial_number;		///< serial number register
	float 			_crit_thr;	///< Critical battery threshold param
	float 			_low_thr;	///< Low battery threshold param
	float 			_emergency_thr;		///< Emergency battery threshold param
};