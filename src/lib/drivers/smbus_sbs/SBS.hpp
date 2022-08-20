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
 * @file SBS.hpp
 *
 * Header for a core SBS 1.1 specification of Smart Battery
 * http://sbs-forum.org/specs/sbdat110.pdf
 *
 * @author Eohan George <eohan@rotoye.com>
 * @author Nick Belanger <nbelanger@mail.skymul.com>
 */

#pragma once

#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/smbus/SMBus.hpp>
#include <uORB/topics/battery_status.h>
#include <px4_platform_common/param.h>
#include <geo/geo.h>


using namespace time_literals;

#define SBS_MEASUREMENT_INTERVAL_US             	100_ms          ///< time in microseconds, measure at 10Hz (not part of SBS spec)

template<class T>
class SMBUS_SBS_BaseClass : public I2CSPIDriver<T>
{
public:
	SMBUS_SBS_BaseClass(const I2CSPIDriverConfig &config, SMBus *interface);
	SMBUS_SBS_BaseClass();

	~SMBUS_SBS_BaseClass();

	static void print_usage();

	friend SMBus;

	int populate_smbus_data(battery_status_s &msg);

	virtual void RunImpl(); // Can be overridden by derived implimentation

	virtual void custom_method(const BusCLIArguments &cli) = 0; //Has be overriden by derived implimentation

	/**
	 * @brief Returns the SBS serial number of the battery device.
	 * @return Returns the SBS serial number of the battery device.
	 */
	uint16_t get_serial_number();

	/**
	* @brief Read info from battery on startup.
	* @return Returns PX4_OK on success, PX4_ERROR on failure. Can be overridden by derived implimentation
	*/
	virtual int get_startup_info();

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
	 * @brief Enables or disables the cell under voltage protection emergency shut off.
	 */

	void suspend();

	void resume();
	enum {
		BATT_SMBUS_TEMP                         =	0x08,           ///< temperature register
		BATT_SMBUS_VOLTAGE                      =       0x09,           ///< voltage register
		BATT_SMBUS_CURRENT                      =       0x0A,           ///< current register
		BATT_SMBUS_AVERAGE_CURRENT              =       0x0B,           ///< average current register
		BATT_SMBUS_MAX_ERROR                    =       0x0C,           ///< max error
		BATT_SMBUS_RELATIVE_SOC                 =       0x0D,           ///< Relative State Of Charge
		BATT_SMBUS_ABSOLUTE_SOC                 =       0x0E,           ///< Absolute State of charge
		BATT_SMBUS_REMAINING_CAPACITY           =       0x0F,           ///< predicted remaining battery capacity as a percentage
		BATT_SMBUS_FULL_CHARGE_CAPACITY         =       0x10,           ///< capacity when fully charged
		BATT_SMBUS_RUN_TIME_TO_EMPTY            =       0x11,           ///< predicted remaining battery capacity based on the present rate of discharge in min
		BATT_SMBUS_AVERAGE_TIME_TO_EMPTY        =       0x12,           ///< predicted remaining battery capacity based on the present rate of discharge in min
		BATT_SMBUS_CYCLE_COUNT                  =       0x17,           ///< number of cycles the battery has experienced
		BATT_SMBUS_DESIGN_CAPACITY              =       0x18,           ///< design capacity register
		BATT_SMBUS_DESIGN_VOLTAGE               =       0x19,           ///< design voltage register
		BATT_SMBUS_MANUFACTURER_NAME            =       0x20,           ///< manufacturer name
		BATT_SMBUS_MANUFACTURER_NAME_SIZE       =       21,             ///< manufacturer name data size (not part of SBS spec)
		BATT_SMBUS_DEVICE_NAME			=	0x21,		///< character string that contains the battery's name
		BATT_SMBUS_MANUFACTURE_DATE             =       0x1B,           ///< manufacture date register
		BATT_SMBUS_SERIAL_NUMBER                =       0x1C,           ///< serial number register
		BATT_SMBUS_MANUFACTURER_ACCESS          =       0x00,
		BATT_SMBUS_MANUFACTURER_DATA            =       0x23
	} SBS_REGISTERS;

protected:

	SMBus *_interface;

	perf_counter_t _cycle{perf_alloc(PC_ELAPSED, "batmon_cycle")}; // TODO

	/** @param _batt_topic uORB battery topic. */
	orb_advert_t _batt_topic{nullptr};

	/** @param _cell_count Number of series cell (retrieved from cell_count PX4 params) */
	uint8_t _cell_count{0};

	/** @param _batt_capacity Battery design capacity in mAh (0 means unknown). */
	uint16_t _batt_capacity{0};

	/** @param _batt_startup_capacity Battery remaining capacity in mAh on startup. */
	uint16_t _batt_startup_capacity{0};

	/** @param _cycle_count The number of cycles the battery has experienced. */
	uint16_t _cycle_count{0};

	/** @param _serial_number Serial number register. */
	uint16_t _serial_number{0};

	/** @param _manufacturer_name Name of the battery manufacturer. */
	char _manufacturer_name[BATT_SMBUS_MANUFACTURER_NAME_SIZE + 1] {};	// Plus one for terminator

	/** @param _manufacture_date Date of the battery manufacturing. */
	uint16_t _manufacture_date{0};

	/** @param _state_of_health state of health as read on connection  */
	float _state_of_health{0.f}; // Not part of SBS, can be calculated from FullChargeCapacity
	// and DesignCapacity

	void print_man_info();

};

template<class T>
SMBUS_SBS_BaseClass<T>::SMBUS_SBS_BaseClass(const I2CSPIDriverConfig &config, SMBus *interface):
	I2CSPIDriver<T>(config),
	_interface(interface)
{
	battery_status_s new_report = {};
	int SBS_instance_number = 0;
	_batt_topic = orb_advertise_multi(ORB_ID(battery_status), &new_report, &SBS_instance_number);
	_interface->init();
}

template<class T>
SMBUS_SBS_BaseClass<T>::~SMBUS_SBS_BaseClass()
{
	orb_unadvertise(_batt_topic);
	perf_free(_cycle); // TODO

	if (_interface != nullptr) {
		delete _interface;
	}

}

template<class T>
void SMBUS_SBS_BaseClass<T>::suspend()
{
	this->ScheduleClear();
}

template<class T>
void SMBUS_SBS_BaseClass<T>::resume()
{
	this->ScheduleOnInterval(SBS_MEASUREMENT_INTERVAL_US);
}

template<class T>
uint16_t SMBUS_SBS_BaseClass<T>::get_serial_number()
{
	uint16_t serial_num = 0;

	if (_interface->read_word(BATT_SMBUS_SERIAL_NUMBER, serial_num) == PX4_OK) {
		return serial_num;
	}

	return PX4_ERROR;
}

template<class T>
int SMBUS_SBS_BaseClass<T>::get_startup_info()
{
	int ret = PX4_OK;

	ret |= _interface->block_read(BATT_SMBUS_MANUFACTURER_NAME, _manufacturer_name, BATT_SMBUS_MANUFACTURER_NAME_SIZE,
				      true);
	_manufacturer_name[sizeof(_manufacturer_name) - 1] = '\0';

	uint16_t serial_num;
	ret |= _interface->read_word(BATT_SMBUS_SERIAL_NUMBER, serial_num);

	uint16_t remaining_cap;
	ret |= _interface->read_word(BATT_SMBUS_REMAINING_CAPACITY, remaining_cap);

	uint16_t cycle_count;
	ret |= _interface->read_word(BATT_SMBUS_CYCLE_COUNT, cycle_count);

	uint16_t full_cap;
	ret |= _interface->read_word(BATT_SMBUS_FULL_CHARGE_CAPACITY, full_cap);

	uint16_t manufacture_date;
	ret |= _interface->read_word(BATT_SMBUS_MANUFACTURE_DATE, manufacture_date);

	if (!ret) {
		_serial_number = serial_num;
		_batt_startup_capacity = (uint16_t)((float)remaining_cap);
		_cycle_count = cycle_count;
		_batt_capacity = (uint16_t)((float)full_cap);
		_manufacture_date = manufacture_date;
	}

	return ret;
}

template<class T>
int SMBUS_SBS_BaseClass<T>::populate_smbus_data(battery_status_s &data)
{

	// Temporary variable for storing SMBUS reads.
	uint16_t result;

	int ret = _interface->read_word(BATT_SMBUS_VOLTAGE, result);

	// Convert millivolts to volts.
	data.voltage_v = ((float)result) * 0.001f;
	data.voltage_filtered_v = data.voltage_v;

	// Read current.
	ret |= _interface->read_word(BATT_SMBUS_CURRENT, result);

	data.current_a = (-1.0f * ((float)(*(int16_t *)&result)) * 0.001f);
	data.current_filtered_a = data.current_a;

	// Read remaining capacity.
	ret |= _interface->read_word(BATT_SMBUS_RELATIVE_SOC, result);
	data.remaining = (float)result * 0.01f;

	// Read remaining capacity.
	ret |= _interface->read_word(BATT_SMBUS_REMAINING_CAPACITY, result);
	data.discharged_mah = _batt_startup_capacity - result;

	// Read full capacity.
	ret |= _interface->read_word(BATT_SMBUS_FULL_CHARGE_CAPACITY, result);
	data.capacity = result;

	// Read cycle count.
	ret |= _interface->read_word(BATT_SMBUS_CYCLE_COUNT, result);
	data.cycle_count = result;

	// Read serial number.
	ret |= _interface->read_word(BATT_SMBUS_SERIAL_NUMBER, result);
	data.serial_number = result;

	// Read battery temperature and covert to Celsius.
	ret |= _interface->read_word(BATT_SMBUS_TEMP, result);
	data.temperature = ((float)result * 0.1f) + CONSTANTS_ABSOLUTE_NULL_CELSIUS;

	return ret;

}

template<class T>
void SMBUS_SBS_BaseClass<T>::RunImpl()
{

	// Get the current time.
	uint64_t now = hrt_absolute_time();

	// Read data from sensor.
	battery_status_s new_report = {};

	new_report.id = 1;

	// Set time of reading.
	new_report.timestamp = now;

	new_report.connected = true;

	int ret = populate_smbus_data(new_report);

	new_report.cell_count = _cell_count;

	// Only publish if no errors.
	if (!ret) {
		orb_publish(ORB_ID(battery_status), _batt_topic, &new_report);
	}
}
