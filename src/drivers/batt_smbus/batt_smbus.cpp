/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file batt_smbus.cpp
 *
 * Driver for a battery monitor connected via SMBus (I2C).
 *
 * @author Randy Mackay <rmackay9@yahoo.com>
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sched.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <ctype.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <board_config.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/battery_status.h>

#include <float.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_batt_smbus.h>
#include <drivers/device/ringbuffer.h>

#define BATT_SMBUS_ADDR_MIN             0x08	///< lowest possible address
#define BATT_SMBUS_ADDR_MAX             0x7F	///< highest possible address

#define BATT_SMBUS_I2C_BUS              PX4_I2C_BUS_EXPANSION
#define BATT_SMBUS_ADDR                 0x0B	///< I2C address
#define BATT_SMBUS_TEMP                 0x08	///< temperature register
#define BATT_SMBUS_VOLTAGE              0x09	///< voltage register
#define BATT_SMBUS_REMAINING_CAPACITY	0x0f	///< predicted remaining battery capacity as a percentage
#define BATT_SMBUS_FULL_CHARGE_CAPACITY 0x10    ///< capacity when fully charged
#define BATT_SMBUS_DESIGN_CAPACITY	0x18	///< design capacity register
#define BATT_SMBUS_DESIGN_VOLTAGE	0x19	///< design voltage register
#define BATT_SMBUS_MANUFACTURE_DATE   0x1B  ///< manufacture date register
#define BATT_SMBUS_SERIAL_NUMBER      0x1C  ///< serial number register
#define BATT_SMBUS_MANUFACTURER_NAME	0x20	///< manufacturer name
#define BATT_SMBUS_DEVICE_NAME        0x21  ///< device name register
#define BATT_SMBUS_DEVICE_CHEMISTRY   0x22  ///< device chemistry register
#define BATT_SMBUS_MANUFACTURER_DATA		0x23	///< manufacturer data
#define BATT_SMBUS_MANUFACTURE_INFO	0x25	///< cell voltage register
#define BATT_SMBUS_CURRENT              0x2a	///< current register
#define BATT_SMBUS_MEASUREMENT_INTERVAL_US	(1000000 / 10)	///< time in microseconds, measure at 10Hz
#define BATT_SMBUS_TIMEOUT_US			10000000	///< timeout looking for battery 10seconds after startup

#define BATT_SMBUS_BUTTON_DEBOUNCE_MS	300		///< button holds longer than this time will cause a power off event

#define BATT_SMBUS_MANUFACTURER_ACCESS	0x00
#define BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS    0x44

#define BATT_SMBUS_PEC_POLYNOMIAL	0x07	///< Polynomial for calculating PEC

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

struct battery_type {
	char *ManufacturerName;
	char *DeviceName;
	char *DeviceChemistry;
};

// Declaration of the solo battery data, as determined by reading out the data from multiple 3DR Solo batteries
const struct battery_type solo_battery = {(char *)"BMTPOW", (char *)"MA03", (char *)"LIon"};

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
	 * ioctl for retrieving battery capacity and time to empty
	 */
	virtual int     ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Test device
	 *
	 * @return 0 on success, error code on failure
	 */
	virtual int		test();

	/**
	 * Search all possible slave addresses for a smart battery
	 */
	int			search();

	/**
	 * Get the SBS manufacturer name of the battery device
	 *
	 * @param manufacturer_name pointer a buffer into which the manufacturer name is to be written
	* @param max_length the maximum number of bytes to attempt to read from the manufacturer name register, including the null character that is appended to the end
	 *
	 * @return the number of bytes read
	 */
	uint8_t     manufacturer_name(uint8_t *man_name, uint8_t max_length);

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
	uint16_t  manufacture_date();

	/**
	 * Get the SBS device name of the battery device
	 *
	 * @param dev_name pointer a buffer into which the device name is to be written
	* @param max_length the maximum number of bytes to attempt to read from the device name register, including the null character that is appended to the end
	 *
	 * @return the number of bytes read
	 */
	uint8_t     device_name(uint8_t *dev_name, uint8_t max_length);

	/**
	 * Return the SBS serial number of the battery device
	 */
	uint16_t     serial_number();

	/**
	 * Get the SBS device chemistry of the battery device
	 *
	 * @param dev_chem pointer a buffer into which the device chemistry is to be written
	* @param max_length the maximum number of bytes to attempt to read from the device chemistry register, including the null character that is appended to the end
	 *
	 * @return the number of bytes read
	 */
	uint8_t     device_chemistry(uint8_t *dev_chem, uint8_t max_length);

	/**
	 * Checks whether the current SBS battery data corresponds to a 3DR Solo battery
	 */
	bool is_solo_battery();

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
	int			read_reg(uint8_t reg, uint16_t &val);

	/**
	 * Write a word to specified register
	 */
	int			write_reg(uint8_t reg, uint16_t val);

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
	uint8_t			get_PEC(uint8_t cmd, bool reading, const uint8_t buff[], uint8_t len) const;

	/**
	 * Write a word to Manufacturer Access register (0x00)
	 * @param cmd the word to be written to Manufacturer Access
	 */
	uint8_t			ManufacturerAccess(uint16_t cmd);

	/**
	 * Checks if the battery that has been detected is a 3DR Solo Battery. If it is, it sets
	 * the private variable _is_solo_battery to be true
	 */
	void        check_if_solo_battery();

	// internal variables
	bool			_enabled;	///< true if we have successfully connected to battery
	work_s			_work;		///< work queue for scheduling reads
	RingBuffer		*_reports;	///< buffer of recorded voltages, currents
	struct battery_status_s _last_report;	///< last published report, used for test()
	orb_advert_t		_batt_topic;	///< uORB battery topic
	orb_id_t		_batt_orb_id;	///< uORB battery topic ID
	uint64_t		_start_time;	///< system time we first attempt to communicate with battery
	uint16_t		_batt_capacity;	///< battery's design capacity in mAh (0 means unknown)
	char           *_manufacturer_name;  ///< The name of the battery manufacturer
	char           *_device_name;  ///< The name of the battery device
	char           *_device_chemistry;  ///< The battery chemistry
	bool            _is_solo_battery; ///< Boolean as to whether the battery detected is a 3DR Solo Battery or not
	uint8_t			_button_press_counts; ///< count of button presses detected on 3DR Solo Battery
};

namespace
{
BATT_SMBUS *g_batt_smbus;	///< device handle. For now, we only support one BATT_SMBUS device
}

void batt_smbus_usage();

extern "C" __EXPORT int batt_smbus_main(int argc, char *argv[]);

int manufacturer_name();
int manufacture_date();
int device_name();
int serial_number();
int device_chemistry();
int solo_battery_check();

BATT_SMBUS::BATT_SMBUS(int bus, uint16_t batt_smbus_addr) :
	I2C("batt_smbus", BATT_SMBUS0_DEVICE_PATH, bus, batt_smbus_addr, 100000),
	_enabled(false),
	_work{},
	_reports(nullptr),
	_batt_topic(-1),
	_batt_orb_id(nullptr),
	_start_time(0),
	_batt_capacity(0),
	_manufacturer_name(nullptr),
	_device_name(nullptr),
	_device_chemistry(nullptr),
	_is_solo_battery(false),
	_button_press_counts(0)
{
	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));

	// capture startup time
	_start_time = hrt_absolute_time();
}

BATT_SMBUS::~BATT_SMBUS()
{
	// make sure we are truly inactive
	stop();

	if (_reports != nullptr) {
		delete _reports;
	}

	if (_manufacturer_name != nullptr) {
		delete _manufacturer_name;
	}

	if (_device_name != nullptr) {
		delete _device_name;
	}

	if (_device_chemistry != nullptr) {
		delete _device_chemistry;
	}
}

int
BATT_SMBUS::init()
{
	int ret = ENOTTY;

	// attempt to initialise I2C bus
	ret = I2C::init();

	if (ret != OK) {
		errx(1, "failed to init I2C");
		return ret;

	} else {
		// allocate basic report buffers
		_reports = new RingBuffer(2, sizeof(struct battery_status_s));

		if (_reports == nullptr) {
			ret = ENOTTY;

		} else {
			// start work queue
			start();
		}
	}

	// init orb id
	_batt_orb_id = ORB_ID(battery_status);

	return ret;
}

int
BATT_SMBUS::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int ret = -ENODEV;

	switch (cmd) {
	case BATT_SMBUS_GET_CAPACITY:

		/* return battery capacity as uint16 */
		if (_enabled) {
			*((uint16_t *)arg) = _batt_capacity;
			ret = OK;
		}

		break;

	default:
		/* see if the parent class can make any use of it */
		ret = CDev::ioctl(filp, cmd, arg);
		break;
	}

	return ret;
}

int
BATT_SMBUS::test()
{
	int sub = orb_subscribe(ORB_ID(battery_status));
	bool updated = false;
	struct battery_status_s status;
	uint64_t start_time = hrt_absolute_time();

	// loop for 5 seconds
	while ((hrt_absolute_time() - start_time) < 5000000) {

		// display new info that has arrived from the orb
		orb_check(sub, &updated);

		if (updated) {
			if (orb_copy(ORB_ID(battery_status), sub, &status) == OK) {
				warnx("V=%4.2f C=%4.2f DismAh=%4.2f Cap:%d Shutdown:%d", (double)status.voltage_v, (double)status.current_a,
				      (double)status.discharged_mah, (int)_batt_capacity, (int)status.is_powering_off);
			}
		}

		// sleep for 0.05 seconds
		usleep(50000);
	}

	return OK;
}

int
BATT_SMBUS::search()
{
	bool found_slave = false;
	uint16_t tmp;
	int16_t orig_addr = get_address();

	// search through all valid SMBus addresses
	for (uint8_t i = BATT_SMBUS_ADDR_MIN; i <= BATT_SMBUS_ADDR_MAX; i++) {
		set_address(i);

		if (read_reg(BATT_SMBUS_VOLTAGE, tmp) == OK) {
			warnx("battery found at 0x%x", (int)i);
			found_slave = true;
		}

		// short sleep
		usleep(1);
	}

	// restore original i2c address
	set_address(orig_addr);

	// display completion message
	if (found_slave) {
		warnx("Done.");

	} else {
		warnx("No smart batteries found.");
	}

	return OK;
}

uint8_t
BATT_SMBUS::manufacturer_name(uint8_t *man_name, uint8_t max_length)
{
	uint8_t len = read_block(BATT_SMBUS_MANUFACTURER_NAME, man_name, max_length, false);

	if (len > 0) {
		if (len >= max_length - 1) {
			man_name[max_length - 1] = 0;

		} else {
			man_name[len] = 0;
		}
	}

	return len;
}

uint16_t
BATT_SMBUS::manufacture_date()
{
	uint16_t man_date;

	if (read_reg(BATT_SMBUS_MANUFACTURE_DATE, man_date) == OK) {
		return man_date;
	}

	// Return 0 if could not read the date correctly
	return 0;
}

uint8_t
BATT_SMBUS::device_name(uint8_t *dev_name, uint8_t max_length)
{
	uint8_t len = read_block(BATT_SMBUS_DEVICE_NAME, dev_name, max_length, false);

	if (len > 0) {
		if (len >= max_length - 1) {
			dev_name[max_length - 1] = 0;

		} else {
			dev_name[len] = 0;
		}
	}

	return len;
}

uint16_t
BATT_SMBUS::serial_number()
{
	uint16_t serial_num;

	if (read_reg(BATT_SMBUS_SERIAL_NUMBER, serial_num) == OK) {
		return serial_num;
	}

	return -1;
}

uint8_t
BATT_SMBUS::device_chemistry(uint8_t *dev_chem, uint8_t max_length)
{
	uint8_t len = read_block(BATT_SMBUS_DEVICE_CHEMISTRY, dev_chem, max_length, false);

	if (len > 0) {
		if (len >= max_length - 1) {
			dev_chem[max_length - 1] = 0;

		} else {
			dev_chem[len] = 0;
		}
	}

	return len;
}

bool
BATT_SMBUS::is_solo_battery()
{
	check_if_solo_battery();
	return _is_solo_battery;
}

int
BATT_SMBUS::probe()
{
	// always return OK to ensure device starts
	return OK;
}

void
BATT_SMBUS::start()
{
	// reset the report ring and state machine
	_reports->flush();

	// schedule a cycle to start things
	work_queue(HPWORK, &_work, (worker_t)&BATT_SMBUS::cycle_trampoline, this, 1);
}

void
BATT_SMBUS::stop()
{
	work_cancel(HPWORK, &_work);
}

void
BATT_SMBUS::cycle_trampoline(void *arg)
{
	BATT_SMBUS *dev = (BATT_SMBUS *)arg;

	dev->cycle();
}

void
BATT_SMBUS::cycle()
{
	// get current time
	uint64_t now = hrt_absolute_time();

	// exit without rescheduling if we have failed to find a battery after 10 seconds
	if (!_enabled && (now - _start_time > BATT_SMBUS_TIMEOUT_US)) {
		warnx("did not find smart battery");
		return;
	}

	bool perform_solo_battry_check = false; // Only check if it is a solo battery if changes have been made to the SBS data

	// Try and get battery SBS info
	if (_manufacturer_name == nullptr) {
		char man_name[21];
		uint8_t len = manufacturer_name((uint8_t *)man_name, sizeof(man_name));

		if (len > 0) {
			_manufacturer_name = new char[len];
			strcpy(_manufacturer_name, man_name);
			perform_solo_battry_check = true;
		}
	}

	if (_device_name == nullptr) {
		char dev_name[21];
		uint8_t len = device_name((uint8_t *)dev_name, sizeof(dev_name));

		if (len > 0) {
			_device_name = new char[len];
			strcpy(_device_name, dev_name);
			perform_solo_battry_check = true;
		}
	}

	if (_device_chemistry == nullptr) {
		char dev_chem[21];
		uint8_t len = device_chemistry((uint8_t *)dev_chem, sizeof(dev_chem));

		if (len > 0) {
			_device_chemistry = new char[len];
			strcpy(_device_chemistry, dev_chem);
			perform_solo_battry_check = true;
		}
	}

	// If necessary, check if the battery is a 3DR Solo Battery
	if (perform_solo_battry_check) {
		warnx("Checking solo battery");
		check_if_solo_battery();
	}

	// read data from sensor
	struct battery_status_s new_report;

	// set time of reading
	new_report.timestamp = now;

	// read voltage
	uint16_t tmp;

	if (read_reg(BATT_SMBUS_VOLTAGE, tmp) == OK) {
		// initialise new_report
		memset(&new_report, 0, sizeof(new_report));

		// convert millivolts to volts
		new_report.voltage_v = ((float)tmp) / 1000.0f;

		// read current
		uint8_t buff[6];

		if (read_block(BATT_SMBUS_CURRENT, buff, 4, false) == 4) {
			new_report.current_a = -(float)((int32_t)((uint32_t)buff[3] << 24 | (uint32_t)buff[2] << 16 | (uint32_t)buff[1] << 8 |
							(uint32_t)buff[0])) / 1000.0f;
		}

		// read battery design capacity
		if (_batt_capacity == 0) {
			if (read_reg(BATT_SMBUS_FULL_CHARGE_CAPACITY, tmp) == OK) {
				_batt_capacity = tmp;
			}
		}

		// read remaining capacity
		if (_batt_capacity > 0) {
			if (read_reg(BATT_SMBUS_REMAINING_CAPACITY, tmp) == OK) {
				if (tmp < _batt_capacity) {
					new_report.discharged_mah = _batt_capacity - tmp;
				}
			}
		}

		// if it is a solo battery, check for shutdown on button press
		if (_is_solo_battery) {
			// read the button press indicator
			if (read_block(BATT_SMBUS_MANUFACTURER_DATA, buff, 6, false) == 6) {
				bool pressed = (buff[1] >> 3) & 0x01;

				if (_button_press_counts >= ((BATT_SMBUS_BUTTON_DEBOUNCE_MS * 1000) / BATT_SMBUS_MEASUREMENT_INTERVAL_US)) {
					// battery will power off
					new_report.is_powering_off = true;

					// warn only once
					if (_button_press_counts++ == ((BATT_SMBUS_BUTTON_DEBOUNCE_MS * 1000) / BATT_SMBUS_MEASUREMENT_INTERVAL_US)) {
						warnx("system is shutting down NOW...");
					}

				} else if (pressed) {
					// battery will power off if the button is held
					_button_press_counts++;

				} else {
					// button released early, reset counters
					_button_press_counts = 0;
					new_report.is_powering_off = false;
				}
			}
		}


		// publish to orb
		if (_batt_topic != -1) {
			orb_publish(_batt_orb_id, _batt_topic, &new_report);

		} else {
			_batt_topic = orb_advertise(_batt_orb_id, &new_report);

			if (_batt_topic < 0) {
				errx(1, "ADVERT FAIL");
			}
		}

		// copy report for test()
		_last_report = new_report;

		// post a report to the ring
		_reports->force(&new_report);

		// notify anyone waiting for data
		poll_notify(POLLIN);

		// record we are working
		_enabled = true;
	}

	// schedule a fresh cycle call when the measurement is done
	work_queue(HPWORK, &_work, (worker_t)&BATT_SMBUS::cycle_trampoline, this,
		   USEC2TICK(BATT_SMBUS_MEASUREMENT_INTERVAL_US));
}

int
BATT_SMBUS::read_reg(uint8_t reg, uint16_t &val)
{
	uint8_t buff[3];	// 2 bytes of data + PEC

	// read from register
	int ret = transfer(&reg, 1, buff, 3);

	if (ret == OK) {
		// check PEC
		uint8_t pec = get_PEC(reg, true, buff, 2);

		if (pec == buff[2]) {
			val = (uint16_t)buff[1] << 8 | (uint16_t)buff[0];

		} else {
			ret = ENOTTY;
		}
	}

	// return success or failure
	return ret;
}

int
BATT_SMBUS::write_reg(uint8_t reg, uint16_t val)
{
	uint8_t buff[4];  // reg + 2 bytes of data + PEC

	buff[0] = reg;
	buff[2] = uint8_t(val << 8) & 0xff;
	buff[1] = (uint8_t)val;
	buff[3] = get_PEC(reg, false, &buff[1],  2); // Append PEC

	// write bytes to register
	int ret = transfer(buff, 3, nullptr, 0);

	if (ret != OK) {
		debug("Register write error");
	}

	// return success or failure
	return ret;
}

uint8_t
BATT_SMBUS::read_block(uint8_t reg, uint8_t *data, uint8_t max_len, bool append_zero)
{
	uint8_t buff[max_len + 2];  // buffer to hold results

	// read bytes including PEC
	int ret = transfer(&reg, 1, buff, max_len + 2);

	// return zero on failure
	if (ret != OK) {
		return 0;
	}

	// get length
	uint8_t bufflen = buff[0];

	// sanity check length returned by smbus
	if (bufflen == 0 || bufflen > max_len) {
		return 0;
	}

	// check PEC
	uint8_t pec = get_PEC(reg, true, buff, bufflen + 1);

	if (pec != buff[bufflen + 1]) {
		return 0;
	}

	// copy data
	memcpy(data, &buff[1], bufflen);

	// optionally add zero to end
	if (append_zero) {
		data[bufflen] = '\0';
	}

	// return success
	return bufflen;
}

uint8_t
BATT_SMBUS::write_block(uint8_t reg, uint8_t *data, uint8_t len)
{
	uint8_t buff[len + 3];  // buffer to hold results

	usleep(1);

	buff[0] = reg;
	buff[1] = len;
	memcpy(&buff[2], data, len);
	buff[len + 2] = get_PEC(reg, false, &buff[1],  len + 1); // Append PEC

	// send bytes
	int ret = transfer(buff, len + 3, nullptr, 0);

	// return zero on failure
	if (ret != OK) {
		debug("Block write error\n");
		return 0;
	}

	// return success
	return len;
}

uint8_t
BATT_SMBUS::get_PEC(uint8_t cmd, bool reading, const uint8_t buff[], uint8_t len) const
{
	// exit immediately if no data
	if (len <= 0) {
		return 0;
	}

	/**
	 *  Note: The PEC is calculated on all the message bytes. See http://cache.freescale.com/files/32bit/doc/app_note/AN4471.pdf
	 *  and http://www.ti.com/lit/an/sloa132/sloa132.pdf for more details
	 */

	// prepare temp buffer for calculating crc
	uint8_t tmp_buff_len;

	if (reading) {
		tmp_buff_len = len + 3;

	} else {
		tmp_buff_len = len + 2;
	}

	uint8_t tmp_buff[tmp_buff_len];
	tmp_buff[0] = (uint8_t)get_address() << 1;
	tmp_buff[1] = cmd;

	if (reading) {
		tmp_buff[2] = tmp_buff[0] | (uint8_t)reading;
		memcpy(&tmp_buff[3], buff, len);

	} else {
		memcpy(&tmp_buff[2], buff, len);
	}

	// initialise crc to zero
	uint8_t crc = 0;
	uint8_t shift_reg = 0;
	bool do_invert;

	// for each byte in the stream
	for (uint8_t i = 0; i < sizeof(tmp_buff); i++) {
		// load next data byte into the shift register
		shift_reg = tmp_buff[i];

		// for each bit in the current byte
		for (uint8_t j = 0; j < 8; j++) {
			do_invert = (crc ^ shift_reg) & 0x80;
			crc <<= 1;
			shift_reg <<= 1;

			if (do_invert) {
				crc ^= BATT_SMBUS_PEC_POLYNOMIAL;
			}
		}
	}

	// return result
	return crc;
}

uint8_t
BATT_SMBUS::ManufacturerAccess(uint16_t cmd)
{
	// write bytes to Manufacturer Access
	int ret = write_reg(BATT_SMBUS_MANUFACTURER_ACCESS, cmd);

	if (ret != OK) {
		debug("Manufacturer Access error");
	}

	return ret;
}

void
BATT_SMBUS::check_if_solo_battery()
{
	// Check if the SBS information corresponds to that of a 3DR Solo Battery. If, yes, set the solo_battery flag to true;
	if (!strcmp(_manufacturer_name, solo_battery.ManufacturerName) && !strcmp(_device_name, solo_battery.DeviceName)
	    && !strcmp(_device_chemistry, solo_battery.DeviceChemistry)) {
		_is_solo_battery = true;
	}
}

///////////////////////// shell functions ///////////////////////

void
batt_smbus_usage()
{
	warnx("missing command: try 'start', 'test', 'stop', 'search', 'man_name', 'man_date', 'dev_name', 'serial_num', 'dev_chem',  'sbs_info'");
	warnx("options:");
	warnx("    -b i2cbus (%d)", BATT_SMBUS_I2C_BUS);
	warnx("    -a addr (0x%x)", BATT_SMBUS_ADDR);
}

int
manufacturer_name()
{
	uint8_t man_name[21];
	uint8_t len = g_batt_smbus->manufacturer_name(man_name, sizeof(man_name));

	if (len > 0) {
		warnx("The manufacturer name: %s", man_name);
		return OK;

	} else {
		warnx("Unable to read manufacturer name.");
	}

	return -1;
}

int
manufacture_date()
{
	uint16_t man_date = g_batt_smbus->manufacture_date();

	if (man_date > 0) {
		// Convert the uint16_t into human-readable date format
		uint16_t year = ((man_date >> 9) & 0xFF) + 1980;
		uint8_t month = (man_date >> 5) & 0xF;
		uint8_t day = man_date & 0x1F;
		warnx("The manufacturer date is: %d which is %4d-%02d-%02d", man_date, year, month, day);
		return OK;

	} else {
		warnx("Unable to read the manufacturer date.");
	}

	return -1;
}

int
device_name()
{
	uint8_t device_name[21];
	uint8_t len = g_batt_smbus->device_name(device_name, sizeof(device_name));

	if (len > 0) {
		warnx("The device name: %s", device_name);
		return OK;

	} else {
		warnx("Unable to read device name.");
	}

	return -1;
}

int
serial_number()
{
	uint16_t serial_num = g_batt_smbus->serial_number();
	warnx("The serial number: 0x%04x (%d in decimal)", serial_num, serial_num);

	return OK;
}

int
device_chemistry()
{
	uint8_t device_chemistry[5];
	uint8_t len = g_batt_smbus->device_chemistry(device_chemistry, sizeof(device_chemistry));

	if (len > 0) {
		warnx("The device chemistry: %s", device_chemistry);
		return OK;

	} else {
		warnx("Unable to read device chemistry.");
	}

	return -1;
}

int
solo_battery_check()
{
	if (g_batt_smbus->is_solo_battery()) {
		warnx("The battery corresponds to a 3DR Solo Battery");

	} else {
		warnx("The battery does not correspond to a 3DR Solo Battery");
	}

	return OK;
}

int
batt_smbus_main(int argc, char *argv[])
{
	int i2cdevice = BATT_SMBUS_I2C_BUS;
	int batt_smbusadr = BATT_SMBUS_ADDR; // 7bit address

	int ch;

	// jump over start/off/etc and look at options first
	while ((ch = getopt(argc, argv, "a:b:")) != EOF) {
		switch (ch) {
		case 'a':
			batt_smbusadr = strtol(optarg, NULL, 0);
			break;

		case 'b':
			i2cdevice = strtol(optarg, NULL, 0);
			break;

		default:
			batt_smbus_usage();
			exit(0);
		}
	}

	if (optind >= argc) {
		batt_smbus_usage();
		exit(1);
	}

	const char *verb = argv[optind];

	if (!strcmp(verb, "start")) {
		if (g_batt_smbus != nullptr) {
			errx(1, "already started");

		} else {
			// create new global object
			g_batt_smbus = new BATT_SMBUS(i2cdevice, batt_smbusadr);

			if (g_batt_smbus == nullptr) {
				errx(1, "new failed");
			}

			if (OK != g_batt_smbus->init()) {
				delete g_batt_smbus;
				g_batt_smbus = nullptr;
				errx(1, "init failed");
			}
		}

		exit(0);
	}

	// need the driver past this point
	if (g_batt_smbus == nullptr) {
		warnx("not started");
		batt_smbus_usage();
		exit(1);
	}

	if (!strcmp(verb, "test")) {
		g_batt_smbus->test();
		exit(0);
	}

	if (!strcmp(verb, "stop")) {
		delete g_batt_smbus;
		g_batt_smbus = nullptr;
		exit(0);
	}

	if (!strcmp(verb, "search")) {
		g_batt_smbus->search();
		exit(0);
	}

	if (!strcmp(verb, "man_name")) {
		manufacturer_name();
		exit(0);
	}

	if (!strcmp(verb, "man_date")) {
		manufacture_date();
		exit(0);
	}

	if (!strcmp(verb, "dev_name")) {
		device_name();
		exit(0);
	}

	if (!strcmp(verb, "serial_num")) {
		serial_number();
		exit(0);
	}

	if (!strcmp(verb, "dev_chem")) {
		device_chemistry();
		exit(0);
	}

	if (!strcmp(verb, "sbs_info")) {
		manufacturer_name();
		manufacture_date();
		device_name();
		serial_number();
		device_chemistry();
		solo_battery_check();
		exit(0);
	}

	batt_smbus_usage();
	exit(0);
}
