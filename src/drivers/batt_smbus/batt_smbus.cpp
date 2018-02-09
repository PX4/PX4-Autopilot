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
 * Designed for BQ40Z50-R1/R2
 *
 * @author Randy Mackay <rmackay9@yahoo.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 */

#include <float.h>
#include <stdio.h>
#include <string.h>

#include <drivers/drv_hrt.h>

#include <px4_config.h>
#include <px4_workqueue.h>

#include <systemlib/battery.h>
#include <systemlib/perf_counter.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/uORB.h>

#include <drivers/device/i2c.h>

#define BATT_SMBUS_ADDR_MIN             0x00	///< lowest possible address
#define BATT_SMBUS_ADDR_MAX             0xFF	///< highest possible address

#define BATT_SMBUS_I2C_BUS              PX4_I2C_BUS_EXPANSION
#define BATT_SMBUS_ADDR                 0x16	///< Default I2C address
#define BATT_SMBUS_TEMP                 0x08	///< temperature register
#define BATT_SMBUS_VOLTAGE              0x09	///< voltage register
#define BATT_SMBUS_REMAINING_CAPACITY	0x0f	///< predicted remaining battery capacity as a percentage
#define BATT_SMBUS_FULL_CHARGE_CAPACITY 0x10    ///< capacity when fully charged
#define BATT_SMBUS_DESIGN_CAPACITY		0x18	///< design capacity register
#define BATT_SMBUS_DESIGN_VOLTAGE		0x19	///< design voltage register
#define BATT_SMBUS_MANUFACTURE_DATE   	0x1B  	///< manufacture date register
#define BATT_SMBUS_SERIAL_NUMBER      	0x1C  	///< serial number register
#define BATT_SMBUS_MANUFACTURER_NAME	0x20	///< manufacturer name
#define BATT_SMBUS_CURRENT              0x0a	///< current register
#define BATT_SMBUS_MEASUREMENT_INTERVAL_US	(1000000 / 10)	///< time in microseconds, measure at 10Hz
#define BATT_SMBUS_TIMEOUT_US			10000000	///< timeout looking for battery 10seconds after startup

#define BATT_SMBUS_MANUFACTURER_ACCESS	0x00
#define BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS    0x44

#define BATT_SMBUS_PEC_POLYNOMIAL	0x07		///< Polynomial for calculating PEC

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class BATT_SMBUS : public device::I2C
{
public:
	BATT_SMBUS(int bus = PX4_I2C_BUS_EXPANSION, uint16_t batt_smbus_addr = BATT_SMBUS_ADDR, bool pec = false);
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
	 * Return the SBS serial number of the battery device
	 */
	uint16_t     serial_number();

protected:
	/**
	 * Check if the device can be contacted
	 */
	virtual int		probe();

private:

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

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
	 * Convert from 2's compliment to decimal
	 * @return the absolute value of the input in decimal
	 */
	uint16_t	convert_twos_comp(uint16_t val);

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
	uint8_t	get_PEC(uint8_t cmd, bool reading, const uint8_t buff[], uint8_t len);

	// internal variables
	bool			_enabled;	///< true if we have successfully connected to battery
	work_s			_work;		///< work queue for scheduling reads
	orb_advert_t	_battery_pub;	///< uORB battery topic
	orb_id_t		_batt_orb_id;	///< uORB battery topic ID
	uint64_t		_start_time;	///< system time we first attempt to communicate with battery
	uint16_t		_batt_capacity;	///< battery's design capacity in mAh (0 means unknown)
	char           *_manufacturer_name;  ///< The name of the battery manufacturer
	battery_status_s _last_report;	///< last published report, used for test()

	Battery		_battery;			/**< Helper lib to publish battery_status topic. */
	bool		_armed{false};				/**< arming status of the vehicle */

	int		_actuator_ctrl_0_sub{-1};		/**< attitude controls sub */
	int		_vcontrol_mode_sub{-1};		/**< vehicle control mode subscription */

	bool	_enable_pec;		///< PEC enabled or disabled on fuel gauge
};

namespace
{
BATT_SMBUS *g_batt_smbus;	///< device handle. For now, we only support one BATT_SMBUS device
}

void batt_smbus_usage();

extern "C" __EXPORT int batt_smbus_main(int argc, char *argv[]);

int manufacturer_name();
int manufacture_date();
int serial_number();

BATT_SMBUS::BATT_SMBUS(int bus, uint16_t batt_smbus_addr, bool pec) :
	I2C("batt_smbus", "/dev/batt_smbus0", bus, batt_smbus_addr, 100000),
	_enabled(false),
	_work{},
	_battery_pub(nullptr),
	_batt_orb_id(nullptr),
	_start_time(0),
	_batt_capacity(0),
	_manufacturer_name(nullptr),
	_enable_pec(pec)
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

	if (_manufacturer_name != nullptr) {
		delete[] _manufacturer_name;
	}
}

void
BATT_SMBUS::vehicle_control_mode_poll()
{
	struct vehicle_control_mode_s vcontrol_mode;
	bool vcontrol_mode_updated;

	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &vcontrol_mode);
		_armed = vcontrol_mode.flag_armed;
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
		//Find the battery on the bus
		search();

		// start work queue
		start();
	}

	// init orb id
	_batt_orb_id = ORB_ID(battery_status);

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
				PX4_INFO("V=%4.2f C=%4.2f DismAh=%4.2f Cap:%d Shutdown:%d", (double)status.voltage_v, (double)status.current_a,
					 (double)status.discharged_mah, (int)_batt_capacity);
			}
		}

		// sleep for 0.05 seconds
		usleep(100000);
	}

	return OK;
}

int
BATT_SMBUS::search()
{
	bool found_slave = false;
	uint16_t tmp;
	int16_t orig_addr = get_device_address();

	// search through all valid SMBus addresses
	for (uint8_t i = BATT_SMBUS_ADDR_MIN; i < BATT_SMBUS_ADDR_MAX; i++) {
		set_device_address(i);

		if (read_reg(BATT_SMBUS_VOLTAGE, tmp) == OK) {
			if (tmp > 0) {
				PX4_INFO("battery found at 0x%x", (int)i);
				found_slave = true;
				break;
			}
		}

		// short sleep
		usleep(1);
	}

	if (found_slave == false) {
		// restore original i2c address
		set_device_address(orig_addr);
	}

	// display completion message
	if (found_slave) {
		PX4_INFO("smart battery connected");

	} else {
		PX4_INFO("No smart batteries found.");
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

uint16_t
BATT_SMBUS::serial_number()
{
	uint16_t serial_num;

	if (read_reg(BATT_SMBUS_SERIAL_NUMBER, serial_num) == OK) {
		return serial_num;
	}

	return -1;
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
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_actuator_ctrl_0_sub = orb_subscribe(ORB_ID(actuator_controls_0));

	// schedule a cycle to start things
	work_queue(HPWORK, &_work, (worker_t)&BATT_SMBUS::cycle_trampoline, this, 1);
}

void
BATT_SMBUS::stop()
{
	orb_unsubscribe(_actuator_ctrl_0_sub);
	orb_unsubscribe(_vcontrol_mode_sub);
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

	/* check vehicle status for changes to publication state */
	vehicle_control_mode_poll();

	// exit without rescheduling if we have failed to find a battery after 10 seconds
	if (!_enabled && (now - _start_time > BATT_SMBUS_TIMEOUT_US)) {
		PX4_INFO("did not find smart battery");
		return;
	}

	// Try and get battery SBS info
	if (_manufacturer_name == nullptr) {
		char man_name[21];
		uint8_t len = manufacturer_name((uint8_t *)man_name, sizeof(man_name));

		if (len > 0) {
			_manufacturer_name = new char[len + 1];
			strcpy(_manufacturer_name, man_name);
		}
	}

	// read data from sensor
	struct battery_status_s new_report;

	// set time of reading
	new_report.timestamp = now;

	// read voltage
	uint16_t tmp;

	if (read_reg(BATT_SMBUS_VOLTAGE, tmp) == OK) {

		//Consider the battery connected if voltage returns true
		bool connected = true;

		// initialise new_report
		memset(&new_report, 0, sizeof(new_report));

		// convert millivolts to volts
		new_report.voltage_v = ((float)tmp) / 1000.0f;

		// read current
		if (read_reg(BATT_SMBUS_CURRENT, tmp) == OK) {
			new_report.current_a = ((float)convert_twos_comp(tmp)) / 1000.0f;
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

		// publish to orb
		actuator_controls_s ctrl;
		orb_copy(ORB_ID(actuator_controls_0), _actuator_ctrl_0_sub, &ctrl);

		battery_status_s battery_status;
		_battery.updateBatteryStatus(now, new_report.voltage_v, new_report.current_a,
					     connected, 0, 0,
					     ctrl.control[actuator_controls_s::INDEX_THROTTLE],
					     _armed, &battery_status);
		int instance;
		orb_publish_auto(ORB_ID(battery_status), &_battery_pub, &battery_status, &instance, ORB_PRIO_DEFAULT);

		// copy report for test()
		_last_report = new_report;

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
	uint8_t buff[3];	// 2 bytes of data

	// read from register
	int ret = transfer(&reg, 1, buff, 3);

	if (ret == OK) {
		if (_enable_pec) {
			// check PEC
			uint8_t pec = get_PEC(reg, true, buff, 2);

			if (pec == buff[2]) {
				val = (uint16_t)buff[1] << 8 | (uint16_t)buff[0];

			} else {
				ret = ENOTTY;
			}

		} else {
			val = (uint16_t)buff[1] << 8 | (uint16_t)buff[0];
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

	// write bytes to register
	int ret = transfer(buff, 3, nullptr, 0);

	if (ret != OK) {
		PX4_WARN("Register write error");
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
	//buff[len + 2] = get_PEC(reg, false, &buff[1],  len + 1); // Append PEC

	// send bytes
	int ret = transfer(buff, len + 3, nullptr, 0);

	// return zero on failure
	if (ret != OK) {
		PX4_WARN("Block write error\n");
		return 0;
	}

	// return success
	return len;
}

uint8_t
BATT_SMBUS::get_PEC(uint8_t cmd, bool reading, const uint8_t buff[], uint8_t len)
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
	tmp_buff[0] = (uint8_t)get_device_address() << 1;
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

uint16_t
BATT_SMBUS::convert_twos_comp(uint16_t val)
{
	if ((val & 0x8000) == 0x8000) {
		uint16_t tmp;
		tmp = ~val;
		tmp = tmp + 1;
		return tmp;
	}

	return val;
}

/* uint8_t
BATT_SMBUS::ManufacturerAccess(uint16_t cmd)
{
	// write bytes to Manufacturer Access
	int ret = write_reg(BATT_SMBUS_MANUFACTURER_ACCESS, cmd);

	if (ret != OK) {
		PX4_WARN("Manufacturer Access error");
	}

	return ret;
} */

///////////////////////// shell functions ///////////////////////

void
batt_smbus_usage()
{
	PX4_INFO("missing command: try 'start', 'test', 'stop', 'search', 'man_name', 'man_date', 'dev_name', 'serial_num', 'dev_chem',  'sbs_info', 'current'");
	PX4_INFO("options:");
	PX4_INFO("    -b i2cbus (%d)", BATT_SMBUS_I2C_BUS);
	PX4_INFO("    -a addr (0x%x)", BATT_SMBUS_ADDR);
	PX4_INFO("    -p Enable PEC");
}

int
manufacturer_name()
{
	uint8_t man_name[21];
	uint8_t len = g_batt_smbus->manufacturer_name(man_name, sizeof(man_name));

	if (len > 0) {
		PX4_INFO("The manufacturer name: %s", man_name);
		return OK;

	} else {
		PX4_INFO("Unable to read manufacturer name.");
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
		PX4_INFO("The manufacturer date is: %d which is %4d-%02d-%02d", man_date, year, month, day);
		return OK;

	} else {
		PX4_INFO("Unable to read the manufacturer date.");
	}

	return -1;
}

int
serial_number()
{
	uint16_t serial_num = g_batt_smbus->serial_number();
	PX4_INFO("The serial number: 0x%04x (%d in decimal)", serial_num, serial_num);

	return OK;
}

int
batt_smbus_main(int argc, char *argv[])
{
	int i2cdevice = BATT_SMBUS_I2C_BUS;
	int batt_smbusadr = BATT_SMBUS_ADDR; // 7bit address
	bool pec_enable = false; //PEC disabled by default

	int ch;

	// jump over start/off/etc and look at options first
	while ((ch = getopt(argc, argv, "a:b:p")) != EOF) {
		switch (ch) {
		case 'a':
			batt_smbusadr = strtol(optarg, nullptr, 0);
			break;

		case 'b':
			i2cdevice = strtol(optarg, nullptr, 0);
			break;

		case 'p':
			pec_enable = true;

		default:
			batt_smbus_usage();
			return 0;
		}
	}

	if (optind >= argc) {
		batt_smbus_usage();
		return 1;
	}

	const char *verb = argv[optind];

	if (!strcmp(verb, "start")) {
		if (g_batt_smbus != nullptr) {
			errx(1, "already started");

		} else {
			// create new global object
			g_batt_smbus = new BATT_SMBUS(i2cdevice, batt_smbusadr, pec_enable);

			if (g_batt_smbus == nullptr) {
				errx(1, "new failed");
			}

			if (OK != g_batt_smbus->init()) {
				delete g_batt_smbus;
				g_batt_smbus = nullptr;
				errx(1, "init failed");
			}
		}

		return 0;
	}

	// need the driver past this point
	if (g_batt_smbus == nullptr) {
		PX4_INFO("not started");
		batt_smbus_usage();
		return 1;
	}

	if (!strcmp(verb, "test")) {
		g_batt_smbus->test();
		return 0;
	}

	if (!strcmp(verb, "stop")) {
		delete g_batt_smbus;
		g_batt_smbus = nullptr;
		return 0;
	}

	if (!strcmp(verb, "search")) {
		g_batt_smbus->search();
		return 0;
	}

	if (!strcmp(verb, "man_name")) {
		manufacturer_name();
		return 0;
	}

	if (!strcmp(verb, "man_date")) {
		manufacture_date();
		return 0;
	}

	if (!strcmp(verb, "serial_num")) {
		serial_number();
		return 0;
	}

	if (!strcmp(verb, "sbs_info")) {
		manufacturer_name();
		manufacture_date();
		serial_number();
		return 0;
	}

	batt_smbus_usage();
	return 0;
}
