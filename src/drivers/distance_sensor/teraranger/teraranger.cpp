/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file teraranger.cpp
 * @author Luis Rodrigues
 *
 * Driver for the TeraRanger One range finders connected via I2C.
 */

#include <drivers/device/i2c.h>
#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_module.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/uORB.h>

using namespace time_literals;

/* Configuration Constants */
#define TERARANGER_BUS_DEFAULT                  PX4_I2C_BUS_EXPANSION
#define TERARANGER_DEVICE_PATH                  "/dev/teraranger"
#define TERARANGER_ONE_BASEADDR                 0x30 // 7-bit address.
#define TERARANGER_EVO_BASEADDR                 0x31 // 7-bit address.

/* TERARANGER Registers addresses */
#define TERARANGER_MEASURE_REG                  0x00 // Measure range register.
#define TERARANGER_WHO_AM_I_REG                 0x01 // Who am I test register.
#define TERARANGER_WHO_AM_I_REG_VAL             0xA1

/* Device limits */
#define TERARANGER_ONE_MAX_DISTANCE             (14.00f)
#define TERARANGER_ONE_MIN_DISTANCE             (0.20f)

#define TERARANGER_EVO_3M_MAX_DISTANCE          (3.0f)
#define TERARANGER_EVO_3M_MIN_DISTANCE          (0.10f)

#define TERARANGER_EVO_60M_MAX_DISTANCE         (60.0f)
#define TERARANGER_EVO_60M_MIN_DISTANCE         (0.50f)

#define TERARANGER_EVO_600HZ_MAX_DISTANCE       (8.0f)
#define TERARANGER_EVO_600HZ_MIN_DISTANCE       (0.75f)

#define TERARANGER_MEASUREMENT_INTERVAL         50_us

class TERARANGER : public device::I2C, public px4::ScheduledWorkItem
{
public:
	TERARANGER(const int bus = TERARANGER_BUS_DEFAULT,
		   const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING,
		   const int address = TERARANGER_ONE_BASEADDR);

	virtual ~TERARANGER();

	virtual int init() override;

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void print_info();

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void stop();

protected:

	virtual int probe() override;

private:

	/**
	 * Collects the most recent sensor measurement data from the i2c bus.
	 */
	int collect();

	/**
	 * Sends an i2c measure command to the sensors.
	 */
	int measure();

	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address The I2C bus address to probe.
	* @return True if the device is present.
	*/
	int probe_address(const uint8_t address);

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void Run() override;

	bool _collect_phase{false};

	int _orb_class_instance{-1};

	uint8_t _orientation{0};

	float _max_distance{0};
	float _min_distance{0};

	orb_advert_t _distance_sensor_topic{nullptr};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "tr1_comm_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "tr1_read")};
};

static const uint8_t crc_table[] = {
	0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
	0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
	0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
	0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
	0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
	0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
	0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
	0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
	0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
	0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
	0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
	0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
	0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
	0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
	0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
	0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
	0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
	0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
	0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
	0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
	0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
	0xfa, 0xfd, 0xf4, 0xf3
};

static uint8_t crc8(uint8_t *p, uint8_t len)
{
	uint16_t i;
	uint16_t crc = 0x0;

	while (len--) {
		i = (crc ^ *p++) & 0xFF;
		crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
	}

	return crc & 0xFF;
}


TERARANGER::TERARANGER(const int bus, const uint8_t orientation, const int address) :
	I2C("TERARANGER", TERARANGER_DEVICE_PATH, bus, address, 100000),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_orientation(orientation)
{
	// up the retries since the device misses the first measure attempts
	I2C::_retries = 3;
}

TERARANGER::~TERARANGER()
{
	// Ensure we are truly inactive.
	stop();

	// Unadvertise the distance sensor topic.
	if (_distance_sensor_topic != nullptr) {
		orb_unadvertise(_distance_sensor_topic);
	}

	// Free perf counters.
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
TERARANGER::collect()
{
	if (!_collect_phase) {
		return measure();
	}

	perf_begin(_sample_perf);

	uint8_t val[3] = {};

	// Transfer data from the bus.
	int ret_val = transfer(nullptr, 0, &val[0], 3);

	if (ret_val < 0) {
		PX4_ERR("error reading from sensor: %d", ret_val);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret_val;
	}

	uint16_t distance_mm = (val[0] << 8) | val[1];
	float distance_m = static_cast<float>(distance_mm) * 1e-3f;

	distance_sensor_s report {};
	report.current_distance = distance_m;
	report.id               = 0; // TODO: set proper ID.
	report.min_distance     = _min_distance;
	report.max_distance     = _max_distance;
	report.orientation      = _orientation;
	report.signal_quality   = -1;
	report.timestamp        = hrt_absolute_time();
	// NOTE: There is no enum item for a combined LASER and ULTRASOUND which this should be.
	report.type             = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	report.variance         = 0.0f;

	if (crc8(val, 2) == val[2] &&
	    (float)report.current_distance > report.min_distance &&
	    (float)report.current_distance < report.max_distance) {
		int instance_id;
		orb_publish_auto(ORB_ID(distance_sensor), &_distance_sensor_topic, &report, &instance_id, ORB_PRIO_DEFAULT);
	}

	// Next phase is measurement.
	_collect_phase = false;

	perf_count(_sample_perf);
	perf_end(_sample_perf);
	return PX4_OK;
}

int
TERARANGER::init()
{
	int hw_model = 0;
	param_get(param_find("SENS_EN_TRANGER"), &hw_model);

	switch (hw_model) {
	case 0: // Disabled
		PX4_WARN("Disabled");
		return PX4_ERROR;

	case 1: // Autodetect - assume default Teraranger One
		set_device_address(TERARANGER_ONE_BASEADDR);

		if (I2C::init() != OK) {
			set_device_address(TERARANGER_EVO_BASEADDR);

			if (I2C::init() != OK) {
				return PX4_ERROR;

			} else {
				// Assume minimum and maximum possible distances acros Evo family
				_min_distance = TERARANGER_EVO_3M_MIN_DISTANCE;
				_max_distance = TERARANGER_EVO_60M_MAX_DISTANCE;
			}

		} else {
			_min_distance = TERARANGER_ONE_MIN_DISTANCE;
			_max_distance = TERARANGER_ONE_MAX_DISTANCE;
		}

		break;

	case 2: // Teraranger One.
		set_device_address(TERARANGER_ONE_BASEADDR);

		if (I2C::init() != OK) {
			return PX4_ERROR;
		}

		_min_distance = TERARANGER_ONE_MIN_DISTANCE;
		_max_distance = TERARANGER_ONE_MAX_DISTANCE;
		break;

	case 3: // Teraranger Evo60m.
		set_device_address(TERARANGER_EVO_BASEADDR);

		// I2C init (and probe) first.
		if (I2C::init() != OK) {
			return PX4_ERROR;
		}

		_min_distance = TERARANGER_EVO_60M_MIN_DISTANCE;
		_max_distance = TERARANGER_EVO_60M_MAX_DISTANCE;
		break;

	case 4: // Teraranger Evo600Hz.
		set_device_address(TERARANGER_EVO_BASEADDR);

		// I2C init (and probe) first.
		if (I2C::init() != OK) {
			return PX4_ERROR;
		}

		_min_distance = TERARANGER_EVO_600HZ_MIN_DISTANCE;
		_max_distance = TERARANGER_EVO_600HZ_MAX_DISTANCE;
		break;

	case 5: // Teraranger Evo3m.
		set_device_address(TERARANGER_EVO_BASEADDR);

		// I2C init (and probe) first.
		if (I2C::init() != OK) {
			return PX4_ERROR;
		}

		_min_distance = TERARANGER_EVO_3M_MIN_DISTANCE;
		_max_distance = TERARANGER_EVO_3M_MAX_DISTANCE;
		break;

	default:
		PX4_ERR("invalid HW model %d.", hw_model);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
TERARANGER::measure()
{
	// Send the command to begin a measurement.
	const uint8_t cmd = TERARANGER_MEASURE_REG;
	int ret_val = transfer(&cmd, sizeof(cmd), nullptr, 0);

	if (ret_val != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret_val);
		return ret_val;
	}

	_collect_phase = true;
	return PX4_OK;
}

int
TERARANGER::probe()
{
	uint8_t who_am_i = 0;

	const uint8_t cmd = TERARANGER_WHO_AM_I_REG;

	// Can't use a single transfer as Teraranger needs a bit of time for internal processing.
	if (transfer(&cmd, 1, nullptr, 0) == OK) {
		if (transfer(nullptr, 0, &who_am_i, 1) == OK && who_am_i == TERARANGER_WHO_AM_I_REG_VAL) {
			return measure();
		}
	}

	PX4_DEBUG("WHO_AM_I byte mismatch 0x%02x should be 0x%02x\n",
		  (unsigned)who_am_i,
		  TERARANGER_WHO_AM_I_REG_VAL);

	// Not found on any address.
	return -EIO;
}

void
TERARANGER::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u\n", TERARANGER_MEASUREMENT_INTERVAL);
}

void
TERARANGER::Run()
{
	// Perform data collection.
	collect();
}

void
TERARANGER::start()
{
	_collect_phase = false;

	// Schedule the driver to run on a set interval
	ScheduleOnInterval(TERARANGER_MEASUREMENT_INTERVAL);
}

void
TERARANGER::stop()
{
	ScheduleClear();
}

/**
 * Local functions in support of the shell command.
 */
namespace teraranger
{

TERARANGER *g_dev;

int start(const uint8_t orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
int start_bus(const int i2c_bus = TERARANGER_BUS_DEFAULT,
	      const uint8_t orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
int status();
int stop();
int usage();

/**
 *
 * Attempt to start driver on all available I2C busses.
 *
 * This function will return as soon as the first sensor
 * is detected on one of the available busses or if no
 * sensors are detected.
 *
 */
int
start(const uint8_t orientation)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	for (size_t i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
		if (start_bus(i2c_bus_options[i], orientation) == PX4_OK) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

/**
 * Start the driver on a specific bus.
 *
 * This function only returns if the sensor is up and running
 * or could not be detected successfully.
 */
int
start_bus(const int i2c_bus, const uint8_t orientation)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

	// Instantiate the driver.
	g_dev = new TERARANGER(i2c_bus, orientation);

	if (g_dev == nullptr) {
		delete g_dev;
		return PX4_ERROR;
	}

	// Initialize the sensor.
	if (g_dev->init() != PX4_OK) {
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	// Start the driver.
	g_dev->start();

	PX4_INFO("driver started");
	return PX4_OK;
}

/**
 * Stop the driver
 */
int
stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	}

	PX4_INFO("driver stopped");
	return PX4_OK;
}

/**
 * Print the driver status.
 */
int
status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	g_dev->print_info();

	return PX4_OK;
}

int
usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

I2C bus driver for TeraRanger rangefinders.

The sensor/driver must be enabled using the parameter SENS_EN_TRANGER.

Setup/usage information: https://docs.px4.io/en/sensor/rangefinders.html#teraranger-rangefinders

### Examples
Start driver on any bus (start on bus where first sensor found).
$ teraranger start -a
Start driver on specified bus
$ teraranger start -b 1
Stop driver
$ teraranger stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("teraranger", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
	PRINT_MODULE_USAGE_PARAM_FLAG('a', "Attempt to start driver on all I2C buses (first one found)", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 1, 1, 2000, "Start driver on specific I2C bus", true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 1, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status","Print driver information");

	return PX4_OK;
}

} // namespace

/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int teraranger_main(int argc, char *argv[])
{
	const char *myoptarg = nullptr;

	int ch;
	int i2c_bus = TERARANGER_BUS_DEFAULT;
	int myoptind = 1;

	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	bool start_all = false;

	while ((ch = px4_getopt(argc, argv, "ab:R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'a':
			start_all = true;
			break;

		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		default:
			PX4_WARN("Unknown option!");
			return teraranger::usage();
		}
	}

	if (myoptind >= argc) {
		return teraranger::usage();
	}

	// Start/load the driver.
	if (!strcmp(argv[myoptind], "start")) {

		if (start_all) {
			return teraranger::start(rotation);

		} else {
			return teraranger::start_bus(i2c_bus, rotation);
		}
	}

	// Print the driver status.
	if (!strcmp(argv[myoptind], "status")) {
		return teraranger::status();
	}

	// Stop the driver.
	if (!strcmp(argv[myoptind], "stop")) {
		return teraranger::stop();
	}

	return teraranger::usage();
}
