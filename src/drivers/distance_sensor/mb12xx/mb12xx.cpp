/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file mb12xx.cpp
 * @author Greg Hulands
 * @author Jon Verbeke <jon.verbeke@kuleuven.be>
 *
 * Driver for the Maxbotix sonar range finders connected via I2C.
 */

#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <board_config.h>
#include <containers/Array.hpp>
#include <drivers/device/device.h>
#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

using namespace time_literals;

/* Configuration Constants */
#define MB12XX_BASE_ADDR                        0x70   // 7-bit address is 0x70 = 112. 8-bit address is 0xE0 = 224.
#define MB12XX_MIN_ADDR                         0x5A   // 7-bit address is 0x5A = 90.  8-bit address is 0xB4 = 180.
#define MB12XX_BUS_DEFAULT                      PX4_I2C_BUS_EXPANSION
#define MB12XX_BUS_SPEED                        100000 // 100kHz bus speed.
#define MB12XX_DEVICE_PATH                      "/dev/mb12xx"

/* MB12xx Registers addresses */
#define MB12XX_TAKE_RANGE_REG                   0x51 // Measure range Register.
#define MB12XX_SET_ADDRESS_1                    0xAA // Change address 1 Register.
#define MB12XX_SET_ADDRESS_2                    0xA5 // Change address 2 Register.

/* Device limits */
#define MB12XX_MIN_DISTANCE                     (0.20f)
#define MB12XX_MAX_DISTANCE                     (7.65f)

#define MB12XX_MEASURE_INTERVAL                 100_ms // 60ms minimum for one sonar.
#define MB12XX_INTERVAL_BETWEEN_SUCCESIVE_FIRES 100_ms // 30ms minimum between each sonar measurement (watch out for interference!).

class MB12XX : public device::I2C, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	MB12XX(const int bus = MB12XX_BUS_DEFAULT);
	virtual ~MB12XX();

	virtual int init() override;

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

	/**
	 * Sets a new device address.
	 * @param address The new sensor address to be set: 200-224 even addresses only.
	 * @return Returns PX4_OK iff successful, PX4_ERROR otherwise.
	 */
	int set_address(const uint8_t address = MB12XX_BASE_ADDR);

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

private:

	/**
	 * Collects the most recent sensor measurement data from the i2c bus.
	 */
	int collect();

	/**
	 * Gets the current sensor rotation value.
	 */
	int get_sensor_rotation(const size_t index);

	/**
	 * Sends an i2c measure command to start the next sonar ping.
	 */
	int measure();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void Run() override;

	px4::Array<uint8_t, RANGE_FINDER_MAX_SENSORS> _sensor_addresses {};
	px4::Array<uint8_t, RANGE_FINDER_MAX_SENSORS> _sensor_rotations {};

	int _measure_interval{MB12XX_MEASURE_INTERVAL};	// Initialize the measure interval for a single sensor.
	int _orb_class_instance{-1};

	size_t _sensor_index{0};	// Initialize counter for cycling i2c adresses to zero.

	size_t _sensor_count{0};

	orb_advert_t _distance_sensor_topic{nullptr};

	perf_counter_t _comms_error{perf_alloc(PC_ELAPSED, "mb12xx_comms_error")};
	perf_counter_t _sample_perf{perf_alloc(PC_COUNT, "mb12xx_sample_perf")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_EN_MB12XX>)   _p_sensor_enabled,
		(ParamInt<px4::params::SENS_MB12_0_ROT>)  _p_sensor0_rot,
		(ParamInt<px4::params::SENS_MB12_1_ROT>)  _p_sensor1_rot,
		(ParamInt<px4::params::SENS_MB12_2_ROT>)  _p_sensor2_rot,
		(ParamInt<px4::params::SENS_MB12_3_ROT>)  _p_sensor3_rot,
		(ParamInt<px4::params::SENS_MB12_4_ROT>)  _p_sensor4_rot,
		(ParamInt<px4::params::SENS_MB12_5_ROT>)  _p_sensor5_rot,
		(ParamInt<px4::params::SENS_MB12_6_ROT>)  _p_sensor6_rot,
		(ParamInt<px4::params::SENS_MB12_7_ROT>)  _p_sensor7_rot,
		(ParamInt<px4::params::SENS_MB12_8_ROT>)  _p_sensor8_rot,
		(ParamInt<px4::params::SENS_MB12_9_ROT>)  _p_sensor9_rot,
		(ParamInt<px4::params::SENS_MB12_10_ROT>) _p_sensor10_rot,
		(ParamInt<px4::params::SENS_MB12_11_ROT>) _p_sensor11_rot
	);
};

MB12XX::MB12XX(const int bus) :
	I2C("MB12xx", MB12XX_DEVICE_PATH, bus, MB12XX_BASE_ADDR, MB12XX_BUS_SPEED),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id()))
{
}

MB12XX::~MB12XX()
{
	// Ensure we are truly inactive.
	stop();

	// Unadvertise the distance sensor topic.
	if (_distance_sensor_topic != nullptr) {
		orb_unadvertise(_distance_sensor_topic);
	}

	// Free perf counters.
	perf_free(_comms_error);
	perf_free(_sample_perf);
}

int
MB12XX::collect()
{
	perf_begin(_sample_perf);
	uint8_t val[2] = {};

	// Increment i2c adress to next sensor.
	_sensor_index++;
	_sensor_index %= _sensor_count;

	// Set the sensor i2c adress for the active cycle.
	set_device_address(_sensor_addresses[_sensor_index]);

	// Transfer data from the bus.
	int ret_val = transfer(nullptr, 0, &val[0], 2);

	if (ret_val < 0) {
		PX4_ERR("sensor %i read failed, address: 0x%02X", _sensor_index, get_device_address());
		perf_count(_comms_error);
		perf_end(_sample_perf);
		return ret_val;
	}

	uint16_t distance_cm = val[0] << 8 | val[1];
	float distance_m = static_cast<float>(distance_cm) * 1e-2f;

	distance_sensor_s report;
	report.current_distance = distance_m;
	report.id               = _sensor_addresses[_sensor_index];
	report.max_distance     = MB12XX_MAX_DISTANCE;
	report.min_distance     = MB12XX_MIN_DISTANCE;
	report.orientation      = _sensor_rotations[_sensor_index];
	report.signal_quality   = -1;
	report.timestamp        = hrt_absolute_time();
	report.type             = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
	report.variance         = 0.0f;

	int instance_id;
	orb_publish_auto(ORB_ID(distance_sensor), &_distance_sensor_topic, &report, &instance_id, ORB_PRIO_DEFAULT);

	// Begin the next measurement.
	if (measure() != PX4_OK) {
		PX4_INFO("sensor %i measurement error, address 0x%02X", _sensor_index, get_device_address());
		perf_count(_comms_error);
		perf_end(_sample_perf);
		return ret_val;
	}

	perf_end(_sample_perf);
	return PX4_OK;
}

int
MB12XX::get_sensor_rotation(const size_t index)
{
	switch (index) {
	case 0: return _p_sensor0_rot.get();

	case 1: return _p_sensor1_rot.get();

	case 2: return _p_sensor2_rot.get();

	case 3: return _p_sensor3_rot.get();

	case 4: return _p_sensor4_rot.get();

	case 5: return _p_sensor5_rot.get();

	case 6: return _p_sensor6_rot.get();

	case 7: return _p_sensor7_rot.get();

	case 8: return _p_sensor8_rot.get();

	case 9: return _p_sensor9_rot.get();

	case 10: return _p_sensor10_rot.get();

	case 11: return _p_sensor11_rot.get();

	default: return PX4_ERROR;
	}
}

int
MB12XX::init()
{
	if (_p_sensor_enabled.get() == 0) {
		PX4_WARN("disabled");
		return PX4_ERROR;
	}

	// Initialize the I2C device
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	// Check for connected rangefinders on each i2c port by decrementing from the base address,
	// (MB12XX_BASE_ADDR = 112, MB12XX_MIN_ADDR = 90).
	for (uint8_t address = MB12XX_BASE_ADDR; address > MB12XX_MIN_ADDR ; address--) {
		set_device_address(address);

		if (measure() == PX4_OK) {
			// Store I2C address
			_sensor_addresses[_sensor_count] = address;
			_sensor_rotations[_sensor_count] = get_sensor_rotation(_sensor_count);
			_sensor_count++;

			PX4_INFO("sensor %i at address 0x%02X added", _sensor_count, get_device_address());

			if (_sensor_count >= RANGE_FINDER_MAX_SENSORS) {
				break;
			}

			px4_usleep(_measure_interval);
		}
	}

	// Return an error if no sensors were detected.
	if (_sensor_count == 0) {
		PX4_ERR("no sensors discovered");
		return PX4_ERROR;
	}

	// If more than one sonar is detected, adjust the meaure interval to avoid sensor interference.
	if (_sensor_count > 1) {
		_measure_interval = MB12XX_INTERVAL_BETWEEN_SUCCESIVE_FIRES;
	}

	PX4_INFO("Total sensors connected: %i", _sensor_count);
	return PX4_OK;
}

int
MB12XX::measure()
{
	// Send the command to take a measurement.
	uint8_t cmd = MB12XX_TAKE_RANGE_REG;
	int ret_val = transfer(&cmd, 1, nullptr, 0);

	return ret_val;
}

void
MB12XX::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_error);
	PX4_INFO("poll interval:  %ums", _measure_interval / 1000);

	for (size_t i = 0; i < _sensor_count; i++) {
		PX4_INFO("sensor: %u, address %u", i, _sensor_addresses[i]);
	}
}

void
MB12XX::Run()
{
	// Collect the sensor data.
	if (collect() != PX4_OK) {
		PX4_INFO("collection error");
	}
}

int
MB12XX::set_address(const uint8_t address)
{
	if (_sensor_count > 1) {
		PX4_INFO("multiple sensors are connected");
		return PX4_ERROR;
	}

	if (address < 2    ||
	    address == 80  ||
	    address == 164 ||
	    address == 170 ||
	    address > 224) {
		PX4_ERR("incompatible address requested");
		return PX4_ERROR;
	}

	PX4_INFO("requested address: %u", address);

	uint8_t shifted_address = (address << 1);
	uint8_t cmd[3] = {MB12XX_SET_ADDRESS_1, MB12XX_SET_ADDRESS_2, shifted_address};

	if (transfer(cmd, sizeof(cmd), nullptr, 0) != PX4_OK) {
		PX4_INFO("could not set the address");
	}

	set_device_address(address);
	PX4_INFO("device address: %u", get_device_address());
	return PX4_OK;
}

void
MB12XX::start()
{
	// Fetch parameter values.
	ModuleParams::updateParams();

	// Schedule the driver cycle at regular intervals.
	ScheduleOnInterval(_measure_interval);
}

void
MB12XX::stop()
{
	ScheduleClear();
}

/**
 * Local functions in support of the shell command.
 */
namespace mb12xx
{

MB12XX	*g_dev;

int reset();
int set_address(const uint8_t address = MB12XX_BASE_ADDR);
int start();
int start_bus(const int i2c_bus = MB12XX_BUS_DEFAULT);
int status();
int stop();
int test();
int usage();

/**
 * Reset the driver.
 */
int
reset()
{
	PX4_INFO("driver resetting");
	stop();
	start();
	return PX4_OK;
}

int
set_address(const uint8_t address)
{
	if (g_dev != nullptr) {
		if (g_dev->set_address(address) != PX4_OK) {
			PX4_ERR("address not set");
			return PX4_ERROR;
		}
	}

	reset();
	return PX4_OK;
}

/**
 * Attempt to start driver on all available I2C busses.
 *
 * This function will return as soon as the first sensor
 * is detected on one of the available busses or if no
 * sensors are detected.
 */
int
start()
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	for (unsigned i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
		if (start_bus(i2c_bus_options[i]) == PX4_OK) {
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
start_bus(const int i2c_bus)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

	// Instantiate the driver.
	g_dev = new MB12XX(i2c_bus);

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

/**
 * Stop the driver.
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
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
int
test()
{
	int fd = px4_open(MB12XX_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'mb12xx start' if the driver is not running)", MB12XX_DEVICE_PATH);
		return PX4_ERROR;
	}

	// Perform a simple demand read.
	distance_sensor_s report {};
	ssize_t sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_ERR("immediate read failed");
		return PX4_ERROR;
	}

	print_message(report);

	PX4_INFO("PASS");
	return PX4_OK;
}

/**
 * Print information about the driver usage.
 */
int
usage()
{
	PX4_INFO("usage: mb12xx <command> [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-a --all available busses");
	PX4_INFO("\t-b --bus i2cbus (%d)", MB12XX_BUS_DEFAULT);
	PX4_INFO("\t-A --set device address (0-112, 0x00-0x70)");
	PX4_INFO("command:");
	PX4_INFO("\treset|set_address|start|status|stop|test|usage");
	return PX4_OK;
}

} // namespace mb12xx

/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int mb12xx_main(int argc, char *argv[])
{
	const char *myoptarg = nullptr;

	int ch = 0;
	int i2c_bus = MB12XX_BUS_DEFAULT;
	int myoptind = 1;

	uint8_t address = MB12XX_BASE_ADDR;

	bool start_all = false;

	while ((ch = px4_getopt(argc, argv, "abs:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'a':
			start_all = true;
			break;

		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

		case 's':
			address = (uint8_t)atoi(myoptarg);
			break;

		default:
			PX4_WARN("Unknown option!");
			return mb12xx::usage();
		}
	}

	if (myoptind >= argc) {
		return mb12xx::usage();
	}

	// Reset the driver.
	if (!strcmp(argv[myoptind], "reset")) {
		return mb12xx::reset();
	}

	// Set the device I2C address.
	if (!strcmp(argv[myoptind], "set_address")) {
		return mb12xx::set_address(address);
	}

	// Start/load the driver.
	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			return mb12xx::start();

		} else {
			return mb12xx::start_bus(i2c_bus);
		}
	}

	// Print the driver status.
	if (!strcmp(argv[myoptind], "status")) {
		return mb12xx::status();
	}

	// Stop the driver.
	if (!strcmp(argv[myoptind], "stop")) {
		return mb12xx::stop();
	}

	// Test the driver/device.
	if (!strcmp(argv[myoptind], "test")) {
		return mb12xx::test();
	}

	// Print driver usage information.
	return mb12xx::usage();
}
