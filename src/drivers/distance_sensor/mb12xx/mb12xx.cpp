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
#include <px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>


/* Configuration Constants */
#define MB12XX_BASEADDR                         0x70 // 7-bit address. 8-bit address is 0xE0.
#define MB12XX_BUS_DEFAULT                      PX4_I2C_BUS_EXPANSION
#define MB12XX_DEVICE_PATH                      "/dev/mb12xx"

/* MB12xx Registers addresses */
#define MB12XX_TAKE_RANGE_REG                   0x51 // Measure range Register.
#define MB12XX_SET_ADDRESS_1                    0xAA // Change address 1 Register.
#define MB12XX_SET_ADDRESS_2                    0xA5 // Change address 2 Register.

/* Device limits */
#define MB12XX_MIN_DISTANCE                     (0.20f)
#define MB12XX_MAX_DISTANCE                     (7.65f)

#define MB12XX_CONVERSION_INTERVAL              100000 // 60ms for one sonar.
#define MB12XX_INTERVAL_BETWEEN_SUCCESIVE_FIRES 100000 // 30ms between each sonar measurement (watch out for interference!).

class MB12XX : public device::I2C, public px4::ScheduledWorkItem
{
public:
	MB12XX(uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING,
	       int bus = MB12XX_BUS_DEFAULT, int address = MB12XX_BASEADDR);
	virtual ~MB12XX();

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

	int collect();

	int measure();

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 *
	 * @param address The I2C bus address to probe.
	 * @return True if the device is present.
	 */
	int probe_address(uint8_t address);

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void Run() override;

	px4::Array<uint8_t, RANGE_FINDER_MAX_SENSORS> addr_ind;	// Temp sonar i2c address vector.

	bool _collect_phase{false};
	bool _sensor_ok{false};

	int _cycling_rate{0};		// Initialize cycling rate (which can differ depending on one sonar or multiple).
	int _measure_interval{0};
	int _orb_class_instance{-1};

	uint8_t _cycle_counter{0};	// Initialize counter for cycling i2c adresses to zero.
	uint8_t _index_counter{0};	// Initialize temp sonar i2c address to zero.
	uint8_t _rotation{0};

	float _max_distance{MB12XX_MAX_DISTANCE};
	float _min_distance{MB12XX_MIN_DISTANCE};

	orb_advert_t _distance_sensor_topic{nullptr};

	perf_counter_t _comms_errors{perf_alloc(PC_ELAPSED, "mb12xx_read")};
	perf_counter_t _sample_perf{perf_alloc(PC_COUNT, "mb12xx_com_err")};
};


MB12XX::MB12XX(uint8_t rotation, int bus, int address) :
	I2C("MB12xx", MB12XX_DEVICE_PATH, bus, address, 100000),
	ScheduledWorkItem(px4::device_bus_to_wq(get_device_id())),
	_rotation(rotation)
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
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
MB12XX::collect()
{
	uint8_t val[2] = {0, 0};
	perf_begin(_sample_perf);

	// Read from the sensor.
	int ret = transfer(nullptr, 0, &val[0], 2);

	if (ret != PX4_OK) {
		PX4_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint16_t distance_cm = val[0] << 8 | val[1];
	float distance_m = static_cast<float>(distance_cm) * 1e-2f;

	distance_sensor_s report;
	report.current_distance = distance_m;
	report.id               = 0; // TODO: set proper ID.
	report.max_distance     = _max_distance;
	report.min_distance     = _min_distance;
	report.orientation      = _rotation;
	report.signal_quality   = -1;
	report.timestamp        = hrt_absolute_time();
	report.type             = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
	report.variance         = 0.0f;

	// Publish it, if we are the primary */
	if (_distance_sensor_topic != nullptr) {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
	}

	// Notify anyone waiting for data.
	poll_notify(POLLIN);
	perf_end(_sample_perf);

	return PX4_OK;
}

int
MB12XX::init()
{
	// Initialize the I2C device
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	_index_counter = MB12XX_BASEADDR;   // Set temp sonar i2c address to base adress.
	set_device_address(_index_counter); // Set I2c port to temp sonar i2c adress.

	/* get a publish handle on the range finder topic */
	distance_sensor_s ds_report {};
	_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
				 &_orb_class_instance, ORB_PRIO_LOW);

	if (_distance_sensor_topic == nullptr) {
		PX4_ERR("failed to create distance_sensor object");
	}

	px4_usleep(10_ms);

	/* Check for connected rangefinders on each i2c port:
	   We start from i2c base address (0x70 = 112) and count downwards
	   So second iteration it uses i2c address 111, third iteration 110 and so on. */
	for (unsigned counter = 0; counter <= RANGE_FINDER_MAX_SENSORS; counter++) {
		_index_counter = MB12XX_BASEADDR - counter;	/* set temp sonar i2c address to base adress - counter */
		set_device_address(_index_counter);			/* set I2c port to temp sonar i2c adress */

		if (measure() == 0) { /* sonar is present -> store address_index in array */
			addr_ind.push_back(_index_counter);
			PX4_DEBUG("sonar added");
		}
	}

	_index_counter = MB12XX_BASEADDR;
	set_device_address(_index_counter); // Set i2c port back to base adress for rest of driver.

	// If only one sonar detected, no special timing is required between firing, so use default.
	if (addr_ind.size() == 1) {
		_cycling_rate = MB12XX_CONVERSION_INTERVAL;

	} else {
		_cycling_rate = MB12XX_INTERVAL_BETWEEN_SUCCESIVE_FIRES;
	}

	// Show the connected sonars in terminal.
	for (unsigned i = 0; i < addr_ind.size(); i++) {
		PX4_DEBUG("sonar %d with address %d added", (i + 1), addr_ind[i]);
	}

	PX4_INFO("Number of sonars connected: %lu", addr_ind.size());

	// Sensor is ok, but we don't really know if it is within range.
	_sensor_ok = true;

	return PX4_OK;
}

int
MB12XX::probe()
{
	return measure();
}

int
MB12XX::measure()
{
	// Send the command to begin a measurement.
	uint8_t cmd = MB12XX_TAKE_RANGE_REG;
	int ret = transfer(&cmd, 1, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	return PX4_OK;
}

void
MB12XX::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u\n", _measure_interval);
}

void
MB12XX::Run()
{
	if (_collect_phase) {
		_index_counter = addr_ind[_cycle_counter]; // Sonar from previous iteration collect is now read out.
		set_device_address(_index_counter);

		/* perform collection */
		if (OK != collect()) {
			PX4_DEBUG("collection error");
			/* if error restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/* change i2c adress to next sonar */
		_cycle_counter = _cycle_counter + 1;

		if (_cycle_counter >= addr_ind.size()) {
			_cycle_counter = 0;
		}

		/* Is there a collect->measure gap? Yes, and the timing is set equal to the cycling_rate
		   Otherwise the next sonar would fire without the first one having received its reflected sonar pulse */
		if (_measure_interval > _cycling_rate) {

			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - _cycling_rate);

			return;
		}
	}

	// Ensure sonar i2c adress is correct.
	_index_counter = addr_ind[_cycle_counter];
	set_device_address(_index_counter);

	// Perform measurement.
	if (OK != measure()) {
		PX4_DEBUG("measure error sonar adress %d", _index_counter);
	}

	// Next phase is collection.
	_collect_phase = true;
}

void
MB12XX::start()
{
	// Reset the collect state.
	_collect_phase = false;

	// Schedule the driver to run at regular interval.
	ScheduleOnInterval(_cycling_rate);
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
int start(const uint8_t rotation = 0);
int start_bus(const uint8_t rotation = 0, const int i2c_bus = MB12XX_BUS_DEFAULT);
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
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	g_dev->stop();
	g_dev->start();
	PX4_INFO("driver reset");
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
start(const uint8_t rotation)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	for (unsigned i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
		if (start_bus(rotation, i2c_bus_options[i]) == PX4_OK) {
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
start_bus(const uint8_t rotation, const int i2c_bus)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

	// Instantiate the driver.
	g_dev = new MB12XX(rotation, i2c_bus);

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
	int sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_ERR("immediate read failed");
		return PX4_ERROR;
	}

	print_message(report);

	PX4_INFO("PASS");
	return PX4_OK;
}

int
usage()
{
	PX4_INFO("usage: mb12xx command [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-a --all available busses");
	PX4_INFO("\t-b --bus i2cbus (%d)", MB12XX_BUS_DEFAULT);
	PX4_INFO("\t-R --rotation (%d)", distance_sensor_s::ROTATION_DOWNWARD_FACING);
	PX4_INFO("command:");
	PX4_INFO("\treset|start|status|stop|test|usage");
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

	// Start/load the driver.
	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			return mb12xx::start(rotation);

		} else {
			return mb12xx::start_bus(rotation, i2c_bus);
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