/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file srf02.cpp
 *
 * Driver for the SRF02 sonar range finder adapted from the Maxbotix sonar range finder driver (srf02).
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <containers/Array.hpp>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <perf/perf_counter.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

#include <board_config.h>

/* Configuration Constants */
#define SRF02_BASEADDR 				0x70 	// 7-bit address. 8-bit address is 0xE0.
#define SRF02_BUS_DEFAULT			PX4_I2C_BUS_EXPANSION
#define SRF02_DEVICE_PATH			"/dev/srf02"

/* SRF02 Registers addresses */
#define SRF02_TAKE_RANGE_REG			0x51	// Measure range Register.
#define SRF02_SET_ADDRESS_0			0xA0	// Change address 0 Register.
#define SRF02_SET_ADDRESS_1			0xAA	// Change address 1 Register.
#define SRF02_SET_ADDRESS_2			0xA5	// Change address 2 Register.

/* Device limits */
#define SRF02_MIN_DISTANCE 			(0.20f)
#define SRF02_MAX_DISTANCE 			(7.65f)

#define SRF02_CONVERSION_INTERVAL 		100000	// 60ms for one sonar.
#define SRF02_INTERVAL_BETWEEN_SUCCESIVE_FIRES 	100000	// 30ms between each sonar measurement (watch out for interference!).

class SRF02 : public device::I2C, public px4::ScheduledWorkItem
{
public:
	SRF02(uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING, int bus = SRF02_BUS_DEFAULT,
	      int address = SRF02_BASEADDR);
	virtual ~SRF02();

	int init() override;

	int ioctl(device::file_t *filp, int cmd, unsigned long arg) override;

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

	ssize_t read(device::file_t *filp, char *buffer, size_t buflen) override;

protected:
	int probe() override;

private:

	int collect();

	int measure();

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 * @param address The I2C bus address to probe.
	 * @return True if the device is present.
	 */
	int probe_address(uint8_t address);

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void Run() override;

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

	px4::Array<uint8_t, RANGE_FINDER_MAX_SENSORS>	addr_ind; 	// Temp sonar i2c address vector.

	bool _sensor_ok{false};
	bool _collect_phase{false};

	int _class_instance{-1};
	int _cycling_rate{0};           // Initialize cycling rate to zero, (can differ depending on one sonar or multiple).
	int _measure_interval{0};
	int _orb_class_instance{-1};

	uint8_t _cycle_counter{0};      // Initialize counter to zero - used to change i2c adresses for multiple devices.
	uint8_t _index_counter{0};      // Initialize temp sonar i2c address to zero.
	uint8_t _rotation;

	float _max_distance{SRF02_MAX_DISTANCE};
	float _min_distance{SRF02_MIN_DISTANCE};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "srf02_com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "srf02_read")};

	orb_advert_t _distance_sensor_topic{nullptr};

	ringbuffer::RingBuffer *_reports{nullptr};
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int srf02_main(int argc, char *argv[]);

SRF02::SRF02(uint8_t rotation, int bus, int address) :
	I2C("SRF02", SRF02_DEVICE_PATH, bus, address, 100000),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_rotation(rotation)
{
}

SRF02::~SRF02()
{
	// Ensure we are truly inactive.
	stop();

	// Free any existing reports.
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	// Free perf counters.
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
SRF02::collect()
{
	// Read from the sensor.
	uint8_t val[2] = {0, 0};
	uint8_t cmd = 0x02;
	perf_begin(_sample_perf);

	int ret = transfer(&cmd, 1, nullptr, 0);
	ret = transfer(nullptr, 0, &val[0], 2);

	if (ret < 0) {
		PX4_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint16_t distance_cm = val[0] << 8 | val[1];
	float distance_m = float(distance_cm) * 1e-2f;

	struct distance_sensor_s report;
	report.current_distance = distance_m;
	report.id 		= 0;	// TODO: set proper ID.
	report.max_distance 	= _max_distance;
	report.min_distance 	= _min_distance;
	report.orientation 	= _rotation;
	report.signal_quality 	= -1;
	report.variance 	= 0.0f;
	report.timestamp 	= hrt_absolute_time();
	report.type 		= distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;

	// Publish it, if we are the primary.
	if (_distance_sensor_topic != nullptr) {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
	}

	_reports->force(&report);

	// Notify anyone waiting for data.
	poll_notify(POLLIN);
	perf_end(_sample_perf);
	return PX4_OK;
}

int
SRF02::init()
{
	// I2C init (and probe) first.
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	// Allocate basic report buffers.
	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

	_index_counter = SRF02_BASEADDR;        // Set temp sonar i2c address to base adress.
	set_device_address(_index_counter);     // Set I2c port to temp sonar i2c adress.

	if (_reports == nullptr) {
		return PX4_ERROR;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	// Get a publish handle on the range finder topic.
	struct distance_sensor_s ds_report = {};

	_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
				 &_orb_class_instance, ORB_PRIO_LOW);

	if (_distance_sensor_topic == nullptr) {
		PX4_ERR("failed to create distance_sensor object");
	}

	// XXX we should find out why we need to wait 200 ms here
	px4_usleep(200000);

	/* Check for connected rangefinders on each i2c port:
	 *   We start from i2c base address (0x70 = 112) and count downwards,
	 *   so the second iteration it uses i2c address 111, third iteration 110, and so on. */
	for (unsigned counter = 0; counter <= RANGE_FINDER_MAX_SENSORS; counter++) {
		_index_counter = SRF02_BASEADDR - counter;      // Set temp sonar i2c address to base adress - counter.
		set_device_address(_index_counter);             // Set I2c port to temp sonar i2c adress.

		int ret = measure();

		if (ret == 0) {
			// Sonar is present -> store address_index in array.
			addr_ind.push_back(_index_counter);
			PX4_DEBUG("sonar added");
		}
	}

	_index_counter = SRF02_BASEADDR;
	set_device_address(_index_counter); // Set i2c port back to base adress for rest of driver.

	// If only one sonar detected, no special timing is required between firing, so use default.
	if (addr_ind.size() == 1) {
		_cycling_rate = SRF02_CONVERSION_INTERVAL;

	} else {
		_cycling_rate = SRF02_INTERVAL_BETWEEN_SUCCESIVE_FIRES;
	}

	// Show the connected sonars in terminal.
	for (unsigned i = 0; i < addr_ind.size(); i++) {
		PX4_DEBUG("sonar %d with address %d added", (i + 1), addr_ind[i]);
	}

	PX4_DEBUG("Number of sonars connected: %zu", addr_ind.size());

	// Sensor is ok, but we don't really know if it is within range.
	_sensor_ok = true;

	return PX4_OK;
}

int
SRF02::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			// Zero would be bad.
			case 0:
				return -EINVAL;

			// Set default polling rate.
			case SENSOR_POLLRATE_DEFAULT: {
					// Do we need to start internal polling?.
					bool want_start = (_measure_interval == 0);

					// Set interval for next measurement to minimum legal value.
					_measure_interval = _cycling_rate;

					// If we need to start the poll state machine, do it.
					if (want_start) {
						start();

					}

					return OK;
				}

			// Adjust to a legal polling interval in Hz.
			default: {
					// Do we need to start internal polling?.
					bool want_start = (_measure_interval == 0);

					// Convert hz to tick interval via microseconds.
					int interval = (1000000 / arg);

					// check against maximum rate.
					if (interval < _cycling_rate) {
						return -EINVAL;
					}

					// Update interval for next measurement.
					_measure_interval = interval;

					// If we need to start the poll state machine, do it.
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	default:
		// Give it to the superclass.
		return I2C::ioctl(filp, cmd, arg);
	}
}

int
SRF02::measure()
{
	uint8_t cmd[2];
	cmd[0] = 0x00;
	cmd[1] = SRF02_TAKE_RANGE_REG;

	// Send the command to begin a measurement.
	int ret = transfer(cmd, 2, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	return PX4_OK;
}

void
SRF02::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u\n", _measure_interval);
	_reports->print_info("report queue");
}

int
SRF02::probe()
{
	return measure();
}

ssize_t
SRF02::read(device::file_t *filp, char *buffer, size_t buflen)
{

	unsigned count = buflen / sizeof(struct distance_sensor_s);
	struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);
	int ret = 0;

	// Buffer must be large enough.
	if (count < 1) {
		return -ENOSPC;
	}

	// If automatic measurement is enabled.
	if (_measure_interval > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		// If there was no data, warn the caller.
		return ret ? ret : -EAGAIN;
	}

	// Manual measurement - run one conversion.
	do {
		_reports->flush();

		// Trigger a measurement.
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		// Wait for it to complete.
		px4_usleep(_cycling_rate * 2);

		// Run the collection phase.
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		// State machine will have generated a report, copy it out.
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}

void
SRF02::Run()
{
	if (_collect_phase) {
		_index_counter = addr_ind[_cycle_counter]; // Sonar from previous iteration collect is now read out.
		set_device_address(_index_counter);

		// Perform collection.
		if (OK != collect()) {
			PX4_DEBUG("collection error");
			// If error restart the measurement state machine.
			start();
			return;
		}

		// Next phase is measurement.
		_collect_phase = false;

		// Change i2c adress to next sonar.
		_cycle_counter = _cycle_counter + 1;

		if (_cycle_counter >= addr_ind.size()) {
			_cycle_counter = 0;
		}

		// Is there a collect->measure gap? Yes, and the timing is set equal to the cycling_rate
		// Otherwise the next sonar would fire without the first one having received its reflected sonar pulse.
		if (_measure_interval > _cycling_rate) {

			// schedule a fresh cycle call when we are ready to measure again.
			ScheduleDelayed(_measure_interval - _cycling_rate);
			return;
		}
	}

	// Measurement (firing) phase - Ensure sonar i2c adress is still correct.
	_index_counter = addr_ind[_cycle_counter];
	set_device_address(_index_counter);

	// Perform measurement.
	if (OK != measure()) {
		PX4_DEBUG("measure error sonar adress %d", _index_counter);
	}

	// Next phase is collection.
	_collect_phase = true;

	// Schedule a fresh cycle call when the measurement is done.
	ScheduleDelayed(_cycling_rate);
}

void
SRF02::start()
{
	// Reset the report ring and state machine.
	_collect_phase = false;
	_reports->flush();

	// Schedule a cycle to start things.
	ScheduleDelayed(5);
}

void
SRF02::stop()
{
	ScheduleClear();
}


/**
 * Local functions in support of the shell command.
 */
namespace srf02
{

SRF02   *g_dev;

int     reset();
int     start(uint8_t rotation);
int     start_bus(uint8_t rotation, int i2c_bus);
int     status();
int     stop();
int     test();
int     usage();

/**
 * Reset the driver.
 */
int
reset()
{
	int fd = px4_open(SRF02_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("failed");
		return PX4_ERROR;
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		return PX4_ERROR;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		return PX4_ERROR;
	}

	px4_close(fd);
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
start(uint8_t rotation)
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
start_bus(uint8_t rotation, int i2c_bus)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	// Create the driver.
	g_dev = new SRF02(rotation, i2c_bus);

	if (g_dev == nullptr) {
		PX4_ERR("failed to instantiate the device");
		return PX4_ERROR;
	}

	if (OK != g_dev->init()) {
		PX4_ERR("failed to initialize the device");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	// Set the poll rate to default, starts automatic data collection.
	int fd = px4_open(SRF02_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		delete g_dev;
		g_dev = nullptr;
		px4_close(fd);
		return PX4_ERROR;
	}

	px4_close(fd);
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

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

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

	} else {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

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
	int fd = px4_open(SRF02_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'srf02 start' if the driver is not running)", SRF02_DEVICE_PATH);
		return PX4_ERROR;
	}

	struct distance_sensor_s report;

	// Perform a simple demand read.
	ssize_t sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_ERR("immediate read failed");
		return PX4_ERROR;
	}

	print_message(report);

	// Start the sensor polling at 2Hz.
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		PX4_ERR("failed to set 2Hz poll rate");
		return PX4_ERROR;
	}

	// Read the sensor 5x and report each value.
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		// Wait for data to be ready.
		fds.fd = fd;
		fds.events = POLLIN;
		int ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			PX4_ERR("timed out waiting for sensor data");
			return PX4_ERROR;
		}

		// Now go get it.
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_ERR("periodic read failed");
			return PX4_ERROR;
		}

		print_message(report);
	}

	// Reset the sensor polling to default rate.
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		PX4_ERR("failed to set default poll rate");
		return PX4_ERROR;
	}

	PX4_INFO("PASS");
	return PX4_OK;
}

int
usage()
{
	PX4_INFO("usage: srf02 command [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-b --bus i2cbus (%d)", SRF02_BUS_DEFAULT);
	PX4_INFO("\t-a --all");
	PX4_INFO("\t-R --rotation (%d)", distance_sensor_s::ROTATION_DOWNWARD_FACING);
	PX4_INFO("command:");
	PX4_INFO("\tstart|stop|test|reset|info");
	return PX4_OK;
}

} // namespace srf02


/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int srf02_main(int argc, char *argv[])
{
	const char *myoptarg = nullptr;

	int ch;
	int myoptind = 1;
	int i2c_bus = SRF02_BUS_DEFAULT;

	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	bool start_all = false;

	while ((ch = px4_getopt(argc, argv, "ab:R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

		case 'a':
			start_all = true;
			break;

		default:
			PX4_WARN("Unknown option!");
			return srf02::usage();
		}
	}

	if (myoptind >= argc) {
		return srf02::usage();
	}

	// Start/load the driver.
	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			return srf02::start(rotation);

		} else {
			return srf02::start_bus(rotation, i2c_bus);
		}
	}

	// Stop the driver.
	if (!strcmp(argv[myoptind], "stop")) {
		return srf02::stop();
	}

	// Test the driver/device.
	if (!strcmp(argv[myoptind], "test")) {
		return srf02::test();
	}

	// Reset the driver.
	if (!strcmp(argv[myoptind], "reset")) {
		return srf02::reset();
	}

	// Print driver information.
	if (!strcmp(argv[myoptind], "info") ||
	    !strcmp(argv[myoptind], "status")) {
		return srf02::status();
	}

	return srf02::usage();
}
