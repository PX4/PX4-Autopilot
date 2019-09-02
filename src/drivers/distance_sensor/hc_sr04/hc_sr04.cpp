/****************************************************************************
 *
 *   Copyright (c) 2015, 2016 Airmind Development Team. All rights reserved.
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
 * 3. Neither the name Airmind nor the names of its contributors may be
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
 * @file hc_sr04.cpp
 *
 * Driver for the hc_sr04 sonar range finders .
 */

#include <lib/drivers/distance_sensor/DistanceSensor.h>

#define SR04_ID_BASE			0x10
#define SR04_DEVICE_PATH		"/dev/hc_sr04"
#define SR04_MAX_RANGEFINDERS		6

/* Device limits */
#define SR04_MIN_DISTANCE		(0.10f)
#define SR04_MAX_DISTANCE		(4.00f)

#define SR04_CONVERSION_INTERVAL	100000 /* 100ms for one sonar */

#define GPIO_GPIO8_INPUT		0	// Not currrently defined for any board.
#define GPIO_GPIO9_INPUT		0	// Not currrently defined for any board.
#define GPIO_GPIO10_INPUT		0	// Not currrently defined for any board.
#define GPIO_GPIO11_INPUT		0	// Not currrently defined for any board.
#define GPIO_GPIO12_INPUT		0	// Not currrently defined for any board.

const HC_SR04::GPIOConfig HC_SR04::_gpio_tab[] = {
	{GPIO_GPIO6_OUTPUT, GPIO_GPIO7_INPUT,  0},
	{GPIO_GPIO6_OUTPUT, GPIO_GPIO8_INPUT,  0},
	{GPIO_GPIO6_OUTPUT, GPIO_GPIO9_INPUT,  0},
	{GPIO_GPIO6_OUTPUT, GPIO_GPIO10_INPUT, 0},
	{GPIO_GPIO6_OUTPUT, GPIO_GPIO11_INPUT, 0},
	{GPIO_GPIO6_OUTPUT, GPIO_GPIO12_INPUT, 0}
};


class HC_SR04 : public DistanceSensor, public cdev::CDev
{
public:
	HC_SR04();
	virtual ~HC_SR04();

	virtual int init();

	void interrupt(unsigned time);

protected:

private:

	int collect();

	int measure();

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 *
	 * @param address The I2C bus address to probe.
	 * @return        True if the device is present.
	 */
	int probe_address(uint8_t address);

	/**
	 * Stop the automatic measurement state machine.
	 */
	void dev_stop();

	struct GPIOConfig {
		uint32_t alt;
		uint32_t echo_port;
		uint32_t trig_port;
	};

	static const GPIOConfig _gpio_tab[];

	px4::Array<float, 6> _latest_sonar_measurements {};	// Array to store latest sonar measurements in before writing to report.

	bool _collect_phase{false};
	bool _sensor_ok{false};

	int _class_instance{-1};
	int _cycling_rate{0};		// Initialize cycling rate to zero, (can differ depending on one sonar or multiple).
	int _measure_interval{0};
	int _orb_class_instance{-1};

	unsigned int _falling_time{0};
	unsigned int _raising_time{0};
	unsigned int _sonars{6};
	unsigned int _status{0};

	uint8_t _cycle_counter{0};	// Initialize counter for cycling function to zero.
	uint8_t _index_counter{0};	// Initialize temp sonar i2c address to zero.

	float _max_distance{SR04_MIN_DISTANCE};
	float _min_distance{SR04_MAX_DISTANCE};

	ringbuffer::RingBuffer	*_reports{nullptr};

	orb_advert_t _distance_sensor_topic{nullptr};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "hc_sr04_comms_errors")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "hc_sr04_read")};
};


HC_SR04::HC_SR04() :
	CDev(SR04_DEVICE_PATH)
{
}

HC_SR04::~HC_SR04()
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
HC_SR04::collect()
{
	int	ret = -EIO;
#if 0
	perf_begin(_sample_perf);

	/* read from the sensor */
	if (_status != 2) {
		PX4_DEBUG("erro sonar %d ,status=%d", _cycle_counter, _status);
		px4_arch_gpiosetevent(_gpio_tab[_cycle_counter].echo_port, true, true, false, nullptr);
		perf_end(_sample_perf);
		return (ret);
	}

	unsigned  distance_time = _falling_time - _raising_time ;

	float si_units = (distance_time * 0.000170f) ; /* meter */
	struct distance_sensor_s report;

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);

	/* if only one sonar, write it to the original distance parameter so that it's still used as altitude sonar */
	if (_sonars == 1) {
		report.distance = si_units;

		for (unsigned i = 0; i < (SRF02_MAX_RANGEFINDERS); i++) {
			report.id[i] = 0;
			report.distance_vector[i] = 0;
		}

		report.id[0] = SR04_ID_BASE;
		report.distance_vector[0] = si_units; //  将测量值填入向量中，适应test()的要求
		report.just_updated = 1;

	} else {
		/* for multiple sonars connected */

		_latest_sonar_measurements[_cycle_counter] = si_units;
		report.just_updated = 0;

		for (unsigned i = 0; i < SRF02_MAX_RANGEFINDERS; i++) {
			if (i < _sonars) {
				report.distance_vector[i] = _latest_sonar_measurements[i];
				report.id[i] = SR04_ID_BASE + i;
				report.just_updated++;

			} else {
				report.distance_vector[i] = 0;
				report.id[i] = 0;
			}

		}

		report.distance =  _latest_sonar_measurements[0]; //
	}

	report.minimum_distance = get_minimum_distance();
	report.maximum_distance = get_maximum_distance();
	report.valid = si_units > get_minimum_distance() && si_units < get_maximum_distance() ? 1 : 0;

	/* publish it, if we are the primary */
	if (_distance_sensor_topic != nullptr) {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
	}

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	px4_arch_gpiosetevent(_gpio_tab[_cycle_counter].echo_port, true, true, false, nullptr); /* close interrupt */
	perf_end(_sample_perf);
#endif
	return ret;
}

int
HC_SR04::init()
{
	// Perform I2C init (and probe) first.
	if (CDev::init() != OK) {
		return PX4_ERROR;
	}

	// Allocate basic report buffers.
	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

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

	// Init echo port:
	for (unsigned i = 0; i <= _sonars; i++) {
		px4_arch_configgpio(_gpio_tab[i].trig_port);
		px4_arch_gpiowrite(_gpio_tab[i].trig_port, false);
		px4_arch_configgpio(_gpio_tab[i].echo_port);
		_latest_sonar_measurements.push_back(0);
	}

	usleep(200000); // Wait for 200ms for the sensor to configure.

	// sensor is ok, but we don't really know if it is within range.
	_sensor_ok = true;
	_cycling_rate = SR04_CONVERSION_INTERVAL;

	// Show the connected sonars in terminal.
	PX4_INFO("Number of sonars set: %d", _sonars);

	return PX4_OK;
}

void
HC_SR04::interrupt(unsigned time)
{
	if (_status == 0) {
		_raising_time = time;
		_status++;
		return;

	} else if (_status == 1) {
		_falling_time = time;
		_status++;
		return;
	}

	return;
}

int
HC_SR04::measure()
{
	// Send a plus begin a measurement.
	px4_arch_gpiowrite(_gpio_tab[_cycle_counter].trig_port, true);

	usleep(10);  // Allow 10us for register write to complete.

	px4_arch_gpiowrite(_gpio_tab[_cycle_counter].trig_port, false);
	px4_arch_gpiosetevent(_gpio_tab[_cycle_counter].echo_port, true, true, false, sonar_isr);

	_status = 0;

	return PX4_OK;
}

void
HC_SR04::Run()
{
	int ret = collect();

	// perform collection */
	if (ret != PX4_OK) {
		PX4_DEBUG("collection error");
	}

	// change to next sonar.
	_cycle_counter++;
	_cycle_counter %= _sonars;

	ret = measure();

	if (ret != PX4_OK) {
		PX4_DEBUG("measure error sonar adress %d", _cycle_counter);
	}

	ScheduleDelayed(_cycling_rate);
}

void
HC_SR04::start()
{
	// Reset the report ring and state machine.
	_collect_phase = false;
	_reports->flush();

	// Begin measure.
	measure();

	// Schedule a cycle to start things.
	ScheduleDelayed(_cycling_rate);
}

void
HC_SR04::stop()
{
	ScheduleClear();
}

/**
 * Local functions in support of the shell command.
 */
namespace  hc_sr04
{

HC_SR04	*g_dev;

int reset(const char *port = DEFAULT_SERIAL_PORT);
int start(const char *port = DEFAULT_SERIAL_PORT,
	  const uint8_t orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
int status();
int stop();
int test(const char *port = DEFAULT_SERIAL_PORT);
int usage();

/**
 * Reset the driver.
 */
int
reset(const char *port)
{
	if (stop() == PX4_OK) {
		return start(port);
	}

	return PX4_ERROR;
}

/**
 * Start the driver.
 */
int
start(const char *port, const uint8_t orientation)
{
	if (g_dev != nullptr) {
		PX4_INFO("already started");
		return PX4_OK;
	}

	// Instantiate the driver.
	g_dev = new HC_SR04(port, orientation);

	if (g_dev == nullptr) {
		PX4_ERR("object instantiate failed");
		return PX4_ERROR;
	}

	if (g_dev->DistanceSensor::init() != PX4_OK) {
		PX4_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

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
int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return PX4_ERROR;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
int
test(const char *port)
{
	int fd = open(port, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'hc_sr04 start' if the driver is not running", port);
		return PX4_ERROR;
	}

	// Perform a simple demand read.
	distance_sensor_s report;
	ssize_t bytes_read = read(fd, &report, sizeof(report));

	if (bytes_read != sizeof(report)) {
		PX4_ERR("immediate read failed");
		return PX4_ERROR;
	}

	close(fd);
	print_message(report);
	return PX4_OK;
}

int
usage()
{
	PX4_INFO("usage: hc_sr04 command [options]");
	PX4_INFO("command:");
	PX4_INFO("\treset|start|status|stop|test");
	PX4_INFO("options:");
	PX4_INFO("\t-R --rotation (%d)", distance_sensor_s::ROTATION_DOWNWARD_FACING);
	PX4_INFO("\t-d --device_path");
	return PX4_OK;
}

} /* namespace */


/**
 * Driver 'main' command.
 */
extern "C"  __EXPORT int hc_sr04_main(int argc, char *argv[])
{
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	const char *device_path = CM8JL65_DEFAULT_PORT;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:R", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_path = myoptarg;
			break;

		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		default:
			PX4_WARN("Unknown option!");
			return hc_sr04::usage();
		}
	}

	if (myoptind >= argc) {
		return hc_sr04::usage();
	}

	// Reset the driver.
	if (!strcmp(argv[myoptind], "reset")) {
		return hc_sr04::reset(device_path);
	}

	// Start/load the driver.
	if (!strcmp(argv[myoptind], "start")) {
		return hc_sr04::start(device_path, rotation);
	}

	// Print driver information.
	if (!strcmp(argv[myoptind], "status")) {
		return hc_sr04::status();
	}

	// Stop the driver
	if (!strcmp(argv[myoptind], "stop")) {
		return hc_sr04::stop();
	}

	// Test the driver/device.
	if (!strcmp(argv[myoptind], "test")) {
		return hc_sr04::test();
	}

	// Print driver usage information.
	return hc_sr04::usage();
}
