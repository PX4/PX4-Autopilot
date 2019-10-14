/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file px4flow.cpp
 * @author Dominik Honegger
 * @author Ban Siesta <bansiesta@gmail.com>
 *
 * Driver for the PX4FLOW module connected via I2C.
 */

#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <lib/conversion/rotation.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/optical_flow.h>

#define PX4FLOW0_DEVICE_PATH	"/dev/px4flow0"

/* Configuration Constants */
#define I2C_FLOW_ADDRESS_DEFAULT    0x42	///< 7-bit address. 8-bit address is 0x84, range 0x42 - 0x49
#define I2C_FLOW_ADDRESS_MIN        0x42	///< 7-bit address.
#define I2C_FLOW_ADDRESS_MAX        0x49	///< 7-bit address.

/* PX4FLOW Registers addresses */
#define PX4FLOW_REG			0x16	///< Measure Register 22

#define PX4FLOW_CONVERSION_INTERVAL_DEFAULT 100000	///< in microseconds! = 10Hz
#define PX4FLOW_CONVERSION_INTERVAL_MIN      10000	///< in microseconds! = 100 Hz
#define PX4FLOW_CONVERSION_INTERVAL_MAX    1000000	///< in microseconds! = 1 Hz

#define PX4FLOW_I2C_MAX_BUS_SPEED	400000	///< 400 KHz maximum speed

#define PX4FLOW_MAX_DISTANCE 5.0f
#define PX4FLOW_MIN_DISTANCE 0.3f

#include "i2c_frame.h"

class PX4FLOW: public device::I2C, public px4::ScheduledWorkItem
{
public:
	PX4FLOW(int bus, int address = I2C_FLOW_ADDRESS_DEFAULT, enum Rotation rotation = (enum Rotation)0,
		int conversion_interval = PX4FLOW_CONVERSION_INTERVAL_DEFAULT,
		uint8_t sonar_rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	virtual ~PX4FLOW();

	virtual int 		init();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void				print_info();

protected:
	virtual int			probe();

private:

	uint8_t _sonar_rotation;
	bool				_sensor_ok{false};
	bool				_collect_phase{false};
	int			_class_instance{-1};
	int			_orb_class_instance{-1};
	orb_advert_t		_px4flow_topic{nullptr};
	orb_advert_t		_distance_sensor_topic{nullptr};

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	enum Rotation       _sensor_rotation;
	float 				_sensor_min_range{0.0f};
	float 				_sensor_max_range{0.0f};
	float 				_sensor_max_flow_rate{0.0f};

	i2c_frame _frame;
	i2c_integral_frame _frame_integral;

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 *
	 * @param address	The I2C bus address to probe.
	 * @return		True if the device is present.
	 */
	int					probe_address(uint8_t address);

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void				start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void				stop();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void				Run() override;

	int					measure();
	int					collect();

};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int px4flow_main(int argc, char *argv[]);

PX4FLOW::PX4FLOW(int bus, int address, enum Rotation rotation, int conversion_interval, uint8_t sonar_rotation) :
	I2C("PX4FLOW", PX4FLOW0_DEVICE_PATH, bus, address, PX4FLOW_I2C_MAX_BUS_SPEED), /* 100-400 KHz */
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_sonar_rotation(sonar_rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, "px4f_read")),
	_comms_errors(perf_alloc(PC_COUNT, "px4f_com_err")),
	_sensor_rotation(rotation)
{
}

PX4FLOW::~PX4FLOW()
{
	/* make sure we are truly inactive */
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
PX4FLOW::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ret;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	/* get a publish handle on the range finder topic */
	struct distance_sensor_s ds_report = {};

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
					 &_orb_class_instance, ORB_PRIO_HIGH);

		if (_distance_sensor_topic == nullptr) {
			PX4_ERR("failed to create distance_sensor object");
		}

	} else {
		DEVICE_LOG("not primary range device, not advertising");
	}

	ret = OK;
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;

	/* get yaw rotation from sensor frame to body frame */
	param_t rot = param_find("SENS_FLOW_ROT");

	if (rot != PARAM_INVALID) {
		int32_t val = 6; // the recommended installation for the flow sensor is with the Y sensor axis forward
		param_get(rot, &val);

		_sensor_rotation = (enum Rotation)val;
	}

	/* get operational limits of the sensor */
	param_t hmin = param_find("SENS_FLOW_MINHGT");

	if (hmin != PARAM_INVALID) {
		float val = 0.7;
		param_get(hmin, &val);

		_sensor_min_range = val;
	}

	param_t hmax = param_find("SENS_FLOW_MAXHGT");

	if (hmax != PARAM_INVALID) {
		float val = 3.0;
		param_get(hmax, &val);

		_sensor_max_range = val;
	}

	param_t ratemax = param_find("SENS_FLOW_MAXR");

	if (ratemax != PARAM_INVALID) {
		float val = 2.5;
		param_get(ratemax, &val);

		_sensor_max_flow_rate = val;
	}

	start();

	return ret;
}

int
PX4FLOW::probe()
{
	uint8_t val[I2C_FRAME_SIZE] {};

	// to be sure this is not a ll40ls Lidar (which can also be on
	// 0x42) we check if a I2C_FRAME_SIZE byte transfer works from address
	// 0. The ll40ls gives an error for that, whereas the flow
	// happily returns some data
	if (transfer(nullptr, 0, &val[0], 22) != OK) {
		return -EIO;
	}

	// that worked, so start a measurement cycle
	return measure();
}

int
PX4FLOW::measure()
{
	/*
	 * Send the command to begin a measurement.
	 */
	uint8_t cmd = PX4FLOW_REG;
	int ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		DEVICE_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	return PX4_OK;
}

int
PX4FLOW::collect()
{
	int ret = -EIO;

	/* read from the sensor */
	uint8_t val[I2C_FRAME_SIZE + I2C_INTEGRAL_FRAME_SIZE] = { };

	perf_begin(_sample_perf);

	if (PX4FLOW_REG == 0x00) {
		ret = transfer(nullptr, 0, &val[0], I2C_FRAME_SIZE + I2C_INTEGRAL_FRAME_SIZE);
	}

	if (PX4FLOW_REG == 0x16) {
		ret = transfer(nullptr, 0, &val[0], I2C_INTEGRAL_FRAME_SIZE);
	}

	if (ret < 0) {
		DEVICE_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	if (PX4FLOW_REG == 0) {
		memcpy(&_frame, val, I2C_FRAME_SIZE);
		memcpy(&_frame_integral, &(val[I2C_FRAME_SIZE]), I2C_INTEGRAL_FRAME_SIZE);
	}

	if (PX4FLOW_REG == 0x16) {
		memcpy(&_frame_integral, val, I2C_INTEGRAL_FRAME_SIZE);
	}


	optical_flow_s report{};

	report.timestamp = hrt_absolute_time();
	report.pixel_flow_x_integral = static_cast<float>(_frame_integral.pixel_flow_x_integral) / 10000.0f;//convert to radians
	report.pixel_flow_y_integral = static_cast<float>(_frame_integral.pixel_flow_y_integral) / 10000.0f;//convert to radians
	report.frame_count_since_last_readout = _frame_integral.frame_count_since_last_readout;
	report.ground_distance_m = static_cast<float>(_frame_integral.ground_distance) / 1000.0f;//convert to meters
	report.quality = _frame_integral.qual; //0:bad ; 255 max quality
	report.gyro_x_rate_integral = static_cast<float>(_frame_integral.gyro_x_rate_integral) / 10000.0f; //convert to radians
	report.gyro_y_rate_integral = static_cast<float>(_frame_integral.gyro_y_rate_integral) / 10000.0f; //convert to radians
	report.gyro_z_rate_integral = static_cast<float>(_frame_integral.gyro_z_rate_integral) / 10000.0f; //convert to radians
	report.integration_timespan = _frame_integral.integration_timespan; //microseconds
	report.time_since_last_sonar_update = _frame_integral.sonar_timestamp;//microseconds
	report.gyro_temperature = _frame_integral.gyro_temperature;//Temperature * 100 in centi-degrees Celsius
	report.sensor_id = 0;
	report.max_flow_rate = _sensor_max_flow_rate;
	report.min_ground_distance = _sensor_min_range;
	report.max_ground_distance = _sensor_max_range;

	/* rotate measurements in yaw from sensor frame to body frame according to parameter SENS_FLOW_ROT */
	float zeroval = 0.0f;

	rotate_3f(_sensor_rotation, report.pixel_flow_x_integral, report.pixel_flow_y_integral, zeroval);
	rotate_3f(_sensor_rotation, report.gyro_x_rate_integral, report.gyro_y_rate_integral, report.gyro_z_rate_integral);

	if (_px4flow_topic == nullptr) {
		_px4flow_topic = orb_advertise(ORB_ID(optical_flow), &report);

	} else {
		/* publish it */
		orb_publish(ORB_ID(optical_flow), _px4flow_topic, &report);
	}

	/* publish to the distance_sensor topic as well */
	distance_sensor_s distance_report{};
	distance_report.timestamp = report.timestamp;
	distance_report.min_distance = PX4FLOW_MIN_DISTANCE;
	distance_report.max_distance = PX4FLOW_MAX_DISTANCE;
	distance_report.current_distance = report.ground_distance_m;
	distance_report.variance = 0.0f;
	distance_report.signal_quality = -1;
	distance_report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
	/* TODO: the ID needs to be properly set */
	distance_report.id = 0;
	distance_report.orientation = _sonar_rotation;

	orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &distance_report);

	perf_end(_sample_perf);

	return PX4_OK;
}

void
PX4FLOW::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

void
PX4FLOW::stop()
{
	ScheduleClear();
}

void
PX4FLOW::Run()
{
	if (OK != measure()) {
		DEVICE_DEBUG("measure error");
	}

	/* perform collection */
	if (OK != collect()) {
		DEVICE_DEBUG("collection error");
		/* restart the measurement state machine */
		start();
		return;
	}

	ScheduleDelayed(PX4FLOW_CONVERSION_INTERVAL_DEFAULT);
}

void
PX4FLOW::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

/**
 * Local functions in support of the shell command.
 */
namespace px4flow
{

PX4FLOW	*g_dev = nullptr;
bool start_in_progress = false;

const int START_RETRY_COUNT = 5;
const int START_RETRY_TIMEOUT = 1000;

int start(int argc, char *argv[]);
int stop();
int info();
void usage();

/**
 * Start the driver.
 */
int
start(int argc, char *argv[])
{
	/* entry check: */
	if (start_in_progress) {
		PX4_WARN("start already in progress");
		return 1;
	}

	if (g_dev != nullptr) {
		start_in_progress = false;
		PX4_WARN("already started");
		return 1;
	}

	/* parse command line options */
	int address = I2C_FLOW_ADDRESS_DEFAULT;
	int conversion_interval = PX4FLOW_CONVERSION_INTERVAL_DEFAULT;

	/* don't exit from getopt loop to leave getopt global variables in consistent state,
	 * set error flag instead */
	bool err_flag = false;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	uint8_t sonar_rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	while ((ch = px4_getopt(argc, argv, "a:i:R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'a':
			address = strtoul(myoptarg, nullptr, 16);

			if (address < I2C_FLOW_ADDRESS_MIN || address > I2C_FLOW_ADDRESS_MAX) {
				PX4_WARN("invalid i2c address '%s'", myoptarg);
				err_flag = true;

			}

			break;

		case 'i':
			conversion_interval = strtoul(myoptarg, nullptr, 10);

			if (conversion_interval < PX4FLOW_CONVERSION_INTERVAL_MIN || conversion_interval > PX4FLOW_CONVERSION_INTERVAL_MAX) {
				PX4_WARN("invalid conversion interval '%s'", myoptarg);
				err_flag = true;
			}

			break;

		case 'R':
			sonar_rotation = (uint8_t)atoi(myoptarg);
			PX4_INFO("Setting sonar orientation to %d", (int)sonar_rotation);

			break;

		default:
			err_flag = true;
			break;
		}
	}

	if (err_flag) {
		usage();
		return PX4_ERROR;
	}

	/* starting */
	start_in_progress = true;
	PX4_INFO("scanning I2C buses for device..");

	int retry_nr = 0;

	while (1) {
		const int busses_to_try[] = {
			PX4_I2C_BUS_EXPANSION,
#ifdef PX4_I2C_BUS_ESC
			PX4_I2C_BUS_ESC,
#endif
#ifdef PX4_I2C_BUS_ONBOARD
			PX4_I2C_BUS_ONBOARD,
#endif
#ifdef PX4_I2C_BUS_EXPANSION1
			PX4_I2C_BUS_EXPANSION1,
#endif
#ifdef PX4_I2C_BUS_EXPANSION2
			PX4_I2C_BUS_EXPANSION2,
#endif

			-1
		};

		const int *cur_bus = busses_to_try;

		while (*cur_bus != -1) {
			/* create the driver */
			/* PX4_WARN("trying bus %d", *cur_bus); */
			g_dev = new PX4FLOW(*cur_bus, address, (enum Rotation)0, conversion_interval, sonar_rotation);

			if (g_dev == nullptr) {
				/* this is a fatal error */
				break;
			}

			/* init the driver: */
			if (OK == g_dev->init()) {
				/* success! */
				break;
			}

			/* destroy it again because it failed. */
			delete g_dev;
			g_dev = nullptr;

			/* try next! */
			cur_bus++;
		}

		/* check whether we found it: */
		if (*cur_bus != -1) {

			/* check for failure: */
			if (g_dev == nullptr) {
				break;
			}

			/* success! */
			start_in_progress = false;
			return 0;
		}

		if (retry_nr < START_RETRY_COUNT) {
			/* lets not be too verbose */
			// PX4_WARN("PX4FLOW not found on I2C busses. Retrying in %d ms. Giving up in %d retries.", START_RETRY_TIMEOUT, START_RETRY_COUNT - retry_nr);
			px4_usleep(START_RETRY_TIMEOUT * 1000);
			retry_nr++;

		} else {
			break;
		}
	}

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	start_in_progress = false;

	return PX4_OK;
}

/**
 * Stop the driver
 */
int
stop()
{
	start_in_progress = false;

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

		return PX4_OK;

	} else {
		PX4_WARN("driver not running");
	}

	return PX4_ERROR;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_WARN("driver not running");
		return PX4_ERROR;
	}

	g_dev->print_info();

	return PX4_OK;
}

void usage()
{
	PX4_INFO("usage: px4flow {start|info'}");
	PX4_INFO("    [-a i2c_address]");
	PX4_INFO("    [-i i2c_interval]");
}

} // namespace

int
px4flow_main(int argc, char *argv[])
{
	if (argc < 2) {
		px4flow::usage();
		return 1;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		return px4flow::start(argc, argv);
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		return px4flow::stop();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		return px4flow::info();
	}

	px4flow::usage();

	return PX4_OK;
}
