/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file pmw3901.cpp
 * @author Daniele Pettenuzzo
 *
 * Driver for the pmw3901 optical flow sensor connected via SPI.
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/device/spi.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <float.h>

#include <conversion/rotation.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <parameters/param.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/sensor_gyro.h>

#include <board_config.h>

/* Configuration Constants */

#if defined PX4_SPI_BUS_EXPANSION   // crazyflie
# define PMW3901_BUS PX4_SPI_BUS_EXPANSION
#elif defined PX4_SPI_BUS_EXTERNAL1   // fmu-v5
# define PMW3901_BUS PX4_SPI_BUS_EXTERNAL1
#elif defined PX4_SPI_BUS_EXTERNAL    // fmu-v4 extspi
# define PMW3901_BUS PX4_SPI_BUS_EXTERNAL
#else
# error "add the required spi bus from board_config.h here"
#endif

#if defined PX4_SPIDEV_EXPANSION_2    // crazyflie flow deck
# define PMW3901_SPIDEV PX4_SPIDEV_EXPANSION_2
#elif defined PX4_SPIDEV_EXTERNAL1_1    // fmu-v5 ext CS1
# define PMW3901_SPIDEV PX4_SPIDEV_EXTERNAL1_1
#elif defined PX4_SPIDEV_EXTERNAL   // fmu-v4 extspi
# define PMW3901_SPIDEV PX4_SPIDEV_EXTERNAL
#else
# error "add the required spi dev from board_config.h here"
#endif

#define PMW3901_SPI_BUS_SPEED (2000000L) // 2MHz

#define DIR_WRITE(a) ((a) | (1 << 7))
#define DIR_READ(a) ((a) & 0x7f)

#define PMW3901_DEVICE_PATH "/dev/pmw3901"

/* PMW3901 Registers addresses */
#define PMW3901_US 1000 /*   1 ms */
#define PMW3901_SAMPLE_RATE 10000 /*  10 ms */


class PMW3901 : public device::SPI, public px4::ScheduledWorkItem
{
public:
	PMW3901(int bus = PMW3901_BUS, enum Rotation yaw_rotation = (enum Rotation)0);

	virtual ~PMW3901();

	virtual int init();

	virtual ssize_t read(device::file_t *filp, char *buffer, size_t buflen);

	virtual int ioctl(device::file_t *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void print_info();

protected:
	virtual int probe();

private:
	ringbuffer::RingBuffer *_reports;
	bool _sensor_ok;
	int _measure_interval;
	int _class_instance;
	int _orb_class_instance;

	const uint64_t _collect_time = 15000; // usecs, optical flow data publish rate

	orb_advert_t _optical_flow_pub;
	orb_advert_t _subsystem_pub;

	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;

	uint64_t _previous_collect_timestamp;

	enum Rotation _yaw_rotation;
	int _flow_sum_x = 0;
	int _flow_sum_y = 0;
	uint64_t _flow_dt_sum_usec = 0;


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

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void Run() override;

	int	collect();

	int readRegister(unsigned reg, uint8_t *data, unsigned count);
	int writeRegister(unsigned reg, uint8_t data);

	int sensorInit();
	int readMotionCount(int16_t &deltaX, int16_t &deltaY);
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int pmw3901_main(int argc, char *argv[]);

PMW3901::PMW3901(int bus, enum Rotation yaw_rotation) :
	SPI("PMW3901", PMW3901_DEVICE_PATH, bus, PMW3901_SPIDEV, SPIDEV_MODE0, PMW3901_SPI_BUS_SPEED),
	ScheduledWorkItem(px4::device_bus_to_wq(get_device_id())),
	_reports(nullptr),
	_sensor_ok(false),
	_measure_interval(0),
	_class_instance(-1),
	_orb_class_instance(-1),
	_optical_flow_pub(nullptr),
	_subsystem_pub(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "pmw3901_read")),
	_comms_errors(perf_alloc(PC_COUNT, "pmw3901_com_err")),
	_previous_collect_timestamp(0),
	_yaw_rotation(yaw_rotation)
{
}

PMW3901::~PMW3901()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}


int
PMW3901::sensorInit()
{
	uint8_t data[5];

	// Power on reset
	writeRegister(0x3A, 0x5A);
	usleep(5000);

	// Reading the motion registers one time
	readRegister(0x02, &data[0], 1);
	readRegister(0x03, &data[1], 1);
	readRegister(0x04, &data[2], 1);
	readRegister(0x05, &data[3], 1);
	readRegister(0x06, &data[4], 1);

	usleep(1000);

	// set performance optimization registers
	writeRegister(0x7F, 0x00);
	writeRegister(0x61, 0xAD);
	writeRegister(0x7F, 0x03);
	writeRegister(0x40, 0x00);
	writeRegister(0x7F, 0x05);
	writeRegister(0x41, 0xB3);
	writeRegister(0x43, 0xF1);
	writeRegister(0x45, 0x14);
	writeRegister(0x5B, 0x32);
	writeRegister(0x5F, 0x34);
	writeRegister(0x7B, 0x08);
	writeRegister(0x7F, 0x06);
	writeRegister(0x44, 0x1B);
	writeRegister(0x40, 0xBF);
	writeRegister(0x4E, 0x3F);
	writeRegister(0x7F, 0x08);
	writeRegister(0x65, 0x20);
	writeRegister(0x6A, 0x18);
	writeRegister(0x7F, 0x09);
	writeRegister(0x4F, 0xAF);
	writeRegister(0x5F, 0x40);
	writeRegister(0x48, 0x80);
	writeRegister(0x49, 0x80);
	writeRegister(0x57, 0x77);
	writeRegister(0x60, 0x78);
	writeRegister(0x61, 0x78);
	writeRegister(0x62, 0x08);
	writeRegister(0x63, 0x50);
	writeRegister(0x7F, 0x0A);
	writeRegister(0x45, 0x60);
	writeRegister(0x7F, 0x00);
	writeRegister(0x4D, 0x11);
	writeRegister(0x55, 0x80);
	writeRegister(0x74, 0x1F);
	writeRegister(0x75, 0x1F);
	writeRegister(0x4A, 0x78);
	writeRegister(0x4B, 0x78);
	writeRegister(0x44, 0x08);
	writeRegister(0x45, 0x50);
	writeRegister(0x64, 0xFF);
	writeRegister(0x65, 0x1F);
	writeRegister(0x7F, 0x14);
	writeRegister(0x65, 0x60);
	writeRegister(0x66, 0x08);
	writeRegister(0x63, 0x78);
	writeRegister(0x7F, 0x15);
	writeRegister(0x48, 0x58);
	writeRegister(0x7F, 0x07);
	writeRegister(0x41, 0x0D);
	writeRegister(0x43, 0x14);
	writeRegister(0x4B, 0x0E);
	writeRegister(0x45, 0x0F);
	writeRegister(0x44, 0x42);
	writeRegister(0x4C, 0x80);
	writeRegister(0x7F, 0x10);
	writeRegister(0x5B, 0x02);
	writeRegister(0x7F, 0x07);
	writeRegister(0x40, 0x41);
	writeRegister(0x70, 0x00);

	usleep(10000);

	writeRegister(0x32, 0x44);
	writeRegister(0x7F, 0x07);
	writeRegister(0x40, 0x40);
	writeRegister(0x7F, 0x06);
	writeRegister(0x62, 0xf0);
	writeRegister(0x63, 0x00);
	writeRegister(0x7F, 0x0D);
	writeRegister(0x48, 0xC0);
	writeRegister(0x6F, 0xd5);
	writeRegister(0x7F, 0x00);
	writeRegister(0x5B, 0xa0);
	writeRegister(0x4E, 0xA8);
	writeRegister(0x5A, 0x50);
	writeRegister(0x40, 0x80);

	writeRegister(0x7F, 0x00);
	writeRegister(0x5A, 0x10);
	writeRegister(0x54, 0x00);

	return OK;

}

int
PMW3901::init()
{
	int ret = PX4_ERROR;

	// get yaw rotation from sensor frame to body frame
	param_t rot = param_find("SENS_FLOW_ROT");

	if (rot != PARAM_INVALID) {
		int32_t val = 0;
		param_get(rot, &val);

		_yaw_rotation = (enum Rotation)val;
	}

	/* For devices competing with NuttX SPI drivers on a bus (Crazyflie SD Card expansion board) */
	SPI::set_lockmode(LOCK_THREADS);

	/* do SPI init (and probe) first */
	if (SPI::init() != OK) {
		goto out;
	}

	sensorInit();

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(optical_flow_s));

	if (_reports == nullptr) {
		goto out;
	}

	ret = OK;
	_sensor_ok = true;
	_previous_collect_timestamp = hrt_absolute_time();

out:
	return ret;

}


int
PMW3901::probe()
{
	uint8_t data[2] = { 0, 0 };

	readRegister(0x00, &data[0], 1); // chip id

	// Test the SPI communication, checking chipId and inverse chipId
	if (data[0] == 0x49) {
		return OK;
	}

	// not found on any address
	return -EIO;
}


int
PMW3901::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			case 0:
				return -EINVAL;

			case SENSOR_POLLRATE_DEFAULT: {

					/* do we need to start internal polling? */
					bool want_start = (_measure_interval == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_interval = (PMW3901_SAMPLE_RATE);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_interval == 0);

					/* convert hz to tick interval via microseconds */
					unsigned interval = (1000000 / arg);

					/* check against maximum rate */
					if (interval < (PMW3901_SAMPLE_RATE)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_interval = interval;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

ssize_t
PMW3901::read(device::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct optical_flow_s);
	struct optical_flow_s *rbuf = reinterpret_cast<struct optical_flow_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
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

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}


int
PMW3901::readRegister(unsigned reg, uint8_t *data, unsigned count)
{
	uint8_t cmd[5]; 					// read up to 4 bytes
	int ret;

	cmd[0] = DIR_READ(reg);

	ret = transfer(&cmd[0], &cmd[0], count + 1);

	if (OK != ret) {
		perf_count(_comms_errors);
		DEVICE_LOG("spi::transfer returned %d", ret);
		return ret;
	}

	memcpy(&data[0], &cmd[1], count);

	return ret;

}


int
PMW3901::writeRegister(unsigned reg, uint8_t data)
{
	uint8_t cmd[2]; 						// write 1 byte
	int ret;

	cmd[0] = DIR_WRITE(reg);
	cmd[1] = data;

	ret = transfer(&cmd[0], nullptr, 2);

	if (OK != ret) {
		perf_count(_comms_errors);
		DEVICE_LOG("spi::transfer returned %d", ret);
		return ret;
	}

	return ret;

}


int
PMW3901::collect()
{
	int ret = OK;
	int16_t delta_x_raw, delta_y_raw;
	float delta_x, delta_y;

	perf_begin(_sample_perf);

	uint64_t timestamp = hrt_absolute_time();
	uint64_t dt_flow = timestamp - _previous_collect_timestamp;
	_previous_collect_timestamp = timestamp;

	_flow_dt_sum_usec += dt_flow;

	readMotionCount(delta_x_raw, delta_y_raw);

	_flow_sum_x += delta_x_raw;
	_flow_sum_y += delta_y_raw;

	// returns if the collect time has not been reached
	if (_flow_dt_sum_usec < _collect_time) {

		return ret;
	}

	delta_x = (float)_flow_sum_x / 500.0f;		// proportional factor + convert from pixels to radians
	delta_y = (float)_flow_sum_y / 500.0f;		// proportional factor + convert from pixels to radians

	struct optical_flow_s report;

	report.timestamp = timestamp;

	report.pixel_flow_x_integral = static_cast<float>(delta_x);
	report.pixel_flow_y_integral = static_cast<float>(delta_y);

	// rotate measurements in yaw from sensor frame to body frame according to parameter SENS_FLOW_ROT
	float zeroval = 0.0f;
	rotate_3f(_yaw_rotation, report.pixel_flow_x_integral, report.pixel_flow_y_integral, zeroval);
	rotate_3f(_yaw_rotation, report.gyro_x_rate_integral, report.gyro_y_rate_integral, report.gyro_z_rate_integral);

	report.frame_count_since_last_readout = 4;				//microseconds
	report.integration_timespan = _flow_dt_sum_usec; 		//microseconds

	report.sensor_id = 0;

	// This sensor doesn't provide any quality metric. However if the sensor is unable to calculate the optical flow it will
	// output 0 for the delta. Hence, we set the measurement to "invalid" (quality = 0) if the values are smaller than FLT_EPSILON
	if (fabsf(report.pixel_flow_x_integral) < FLT_EPSILON && fabsf(report.pixel_flow_y_integral) < FLT_EPSILON) {
		report.quality = 0;

	} else {
		report.quality = 255;
	}

	/* No gyro on this board */
	report.gyro_x_rate_integral = NAN;
	report.gyro_y_rate_integral = NAN;
	report.gyro_z_rate_integral = NAN;

	// set (conservative) specs according to datasheet
	report.max_flow_rate = 5.0f;       // Datasheet: 7.4 rad/s
	report.min_ground_distance = 0.1f; // Datasheet: 80mm
	report.max_ground_distance = 30.0f; // Datasheet: infinity

	_flow_dt_sum_usec = 0;
	_flow_sum_x = 0;
	_flow_sum_y = 0;

	if (_optical_flow_pub == nullptr) {

		_optical_flow_pub = orb_advertise(ORB_ID(optical_flow), &report);

	} else {

		orb_publish(ORB_ID(optical_flow), _optical_flow_pub, &report);
	}

	/* post a report to the ring */
	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	perf_end(_sample_perf);

	return ret;

}


int
PMW3901::readMotionCount(int16_t &deltaX, int16_t &deltaY)
{
	int ret;

	uint8_t data[10] = { DIR_READ(0x02), 0, DIR_READ(0x03), 0, DIR_READ(0x04), 0,
			     DIR_READ(0x05), 0, DIR_READ(0x06), 0
			   };

	ret = transfer(&data[0], &data[0], 10);

	if (OK != ret) {
		perf_count(_comms_errors);
		DEVICE_LOG("spi::transfer returned %d", ret);
		return ret;
	}

	deltaX = ((int16_t)data[5] << 8) | data[3];
	deltaY = ((int16_t)data[9] << 8) | data[7];

	ret = OK;

	return ret;

}


void
PMW3901::start()
{
	/* reset the report ring and state machine */
	_reports->flush();

	/* schedule a cycle to start things */
	ScheduleDelayed(PMW3901_US);

	/* notify about state change */
	struct subsystem_info_s info = {};

	info.timestamp = hrt_absolute_time();
	info.present = true;
	info.enabled = true;
	info.ok = true;
	info.subsystem_type = subsystem_info_s::SUBSYSTEM_TYPE_OPTICALFLOW;

	if (_subsystem_pub != nullptr) {
		orb_publish(ORB_ID(subsystem_info), _subsystem_pub, &info);

	} else {
		_subsystem_pub = orb_advertise(ORB_ID(subsystem_info), &info);
	}
}

void
PMW3901::stop()
{
	ScheduleClear();
}

void
PMW3901::Run()
{
	collect();

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(PMW3901_SAMPLE_RATE);
}

void
PMW3901::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u\n", _measure_interval);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace pmw3901
{

PMW3901	*g_dev;

void	start(int spi_bus);
void	stop();
void	test();
void	reset();
void	info();
void	usage();


/**
 * Start the driver.
 */
void
start(int spi_bus)
{
	int fd;

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	g_dev = new PMW3901(spi_bus, (enum Rotation)0);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(PMW3901_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}


/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{

	struct optical_flow_s report;
	ssize_t sz;

	int fd = open(PMW3901_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'pmw3901 start' if the driver is not running)", PMW3901_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_ERR("ret: %d, expected: %d", sz, sizeof(report));
		err(1, "immediate acc read failed");
	}

	print_message(report);

	errx(0, "PASS");
}


/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

/**
 * Print a little info about how to start/stop/use the driver
 */
void usage()
{
	PX4_INFO("usage: pmw3901 {start|test|reset|info'}");
	PX4_INFO("    [-b SPI_BUS]");
}

} // namespace pmw3901


int
pmw3901_main(int argc, char *argv[])
{
	if (argc < 2) {
		pmw3901::usage();
		return PX4_ERROR;
	}

	// don't exit from getopt loop to leave getopt global variables in consistent state,
	// set error flag instead
	bool err_flag = false;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	int spi_bus = PMW3901_BUS;

	while ((ch = px4_getopt(argc, argv, "b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			spi_bus = (uint8_t)atoi(myoptarg);

			break;

		default:
			err_flag = true;
			break;
		}
	}

	if (err_flag) {
		pmw3901::usage();
		return PX4_ERROR;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		pmw3901::start(spi_bus);
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		pmw3901::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[myoptind], "test")) {
		pmw3901::test();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		pmw3901::info();
	}

	pmw3901::usage();
	return PX4_ERROR;
}
