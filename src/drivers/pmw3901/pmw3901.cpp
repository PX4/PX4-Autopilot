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
 * @file pmw3901.cpp
 * @author Daniele Pettenuzzo
 *
 * Driver for the pmw3901 optical flow sensor connected via SPI.
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>

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

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/distance_sensor.h>

#include <board_config.h>

/* Configuration Constants */
#ifdef PX4_SPI_BUS_EXT
#define PMW3901_BUS PX4_SPI_BUS_EXT
#else
#define PMW3901_BUS 0
#endif

#define PMW3901_SPI_BUS_SPEED    2*1000*1000      // 2MHz

#define DIR_READ(a)                     ((a) | (1 << 7))
#define DIR_WRITE(a)                    ((a) & 0x7f)

#define PMW3901_BASEADDR      	0b0101001 					// set camera address
#define PMW3901_DEVICE_PATH   	"/dev/pmw3901"

/* VL53LXX Registers addresses */
#define VL53LXX_CONVERSION_INTERVAL 1000 /* 1ms */

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class PMW3901 : public device::SPI
{
public:
	PMW3901(uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING,
		   int bus = PMW3901_BUS, int address = PMW3901_BASEADDR);
	virtual ~PMW3901();

	virtual int 		init();

	//virtual ssize_t		read(device::file_t *filp, char *buffer, size_t buflen);
	virtual int			ioctl(device::file_t *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

private:
	uint8_t _rotation;
	float				_min_distance;
	float				_max_distance;
	work_s				_work;
	ringbuffer::RingBuffer		*_reports;
	bool				_sensor_ok;
	uint8_t				_valid;
	int					_measure_ticks;
	bool				_collect_phase;
	int					_class_instance;
	int					_orb_class_instance;

	orb_advert_t		_distance_sensor_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	uint8_t 			stop_variable_;


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
	void				cycle();
	int					measure();
	int					collect();

	int 				write(unsigned reg, uint8_t *data, unsigned count);
	int 				read(unsigned reg, uint8_t *data, unsigned count);
  int         writeRegister(unsigned reg, uint8_t data);

	int 				sensorInit();
  void        readMotionCount(uint16_t &deltaX, uint16_t &deltaY);

	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void		cycle_trampoline(void *arg);


};


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int pmw3901_main(int argc, char *argv[]);

PMW3901::PMW3901(uint8_t rotation, int bus, int address) :
	SPI("PMW3901", PMW3901_DEVICE_PATH, bus, address, SPIDEV_MODE3, PMW3901_SPI_BUS_SPEED),
	_rotation(rotation),
	_min_distance(-1.0f),
	_max_distance(-1.0f),
	_reports(nullptr),
	_sensor_ok(false),
	_valid(0),
	_measure_ticks(0),
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_distance_sensor_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "tr1_read")),
	_comms_errors(perf_alloc(PC_COUNT, "tr1_com_err"))
{
	// up the retries since the device misses the first measure attempts
	//I2C::_retries = 3;

	// enable debug() calls
	_debug_enabled = false;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

PMW3901::~PMW3901()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}


int
PMW3901::sensorInit()
{
	//uint8_t val = 0;
	int ret;
  uint8_t data[5];

  // initialize pmw3901 flow sensor

  // Power on reset
  writeRegister(0x3A, 0x5A);
  usleep(5000);

   // Test the SPI communication, checking chipId and inverse chipId
  read(0x00, &data[0], 1); // chip id
  read(0x5F, &data[1], 1); // inverse chip id

  if (data[0] != 0x49 && data[1] != 0xB8) return false;

  // Reading the motion registers one time
  read(0x02, &data[0], 1);
  read(0x03, &data[1], 1);
  read(0x04, &data[2], 1);
  read(0x05, &data[3], 1);
  read(0x06, &data[4], 1);
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

  usleep(100000);

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

	ret = OK;

	return ret;

}

int
PMW3901::init()
{
	int ret = PX4_ERROR;

	set_device_address(PMW3901_BASEADDR);

	/* do I2C init (and probe) first */
	if (SPI::init() != OK) {
		goto out;
	}

  set_frequency(PMW3901_SPI_BUS_SPEED);

	sensorInit();

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

	if (_reports == nullptr) {
		goto out;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {               // change to optical flow topic
		/* get a publish handle on the range finder topic */
		struct distance_sensor_s ds_report;
		//measure();
		_reports->get(&ds_report);

		_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
					 &_orb_class_instance, ORB_PRIO_LOW);

		if (_distance_sensor_topic == nullptr) {
			DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
		}
	}

	ret = OK;
	_sensor_ok = true;

out:
	return ret;
}


int
PMW3901::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	printf("ioctl\n");
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

				case SENSOR_POLLRATE_DEFAULT: {
						/* do we need to start internal polling? */
						//bool want_start = (_measure_ticks == 0);

						/* set interval for next measurement to minimum legal value */
						_measure_ticks = USEC2TICK(VL53LXX_CONVERSION_INTERVAL);

						/* if we need to start the poll state machine, do it */
						// if (want_start) {
						// 	start();
						// }
						start();

						return OK;
				}

				/* adjust to a legal polling interval in Hz */
				default: {
						/* do we need to start internal polling? */
						//bool want_start = (_measure_ticks == 0);

						/* convert hz to tick interval via microseconds */
						unsigned ticks = USEC2TICK(1000000 / arg);

						/* update interval for next measurement */
						_measure_ticks = ticks;

						/* if we need to start the poll state machine, do it */
						//if (want_start) {
							start();
						//}

						return OK;
					}
				}
			}

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

// ssize_t
// VL53LXX::read(device::file_t *filp, char *buffer, size_t buflen)
// {
// 	unsigned count = buflen / sizeof(struct distance_sensor_s);
// 	struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);
// 	int ret = 0;

// 	/* buffer must be large enough */
// 	if (count < 1) {
// 		return -ENOSPC;
// 	}

// 	/* if automatic measurement is enabled */
// 	if (_measure_ticks > 0) {

// 		/*
// 		 * While there is space in the caller's buffer, and reports, copy them.
// 		 * Note that we may be pre-empted by the workq thread while we are doing this;
// 		 * we are careful to avoid racing with them.
// 		 */
// 		while (count--) {
// 			if (_reports->get(rbuf)) {
// 				ret += sizeof(*rbuf);
// 				rbuf++;
// 			}
// 		}

// 		/* if there was no data, warn the caller */
// 		return ret ? ret : -EAGAIN;
// 	}

// 	/* manual measurement - run one conversion */
// 	do {
// 		_reports->flush();

// 		/* trigger a measurement */
// 		if (OK != measure()) {
// 			ret = -EIO;
// 			break;
// 		}

// 		/* wait for it to complete */
// 		usleep(VL53LXX_CONVERSION_INTERVAL);

// 		/* run the collection phase */
// 		if (OK != collect()) {
// 			ret = -EIO;
// 			break;
// 		}

// 		/* state machine will have generated a report, copy it out */
// 		if (_reports->get(rbuf)) {
// 			ret = sizeof(*rbuf);
// 		}

// 	} while (0);

// 	return ret;
// }


int
PMW3901::read(unsigned reg, uint8_t *data, unsigned count)
{
  uint8_t cmd[5]; // read up to 4 bytes
  int ret;

  cmd[0] = DIR_READ(reg);
  //cmd[1] = 0;

  transfer(cmd, cmd, count+1);
  memcpy(&data[0], &cmd[1], count);

  ret = OK;

  return ret;

}


int
PMW3901::write(unsigned reg, uint8_t *data, unsigned count)
{
  uint8_t cmd[5]; // write up to 4 bytes
  int ret;

	if (sizeof(cmd) < (count + 1)) {
		return -EIO;
	}

	cmd[0] = DIR_WRITE(reg);
	//cmd[1] = *(uint8_t *)data;
  memcpy(&cmd[1], &data[0], count);

	transfer(&cmd[0], nullptr, count + 1);

  ret = OK;

  return ret;

}


int
PMW3901::writeRegister(unsigned reg, uint8_t data)
{
  uint8_t cmd[2]; // write up to 4 bytes
  int ret;

	cmd[0] = DIR_WRITE(reg);

  cmd[1] = data;

	transfer(&cmd[0], nullptr, 2);

  ret = OK;

  return ret;

}


int
PMW3901::measure()
{
	int ret;
  uint16_t deltaX, deltaY;

  readMotionCount(deltaX, deltaY);

  printf("deltaX= %u, deltaY= %u", deltaX, deltaY);

	ret = OK;

	return ret;
}


void PMW3901::readMotionCount(uint16_t &deltaX, uint16_t &deltaY)
{
 uint8_t tmp;
 uint8_t dX[2];
 uint8_t dY[2];

 read(0x02, &tmp, 1);

 read(0x03, &dX[0], 1);
 read(0x04, &dX[1], 1);
 deltaX = ((int16_t)dX[1] << 8) | dX[0];

 read(0x05, &dY[0], 1);
 read(0x06, &dY[1], 1);
 deltaY = ((int16_t)dY[1] << 8) | dY[0];

 return;

}


int
PMW3901::collect()
{
	int ret = -EIO;

	/* read from the sensor */
	//uint8_t val[2] = {0, 0};

	perf_begin(_sample_perf);

	// ret = transfer(nullptr, 0, &val[0], 2);                          // change to spi transfer
  //
	// if (ret < 0) {
	// 	DEVICE_LOG("error reading from sensor: %d", ret);
	// 	perf_count(_comms_errors);
	// 	perf_end(_sample_perf);
	// 	return ret;
	// }
  //
	// uint16_t distance_mm = (val[0] << 8) | val[1];
	// float distance_m = float(distance_mm) *  1e-3f;
	// struct distance_sensor_s report;
  //
	// report.timestamp = hrt_absolute_time();                              // change to flow topic
	// report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	// report.orientation = _rotation;
	// report.current_distance = distance_m;
	// report.min_distance = 0.0f;
	// report.max_distance = 2.0f;
	// report.covariance = 0.0f;
	// /* TODO: set proper ID */
	// report.id = 0;
  //
	// /* publish it, if we are the primary */
	// if (_distance_sensor_topic != nullptr) {
	// 	orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
	// }
  //
	// _reports->force(&report);
  //
	// /* notify anyone waiting for data */
	// poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
PMW3901::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&PMW3901::cycle_trampoline, this, 1000);

	/* notify about state change */
	struct subsystem_info_s info = {};
	info.present = true;
	info.enabled = true;
	info.ok = true;
	info.subsystem_type = subsystem_info_s::SUBSYSTEM_TYPE_RANGEFINDER;

	static orb_advert_t pub = nullptr;

	if (pub != nullptr) {
		orb_publish(ORB_ID(subsystem_info), pub, &info);

	} else {
		pub = orb_advertise(ORB_ID(subsystem_info), &info);
	}
}

void
PMW3901::stop()
{
	work_cancel(HPWORK, &_work);
}

void
PMW3901::cycle_trampoline(void *arg)
{
	PMW3901 *dev = (PMW3901 *)arg;

	dev->cycle();
}

void
PMW3901::cycle()
{
	measure();
	//collect();

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&PMW3901::cycle_trampoline,
		   this,
		   USEC2TICK(20000));

}

void
PMW3901::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}


/**
 * Local functions in support of the shell command.
 */
namespace pmw3901
{

PMW3901	*g_dev;

void	start(uint8_t rotation);
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start(uint8_t rotation)
{
	int fd;

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	g_dev = new PMW3901(rotation, PMW3901_BUS);


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
	//struct distance_sensor_s report;                   // change report type

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	g_dev = new PMW3901(0, PMW3901_BUS);

	if (g_dev == nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	if (OK != g_dev->init()) {
		delete g_dev;
		g_dev = nullptr;

	}

	int fd = open(PMW3901_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'vl53lxx start' if the driver is not running", PMW3901_DEVICE_PATH);
	}

	// warnx("single read");
	// warnx("measurement: %0.2f m", (double)report.current_distance);
	// warnx("time:        %llu", report.timestamp);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(PMW3901_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
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

} // namespace

int
pmw3901_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	while ((ch = px4_getopt(argc, argv, "R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			PX4_INFO("Setting sensor orientation to %d", (int)rotation);
			break;

		default:
			PX4_WARN("Unknown option!");
		}
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		pmw3901::start(rotation);
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
	 * Reset the driver.
	 */
	if (!strcmp(argv[myoptind], "reset")) {
		pmw3901::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[1], "status")) {
		pmw3901::info();
	}

	PX4_ERR("unrecognized command, try 'start', 'test', 'reset' or 'info'");
	return PX4_ERROR;
}
