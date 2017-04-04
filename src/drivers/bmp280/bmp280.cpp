/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file bmp280.cpp
 * Driver for the BMP280 barometric pressure sensor connected via I2C TODO or SPI.
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>
#include <board_config.h>
#include <drivers/bmp280/bmp280.h>

#include <drivers/device/device.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>


enum BMP280_BUS {
	BMP280_BUS_ALL = 0,
	BMP280_BUS_I2C_INTERNAL,
	BMP280_BUS_I2C_EXTERNAL,
	BMP280_BUS_SPI_INTERNAL,
	BMP280_BUS_SPI_EXTERNAL
};

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

/*
 * BMP280 internal constants and data structures.
 */

class BMP280 : public device::CDev
{
public:
	BMP280(bmp280::IBMP280 *interface, const char *path);
	~BMP280();

	virtual int		init();

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

private:
	bmp280::IBMP280	*_interface;

	uint8_t				_curr_ctrl;

	struct work_s		_work;
	unsigned			_report_ticks; // 0 - no cycling, otherwise period of sending a report
	unsigned			_max_mesure_ticks; //ticks needed to measure

	ringbuffer::RingBuffer	*_reports;

	bool			_collect_phase;

	/* altitude conversion calibration */
	unsigned		_msl_pressure;	/* in Pa */

	orb_advert_t		_baro_topic;
	int					_orb_class_instance;
	int					_class_instance;

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;

	struct bmp280::calibration_s *_cal; //stored calibration constants
	struct bmp280::fcalibration_s _fcal; //pre processed calibration constants

	float			_P; /* in Pa */
	float			_T; /* in K */


	/* periodic execution helpers */
	void			start_cycle();
	void			stop_cycle();
	void			cycle(); //main execution
	static void		cycle_trampoline(void *arg);

	int		measure(); //start measure
	int		collect(); //get results and publish
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int bmp280_main(int argc, char *argv[]);

BMP280::BMP280(bmp280::IBMP280 *interface, const char *path) :
	CDev("BMP280", path),
	_interface(interface),
	_report_ticks(0),
	_reports(nullptr),
	_collect_phase(false),
	_msl_pressure(101325),
	_baro_topic(nullptr),
	_orb_class_instance(-1),
	_class_instance(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmp280_read")),
	_measure_perf(perf_alloc(PC_ELAPSED, "bmp280_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "bmp280_comms_errors"))
{
	_device_id.devid_s.devtype = DRV_BARO_DEVTYPE_BMP280;

	// work_cancel in stop_cycle called from the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

BMP280::~BMP280()
{
	/* make sure we are truly inactive */
	stop_cycle();

	if (_class_instance != -1) {
		unregister_class_devname(get_devname(), _class_instance);
	}

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);

	delete _interface;


}

int
BMP280::init()
{
	int ret;

	ret = CDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("CDev init failed");
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(baro_report));

	if (_reports == nullptr) {
		DEVICE_DEBUG("can't get memory for reports");
		ret = -ENOMEM;
		return ret;
	}

	/* register alternate interfaces if we have to */
	_class_instance = register_class_devname(BARO_BASE_DEVICE_PATH);

	/* reset sensor */
	_interface->set_reg(BPM280_VALUE_RESET, BPM280_ADDR_RESET);
	usleep(10000);

	/* check  id*/
	if (_interface->get_reg(BPM280_ADDR_ID) != BPM280_VALUE_ID) {
		warnx("id of your baro is not: 0x%02x", BPM280_VALUE_ID);
		return -EIO;
	}

	/* set config, recommended settings */
	_curr_ctrl = BPM280_CTRL_P16 | BPM280_CTRL_T2;
	_interface->set_reg(_curr_ctrl, BPM280_ADDR_CTRL);
	_max_mesure_ticks = USEC2TICK(BPM280_MT_INIT + BPM280_MT * (16 - 1 + 2 - 1));
	_interface->set_reg(BPM280_CONFIG_F16, BPM280_ADDR_CONFIG);

	/* get calibration and pre process them*/
	_cal = _interface->get_calibration(BPM280_ADDR_CAL);

	_fcal.t1 =  _cal->t1 * powf(2,  4);
	_fcal.t2 =  _cal->t2 * powf(2, -14);
	_fcal.t3 =  _cal->t3 * powf(2, -34);

	_fcal.p1 = _cal->p1            * (powf(2,  4) / -100000.0f);
	_fcal.p2 = _cal->p1 * _cal->p2 * (powf(2, -31) / -100000.0f);
	_fcal.p3 = _cal->p1 * _cal->p3 * (powf(2, -51) / -100000.0f);

	_fcal.p4 = _cal->p4 * powf(2,  4) - powf(2, 20);
	_fcal.p5 = _cal->p5 * powf(2, -14);
	_fcal.p6 = _cal->p6 * powf(2, -31);

	_fcal.p7 = _cal->p7 * powf(2, -4);
	_fcal.p8 = _cal->p8 * powf(2, -19) + 1.0f;
	_fcal.p9 = _cal->p9 * powf(2, -35);

	/* do a first measurement cycle to populate reports with valid data */
	struct baro_report brp;
	_reports->flush();

	if (measure()) {
		return -EIO;
	}

	usleep(TICK2USEC(_max_mesure_ticks));

	if (collect()) {
		return -EIO;
	}

	_reports->get(&brp);

	_baro_topic = orb_advertise_multi(ORB_ID(sensor_baro), &brp,
					  &_orb_class_instance, _interface->is_external() ? ORB_PRIO_HIGH : ORB_PRIO_DEFAULT);

	if (_baro_topic == nullptr) {
		warnx("failed to create sensor_baro publication");
		return -ENOMEM;
	}

	return OK;

}

ssize_t
BMP280::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct baro_report);
	struct baro_report *brp = reinterpret_cast<struct baro_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_report_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(brp)) {
				ret += sizeof(*brp);
				brp++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */

	_reports->flush();

	if (measure()) {
		return -EIO;
	}

	usleep(TICK2USEC(_max_mesure_ticks));

	if (collect()) {
		return -EIO;
	}

	if (_reports->get(brp)) { //get new generated report
		ret = sizeof(*brp);
	}

	return ret;
}

int
BMP280::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {

			unsigned ticks = 0;

			switch (arg) {

			case SENSOR_POLLRATE_MANUAL:
				stop_cycle();
				_report_ticks = 0;
				return OK;

			case SENSOR_POLLRATE_EXTERNAL:
			case 0:
				return -EINVAL;

			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT:
				ticks = _max_mesure_ticks;

			default: {
					if (ticks == 0) {
						ticks = USEC2TICK(USEC_PER_SEC / arg);
					}

					/* do we need to start internal polling? */
					bool want_start = (_report_ticks == 0);

					/* check against maximum rate */
					if (ticks < _max_mesure_ticks) {
						return -EINVAL;
					}

					_report_ticks = ticks;

					if (want_start) {
						start_cycle();
					}

					return OK;
				}
			}

			break;
		}

	case SENSORIOCGPOLLRATE:
		if (_report_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (USEC_PER_SEC / USEC_PER_TICK / _report_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);
			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		/*
		 * Since we are initialized, we do not need to do anything, since the
		 * PROM is correctly read and the part does not need to be configured.
		 */
		return OK;

	case BAROIOCSMSLPRESSURE:

		/* range-check for sanity */
		if ((arg < 80000) || (arg > 120000)) {
			return -EINVAL;
		}

		_msl_pressure = arg;
		return OK;

	case BAROIOCGMSLPRESSURE:
		return _msl_pressure;

	default:
		break;
	}

	return CDev::ioctl(filp, cmd, arg);
}

void
BMP280::start_cycle()
{

	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&BMP280::cycle_trampoline, this, 1);
}

void
BMP280::stop_cycle()
{
	work_cancel(HPWORK, &_work);
}

void
BMP280::cycle_trampoline(void *arg)
{
	BMP280 *dev = reinterpret_cast<BMP280 *>(arg);

	dev->cycle();
}

void
BMP280::cycle()
{
	if (_collect_phase) {
		collect();
		unsigned wait_gap = _report_ticks - _max_mesure_ticks;

		if (wait_gap != 0) {
			work_queue(HPWORK, &_work, (worker_t)&BMP280::cycle_trampoline, this,
				   wait_gap); //need to wait some time before new measurement
			return;
		}

	}

	measure();
	work_queue(HPWORK, &_work, (worker_t)&BMP280::cycle_trampoline, this, _max_mesure_ticks);

}

int
BMP280::measure()
{
	_collect_phase = true;

	perf_begin(_measure_perf);

	/* start measure */
	int ret = _interface->set_reg(_curr_ctrl | BPM280_CTRL_MODE_FORCE, BPM280_ADDR_CTRL);

	if (ret != OK) {
		perf_count(_comms_errors);
		perf_cancel(_measure_perf);
		return -EIO;
	}

	perf_end(_measure_perf);

	return OK;
}

int
BMP280::collect()
{
	_collect_phase = false;

	perf_begin(_sample_perf);

	struct baro_report report;
	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);

	bmp280::data_s *data = _interface->get_data(BPM280_ADDR_DATA);

	if (data == nullptr) {
		perf_count(_comms_errors);
		perf_cancel(_sample_perf);
		return -EIO;
	}

	//convert data to number 20 bit
	uint32_t p_raw =  data->p_msb << 12 | data->p_lsb << 4 | data->p_xlsb >> 4;
	uint32_t t_raw =  data->t_msb << 12 | data->t_lsb << 4 | data->t_xlsb >> 4;

	// Temperature
	float ofs = (float) t_raw - _fcal.t1;
	float t_fine = (ofs * _fcal.t3 + _fcal.t2) * ofs;
	_T = t_fine * (1.0f / 5120.0f);

	// Pressure
	float tf = t_fine - 128000.0f;
	float x1 = (tf * _fcal.p6 + _fcal.p5) * tf + _fcal.p4;
	float x2 = (tf * _fcal.p3 + _fcal.p2) * tf + _fcal.p1;

	float pf = ((float) p_raw + x1) / x2;
	_P = (pf * _fcal.p9 + _fcal.p8) * pf + _fcal.p7;


	report.temperature = _T;
	report.pressure = _P / 100.0f; // to mbar

	/* Get device ID */
	report.device_id = _device_id.devid;

	/* altitude calculations based on http://www.kansasflyer.org/index.asp?nav=Avi&sec=Alti&tab=Theory&pg=1 */

	/* tropospheric properties (0-11km) for standard atmosphere */
	const float T1 = 15.0f + 273.15f;	/* temperature at base height in Kelvin */
	const float a  = -6.5f / 1000.0f;	/* temperature gradient in degrees per metre */
	const float g  = 9.80665f;	/* gravity constant in m/s/s */
	const float R  = 287.05f;	/* ideal gas constant in J/kg/K */
	float pK = _P / _msl_pressure;

	/*
	 * Solve:
	 *
	 *     /        -(aR / g)     \
	 *    | (p / p1)          . T1 | - T1
	 *     \                      /
	 * h = -------------------------------  + h1
	 *                   a
	 */
	report.altitude = (((powf(pK, (-(a * R) / g))) * T1) - T1) / a;


	/* publish it */
	if (!(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_baro), _baro_topic, &report);
	}

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	perf_end(_sample_perf);

	return OK;
}

void
BMP280::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u us \n", _report_ticks * USEC_PER_TICK);
	_reports->print_info("report queue");
	printf("P Pa:              %.3f\n", (double)_P);
	printf("T:              %.3f\n", (double)_T);
	printf("MSL pressure Pa:   %u\n", _msl_pressure);

}

/**
 * Local functions in support of the shell command.
 */
namespace bmp280
{

/*
  list of supported bus configurations
 */
struct bmp280_bus_option {
	enum BMP280_BUS busid;
	const char *devpath;
	BMP280_constructor interface_constructor;
	uint8_t busnum;
	uint8_t device;
	bool external;
	BMP280 *dev;
} bus_options[] = {
#if defined(PX4_SPIDEV_EXT_BARO) && defined(PX4_SPI_BUS_EXT)
	{ BMP280_BUS_SPI_EXTERNAL, "/dev/bmp280_spi_ext", &bmp280_spi_interface, PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_BARO, true, NULL },
#endif
#ifdef PX4_SPIDEV_BARO
	{ BMP280_BUS_SPI_INTERNAL, "/dev/bmp280_spi_int", &bmp280_spi_interface, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_BARO, false, NULL },
#endif
#ifdef PX4_I2C_OBDEV_BMP280
	{ BMP280_BUS_I2C_INTERNAL, "/dev/bmp280_i2c_int", nullptr, PX4_I2C_BUS_ONBOARD, PX4_I2C_OBDEV_BMP280, false, NULL },
#endif
#if defined(PX4_I2C_BUS_EXPANSION) && defined(PX4_I2C_EXT_OBDEV_BMP280)
	{ BMP280_BUS_I2C_EXTERNAL, "/dev/bmp280_i2c_ext", nullptr, PX4_I2C_BUS_EXPANSION, PX4_I2C_EXT_OBDEV_BMP280, true, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

bool	start_bus(struct bmp280_bus_option &bus);
struct bmp280_bus_option &find_bus(enum BMP280_BUS busid);
void	start(enum BMP280_BUS busid);
void	test(enum BMP280_BUS busid);
void	reset(enum BMP280_BUS busid);
void	info();
void	calibrate(unsigned altitude, enum BMP280_BUS busid);
void	usage();


/**
 * Start the driver.
 */
bool
start_bus(struct bmp280_bus_option &bus)
{
	if (bus.dev != nullptr) {
		errx(1, "bus option already started");
	}

	bmp280::IBMP280 *interface = bus.interface_constructor(bus.busnum, bus.device, bus.external);

	if (interface->init() != OK) {
		delete interface;
		warnx("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new BMP280(interface, bus.devpath);

	if (bus.dev != nullptr && OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = NULL;
		return false;
	}

	int fd = open(bus.devpath, O_RDONLY);

	/* set the poll rate to default, starts automatic data collection */
	if (fd == -1) {
		errx(1, "can't open baro device");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		close(fd);
		errx(1, "failed setting default poll rate");
	}

	close(fd);
	return true;
}


/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
void
start(enum BMP280_BUS busid)
{
	uint8_t i;
	bool started = false;

	for (i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == BMP280_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != BMP280_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i]);
	}

	if (!started) {
		errx(1, "driver start failed");
	}

	// one or more drivers started OK
	exit(0);
}


/**
 * find a bus structure for a busid
 */
struct bmp280_bus_option &find_bus(enum BMP280_BUS busid)
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == BMP280_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	errx(1, "bus %u not started", (unsigned)busid);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(enum BMP280_BUS busid)
{
	struct bmp280_bus_option &bus = find_bus(busid);
	struct baro_report report;
	ssize_t sz;
	int ret;

	int fd;

	fd = open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		err(1, "open failed (try 'bmp280 start' if the driver is not running)");
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("pressure:    %10.4f", (double)report.pressure);
	warnx("altitude:    %11.4f", (double)report.altitude);
	warnx("temperature: %8.4f", (double)report.temperature);
	warnx("time:        %lld", report.timestamp);

	/* set the queue depth to 10 */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10)) {
		errx(1, "failed to set queue depth");
	}

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		warnx("periodic read %u", i);
		warnx("pressure:    %10.4f", (double)report.pressure);
		warnx("altitude:    %11.4f", (double)report.altitude);
		warnx("temperature K: %8.4f", (double)report.temperature);
		warnx("time:        %lld", report.timestamp);
	}

	close(fd);
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset(enum BMP280_BUS busid)
{
	struct bmp280_bus_option &bus = find_bus(busid);
	int fd;

	fd = open(bus.devpath, O_RDONLY);

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
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		struct bmp280_bus_option &bus = bus_options[i];

		if (bus.dev != nullptr) {
			warnx("%s", bus.devpath);
			bus.dev->print_info();
		}
	}

	exit(0);
}

/**
 * Calculate actual MSL pressure given current altitude
 */
void
calibrate(unsigned altitude, enum BMP280_BUS busid)
{
	struct bmp280_bus_option &bus = find_bus(busid);
	struct baro_report report;
	float	pressure;
	float	p1;

	int fd;

	fd = open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		err(1, "open failed (try 'bmp280 start' if the driver is not running)");
	}

	/* start the sensor polling at max */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MAX)) {
		errx(1, "failed to set poll rate");
	}

	/* average a few measurements */
	pressure = 0.0f;

	for (unsigned i = 0; i < 20; i++) {
		struct pollfd fds;
		int ret;
		ssize_t sz;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 1000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "sensor read failed");
		}

		pressure += report.pressure;
	}

	pressure /= 20;		/* average */
	pressure /= 10;		/* scale from millibar to kPa */

	/* tropospheric properties (0-11km) for standard atmosphere */
	const float T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
	const float a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
	const float g  = 9.80665f;	/* gravity constant in m/s/s */
	const float R  = 287.05f;	/* ideal gas constant in J/kg/K */

	warnx("averaged pressure %10.4fkPa at %um", (double)pressure, altitude);

	p1 = pressure * (powf(((T1 + (a * (float)altitude)) / T1), (g / (a * R))));

	warnx("calculated MSL pressure %10.4fkPa", (double)p1);

	/* save as integer Pa */
	p1 *= 1000.0f;

	if (ioctl(fd, BAROIOCSMSLPRESSURE, (unsigned long)p1) != OK) {
		err(1, "BAROIOCSMSLPRESSURE");
	}

	close(fd);
	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'test2', 'reset', 'calibrate'");
	warnx("options:");
	warnx("    -X    (external I2C bus TODO)");
	warnx("    -I    (internal I2C bus TODO)");
	warnx("    -S    (external SPI bus)");
	warnx("    -s    (internal SPI bus)");
}

} // namespace

int
bmp280_main(int argc, char *argv[])
{
	enum BMP280_BUS busid = BMP280_BUS_ALL;
	int ch;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "XISs")) != EOF) {
		switch (ch) {
		case 'X':
			busid = BMP280_BUS_I2C_EXTERNAL;
			errx(1, "not supported yet");
			break;

		case 'I':
			busid = BMP280_BUS_I2C_INTERNAL;
			errx(1, "not supported yet");
			break;

		case 'S':
			busid = BMP280_BUS_SPI_EXTERNAL;
			break;

		case 's':
			busid = BMP280_BUS_SPI_INTERNAL;
			break;

		default:
			bmp280::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		bmp280::start(busid);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		bmp280::test(busid);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		bmp280::reset(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		bmp280::info();
	}

	/*
	 * Perform MSL pressure calibration given an altitude in metres
	 */
	if (!strcmp(verb, "calibrate")) {
		if (argc < 2) {
			errx(1, "missing altitude");
		}

		long altitude = strtol(argv[optind + 1], nullptr, 10);

		bmp280::calibrate(altitude, busid);
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
