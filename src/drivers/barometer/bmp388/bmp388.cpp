/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file bmp388.cpp
 * Driver for the BMP388 barometric pressure sensor connected via I2C TODO or SPI.
 */

#include "bmp388.h"

#include <lib/drivers/barometer/PX4Barometer.hpp>

enum BMP388_BUS {
	BMP388_BUS_ALL = 0,
	BMP388_BUS_I2C_INTERNAL,
	BMP388_BUS_I2C_EXTERNAL,
	BMP388_BUS_SPI_INTERNAL,
	BMP388_BUS_SPI_EXTERNAL
};

/*
 * BMP388 internal constants and data structures.
 */

class BMP388 : public cdev::CDev, public px4::ScheduledWorkItem
{
public:
	BMP388(bmp388::IBMP388 *interface, const char *path);
	virtual ~BMP388();

	virtual int		init();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

private:
	PX4Barometer		_px4_baro;

	bmp388::IBMP388		*_interface;

	bool               	_running{false};

	uint8_t				_curr_ctrl{0};

	unsigned			_report_interval{0}; // 0 - no cycling, otherwise period of sending a report
	unsigned			_max_measure_interval{0}; // interval in microseconds needed to measure


	bool			_collect_phase{false};

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;

	struct bmp388::calibration_s *_cal; // stored calibration constants
	struct bmp388::fcalibration_s _fcal; // pre processed calibration constants

	float			_P; /* in Pa */
	float			_T; /* in K */


	/* periodic execution helpers */
	void			start_cycle();
	void			stop_cycle();

	void			Run() override;

	int		measure(); //start measure
	int		collect(); //get results and publish
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int bmp388_main(int argc, char *argv[]);

BMP388::BMP388(bmp388::IBMP388 *interface, const char *path) :
	CDev(path),
	ScheduledWorkItem(px4::device_bus_to_wq(interface->get_device_id())),
	_px4_baro(interface->get_device_id()),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmp388: read")),
	_measure_perf(perf_alloc(PC_ELAPSED, "bmp388: measure")),
	_comms_errors(perf_alloc(PC_COUNT, "bmp388: comms errors"))
{
	_px4_baro.set_device_type(DRV_BARO_DEVTYPE_BMP388);
}

BMP388::~BMP388()
{
	// make sure we are truly inactive
	stop_cycle();

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
BMP388::init()
{
	int ret = CDev::init();

	if (ret != OK) {
		PX4_ERR("CDev init failed");
		return ret;
	}

	/* reset sensor */
	_interface->set_reg(BMP388_VALUE_RESET, BMP388_ADDR_RESET);
	usleep(10000);

	/* check  id*/
	if (_interface->get_reg(BMP388_ADDR_ID) != BMP388_VALUE_ID) {
		PX4_WARN("id of your baro is not: 0x%02x", BMP388_VALUE_ID);
		return -EIO;
	}

	/* set config, recommended settings */
	_curr_ctrl = BMP388_CTRL_P16 | BMP388_CTRL_T2;
	_interface->set_reg(_curr_ctrl, BMP388_ADDR_CTRL);
	_max_measure_interval = (BMP388_MT_INIT + BMP388_MT * (16 - 1 + 2 - 1));
	_interface->set_reg(BMP388_CONFIG_F16, BMP388_ADDR_CONFIG);

	/* get calibration and pre process them*/
	_cal = _interface->get_calibration(BMP388_ADDR_CAL);

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

	return OK;
}

void
BMP388::start_cycle()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_running = true;

	/* schedule a cycle to start things */
	ScheduleNow();
}

void
BMP388::stop_cycle()
{
	_running = false;

	ScheduleClear();
}

void
BMP388::Run()
{
	if (_collect_phase) {
		collect();
		unsigned wait_gap = _report_interval - _max_measure_interval;

		if ((wait_gap != 0) && (_running)) {
			//need to wait some time before new measurement
			ScheduleDelayed(wait_gap);

			return;
		}

	}

	measure();

	if (_running) {
		ScheduleDelayed(_max_measure_interval);
	}
}

int
BMP388::measure()
{
	_collect_phase = true;

	perf_begin(_measure_perf);

	/* start measure */
	int ret = _interface->set_reg(_curr_ctrl | BMP388_CTRL_MODE_FORCE, BMP388_ADDR_CTRL);

	if (ret != OK) {
		perf_count(_comms_errors);
		perf_cancel(_measure_perf);
		return -EIO;
	}

	perf_end(_measure_perf);

	return OK;
}

int
BMP388::collect()
{
	_collect_phase = false;

	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	_px4_baro.set_error_count(perf_event_count(_comms_errors));

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	bmp388::data_s *data = _interface->get_data(BMP388_ADDR_DATA);

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

	_px4_baro.set_temperature(_T);
	_px4_baro.update(timestamp_sample, _P / 100.0f); // to mbar

	perf_end(_sample_perf);

	return OK;
}

void
BMP388::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u us \n", _report_interval);

	_px4_baro.print_status();
}

/**
 * Local functions in support of the shell command.
 */
namespace bmp388
{

/*
  list of supported bus configurations
 */
struct bmp388_bus_option {
	enum BMP388_BUS busid;
	const char *devpath;
	BMP388_constructor interface_constructor;
	uint8_t busnum;
	uint32_t device;
	bool external;
	BMP388 *dev;
} bus_options[] = {
#if defined(PX4_SPIDEV_EXT_BARO) && defined(PX4_SPI_BUS_EXT)
	{ BMP388_BUS_SPI_EXTERNAL, "/dev/bmp388_spi_ext", &bmp388_spi_interface, PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_BARO, true, NULL },
#endif
#if defined(PX4_SPIDEV_BARO)
#  if defined(PX4_SPIDEV_BARO_BUS)
	{ BMP388_BUS_SPI_INTERNAL, "/dev/bmp388_spi_int", &bmp388_spi_interface, PX4_SPIDEV_BARO_BUS, PX4_SPIDEV_BARO, false, NULL },
#  else
	{ BMP388_BUS_SPI_INTERNAL, "/dev/bmp388_spi_int", &bmp388_spi_interface, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_BARO, false, NULL },
#  endif
#endif
#if defined(PX4_I2C_BUS_ONBOARD) && defined(PX4_I2C_OBDEV_BMP388)
	{ BMP388_BUS_I2C_INTERNAL, "/dev/bmp388_i2c_int", &bmp388_i2c_interface, PX4_I2C_BUS_ONBOARD, PX4_I2C_OBDEV_BMP388, false, NULL },
#endif
#if defined(PX4_I2C_BUS_EXPANSION) && defined(PX4_I2C_OBDEV_BMP388)
	{ BMP388_BUS_I2C_EXTERNAL, "/dev/bmp388_i2c_ext", &bmp388_i2c_interface, PX4_I2C_BUS_EXPANSION, PX4_I2C_OBDEV_BMP388, true, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

bool	start_bus(struct bmp388_bus_option &bus);
struct bmp388_bus_option &find_bus(enum BMP388_BUS busid);
void	start(enum BMP388_BUS busid);
void	test(enum BMP388_BUS busid);
void	reset(enum BMP388_BUS busid);
void	info();
void	usage();


/**
 * Start the driver.
 */
bool
start_bus(struct bmp388_bus_option &bus)
{
	if (bus.dev != nullptr) {
		PX4_ERR("bus option already started");
		exit(1);
	}

	bmp388::IBMP388 *interface = bus.interface_constructor(bus.busnum, bus.device, bus.external);

	if (interface->init() != OK) {
		delete interface;
		PX4_WARN("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new BMP388(interface, bus.devpath);

	if (bus.dev == nullptr) {
		return false;
	}

	if (OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = nullptr;
		return false;
	}

	int fd = open(bus.devpath, O_RDONLY);

	/* set the poll rate to default, starts automatic data collection */
	if (fd == -1) {
		PX4_ERR("can't open baro device");
		exit(1);
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		close(fd);
		PX4_ERR("failed setting default poll rate");
		exit(1);
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
start(enum BMP388_BUS busid)
{
	uint8_t i;
	bool started = false;

	for (i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == BMP388_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != BMP388_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i]);
	}

	if (!started) {
		PX4_WARN("bus option number is %d", i);
		PX4_ERR("driver start failed");
		exit(1);
	}

	// one or more drivers started OK
	exit(0);
}


/**
 * find a bus structure for a busid
 */
struct bmp388_bus_option &find_bus(enum BMP388_BUS busid)
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == BMP388_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	PX4_ERR("bus %u not started", (unsigned)busid);
	exit(1);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		struct bmp388_bus_option &bus = bus_options[i];

		if (bus.dev != nullptr) {
			PX4_WARN("%s", bus.devpath);
			bus.dev->print_info();
		}
	}

	exit(0);
}

void
usage()
{
	PX4_WARN("missing command: try 'start', 'info'");
	PX4_WARN("options:");
	PX4_WARN("    -X    (external I2C bus TODO)");
	PX4_WARN("    -I    (internal I2C bus TODO)");
	PX4_WARN("    -S    (external SPI bus)");
	PX4_WARN("    -s    (internal SPI bus)");
}

} // namespace

int
bmp388_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	enum BMP388_BUS busid = BMP388_BUS_ALL;

	while ((ch = px4_getopt(argc, argv, "XISs", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = BMP388_BUS_I2C_EXTERNAL;
			break;

		case 'I':
			busid = BMP388_BUS_I2C_INTERNAL;
			break;

		case 'S':
			busid = BMP388_BUS_SPI_EXTERNAL;
			break;

		case 's':
			busid = BMP388_BUS_SPI_INTERNAL;
			break;

		default:
			bmp388::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		bmp388::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		bmp388::start(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		bmp388::info();
	}

	PX4_ERR("unrecognized command, try 'start', 'test', 'reset' or 'info'");
	return -1;
}
