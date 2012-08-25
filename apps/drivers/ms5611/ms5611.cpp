/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file ms5611.cpp
 * Driver for the MS5611 barometric pressure sensor connected via I2C.
 */

#include <nuttx/config.h>

#include <drivers/device/i2c.h>

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

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/up_hrt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_baro.h>

/**
 * Calibration PROM as reported by the device.
 */
#pragma pack(push,1)
struct ms5611_prom_s {
	uint16_t factory_setup;
	uint16_t c1_pressure_sens;
	uint16_t c2_pressure_offset;
	uint16_t c3_temp_coeff_pres_sens;
	uint16_t c4_temp_coeff_pres_offset;
	uint16_t c5_reference_temp;
	uint16_t c6_temp_coeff_temp;
	uint16_t serial_and_crc;
};

/**
 * Grody hack for crc4()
 */
union ms5611_prom_u {
	uint16_t c[8];
	struct ms5611_prom_s s;
};
#pragma pack(pop)

class MS5611 : public device::I2C
{
public:
	MS5611(int bus);
	~MS5611();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int		open_first(struct file *filp);
	virtual int		close_last(struct file *filp);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	virtual int		probe();

private:
	union ms5611_prom_u	_prom;

	struct work_s		_work;
	unsigned		_measure_ticks;

	unsigned		_num_reports;
	volatile unsigned	_next_report;
	volatile unsigned	_oldest_report;
	struct baro_report	*_reports;

	bool			_collect_phase;
	unsigned		_measure_phase;

	int32_t			_dT;
	int64_t			_temp64;

	orb_advert_t		_baro_topic;

	unsigned		_reads;
	unsigned		_measure_errors;
	unsigned		_read_errors;
	unsigned		_buf_overflows;

	perf_counter_t		_sample_perf;

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 *
	 * @param address	The I2C bus address to probe.
	 * @return		True if the device is present.
	 */
	int			probe_address(uint8_t address);

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * Issue a measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int			collect();

	/**
	 * Read the MS5611 PROM
	 *
	 * @return		OK if the PROM reads successfully.
	 */
	int			read_prom();

	/**
	 * PROM CRC routine ported from MS5611 application note
	 *
	 * @param n_prom	Pointer to words read from PROM.
	 * @return		True if the CRC matches.
	 */
	bool			crc4(uint16_t *n_prom);

};

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { _x++; if (_x >= _lim) _x = 0; } while(0)

/*
 * MS5611 internal constants and data structures.
 */

/* internal conversion time: 9.17 ms, so should not be read at rates higher than 100 Hz */
#define MS5611_CONVERSION_INTERVAL	10000	/* microseconds */
#define MS5611_MEASUREMENT_RATIO	3	/* pressure measurements per temperature measurement */

#define MS5611_ADDRESS_1		0x76    /* address select pins pulled high (PX4FMU series v1.6+) */
#define MS5611_ADDRESS_2		0x77    /* address select pins pulled low (PX4FMU prototypes) */

#define ADDR_RESET_CMD			0x1E /* read from this address to reset chip (0b0011110 on bus) */
#define ADDR_CMD_CONVERT_D1		0x48 /* 4096 samples to this address to start conversion (0b01001000 on bus) */
#define ADDR_CMD_CONVERT_D2		0x58 /* 4096 samples */
#define ADDR_DATA				0x00 /* address of 3 bytes / 32bit pressure data */
#define ADDR_PROM_SETUP			0xA0 /* address of 8x 2 bytes factory and calibration data */
#define ADDR_PROM_C1			0xA2 /* address of 6x 2 bytes calibration data */

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ms5611_main(int argc, char *argv[]);


MS5611::MS5611(int bus) :
	I2C("MS5611", BARO_DEVICE_PATH, bus, 0, 400000),
	_measure_ticks(0),
	_num_reports(0),
	_next_report(0),
	_oldest_report(0),
	_reports(nullptr),
	_collect_phase(false),
	_measure_phase(0),
	_dT(0),
	_temp64(0),
	_baro_topic(-1),
	_reads(0),
	_measure_errors(0),
	_read_errors(0),
	_buf_overflows(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "ms5611_read"))
{
	// enable debug() calls
	_debug_enabled = true;

	// work_cancel in the dtor will explode if we don't do this...
	_work.worker = nullptr;
}

MS5611::~MS5611()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
		delete[] _reports;
}

int
MS5611::init()
{
	int ret;

	/* do I2C init (and probe) first */
	ret = I2C::init();

	return ret;
}

int
MS5611::open_first(struct file *filp)
{
	/* reset to manual-poll mode */
	_measure_ticks = 0;

	/* allocate basic report buffers */
	_num_reports = 2;
	_reports = new struct baro_report[_num_reports];
	_oldest_report = _next_report = 0;

	return OK;
}

int
MS5611::close_last(struct file *filp)
{
	/* stop measurement */
	stop();

	/* free report buffers */
	if (_reports != nullptr) {
		delete[] _reports;
		_num_reports = 0;
	}

	_measure_ticks = 0;

	return OK;
}

int
MS5611::probe()
{
	if (OK == probe_address(MS5611_ADDRESS_1))
		return OK;

	if (OK == probe_address(MS5611_ADDRESS_2))
		return OK;

	return -EIO;
}

int
MS5611::probe_address(uint8_t address)
{
	uint8_t cmd = ADDR_RESET_CMD;

	/* select the address we are going to try */
	set_address(address);

	/* send reset command */
	if (OK != transfer(&cmd, 1, nullptr, 0))
		return -EIO;

	/* wait for PROM contents to be in the device (2.8 ms) */
	usleep(3000);

	/* read PROM */
	if (OK != read_prom())
		return -EIO;

	return OK;
}

ssize_t
MS5611::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct baro_report);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_oldest_report != _next_report) {
				memcpy(buffer, _reports + _oldest_report, sizeof(*_reports));
				ret += sizeof(_reports[0]);
				INCREMENT(_oldest_report, _num_reports);
			}
		}

		_reads++;

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	/* XXX really it'd be nice to lock against other readers here */
	do {
		_measure_phase = 0;
		_oldest_report = _next_report = 0;

		/* do temperature first */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		usleep(MS5611_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* now do a pressure measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		usleep(MS5611_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		memcpy(buffer, _reports, sizeof(*_reports));
		ret = sizeof(*_reports);
		_reads++;

	} while (0);

	return ret;
}

int
MS5611::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

				/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

				/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

				/* zero would be bad */
			case 0:
				return -EINVAL;

				/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(MS5611_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}

				/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(MS5611_CONVERSION_INTERVAL))
						return -EINVAL;

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0)
			return SENSOR_POLLRATE_MANUAL;
		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* add one to account for the sentinel in the ring */
			arg++;

			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 2) || (arg > 100))
				return -EINVAL;

			/* allocate new buffer */
			struct baro_report *buf = new struct baro_report[arg];

			if (nullptr == buf)
				return -ENOMEM;

			/* reset the measurement state machine with the new buffer, free the old */
			stop();
			delete[] _reports;
			_num_reports = arg;
			_reports = buf;
			start();

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _num_reports - 1;

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

void
MS5611::start()
{
	/* make sure we are stopped first */
	stop();

	/* reset the report ring and state machine */
	_collect_phase = false;
	_measure_phase = 0;
	_oldest_report = _next_report = 0;

	/* schedule a cycle to start things */
	work_queue(&_work, (worker_t)&MS5611::cycle_trampoline, this, 1);
}

void
MS5611::stop()
{
	work_cancel(&_work);
}

void
MS5611::cycle_trampoline(void *arg)
{
	MS5611 *dev = (MS5611 *)arg;

	dev->cycle();
}

void
MS5611::cycle()
{
	/*
	 * We have to publish the baro topic in the context of the workq
	 * in order to ensure that the descriptor is valid when we go to publish.
	 *
	 * @bug	We can't really ever be torn down and restarted, since this
	 *      descriptor will never be closed and on the restart we will be
	 *      unable to re-advertise.
	 */
	if (_baro_topic == -1) {
		struct baro_report b;

		/* if this fails (e.g. no object in the system) we will cope */
		memset(&b, 0, sizeof(b));
		_baro_topic = orb_advertise(ORB_ID(sensor_baro), &b);

		if (_baro_topic < 0)
			debug("failed to create sensor_baro object");
	}

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			log("FATAL collection error - restarting\n");
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 * Don't inject one after temperature measurements, so we can keep
		 * doing pressure measurements at something close to the desired rate.
		 */
		if ((_measure_phase != 0) &&
		    (_measure_ticks > USEC2TICK(MS5611_CONVERSION_INTERVAL))) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(&_work,
				   (worker_t)&MS5611::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(MS5611_CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		log("FATAL measure error - restarting\n");
		start();
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(&_work,
		   (worker_t)&MS5611::cycle_trampoline,
		   this,
		   USEC2TICK(MS5611_CONVERSION_INTERVAL));
}

int
MS5611::measure()
{
	int ret;

	/*
	 * In phase zero, request temperature; in other phases, request pressure.
	 */
	uint8_t	cmd_data = (_measure_phase == 0) ? ADDR_CMD_CONVERT_D2 : ADDR_CMD_CONVERT_D1;

	/*
	 * Send the command to begin measuring.
	 */
	ret = transfer(&cmd_data, 1, nullptr, 0);

	if (OK != ret)
		_measure_errors++;

	return ret;
}

int
MS5611::collect()
{
	uint8_t cmd;
	uint8_t data[3];

	/* read the most recent measurement */
	cmd = 0;

	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	_reports[_next_report].timestamp = hrt_absolute_time();

	if (OK != transfer(&cmd, 1, &data[0], 3)) {
		_read_errors++;
		return -EIO;
	}

	/* fetch the raw value */
	uint32_t raw = (((uint32_t)data[0]) << 16) | (((uint32_t)data[1]) << 8) | ((uint32_t)data[2]);

	/* handle a measurement */
	if (_measure_phase == 0) {

		/* temperature calculation */
		_dT = raw - (((int32_t)_prom.s.c5_reference_temp) * 256);
		_temp64 = 2000 + (((int64_t)_dT) * _prom.s.c6_temp_coeff_temp) / 8388608;

	} else {

		/* pressure calculation */
		int64_t offset = (int64_t)_prom.s.c2_pressure_offset * 65536 + ((int64_t)_dT * _prom.s.c4_temp_coeff_pres_offset) / 128;
		int64_t sens = (int64_t)_prom.s.c1_pressure_sens * 32768 + ((int64_t)_dT * _prom.s.c3_temp_coeff_pres_sens) / 256;

		/* it's pretty cold, second order temperature compensation needed */
		if (_temp64 < 2000) {
			/* second order temperature compensation */
			int64_t temp2 = (((int64_t)_dT) * _dT) >> 31;
			int64_t tmp_64 = (_temp64 - 2000) * (_temp64 - 2000);
			int64_t offset2 = (5 * tmp_64) >> 1;
			int64_t sens2 = (5 * tmp_64) >> 2;
			_temp64 = _temp64 - temp2;
			offset = offset - offset2;
			sens = sens - sens2;
		}

		int64_t press_int64 = (((raw * sens) / 2097152 - offset) / 32768);

		/* generate a new report */
		_reports[_next_report].temperature = _temp64 / 100.0f;
		_reports[_next_report].pressure = press_int64 / 100.0f;
		/* convert as double for max. precision, store as float (more than enough precision) */
		_reports[_next_report].altitude = (44330.0 * (1.0 - pow((press_int64 / 101325.0), 0.190295)));

		/* publish it */
		orb_publish(ORB_ID(sensor_baro), _baro_topic, &_reports[_next_report]);

		/* post a report to the ring - note, not locked */
		INCREMENT(_next_report, _num_reports);

		/* if we are running up against the oldest report, toss it */
		if (_next_report == _oldest_report) {
			_buf_overflows++;
			INCREMENT(_oldest_report, _num_reports);
		}

		/* notify anyone waiting for data */
		poll_notify(POLLIN);
	}

	/* update the measurement state machine */
	INCREMENT(_measure_phase, MS5611_MEASUREMENT_RATIO + 1);

	perf_end(_sample_perf);

	return OK;
}

int
MS5611::read_prom()
{
	/* read PROM data */
	uint8_t prom_buf[2] = {255, 255};

	for (int i = 0; i < 8; i++) {
		uint8_t cmd = ADDR_PROM_SETUP + (i * 2);

		if (OK != transfer(&cmd, 1, &prom_buf[0], 2))
			break;

		/* assemble 16 bit value and convert from big endian (sensor) to little endian (MCU) */
		_prom.c[i] = (((uint16_t)prom_buf[0]) << 8) | ((uint16_t)prom_buf[1]);

	}

	/* calculate CRC and return false */
	return crc4(&_prom.c[0]) ? OK : -EIO;
}

bool
MS5611::crc4(uint16_t *n_prom)
{
	int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;

	/* save the read crc */
	crc_read = n_prom[7];

	/* remove CRC byte */
	n_prom[7] = (0xFF00 & (n_prom[7]));

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

		} else {
			n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;

			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	/* final 4 bit remainder is CRC value */
	n_rem = (0x000F & (n_rem >> 12));
	n_prom[7] = crc_read;

	/* return true if CRCs match */
	return (0x000F & crc_read) == (n_rem ^ 0x00);
}

void
MS5611::print_info()
{
	printf("reads:          %u\n", _reads);
	printf("measure errors: %u\n", _measure_errors);
	printf("read errors:    %u\n", _read_errors);
	printf("read overflows: %u\n", _buf_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	printf("report queue:   %u (%u/%u @ %p)\n",
	       _num_reports, _oldest_report, _next_report, _reports);
	printf("dT/temp64:      %d/%lld\n", _dT, _temp64);
}

/**
 * Local functions in support of the shell command.
 */
namespace ms5611
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

MS5611	*g_dev;

void	start();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	/* XXX HORRIBLE hack - the bus number should not come from here */
	g_dev = new MS5611(2);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(BARO_DEVICE_PATH, O_RDONLY);
	if (fd < 0)
		goto fail;
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		goto fail;
	exit(0);

fail:
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}
	errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	struct baro_report report;
	ssize_t sz;
	int ret;

	int fd = open(BARO_DEVICE_PATH, O_RDONLY);
	if (fd < 0)
		err(1, "%s open failed (try 'ms5611 start' if the driver is not running", BARO_DEVICE_PATH);

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));
	if (sz != sizeof(report))
		err(1, "immediate read failed");

	warnx("single read");
	warnx("pressure:    %u", (unsigned)report.pressure);
	warnx("altitude:    %u", (unsigned)report.altitude);
	warnx("temperature: %u", (unsigned)report.temperature);
	warnx("time:        %lld", report.timestamp);

	/* set the queue depth to 10 */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10))
		errx(1, "failed to set queue depth");

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2))
		errx(1, "failed to set 2Hz poll rate");

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1)
			errx(1, "timed out waiting for sensor data");

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report))
			err(1, "periodic read failed");

		warnx("periodic read %u", i);
		warnx("pressure:    %u", (unsigned)report.pressure);
		warnx("altitude:    %u", (unsigned)report.altitude);
		warnx("temperature: %u", (unsigned)report.temperature);
		warnx("time:        %lld", report.timestamp);
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(BARO_DEVICE_PATH, O_RDONLY);
	if (fd < 0)
		err(1, "failed ");
	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "driver poll restart failed");

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}


} // namespace

int
ms5611_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
		ms5611::start();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		ms5611::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		ms5611::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info"))
		ms5611::info();

	errx(1, "unrecognised command, try 'start', 'test', 'reset' or 'info'");
}
