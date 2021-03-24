/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file si7210.cpp
 * Driver for the SI7210 connected via I2C.
 *
 * Author: Amir Melzer  <amir.melzer@mavt.ethz.ch>
 */

#include "si7210.h"

#include <px4_getopt.h>

/** driver 'main' command */
extern "C" { __EXPORT int si7210_main(int argc, char *argv[]); }

namespace si7210
{

SI7210 *g_dev_0;
SI7210 *g_dev_1;
SI7210 *g_dev_2;
SI7210 *g_dev_3;

typedef struct {
	float       mag_T;
	float       temp_C;
} si7210_measurements_t;

void start(bool, SI7210::Instance);
void test(bool, SI7210::Instance);
void reset(bool, SI7210::Instance);
void info(bool, SI7210::Instance);
void usage();
int get_i2c_address(SI7210::Instance);
const char *get_path(SI7210::Instance);
SI7210 **get_g_dev_ptr(SI7210::Instance);


/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void start(bool external_bus, SI7210::Instance instance)
{
	if (!external_bus) {
		PX4_ERR("Internal bus currently not supported");
		exit(1);
	}

	int fd;
	SI7210 **g_dev_ptr = get_g_dev_ptr(instance);
	const char *path = get_path(instance);

	if (*g_dev_ptr != nullptr) {
		/* if already started, the command still succeeded */
		PX4_ERR("driver instance %d at address 0x%x already started", instance, get_i2c_address(instance));
		exit(0);
	}

	/* create the driver */
	if (external_bus) {
#if defined(PX4_I2C_BUS_EXPANSION)
		*g_dev_ptr = new SI7210(PX4_I2C_BUS_EXPANSION, instance, path);
#else
		PX4_ERR("External I2C not available");
		exit(0);
#endif

	} else {
		PX4_ERR("Internal I2C not available");
		exit(0);
	}

	if (*g_dev_ptr == nullptr) {
		goto fail;
	}


	if (OK != (*g_dev_ptr)->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(path, O_RDONLY);


	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	close(fd);

	exit(0);
fail:

	if (*g_dev_ptr != nullptr) {
		delete (*g_dev_ptr);
		*g_dev_ptr = nullptr;
	}

	PX4_ERR("driver instance %d start at address 0x%x failed", instance, get_i2c_address(instance));
	exit(1);
}

void test(bool external_bus, SI7210::Instance instance)
{
	int fd = -1;
	const char *path = get_path(instance);

	struct si7210_report p_report;
	ssize_t sz;

	/* get the driver */
	fd = open(path, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'si7210 start -a %d' if the driver is not running)",
			path, instance);
		exit(1);
	}

	/* reset to Max polling rate*/
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MAX) < 0) {
		PX4_ERR("reset to Max polling rate");
		exit(1);
	}

	/* do a simple demand read */
	sz = read(fd, &p_report, sizeof(p_report));

	if (sz != sizeof(p_report)) {
		PX4_ERR("immediate si7210 read failed");
		exit(1);
	}

	PX4_INFO("single read");
	PX4_INFO("time:             %lld", p_report.timestamp);
	PX4_INFO("magnetic field: %10.4f", (double)p_report.mag_T);
	PX4_INFO("temperature:    %10.4f", (double)p_report.temp_C);

	PX4_INFO("PASS");
	exit(0);
}

void
reset(bool external_bus, SI7210::Instance instance)
{
	const char *path = get_path(instance);

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'si7210 start -a %d' if the driver is not running)",
			path, instance);
		exit(1);
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		exit(1);
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		exit(1);
	}

	exit(0);
}

void
info(bool external_bus, SI7210::Instance instance)
{
	SI7210 **g_dev_ptr = get_g_dev_ptr(instance);

	if (*g_dev_ptr == nullptr) {
		PX4_ERR("driver instance %d at address 0x%x not running", instance, get_i2c_address(instance));
		exit(1);
	}

	printf("state @ %p\n", *g_dev_ptr);
	(*g_dev_ptr)->print_info();

	exit(0);
}

void
usage()
{
	PX4_INFO("required:");
	PX4_INFO("    'start', 'info', 'test', 'stop', 'reset' ");
	PX4_INFO("    -i    (sensor instance (0-3))");
	PX4_INFO("optional:");
	PX4_INFO("    -X    (external bus)");
	PX4_INFO("example:");
	PX4_INFO("    si7210 start -i 0");
}

/* Get i2c address */
int
get_i2c_address(SI7210::Instance instance)
{
	int i2c_address(0);

	switch (instance) {
	case SI7210::Instance::ID_0:
		i2c_address = SI7210_SLAVE_ADDRESS_0;
		break;

	case SI7210::Instance::ID_1:
		i2c_address = SI7210_SLAVE_ADDRESS_1;
		break;

	case SI7210::Instance::ID_2:
		i2c_address = SI7210_SLAVE_ADDRESS_2;
		break;

	case SI7210::Instance::ID_3:
		i2c_address = SI7210_SLAVE_ADDRESS_3;
		break;

	default:
		PX4_ERR("Invalid instance, defaulting to i2c address 0x%x", SI7210_SLAVE_ADDRESS_0);
	}

	return i2c_address;
}

const char *
get_path(SI7210::Instance instance)
{
	switch (instance) {
	case SI7210::Instance::ID_0:
		return HALL0_DEVICE_PATH;

	case SI7210::Instance::ID_1:
		return HALL1_DEVICE_PATH;

	case SI7210::Instance::ID_2:
		return HALL2_DEVICE_PATH;

	case SI7210::Instance::ID_3:
		return HALL3_DEVICE_PATH;

	default:
		PX4_ERR("Invalid instance, defaulting to instance 0");
		return HALL0_DEVICE_PATH;
	}

}

SI7210 **
get_g_dev_ptr(SI7210::Instance instance)
{
	switch (instance) {
	case SI7210::Instance::ID_0:
		return &g_dev_0;

	case SI7210::Instance::ID_1:
		return &g_dev_1;

	case SI7210::Instance::ID_2:
		return &g_dev_2;

	case SI7210::Instance::ID_3:
		return &g_dev_3;

	default:
		PX4_ERR("Invalid instance, defaulting to instance 0");
		return &g_dev_0;
	}
}

} // namespace si7210

SI7210 :: SI7210(int bus, SI7210::Instance instance, const char *path) :
	I2C("SI7210", path, bus, si7210::get_i2c_address(instance), SI7210_BUS_SPEED),
	_running(false),
	_call_interval(0),
	_reports(nullptr),
	_collect_phase(false),
	_hall_topic(nullptr),
	_orb_class_instance(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "si7210_read")),
	_bad_transfers(perf_alloc(PC_COUNT, "si7210_bad_transfers")),
	_good_transfers(perf_alloc(PC_COUNT, "si7210_good_transfers")),
	_measure_perf(perf_alloc(PC_ELAPSED, "si7210_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "si7210_comms_errors")),
	_duplicates(perf_alloc(PC_COUNT, "si7210_duplicates")),
	_got_duplicate(false),
	_instance(instance),
	_params_sub(-1)
{
	_device_id.devid_s.devtype = DRV_HALL_DEVTYPE_SI7210;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));

	// default scaling
	initialize_parameter_handles(_parameter_handles);
}

SI7210 :: ~SI7210()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_hall_topic != nullptr) {
		orb_unadvertise(_hall_topic);
	}

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_bad_transfers);
	perf_free(_good_transfers);
	perf_free(_measure_perf);
	perf_free(_comms_errors);
	perf_free(_duplicates);
}

int SI7210::init()
{
	int ret = OK;

	/* do I2C init (and probe) first */
	ret = I2C::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("I2C setup failed");
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(si7210_report));

	if (_reports == nullptr) {
		return ret;
	}

	up_udelay(10000);

	if (collect()) {
		return -EIO;
	}

	/* advertise sensor topic, measure manually to initialize valid report */
	struct si7210_report prb;
	_reports->get(&prb);

	/* measurement will have generated a report, publish */
	_hall_topic = orb_advertise_multi(ORB_ID(sensor_hall), &prb, &_orb_class_instance, ORB_PRIO_DEFAULT);

	if (_hall_topic == nullptr) {
		PX4_WARN("ADVERT FAIL");
	}

	return ret;
}

int
SI7210::probe()
{
	uint8_t reg;

	if (OK != get_regs(HREVID, &reg)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (((reg & 0xf0) >> 4) != IDCHIPID) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if ((reg & 0x0f) != REVID) {
		perf_count(_comms_errors);
		return -EIO;
	}

	return OK;
}

void
SI7210::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_running = true;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&SI7210::cycle_trampoline, this, 1);
}

void
SI7210::stop()
{
	_running = false;
	work_cancel(HPWORK, &_work);
}

ssize_t
SI7210::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(si7210_report);
	struct si7210_report *si7210_buf = reinterpret_cast<struct si7210_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_call_interval > 0) {
		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(si7210_buf)) {
				ret += sizeof(struct si7210_report);
				si7210_buf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* wait for it to complete */
		usleep(SI7210_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}


		if (_reports->get(si7210_buf)) {
			ret = sizeof(struct si7210_report);
		}
	} while (0);

	/* return the number of bytes transferred */
	return ret;

}

void
SI7210::cycle_trampoline(void *arg)
{
	SI7210 *dev = reinterpret_cast<SI7210 *>(arg);

	/* make measurement */
	dev->cycle();
}

void
SI7210::cycle()
{
	bool force_update = false;
	bool param_updated = false;

	if (!_running) {
		if (_params_sub >= 0) {
			orb_unsubscribe(_params_sub);
		}

	} else {
		if (_params_sub < 0) {
			_params_sub = orb_subscribe(ORB_ID(parameter_update));
			force_update = true;
		}
	}

	/* Check if any parameter changed */
	orb_check(_params_sub, &param_updated);

	if (param_updated || force_update) {

		struct parameter_update_s update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &update);
		parameters_update();
	}

	if (_collect_phase) {
		collect();
		unsigned wait_gap = _call_interval - USEC2TICK(SI7210_CONVERSION_INTERVAL);

		if ((wait_gap != 0) && (_running)) {
			work_queue(HPWORK, &_work, (worker_t)&SI7210::cycle_trampoline, this,
				   wait_gap); //need to wait some time before new measurement
			return;
		}

	}

	measure();

	if ((_running)) {
		/* schedule a fresh cycle call when the measurement is done */
		work_queue(HPWORK,
			   &_work,
			   (worker_t)&SI7210::cycle_trampoline,
			   this,
			   USEC2TICK(SI7210_CONVERSION_INTERVAL));
	}
}

int
SI7210::parameters_update()
{
	int ret = update_parameters(_parameter_handles, _parameters);
	return ret;
}

int
SI7210::measure()
{
	_collect_phase = true;

	return OK;
}

int
SI7210::collect()
{
	_collect_phase = false;
	bool si7210_notify = true;
	si7210_report  prb;

	uint8_t     reg;

	uint16_t    magRaw;
	uint16_t    tempRaw;

	int8_t 		otpTempOffset;
	int8_t 		otpTempGain;

	/* read the most recent measurement */
	perf_begin(_sample_perf);

	/* capture the magnetic field measurements */
	reg = ARAUTOINC_ARAUTOINC_MASK;

	if (OK != set_regs(ARAUTOINC, reg)) {
		return -EIO;
	}

	reg = DSPSIGSEL_MAG_VAL_SEL;

	if (OK != set_regs(DSPSIGSEL, reg)) {
		return -EIO;
	}

	reg = POWER_CTRL_ONEBURST_MASK;

	if (OK != set_regs(POWER_CTRL, reg)) {
		return -EIO;
	}

	if (OK != get_measurement(DSPSIGM, &magRaw)) {
		return -EIO;
	}

	/* capture the temperate measurements */
	reg = DSPSIGSEL_TEMP_VAL_SEL;

	if (OK != set_regs(DSPSIGSEL, reg)) {
		return -EIO;
	}

	reg = POWER_CTRL_ONEBURST_MASK;

	if (OK != set_regs(POWER_CTRL, reg)) {
		return -EIO;
	}

	if (OK != get_measurement(DSPSIGM, &tempRaw)) {
		return -EIO;
	}

	if (OK != get_sensor_data(OTP_TEMP_OFFSET, &otpTempOffset)) {
		return -EIO;
	}

	if (OK != get_sensor_data(OTP_TEMP_GAIN, &otpTempGain)) {
		return -EIO;
	}

	float _tempOffset = (float)otpTempOffset / 16;
	float _tempGain = 1 + (float)otpTempGain / 2048;

	if (OK == ((magRaw & 0x8000) && (tempRaw & 0x8000))) {
		return -EIO;
	}

	/* generate a new report */
	prb.timestamp = hrt_absolute_time();
	prb.instance = _instance;
	prb.mag_T = (float)(magRaw - 0xC000) * 0.00125F;
	prb.temp_C = (float)((tempRaw & ~0x8000) >> 3);
	prb.temp_C = _tempGain * (-3.83e-6F * prb.temp_C * prb.temp_C + 0.16094F * prb.temp_C - 279.80F - 0.222F * 3.0F) +
		     _tempOffset;

	_reports->force(&prb);

	//PX4_INFO("<mag field, temp[degC]>: %.2f, %.2f", (double)prb.mag_T, (double)prb.temp_C);

	/* notify anyone waiting for data */
	if (si7210_notify) {
		poll_notify(POLLIN);
	}

	if (si7210_notify && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_hall), _hall_topic, &prb);
	}

	perf_end(_sample_perf);
	return OK;
}

int
SI7210::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_interval = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, SI7210_MAX_DATA_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(SI7210_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_call_interval = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_call_interval == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return 1000000 / _call_interval;

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

	case SENSORIOCRESET:
		return reset();

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

int SI7210::reset()
{
	//ToDo: write a reset function
	stop();
	return OK;
}

int
SI7210::self_test()
{
	if (perf_event_count(_sample_perf) == 0) {
		collect();
	}

	/* return 0 on success, 1 else */
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

/* Get registers value */
int
SI7210::get_regs(uint8_t ptr, uint8_t *regs)
{
	uint8_t data;

	if (OK != transfer(&ptr, 1, &data, 1)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	*regs  = data;

	return OK;
}

/* Set registers value */
int
SI7210::set_regs(uint8_t ptr, uint8_t value)
{
	uint8_t data[2];

	data[0] = ptr;
	data[1] = value;

	if (OK != transfer(&data[0], 2, nullptr, 0)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	/* read back the reg and verify */

	if (OK != transfer(&ptr, 1, &data[1], 1)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (data[1] != value) {
		//return -EIO;
	}

	return OK;
}

/* Get measurement value */
int
SI7210::get_measurement(uint8_t ptr, uint16_t *value)
{
	uint8_t data[2];

	if (OK != transfer(&ptr, 1, &data[0], 2)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	*value = (uint16_t)((data[1] & 0xff) + ((data[0] & 0xff) << 8));

	return OK;
}

/* Get sensor data */
int
SI7210::get_sensor_data(uint8_t otpAddr, int8_t *data)
{
	uint8_t optCtrl;
	uint8_t reg;

	if (OK != get_regs(OTP_CTRL, &optCtrl)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != (optCtrl & OTP_CTRL_OPT_BUSY_MASK)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	reg = otpAddr;

	if (OK != set_regs(OTP_ADDR, reg)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	reg = OTP_CTRL_OPT_READ_EN_MASK;

	if (OK != set_regs(OTP_CTRL, reg)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != get_regs(OTP_DATA, (uint8_t *) data)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	return OK;
}

void
SI7210::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_good_transfers);
	_reports->print_info("si7210 queue");
}

int
si7210_main(int argc, char *argv[])
{
	const char *verb = argv[1];

	bool external_bus = true;
	SI7210::Instance instance = SI7210::Instance::INVALID;
	int ch;

	int myoptind = 1;
	const char *myoptarg = nullptr;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "X:i:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;

		case 'i':
			instance = SI7210::Instance(atoi(myoptarg));

			// range check
			if ((instance > 3) || (instance < 0)) {
				PX4_ERR("invalid instance: %d (only 0 through 3 valid)", instance);
				return PX4_ERROR;
			}

			break;

		default:
			PX4_ERR("unknown command line argument");
			si7210::usage();
			return PX4_ERROR;
		}
	}

	if (instance == SI7210::Instance::INVALID) {
		PX4_ERR("the -i option is required");
		si7210::usage();
		exit(1);
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		si7210::start(external_bus, instance);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		si7210::test(external_bus, instance);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		si7210::reset(external_bus, instance);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		si7210::info(external_bus, instance);
	}

	PX4_ERR("missing command");
	si7210::usage();
	exit(1);
}
