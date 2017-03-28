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
 * @file bmp285.cpp
 * Driver for the BMP285 barometric pressure sensor connected via I2C TODO or SPI.
 */



#include <drivers/bmp285/bmp285.hpp>

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif



BMP285::BMP285(int bus, uint16_t address, const char *path, bool external) :
	I2C("BMP285", path, bus, address, BMP285_BUS_SPEED),
	_curr_ctrl(0),
	_report_ticks(0),
	_max_mesure_ticks(0),
	_reports(nullptr),
	_collect_phase(false),
	_msl_pressure(101325),
	_baro_topic(nullptr),
	_orb_class_instance(-1),
	_class_instance(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmp285_read")),
	_measure_perf(perf_alloc(PC_ELAPSED, "bmp285_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "bmp285_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "bmp285_buffer_overflows")),
	_P(0.0f),
	_T(0.0f)
{
	_external = external;
	// work_cancel in stop_cycle called from the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

BMP285::~BMP285()
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
	perf_free(_buffer_overflows);

}

int
BMP285::init()
{
	int ret;

	/* do I2C init (and probe) first */
	ret = I2C::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("I2C setup failed");
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
	write_reg(BPM285_ADDR_RESET, BPM285_VALUE_RESET);
	usleep(10000);

	/* check  id*/
	if (read_reg(BPM285_ADDR_ID) != BPM285_VALUE_ID) {
		warnx("id of your baro is not: 0x%02x", BPM285_VALUE_ID);
		return -EIO;
	}

	/* Put sensor in Normal mode */
	ret = write_reg(BPM285_ADDR_CTRL, _curr_ctrl | BPM285_CTRL_MODE_NORMAL);

	/* set config, recommended settings */
	_curr_ctrl = BPM285_CTRL_P16 | BPM285_CTRL_T2;
	write_reg(BPM285_ADDR_CTRL, _curr_ctrl);
	_max_mesure_ticks = USEC2TICK(BPM285_MT_INIT + BPM285_MT * (16 - 1 + 2 - 1));


	/* Configure filter settings */
	write_reg(BPM285_ADDR_CONFIG, BPM285_CONFIG_F16);

	/* get calibration and pre process them*/
	get_calibration(BPM285_ADDR_CAL);

	_fcal.t1 =  _cal.t1 * powf(2,  4);
	_fcal.t2 =  _cal.t2 * powf(2, -14);
	_fcal.t3 =  _cal.t3 * powf(2, -34);

	_fcal.p1 = _cal.p1            * (powf(2,  4) / -100000.0f);
	_fcal.p2 = _cal.p1 * _cal.p2 * (powf(2, -31) / -100000.0f);
	_fcal.p3 = _cal.p1 * _cal.p3 * (powf(2, -51) / -100000.0f);

	_fcal.p4 = _cal.p4 * powf(2,  4) - powf(2, 20);
	_fcal.p5 = _cal.p5 * powf(2, -14);
	_fcal.p6 = _cal.p6 * powf(2, -31);

	_fcal.p7 = _cal.p7 * powf(2, -4);
	_fcal.p8 = _cal.p8 * powf(2, -19) + 1.0f;
	_fcal.p9 = _cal.p9 * powf(2, -35);

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
					  &_orb_class_instance, (is_external()) ? ORB_PRIO_HIGH : ORB_PRIO_DEFAULT);

	if (_baro_topic == nullptr) {
		warnx("failed to create sensor_baro publication");
		return -ENOMEM;
	}

	return OK;

}

ssize_t
BMP285::read(struct file *filp, char *buffer, size_t buflen)
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
BMP285::ioctl(struct file *filp, int cmd, unsigned long arg)
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
BMP285::start_cycle()
{

	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&BMP285::cycle_trampoline, this, 1);
}

void
BMP285::stop_cycle()
{
	work_cancel(HPWORK, &_work);
}

void
BMP285::cycle_trampoline(void *arg)
{
	BMP285 *dev = reinterpret_cast<BMP285 *>(arg);

	dev->cycle();
}

void
BMP285::cycle()
{
	if (_collect_phase) {
		collect();
		unsigned wait_gap = _report_ticks - _max_mesure_ticks;

		if (wait_gap != 0) {
			work_queue(HPWORK, &_work, (worker_t)&BMP285::cycle_trampoline, this,
				   wait_gap); //need to wait some time before new measurement
			return;
		}

	}

	measure();
	work_queue(HPWORK, &_work, (worker_t)&BMP285::cycle_trampoline, this, _max_mesure_ticks);

}

int
BMP285::measure()
{
	_collect_phase = true;

	perf_begin(_measure_perf);

	/* start measure */
	int ret = write_reg(BPM285_ADDR_CTRL, _curr_ctrl | BPM285_CTRL_MODE_FORCE);

	if (ret != OK) {
		perf_count(_comms_errors);
		perf_cancel(_measure_perf);
		return -EIO;
	}

	perf_end(_measure_perf);

	return OK;
}

int
BMP285::collect()
{
	_collect_phase = false;

	perf_begin(_sample_perf);

	struct baro_report report;
	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);

	bmp285::data_s *data = get_data(BPM285_ADDR_DATA);

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


	/* altitude calculations based on http://www.kansasflyer.org/index.asp?nav=Avi&sec=Alti&tab=Theory&pg=1 */

	/* tropospheric properties (0-11km) for standard atmosphere */
	const float T1 = 15.0f + 273.15f;   /* temperature at base height in Kelvin */
	const float a  = -6.5f / 1000.0f;   /* temperature gradient in degrees per metre */
	const float g  = 9.80665f;  /* gravity constant in m/s/s */
	const float R  = 287.05f;   /* ideal gas constant in J/kg/K */
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

	if (_reports->force(&report)) {
		perf_count(_buffer_overflows);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	perf_end(_sample_perf);

	return OK;
}

void
BMP285::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u us \n", _report_ticks * USEC_PER_TICK);
	_reports->print_info("report queue");
	printf("P Pa:              %.3f\n", (double)_P);
	printf("T:              %.3f\n", (double)_T);
	printf("MSL pressure Pa:   %u\n", _msl_pressure);

}

bool BMP285::is_external()
{
	return _external;
};


uint8_t
BMP285::read_reg(unsigned reg)
{
	const uint8_t cmd = (uint8_t)(reg) ;
	uint8_t result;

	transfer(&cmd, sizeof(cmd), &result, 1);

	return result;
}

int
BMP285::write_reg(unsigned reg, uint8_t value)
{
	uint8_t cmd[2] = {(uint8_t)reg, value};

	return transfer(cmd, sizeof(cmd), nullptr, 0);
}

bmp285::data_s *BMP285::get_data(uint8_t reg)
{
	const uint8_t cmd = (uint8_t)(reg);

	if (transfer(&cmd, sizeof(cmd), (uint8_t *)&_data, sizeof(struct bmp285::data_s)) == OK) {
		return (&_data);

	} else {
		return nullptr;
	}

}
bmp285::calibration_s *BMP285::get_calibration(uint8_t reg)
{
	const uint8_t cmd = (uint8_t)(reg) ;

	if (transfer(&cmd, sizeof(cmd), (uint8_t *)&_cal, sizeof(struct bmp285::calibration_s)) == OK) {
		return &(_cal);

	} else {
		return nullptr;
	}

}

