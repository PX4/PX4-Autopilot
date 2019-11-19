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

#include "BMP280.hpp"

BMP280::BMP280(bmp280::IBMP280 *interface, const char *path) :
	CDev(path),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id())),
	_interface(interface),
	_running(false),
	_report_interval(0),
	_reports(nullptr),
	_collect_phase(false),
	_baro_topic(nullptr),
	_orb_class_instance(-1),
	_class_instance(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmp280_read")),
	_measure_perf(perf_alloc(PC_ELAPSED, "bmp280_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "bmp280_comms_errors"))
{
}

BMP280::~BMP280()
{
	/* make sure we are truly inactive */
	stop_cycle();

	if (_class_instance != -1) {
		unregister_class_devname(BARO_BASE_DEVICE_PATH, _class_instance);
	}

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_baro_topic != nullptr) {
		orb_unadvertise(_baro_topic);
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
	int ret = CDev::init();

	if (ret != OK) {
		PX4_ERR("CDev init failed");
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_baro_s));

	if (_reports == nullptr) {
		PX4_ERR("can't get memory for reports");
		ret = -ENOMEM;
		return ret;
	}

	/* register alternate interfaces if we have to */
	_class_instance = register_class_devname(BARO_BASE_DEVICE_PATH);

	/* reset sensor */
	_interface->set_reg(BMP280_VALUE_RESET, BMP280_ADDR_RESET);
	usleep(10000);

	/* check  id*/
	if (_interface->get_reg(BMP280_ADDR_ID) != BMP280_VALUE_ID) {
		PX4_WARN("id of your baro is not: 0x%02x", BMP280_VALUE_ID);
		return -EIO;
	}

	/* set config, recommended settings */
	_curr_ctrl = BMP280_CTRL_P16 | BMP280_CTRL_T2;
	_interface->set_reg(_curr_ctrl, BMP280_ADDR_CTRL);
	_max_measure_interval = (BMP280_MT_INIT + BMP280_MT * (16 - 1 + 2 - 1));
	_interface->set_reg(BMP280_CONFIG_F16, BMP280_ADDR_CONFIG);

	/* get calibration and pre process them*/
	_cal = _interface->get_calibration(BMP280_ADDR_CAL);

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
	sensor_baro_s brp;
	_reports->flush();

	if (measure()) {
		return -EIO;
	}

	usleep(_max_measure_interval);

	if (collect()) {
		return -EIO;
	}

	_reports->get(&brp);

	_baro_topic = orb_advertise_multi(ORB_ID(sensor_baro), &brp,
					  &_orb_class_instance, _interface->is_external() ? ORB_PRIO_HIGH : ORB_PRIO_DEFAULT);

	if (_baro_topic == nullptr) {
		PX4_WARN("failed to create sensor_baro publication");
		return -ENOMEM;
	}

	return OK;

}

ssize_t
BMP280::read(cdev::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_baro_s);
	sensor_baro_s *brp = reinterpret_cast<sensor_baro_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_report_interval > 0) {

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

	usleep(_max_measure_interval);

	if (collect()) {
		return -EIO;
	}

	if (_reports->get(brp)) { //get new generated report
		ret = sizeof(*brp);
	}

	return ret;
}

int
BMP280::ioctl(cdev::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {

			unsigned interval = 0;

			switch (arg) {

			case 0:
				return -EINVAL;

			case SENSOR_POLLRATE_DEFAULT:
				interval = _max_measure_interval;

			/* FALLTHROUGH */
			default: {
					if (interval == 0) {
						interval = (1000000 / arg);
					}

					/* do we need to start internal polling? */
					bool want_start = (_report_interval == 0);

					/* check against maximum rate */
					if (interval < _max_measure_interval) {
						return -EINVAL;
					}

					_report_interval = interval;

					if (want_start) {
						start_cycle();
					}

					return OK;
				}
			}

			break;
		}

	case SENSORIOCRESET:
		/*
		 * Since we are initialized, we do not need to do anything, since the
		 * PROM is correctly read and the part does not need to be configured.
		 */
		return OK;

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
	_running = true;
	_reports->flush();

	/* schedule a cycle to start things */
	ScheduleNow();
}

void
BMP280::stop_cycle()
{
	_running = false;
	ScheduleClear();
}

void
BMP280::Run()
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
BMP280::measure()
{
	_collect_phase = true;

	perf_begin(_measure_perf);

	/* start measure */
	int ret = _interface->set_reg(_curr_ctrl | BMP280_CTRL_MODE_FORCE, BMP280_ADDR_CTRL);

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

	sensor_baro_s report;
	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);
	report.device_id = _interface->get_device_id();

	bmp280::data_s *data = _interface->get_data(BMP280_ADDR_DATA);

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

	/* publish it */
	orb_publish(ORB_ID(sensor_baro), _baro_topic, &report);

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
	printf("poll interval:  %u us \n", _report_interval);
	_reports->print_info("report queue");

	sensor_baro_s brp = {};
	_reports->get(&brp);
	print_message(brp);
}
