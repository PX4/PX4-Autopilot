/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @file LidarLiteI2C.cpp
 * @author Allyson Kreft
 *
 * Driver for the PulsedLight Lidar-Lite range finders connected via I2C.
 */

#include "LidarLiteI2C.h"
#include <px4_defines.h>
#include <semaphore.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <string.h>
#include <stdio.h>
#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>

LidarLiteI2C::LidarLiteI2C(int bus, const char *path, uint8_t rotation, int address) :
	I2C("LL40LS", path, bus, address, 100000),
	_rotation(rotation),
	_work{},
	_reports(nullptr),
	_sensor_ok(false),
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_distance_sensor_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "ll40ls_i2c_read")),
	_comms_errors(perf_alloc(PC_COUNT, "ll40ls_i2c_comms_errors")),
	_sensor_resets(perf_alloc(PC_COUNT, "ll40ls_i2c_resets")),
	_sensor_zero_resets(perf_alloc(PC_COUNT, "ll40ls_i2c_zero_resets")),
	_last_distance(0),
	_zero_counter(0),
	_acquire_time_usec(0),
	_pause_measurements(false),
	_hw_version(0),
	_sw_version(0),
	_unit_id(0)
{
	// up the retries since the device misses the first measure attempts
	_retries = 3;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

LidarLiteI2C::~LidarLiteI2C()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_distance_sensor_topic != nullptr) {
		orb_unadvertise(_distance_sensor_topic);
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_sensor_resets);
	perf_free(_sensor_zero_resets);
}

int LidarLiteI2C::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(struct distance_sensor_s));

	if (_reports == nullptr) {
		return ret;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	/* get a publish handle on the range finder topic */
	struct distance_sensor_s ds_report = {};
	measure();
	_reports->get(&ds_report);
	_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
				 &_orb_class_instance, ORB_PRIO_LOW);

	if (_distance_sensor_topic == nullptr) {
		PX4_DEBUG("failed to create distance_sensor object. Did you start uOrb?");
	}

	ret = OK;
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;

	return ret;
}

int LidarLiteI2C::read_reg(uint8_t reg, uint8_t &val)
{
	return lidar_transfer(&reg, 1, &val, 1);
}

int LidarLiteI2C::write_reg(uint8_t reg, uint8_t val)
{
	const uint8_t cmd[2] = { reg, val };
	return transfer(&cmd[0], 2, nullptr, 0);
}

/*
  LidarLite specific transfer() function that avoids a stop condition
  with SCL low
 */
int LidarLiteI2C::lidar_transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
{
	if (send != nullptr && send_len > 0) {
		int ret = transfer(send, send_len, nullptr, 0);

		if (ret != OK) {
			return ret;
		}
	}

	if (recv != nullptr && recv_len > 0) {
		return transfer(nullptr, 0, recv, recv_len);
	}

	return OK;
}

int LidarLiteI2C::probe()
{
	// cope with both old and new I2C bus address
	const uint8_t addresses[2] = {LL40LS_BASEADDR, LL40LS_BASEADDR_OLD};

	uint8_t id_high = 0, id_low = 0;

	// more retries for detection
	_retries = 10;

	for (uint8_t i = 0; i < sizeof(addresses); i++) {

		set_device_address(addresses[i]);

		if (addresses[i] == LL40LS_BASEADDR) {

			/*
			  check for unit id. It would be better if
			  we had a proper WHOAMI register
			 */
			if (read_reg(LL40LS_UNIT_ID_HIGH, id_high) == OK && id_high > 0 &&
			    read_reg(LL40LS_UNIT_ID_LOW, id_low) == OK && id_low > 0) {
				_unit_id = (uint16_t)((id_high << 8) | id_low) & 0xFFFF;
				goto ok;
			}

			PX4_DEBUG("probe failed unit_id=0x%02x\n",
				  (unsigned)_unit_id);

		} else {
			/*
			  check for hw and sw versions. It would be better if
			  we had a proper WHOAMI register
			 */
			if (read_reg(LL40LS_HW_VERSION, _hw_version) == OK && _hw_version > 0 &&
			    read_reg(LL40LS_SW_VERSION, _sw_version) == OK && _sw_version > 0) {
				set_maximum_distance(LL40LS_MAX_DISTANCE_V1);
				goto ok;
			}

			PX4_DEBUG("probe failed hw_version=0x%02x sw_version=0x%02x\n",
				  (unsigned)_hw_version,
				  (unsigned)_sw_version);
		}

	}

	// not found on any address
	return -EIO;

ok:
	_retries = 3;
	return reset_sensor();
}

int LidarLiteI2C::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	default: {
			int result = LidarLite::ioctl(filp, cmd, arg);

			if (result == -EINVAL) {
				result = I2C::ioctl(filp, cmd, arg);
			}

			return result;
		}
	}
}

ssize_t LidarLiteI2C::read(device::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct distance_sensor_s);
	struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (getMeasureTicks() > 0) {

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
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		px4_usleep(LL40LS_CONVERSION_INTERVAL);

		/* run the collection phase */
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

int LidarLiteI2C::measure()
{
	int ret;

	if (_pause_measurements) {
		// we are in print_registers() and need to avoid
		// acquisition to keep the I2C peripheral on the
		// sensor active
		return OK;
	}

	/*
	 * Send the command to begin a measurement.
	 */
	const uint8_t cmd[2] = { LL40LS_MEASURE_REG, LL40LS_MSRREG_ACQUIRE };
	ret = lidar_transfer(cmd, sizeof(cmd), nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);

		// if we are getting lots of I2C transfer errors try
		// resetting the sensor
		if (perf_event_count(_comms_errors) % 10 == 0) {
			perf_count(_sensor_resets);
			reset_sensor();
		}

		return ret;
	}

	// remember when we sent the acquire so we can know when the
	// acquisition has timed out
	_acquire_time_usec = hrt_absolute_time();
	ret = OK;

	return ret;
}

/*
  reset the sensor to power on defaults plus additional configurations
 */
int LidarLiteI2C::reset_sensor()
{
	int ret;
	ret = write_reg(LL40LS_MEASURE_REG, LL40LS_MSRREG_RESET);

	if (ret != OK) {
		return ret;
	}

	// wait for sensor reset to complete
	px4_usleep(50000);
	ret = write_reg(LL40LS_SIG_COUNT_VAL_REG, LL40LS_SIG_COUNT_VAL_MAX);

	if (ret != OK) {
		return ret;
	}

	// wait for register write to complete
	px4_usleep(1000);

	return OK;
}

/*
  dump sensor registers for debugging
 */
void LidarLiteI2C::print_registers()
{
	_pause_measurements = true;
	printf("ll40ls registers\n");
	// wait for a while to ensure the lidar is in a ready state
	px4_usleep(50000);

	for (uint8_t reg = 0; reg <= 0x67; reg++) {
		uint8_t val = 0;
		int ret = lidar_transfer(&reg, 1, &val, 1);

		if (ret != OK) {
			printf("%02x:XX ", (unsigned)reg);

		} else {
			printf("%02x:%02x ", (unsigned)reg, (unsigned)val);
		}

		if (reg % 16 == 15) {
			printf("\n");
		}
	}

	printf("\n");
	_pause_measurements = false;
}

int LidarLiteI2C::collect()
{
	int ret = -EIO;

	/* read from the sensor */
	uint8_t val[2] = {0, 0};

	perf_begin(_sample_perf);

	// read the high and low byte distance registers
	uint8_t distance_reg = LL40LS_DISTHIGH_REG | LL40LS_AUTO_INCREMENT;
	ret = lidar_transfer(&distance_reg, 1, &val[0], sizeof(val));

	// if the transfer failed or if the high bit of distance is
	// set then the distance is invalid
	if (ret < 0 || (val[0] & 0x80)) {
		if (hrt_absolute_time() - _acquire_time_usec > LL40LS_CONVERSION_TIMEOUT) {
			/*
			  NACKs from the sensor are expected when we
			  read before it is ready, so only consider it
			  an error if more than 100ms has elapsed.
			 */
			PX4_DEBUG("error reading from sensor: %d", ret);
			perf_count(_comms_errors);

			if (perf_event_count(_comms_errors) % 10 == 0) {
				perf_count(_sensor_resets);
				reset_sensor();
			}
		}

		perf_end(_sample_perf);
		// if we are getting lots of I2C transfer errors try
		// resetting the sensor
		return ret;
	}

	uint16_t distance_cm = (val[0] << 8) | val[1];
	float distance_m = float(distance_cm) * 1e-2f;
	struct distance_sensor_s report;

	if (distance_cm == 0) {
		_zero_counter++;

		if (_zero_counter == 20) {
			/* we have had 20 zeros in a row - reset the
			   sensor. This is a known bad state of the
			   sensor where it returns 16 bits of zero for
			   the distance with a trailing NACK, and
			   keeps doing that even when the target comes
			   into a valid range.
			*/
			_zero_counter = 0;
			perf_end(_sample_perf);
			perf_count(_sensor_zero_resets);
			return reset_sensor();
		}

	} else {
		_zero_counter = 0;
	}

	_last_distance = distance_cm;

	/* Relative signal strength measurement, i.e. the strength of
	 * the main signal peak compared to the general noise level*/
	uint8_t signal_strength_reg = LL40LS_SIGNAL_STRENGTH_REG;
	ret = lidar_transfer(&signal_strength_reg, 1, &val[0], 1);

	// check if the transfer failed
	if (ret < 0) {
		if (hrt_absolute_time() - _acquire_time_usec > LL40LS_CONVERSION_TIMEOUT) {
			/*
			  NACKs from the sensor are expected when we
			  read before it is ready, so only consider it
			  an error if more than 100ms has elapsed.
			 */
			PX4_INFO("signal strength read failed");

			DEVICE_DEBUG("error reading signal strength from sensor: %d", ret);
			perf_count(_comms_errors);

			if (perf_event_count(_comms_errors) % 10 == 0) {
				perf_count(_sensor_resets);
				reset_sensor();
			}
		}

		perf_end(_sample_perf);
		// if we are getting lots of I2C transfer errors try
		// resetting the sensor
		return ret;
	}

	uint8_t ll40ls_signal_strength = val[0];


	/* Absolute peak strength measurement, i.e. absolute strength of main signal peak*/
	uint8_t peak_strength_reg = LL40LS_PEAK_STRENGTH_REG;
	ret = lidar_transfer(&peak_strength_reg, 1, &val[0], 1);

	// check if the transfer failed
	if (ret < 0) {
		if (hrt_absolute_time() - _acquire_time_usec > LL40LS_CONVERSION_TIMEOUT) {
			/*
			  NACKs from the sensor are expected when we
			  read before it is ready, so only consider it
			  an error if more than 100ms has elapsed.
			 */
			PX4_INFO("peak strenght read failed");

			DEVICE_DEBUG("error reading peak strength from sensor: %d", ret);
			perf_count(_comms_errors);

			if (perf_event_count(_comms_errors) % 10 == 0) {
				perf_count(_sensor_resets);
				reset_sensor();
			}
		}

		perf_end(_sample_perf);
		// if we are getting lots of I2C transfer errors try
		// resetting the sensor
		return ret;
	}

	uint8_t ll40ls_peak_strength = val[0];

	/* Final data quality evaluation. This is based on the datasheet and simple heuristics retrieved from experiments*/
	// Step 1: Normalize signal strength to 0...100 percent using the absolute signal peak strength.
	uint8_t signal_quality = 100 * math::max(ll40ls_peak_strength - LL40LS_PEAK_STRENGTH_LOW,
				 0) / (LL40LS_PEAK_STRENGTH_HIGH - LL40LS_PEAK_STRENGTH_LOW);

	// Step 2: Also use ll40ls_signal_strength (a relative measure, i.e. peak strength to noise!) to reject potentially ambiguous measurements
	if (ll40ls_signal_strength <= LL40LS_SIGNAL_STRENGTH_LOW) { signal_quality = 0; }

	// Step 3: Filter physically impossible measurements, which removes some crazy outliers that appear on LL40LS.
	if (distance_m < LL40LS_MIN_DISTANCE) { signal_quality = 0; }

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();
	report.current_distance = distance_m;
	report.min_distance = get_minimum_distance();
	report.max_distance = get_maximum_distance();
	report.covariance = 0.0f;
	report.signal_quality = signal_quality;
	report.type =
		distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;		// the sensor is in fact a laser + sonar but there is no enum for this
	report.orientation = _rotation;
	report.id = 0;	// TODO: set proper ID

	/* publish it, if we are the primary */
	if (_distance_sensor_topic != nullptr) {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
	}

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void LidarLiteI2C::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&LidarLiteI2C::cycle_trampoline, this, 1);
}

void LidarLiteI2C::stop()
{
	work_cancel(HPWORK, &_work);
}

void LidarLiteI2C::cycle_trampoline(void *arg)
{
	LidarLiteI2C *dev = (LidarLiteI2C *)arg;

	dev->cycle();
}

void LidarLiteI2C::cycle()
{
	/* collection phase? */
	if (_collect_phase) {

		/* try a collection */
		if (OK != collect()) {
			PX4_DEBUG("collection error");

			/* if we've been waiting more than 200ms then
			   send a new acquire */
			if (hrt_absolute_time() - _acquire_time_usec > LL40LS_CONVERSION_TIMEOUT * 2) {
				_collect_phase = false;
			}

		} else {
			/* next phase is measurement */
			_collect_phase = false;

			/*
			 * Is there a collect->measure gap?
			 */
			if (getMeasureTicks() > USEC2TICK(LL40LS_CONVERSION_INTERVAL)) {

				/* schedule a fresh cycle call when we are ready to measure again */
				work_queue(HPWORK,
					   &_work,
					   (worker_t)&LidarLiteI2C::cycle_trampoline,
					   this,
					   getMeasureTicks() - USEC2TICK(LL40LS_CONVERSION_INTERVAL));

				return;
			}
		}
	}

	if (_collect_phase == false) {
		/* measurement phase */
		if (OK != measure()) {
			PX4_DEBUG("measure error");

		} else {
			/* next phase is collection. Don't switch to
			   collection phase until we have a successful
			   acquire request I2C transfer */
			_collect_phase = true;
		}
	}

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&LidarLiteI2C::cycle_trampoline,
		   this,
		   USEC2TICK(LL40LS_CONVERSION_INTERVAL));
}

void LidarLiteI2C::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_sensor_resets);
	perf_print_counter(_sensor_zero_resets);
	printf("poll interval:  %u ticks\n", getMeasureTicks());
	_reports->print_info("report queue");
	printf("distance: %ucm (0x%04x)\n",
	       (unsigned)_last_distance, (unsigned)_last_distance);
}

const char *LidarLiteI2C::get_dev_name()
{
	return get_devname();
}
