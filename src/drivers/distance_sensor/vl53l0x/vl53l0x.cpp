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
 * @file vl53l0x.cpp
 * @author Daniele Pettenuzzo
 *
 * Driver for the vl53l0x ToF Sensor from ST Microelectronics connected via I2C.
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <perf/perf_counter.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

#include <board_config.h>

/* Configuration Constants */
#define VL53L0X_BUS_DEFAULT				PX4_I2C_BUS_EXPANSION

#define VL53L0X_BASEADDR				0x52
#define VL53L0X_DEVICE_PATH				"/dev/vl53l0x"

/* VL53L0X Registers addresses */
#define VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HW_REG		0x89
#define MSRC_CONFIG_CONTROL_REG				0x60
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_REG	0x44
#define SYSTEM_SEQUENCE_CONFIG_REG			0x01
#define DYNAMIC_SPAD_REF_EN_START_OFFSET_REG		0x4F
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD_REG		0x4E
#define GLOBAL_CONFIG_REF_EN_START_SELECT_REG		0xB6
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REG		0xB0
#define SYSTEM_INTERRUPT_CONFIG_GPIO_REG		0x0A
#define SYSTEM_SEQUENCE_CONFIG_REG			0x01
#define SYSRANGE_START_REG				0x00
#define RESULT_INTERRUPT_STATUS_REG			0x13
#define SYSTEM_INTERRUPT_CLEAR_REG			0x0B
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REG		0xB0
#define GPIO_HV_MUX_ACTIVE_HIGH_REG			0x84
#define SYSTEM_INTERRUPT_CLEAR_REG			0x0B
#define RESULT_RANGE_STATUS_REG				0x14
#define VL53L0X_RA_IDENTIFICATION_MODEL_ID		0xC0
#define VL53L0X_IDENTIFICATION_MODEL_ID			0xEEAA

#define VL53L0X_US					1000	// 1ms
#define VL53L0X_SAMPLE_RATE				50000	// 50ms

#define VL53L0X_MAX_RANGING_DISTANCE			2.0f
#define VL53L0X_MIN_RANGING_DISTANCE			0.0f

#define VL53L0X_BUS_CLOCK 				400000 // 400kHz bus clock speed

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class VL53L0X : public device::I2C
{
public:
	VL53L0X(uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING,
		int bus = VL53L0X_BUS_DEFAULT, int address = VL53L0X_BASEADDR);

	virtual ~VL53L0X();

	virtual int init();

	virtual int ioctl(device::file_t *filp, int cmd, unsigned long arg);

	virtual ssize_t read(device::file_t *filp, char *buffer, size_t buflen);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

protected:
	virtual int probe();

private:

	/**
	 * Initialise the automatic measurement state machine and start it.
	 */
	void start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void stop();

	int collect();
	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void cycle_trampoline(void *arg);

	int measure();

	int readRegister(uint8_t reg_address, uint8_t &value);
	int readRegisterMulti(uint8_t reg_address, uint8_t *value, uint8_t length);

	int writeRegister(uint8_t reg_address, uint8_t value);
	int writeRegisterMulti(uint8_t reg_address, uint8_t *value, uint8_t length);

	int sensorInit();
	bool sensorTuning();
	bool singleRefCalibration(uint8_t byte);
	bool spadCalculations();

	bool _collect_phase{false};
	bool _measurement_started{false};
	bool _new_measurement{true};
	bool _sensor_ok{false};

	int _class_instance{-1};
	int _measure_ticks{0};
	int _orb_class_instance{-1};

	uint8_t _rotation{0};
	uint8_t _stop_variable{0};

	orb_advert_t _distance_sensor_topic{nullptr};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "vl53l0x_com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "vl53l0x_read")};

	ringbuffer::RingBuffer *_reports{nullptr};

	work_s _work {};
};


VL53L0X::VL53L0X(uint8_t rotation, int bus, int address) :
	I2C("VL53L0X", VL53L0X_DEVICE_PATH, bus, address, VL53L0X_BUS_CLOCK),
	_rotation(rotation)
{
	// Allow 3 retries as the device typically misses the first measure attempts.
	I2C::_retries = 3;

	// Enable debug() calls.
	_debug_enabled = false;
}

VL53L0X::~VL53L0X()
{
	// Ensure we are truly inactive.
	stop();

	// Free any existing reports.
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	// Unadvertise uORB topics.
	if (_distance_sensor_topic != nullptr) {
		orb_unadvertise(_distance_sensor_topic);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}


int
VL53L0X::sensorInit()
{
	uint8_t val = 0;
	float rate_limit = 0.f;

	// I2C at 2.8V on sensor side of level shifter
	int ret = PX4_OK;
	ret |= readRegister(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HW_REG, val);

	if (ret != PX4_OK) {
		return PX4_ERROR;
	}

	ret |= writeRegister(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HW_REG, val | 0x01);

	// set I2C to standard mode
	ret |= writeRegister(0x88, 0x00);

	ret |= writeRegister(0x80, 0x01);
	ret |= writeRegister(0xFF, 0x01);
	ret |= writeRegister(0x00, 0x00);
	ret |= readRegister(0x91, val);
	ret |= writeRegister(0x00, 0x01);
	ret |= writeRegister(0xFF, 0x00);
	ret |= writeRegister(0x80, 0x00);

	if (ret != PX4_OK) {
		return PX4_ERROR;
	}

	_stop_variable = val;

	// disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
	readRegister(MSRC_CONFIG_CONTROL_REG, val);
	writeRegister(MSRC_CONFIG_CONTROL_REG, val | 0x12);

	// Set signal rate limit to 0.1
	rate_limit = 0.1 * 65536;

	uint8_t rate_limit_split[2] = {};
	rate_limit_split[0] = (((uint16_t)rate_limit) >> 8);
	rate_limit_split[1] = (uint16_t)rate_limit;

	writeRegisterMulti(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_REG, &rate_limit_split[0], 2);
	writeRegister(SYSTEM_SEQUENCE_CONFIG_REG, 0xFF);

	spadCalculations();

	return PX4_OK;

}

int
VL53L0X::init()
{
	set_device_address(VL53L0X_BASEADDR);

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

	if (_reports == nullptr) {
		return PX4_ERROR;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;

	return PX4_OK;
}

int
VL53L0X::probe()
{
	if (sensorInit() == OK) {
		return PX4_OK;
	}

	// not found on any address
	return -EIO;
}

int
VL53L0X::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			case 0:
				return -EINVAL;

			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(VL53L0X_SAMPLE_RATE);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					start();

					return PX4_OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return PX4_OK;
				}
			}
		}

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
VL53L0X::read(device::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct distance_sensor_s);
	struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

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

		while (!_collect_phase);

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

int
VL53L0X::readRegister(uint8_t reg_address, uint8_t &value)
{
	int ret;

	/* write register address to the sensor */
	ret = transfer(&reg_address, sizeof(reg_address), nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		return ret;
	}

	/* read from the sensor */
	ret = transfer(nullptr, 0, &value, 1);

	if (OK != ret) {
		perf_count(_comms_errors);
		return ret;
	}

	return ret;

}

int
VL53L0X::readRegisterMulti(uint8_t reg_address, uint8_t *value, uint8_t length)
{
	int ret;

	/* write register address to the sensor */
	ret = transfer(&reg_address, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		return ret;
	}

	/* read from the sensor */
	ret = transfer(nullptr, 0, &value[0], length);

	if (OK != ret) {
		perf_count(_comms_errors);
		return ret;
	}

	return ret;

}

int
VL53L0X::writeRegister(uint8_t reg_address, uint8_t value)
{
	int ret;
	uint8_t cmd[2] = {0, 0};

	cmd[0] = reg_address;
	cmd[1] = value;

	ret = transfer(&cmd[0], 2, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		return ret;
	}

	return ret;

}

int
VL53L0X::writeRegisterMulti(uint8_t reg_address, uint8_t *value,
			    uint8_t length) /* bytes are send in order as they are in the array */
{
	/* be careful: for uint16_t to send first higher byte */
	int ret;
	uint8_t cmd[7] = {};

	if (length > 6 || length < 1) {
		DEVICE_LOG("VL53L0X::writeRegisterMulti length out of range");
		return PX4_ERROR;
	}

	cmd[0] = reg_address;

	memcpy(&cmd[1], &value[0], length);

	ret = transfer(&cmd[0], length + 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		return ret;
	}

	return ret;

}

int
VL53L0X::measure()
{
	int ret = OK;
	uint8_t wait_for_measurement;
	uint8_t system_start;

	/*
	 * Send the command to begin a measurement.
	 */
	const uint8_t cmd = RESULT_RANGE_STATUS_REG + 10;

	if (_new_measurement) {

		_new_measurement = false;

		writeRegister(0x80, 0x01);
		writeRegister(0xFF, 0x01);
		writeRegister(0x00, 0x00);
		writeRegister(0x91, _stop_variable);
		writeRegister(0x00, 0x01);
		writeRegister(0xFF, 0x00);
		writeRegister(0x80, 0x00);

		writeRegister(SYSRANGE_START_REG, 0x01);

		readRegister(SYSRANGE_START_REG, system_start);

		if ((system_start & 0x01) == 1) {
			work_queue(LPWORK, &_work, (worker_t)&VL53L0X::cycle_trampoline, this,
				   USEC2TICK(VL53L0X_US));	// Reschedule every 1 ms until measurement is ready.
			return PX4_OK;

		} else {
			_measurement_started = true;
		}

	}

	if (!_collect_phase && !_measurement_started) {

		readRegister(SYSRANGE_START_REG, system_start);

		if ((system_start & 0x01) == 1) {
			work_queue(LPWORK, &_work, (worker_t)&VL53L0X::cycle_trampoline, this,
				   USEC2TICK(VL53L0X_US));	// Reschedule every 1 ms until measurement is ready.
			return PX4_OK;

		} else {
			_measurement_started = true;
		}
	}

	readRegister(RESULT_INTERRUPT_STATUS_REG, wait_for_measurement);

	if ((wait_for_measurement & 0x07) == 0) {
		work_queue(LPWORK, &_work, (worker_t)&VL53L0X::cycle_trampoline, this,
			   USEC2TICK(VL53L0X_US));	// Reschedule every 1 ms until measurement is ready.
		return PX4_OK;
	}

	_collect_phase = true;

	ret = transfer(&cmd, sizeof(cmd), nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		DEVICE_LOG("i2c::transfer returned %d", ret);
		return ret;
	}

	return PX4_OK;
}

int
VL53L0X::collect()
{
	int ret = -EIO;

	// Read from the sensor.
	uint8_t val[2] = {0, 0};
	perf_begin(_sample_perf);

	_collect_phase = false;

	ret = transfer(nullptr, 0, &val[0], 2);

	if (ret < 0) {
		DEVICE_LOG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint16_t distance_mm = (val[0] << 8) | val[1];

	struct distance_sensor_s report;
	report.timestamp        = hrt_absolute_time();
	report.current_distance = static_cast<float>(distance_mm) * 1e-3f;;
	report.id               = VL53L0X_BASEADDR;
	report.max_distance     = VL53L0X_MAX_RANGING_DISTANCE;
	report.min_distance     = VL53L0X_MIN_RANGING_DISTANCE;
	report.orientation      = _rotation;
	report.signal_quality   = -1;
	report.type             = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	report.variance         = 0.0f;

	// Publish the report data if we have a valid topic.
	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		orb_publish_auto(ORB_ID(distance_sensor), &_distance_sensor_topic, &report,
				 &_orb_class_instance, ORB_PRIO_DEFAULT);
	}

	_reports->force(&report);

	// Notify anyone waiting for data.
	poll_notify(POLLIN);
	perf_end(_sample_perf);

	return PX4_OK;
}

void
VL53L0X::start()
{
	// Reset the report ring and state machine.
	_reports->flush();

	// Schedule a cycle to start things.
	work_queue(LPWORK, &_work, (worker_t)&VL53L0X::cycle_trampoline, this, 0);
}

void
VL53L0X::stop()
{
	work_cancel(LPWORK, &_work);
}

void
VL53L0X::cycle()
{
	measure();

	if (_collect_phase) {

		_collect_phase = false;
		_new_measurement = true;

		collect();
	}

	work_queue(LPWORK, &_work, (worker_t)&VL53L0X::cycle_trampoline, this, _measure_ticks);
}

void
VL53L0X::cycle_trampoline(void *arg)
{
	VL53L0X *dev = (VL53L0X *)arg;
	dev->cycle();
}

void
VL53L0X::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

bool
VL53L0X::spadCalculations()
{
	uint8_t val = 0;

	writeRegister(0x80, 0x01);
	writeRegister(0xFF, 0x01);
	writeRegister(0x00, 0x00);
	writeRegister(0xFF, 0x06);

	readRegister(0x83, val);
	writeRegister(0x83, val | 0x04);

	writeRegister(0xFF, 0x07);
	writeRegister(0x81, 0x01);
	writeRegister(0x80, 0x01);
	writeRegister(0x94, 0x6b);
	writeRegister(0x83, 0x00);

	readRegister(0x83, val);

	while (val == 0x00) {
		readRegister(0x83, val);
	}

	writeRegister(0x83, 0x01);
	readRegister(0x92, val);

	uint8_t spad_count = val & 0x7f;
	bool spad_type_is_aperture = (val >> 7) & 0x01;

	writeRegister(0x81, 0x00);
	writeRegister(0xFF, 0x06);

	readRegister(0x83, val);
	writeRegister(0x83, val & ~0x04);

	writeRegister(0xFF, 0x01);
	writeRegister(0x00, 0x01);
	writeRegister(0xFF, 0x00);
	writeRegister(0x80, 0x00);

	uint8_t ref_spad_map[6] = {};
	readRegisterMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REG, &ref_spad_map[0], 6);

	writeRegister(0xFF, 0x01);
	writeRegister(DYNAMIC_SPAD_REF_EN_START_OFFSET_REG, 0x00);
	writeRegister(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD_REG, 0x2C);
	writeRegister(0xFF, 0x00);
	writeRegister(GLOBAL_CONFIG_REF_EN_START_SELECT_REG, 0xB4);

	uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
	uint8_t spads_enabled = 0;

	for (uint8_t i = 0; i < 48; i++) {
		if (i < first_spad_to_enable || spads_enabled == spad_count) {
			ref_spad_map[i / 8] &= ~(1 << (i % 8));

		} else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
			spads_enabled++;
		}
	}

	writeRegisterMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REG, &ref_spad_map[0], 6);

	sensorTuning();

	writeRegister(SYSTEM_INTERRUPT_CONFIG_GPIO_REG, 4);		// 4: GPIO interrupt on new data

	readRegister(GPIO_HV_MUX_ACTIVE_HIGH_REG, val);

	writeRegister(GPIO_HV_MUX_ACTIVE_HIGH_REG, val & ~0x10);	// Active low.
	writeRegister(SYSTEM_INTERRUPT_CLEAR_REG, 0x01);
	writeRegister(SYSTEM_SEQUENCE_CONFIG_REG, 0xE8);
	writeRegister(SYSTEM_SEQUENCE_CONFIG_REG, 0x01);

	singleRefCalibration(0x40);

	writeRegister(SYSTEM_SEQUENCE_CONFIG_REG, 0x02);

	singleRefCalibration(0x00);

	writeRegister(SYSTEM_SEQUENCE_CONFIG_REG, 0xE8);		// Restore config/

	return PX4_OK;
}


bool
VL53L0X::sensorTuning()
{
	writeRegister(0xFF, 0x01);
	writeRegister(0x00, 0x00);
	writeRegister(0xFF, 0x00);
	writeRegister(0x09, 0x00);
	writeRegister(0x10, 0x00);
	writeRegister(0x11, 0x00);
	writeRegister(0x24, 0x01);
	writeRegister(0x25, 0xFF);
	writeRegister(0x75, 0x00);
	writeRegister(0xFF, 0x01);
	writeRegister(0x4E, 0x2C);
	writeRegister(0x48, 0x00);
	writeRegister(0x30, 0x20);
	writeRegister(0xFF, 0x00);
	writeRegister(0x30, 0x09);
	writeRegister(0x54, 0x00);
	writeRegister(0x31, 0x04);
	writeRegister(0x32, 0x03);
	writeRegister(0x40, 0x83);
	writeRegister(0x46, 0x25);
	writeRegister(0x60, 0x00);
	writeRegister(0x27, 0x00);
	writeRegister(0x50, 0x06);
	writeRegister(0x51, 0x00);
	writeRegister(0x52, 0x96);
	writeRegister(0x56, 0x08);
	writeRegister(0x57, 0x30);
	writeRegister(0x61, 0x00);
	writeRegister(0x62, 0x00);
	writeRegister(0x64, 0x00);
	writeRegister(0x65, 0x00);
	writeRegister(0x66, 0xA0);
	writeRegister(0xFF, 0x01);
	writeRegister(0x22, 0x32);
	writeRegister(0x47, 0x14);
	writeRegister(0x49, 0xFF);
	writeRegister(0x4A, 0x00);
	writeRegister(0xFF, 0x00);
	writeRegister(0x7A, 0x0A);
	writeRegister(0x7B, 0x00);
	writeRegister(0x78, 0x21);
	writeRegister(0xFF, 0x01);
	writeRegister(0x23, 0x34);
	writeRegister(0x42, 0x00);
	writeRegister(0x44, 0xFF);
	writeRegister(0x45, 0x26);
	writeRegister(0x46, 0x05);
	writeRegister(0x40, 0x40);
	writeRegister(0x0E, 0x06);
	writeRegister(0x20, 0x1A);
	writeRegister(0x43, 0x40);
	writeRegister(0xFF, 0x00);
	writeRegister(0x34, 0x03);
	writeRegister(0x35, 0x44);
	writeRegister(0xFF, 0x01);
	writeRegister(0x31, 0x04);
	writeRegister(0x4B, 0x09);
	writeRegister(0x4C, 0x05);
	writeRegister(0x4D, 0x04);
	writeRegister(0xFF, 0x00);
	writeRegister(0x44, 0x00);
	writeRegister(0x45, 0x20);
	writeRegister(0x47, 0x08);
	writeRegister(0x48, 0x28);
	writeRegister(0x67, 0x00);
	writeRegister(0x70, 0x04);
	writeRegister(0x71, 0x01);
	writeRegister(0x72, 0xFE);
	writeRegister(0x76, 0x00);
	writeRegister(0x77, 0x00);
	writeRegister(0xFF, 0x01);
	writeRegister(0x0D, 0x01);
	writeRegister(0xFF, 0x00);
	writeRegister(0x80, 0x01);
	writeRegister(0x01, 0xF8);
	writeRegister(0xFF, 0x01);
	writeRegister(0x8E, 0x01);
	writeRegister(0x00, 0x01);
	writeRegister(0xFF, 0x00);
	writeRegister(0x80, 0x00);

	return PX4_OK;
}


bool
VL53L0X::singleRefCalibration(uint8_t byte)
{
	uint8_t val = 0;

	writeRegister(SYSRANGE_START_REG, byte | 0x01);		// VL53L0X_REG_SYSRANGE_MODE_START_STOP

	do {
		readRegister(RESULT_INTERRUPT_STATUS_REG, val);
	} while ((val & 0x07) == 0);

	writeRegister(SYSTEM_INTERRUPT_CLEAR_REG, 0x01);
	writeRegister(SYSRANGE_START_REG, 0x00);

	return PX4_OK;
}


/**
 * Local functions in support of the shell command.
 */
namespace vl53l0x
{

VL53L0X	*g_dev;

int 	info();
int 	start(uint8_t rotation);
int 	start_bus(uint8_t rotation, int i2c_bus);
int 	stop();
int 	test();

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return PX4_OK;
}

/**
 * Reset the driver.
 */
int
reset()
{
	int fd = px4_open(VL53L0X_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("driver open failed");
		return PX4_ERROR;
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		px4_close(fd);
		return PX4_ERROR;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		px4_close(fd);
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * Attempt to start driver on all available I2C busses.
 *
 * This function will return as soon as the first sensor
 * is detected on one of the available busses or if no
 * sensors are detected.
 */
int
start(uint8_t rotation)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	for (size_t i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
		if (start_bus(rotation, i2c_bus_options[i]) == PX4_OK) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

/**
 * Start the driver on a specific bus.
 *
 * This function only returns if the sensor is up and running
 * or could not be detected successfully.
 */
int
start_bus(uint8_t rotation, int i2c_bus)
{
	int fd = -1;

	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	// Instantiate the driver.
	g_dev = new VL53L0X(rotation, i2c_bus);

	if (g_dev == nullptr) {
		delete g_dev;
		return PX4_ERROR;
	}

	if (OK != g_dev->init()) {
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = px4_open(VL53L0X_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		if (g_dev != nullptr) {
			delete g_dev;
			g_dev = nullptr;
		}

		px4_close(fd);
		return PX4_ERROR;
	}

	// Set the poll rate to default, automatic data collection will begin.
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		if (g_dev != nullptr) {
			delete g_dev;
			g_dev = nullptr;
		}

		px4_close(fd);
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * Stop the driver
 */
int
stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
int
test()
{
	struct distance_sensor_s report;
	ssize_t sz;

	int fd = px4_open(VL53L0X_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'vl53l0x start' if the driver is not running)", VL53L0X_DEVICE_PATH);
		return PX4_ERROR;
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_ERR("immediate read failed");
		return PX4_ERROR;
	}

	print_message(report);

	px4_close(fd);

	PX4_INFO("PASS");
	return PX4_OK;
}

} // namespace vl53l0x


static void
vl53l0x_usage()
{
	PX4_INFO("usage: vl53l0x command [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-a --all available busses");
	PX4_INFO("\t-b --bus i2cbus (%d)", VL53L0X_BUS_DEFAULT);
	PX4_INFO("\t-R --rotation (%d)", distance_sensor_s::ROTATION_DOWNWARD_FACING);
	PX4_INFO("command:");
	PX4_INFO("\tinfo|start|stop|test");
}

/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int vl53l0x_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	bool start_all = false;

	int i2c_bus = VL53L0X_BUS_DEFAULT;

	while ((ch = px4_getopt(argc, argv, "ab:R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'a':
			start_all = true;
			break;

		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		default:
			PX4_WARN("Unknown option!");
			vl53l0x_usage();
			return PX4_ERROR;
		}
	}

	if (myoptind >= argc) {
		vl53l0x_usage();
		return PX4_ERROR;
	}


	// Start/load the driver.
	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			return vl53l0x::start(rotation);

		} else {
			return vl53l0x::start_bus(rotation, i2c_bus);
		}
	}

	// Stop the driver
	if (!strcmp(argv[myoptind], "stop")) {
		return vl53l0x::stop();
	}

	// Test the driver/device.
	if (!strcmp(argv[myoptind], "test")) {
		return vl53l0x::test();
	}

	// Print driver information.
	if (!strcmp(argv[myoptind], "help") ||
	    !strcmp(argv[myoptind], "info") ||
	    !strcmp(argv[myoptind], "status")) {
		return vl53l0x::info();
	}

	vl53l0x_usage();
	return PX4_ERROR;
}
