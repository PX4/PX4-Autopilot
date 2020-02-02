/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file pcf8583.cpp
 *
 * @author ThunderFly s.r.o., Vít Hanousek <hanousekvit@thunderfly.cz>
 * @url https://github.com/ThunderFly-aerospace/TFRPM01
 *
 * Driver for Main Rotor frequency sensor using PCF8583 I2C counter.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>


#include <lib/parameters/param.h>

#include <board_config.h>

#include <px4_log.h>
#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/rpm.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>


/* Configuration Constants */
#define PCF8583_BUS_DEFAULT                  PX4_I2C_BUS_EXPANSION
#define PCF8583_DEVICE_PATH                  "/dev/pcf8583"
#define PCF8583_BASEADDR_DEFAULT             0x50

class PCF8583 : public device::I2C, public px4::ScheduledWorkItem
{
public:
	PCF8583(int bus = PCF8583_BUS_DEFAULT,
		int address = PCF8583_BASEADDR_DEFAULT);

	virtual        ~PCF8583();
	virtual int    init();
	void           print_info();

protected:
	virtual int    probe();

private:
	int            _pool_interval;
	float          _indicated_frequency;
	float          _estimated_accurancy;
	int            _count;
	int            _reset_count;
	int            _magnet_count;
	uint64_t       _lastmeasurement_time;
	orb_advert_t   _rpm_topic;

	virtual void   Run(); //Perform a poll cycle; overide for ScheduledWorkItem

	void           readSensorAndComputeFreqency();
	void           publish();
	int            getCounter();
	void           resetCounter();

	uint8_t        readRegister(uint8_t reg);
	void           setRegister(uint8_t reg, uint8_t value);

};


PCF8583::PCF8583(int bus, int address) :
	I2C("PCF8583", PCF8583_DEVICE_PATH, bus, address, 400000),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_pool_interval(0),
	_indicated_frequency(0.0),
	_estimated_accurancy(0.0),
	_count(0),
	_reset_count(0),
	_magnet_count(0),
	_rpm_topic(nullptr)
{
}

PCF8583::~PCF8583()
{
	ScheduleClear();

	if (_rpm_topic != nullptr) {
		orb_unadvertise(_rpm_topic);
	}
}

int
PCF8583::init()
{
	int ret = PX4_ERROR;

	//load parameters
	int address = PCF8583_BASEADDR_DEFAULT;

	if (param_find("PCF8583_ADDR") != PARAM_INVALID) {
		param_get(param_find("PCF8583_ADDR"), &address);
	}

	set_device_address(address);

	if (param_find("PCF8583_POOL") != PARAM_INVALID) {
		param_get(param_find("PCF8583_POOL"), &_pool_interval);
	}

	if (param_find("PCF8583_RESET") != PARAM_INVALID) {
		param_get(param_find("PCF8583_RESET"), &_reset_count);
	}

	if (param_find("PCF8583_MAGNET") != PARAM_INVALID) {
		param_get(param_find("PCF8583_MAGNET"), &_magnet_count);
	}

	PX4_INFO("addr: %d, pool: %d, reset: %d, magenet: %d", address, _pool_interval, _reset_count, _magnet_count);

	if (I2C::init() != OK) {
		return ret;
	}

	//set counter mode
	setRegister(0x00, 0b00100000);

	//start measurement
	resetCounter();
	_lastmeasurement_time = hrt_absolute_time();
	ScheduleOnInterval(_pool_interval);

	/* get a publish handle on the range finder topic */
	struct rpm_s rf_report = {};
	_rpm_topic = orb_advertise(ORB_ID(rpm), &rf_report);

	if (_rpm_topic == nullptr) {
		PX4_ERR("failed to create rotor_freqency object");
	}

	return PX4_OK;
}

int
PCF8583::getCounter()
{
	uint8_t a = readRegister(0x01);
	uint8_t b = readRegister(0x02);
	uint8_t c = readRegister(0x03);

	return int((a & 0x0f) * 1 + ((a & 0xf0) >> 4) * 10 + (b & 0x0f) * 100 + ((b & 0xf0) >> 4) * 1000 +
		   (c & 0x0f) * 10000 + ((c & 0xf0) >> 4) * 1000000);
}

void
PCF8583::resetCounter()
{
	setRegister(0x01, 0x00);
	setRegister(0x02, 0x00);
	setRegister(0x03, 0x00);
}

void
PCF8583::setRegister(uint8_t reg, uint8_t value)
{
	uint8_t buff[2];
	buff[0] = reg;
	buff[1] = value;
	int ret = transfer(buff, 2, nullptr, 0);

	if (OK != ret) {
		PX4_DEBUG("PCF8583::setRegister : i2c::transfer returned %d", ret);
	}
}

uint8_t
PCF8583::readRegister(uint8_t reg)
{
	uint8_t rcv;
	int ret = transfer(&reg, 1, &rcv, 1);

	if (OK != ret) {
		PX4_DEBUG("PCF8583::readRegister : i2c::transfer returned %d", ret);
	}

	return rcv;
}

int
PCF8583::probe()
{
	return PX4_OK;
}

void
PCF8583::readSensorAndComputeFreqency()
{

	int oldcount = _count;
	uint64_t oldtime = _lastmeasurement_time;

	_count = getCounter();
	_lastmeasurement_time = hrt_absolute_time();

	int diffCount = _count - oldcount;
	uint64_t diffTime = _lastmeasurement_time - oldtime;

	if (_reset_count < _count + diffCount) {
		resetCounter();
		_lastmeasurement_time = hrt_absolute_time();
		_count = 0;
	}

	_indicated_frequency = (float)diffCount / _magnet_count / ((float)diffTime / 1000000);
	_estimated_accurancy = 1 / (float)_magnet_count / ((float)diffTime / 1000000);
}

void PCF8583::publish()
{
	struct rpm_s msg;
	msg.timestamp = hrt_absolute_time();
	msg.indicated_frequency_hz = _indicated_frequency;
	msg.indicated_frequency_rpm = _indicated_frequency * 60;
	msg.estimated_accurancy_hz = _estimated_accurancy;
	msg.estimated_accurancy_rpm = _estimated_accurancy * 60;

	// publish it, if we are the primary
	if (_rpm_topic != nullptr) {
		orb_publish(ORB_ID(rpm), _rpm_topic, &msg);
	}

	// notify anyone waiting for data
	poll_notify(POLLIN);

}

void
PCF8583::Run()
{
	/*Collect results */
	readSensorAndComputeFreqency();
	publish();
}


void
PCF8583::print_info()
{
	printf("poll interval:  %d us\n", _pool_interval);
}

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int pcf8583_main(int argc, char *argv[]);

/**
 * Local functions in support of the shell command.
 */
namespace pcf8583
{

PCF8583    *g_dev;

int     start_bus(int i2c_bus);
int     stop();
int     info();

/**
 * Start the driver on a specific bus.
 *
 * This function only returns if the sensor is up and running
 * or could not be detected successfully.
 */
int
start_bus(int i2c_bus)
{

	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	/* create the driver */
	g_dev = new PCF8583(i2c_bus);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	PX4_INFO("pcf8583 for bus: %d started.", i2c_bus);
	return PX4_OK;

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return PX4_ERROR;
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

} /* namespace */


static void
pcf8583_usage()
{
	PX4_INFO("usage: pcf8583 command [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-b --bus i2cbus (%d)", PCF8583_BUS_DEFAULT);
	PX4_INFO("command:");
	PX4_INFO("\tstart|stop|info");
}

int
pcf8583_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	int i2c_bus = PCF8583_BUS_DEFAULT;

	while ((ch = px4_getopt(argc, argv, "b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

		default:
			PX4_WARN("Unknown option!");
			goto out_error;
		}
	}

	if (myoptind >= argc) {
		goto out_error;
	}

	if (!strcmp(argv[myoptind], "start")) {
		return pcf8583::start_bus(i2c_bus);
	}

	if (!strcmp(argv[myoptind], "stop")) {
		return pcf8583::stop();
	}

	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		return pcf8583::info();
	}

out_error:
	pcf8583_usage();
	return PX4_ERROR;
}
