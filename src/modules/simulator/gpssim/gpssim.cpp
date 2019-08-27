/****************************************************************************
 *
 *   Copyright (c) 2015 Roman Bapst. All rights reserved.
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
 * @file gpssim.cpp
 * Simulated GPS driver
 */

#include <sys/types.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <random>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <px4_platform_common/config.h>
#include <px4_platform_common/tasks.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/device.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/satellite_info.h>

#include <simulator/simulator.h>

#include "DevMgr.hpp"
#include "VirtDevObj.hpp"

using namespace DriverFramework;

#define GPS_DRIVER_MODE_UBX_SIM
#define GPSSIM_DEVICE_PATH "/dev/gpssim"

#define TIMEOUT_100MS 100000
#define RATE_MEASUREMENT_PERIOD 5000000

/* class for dynamic allocation of satellite info data */
class GPS_Sat_Info
{
public:
	struct satellite_info_s 	_data;
};


class GPSSIM : public VirtDevObj
{
public:
	GPSSIM(bool fake_gps, bool enable_sat_info,
	       int fix_type, int num_sat, int noise_multiplier);
	virtual ~GPSSIM();

	virtual int			init();

	virtual int			devIOCTL(unsigned long cmd, unsigned long arg);

	void set(int fix_type, int num_sat, int noise_multiplier);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void				print_info();

protected:
	virtual void			_measure() {}

private:

	bool				_task_should_exit;				///< flag to make the main worker task exit
	volatile int			_task;						///< worker task
	GPS_Sat_Info			*_Sat_Info;					///< instance of GPS sat info data object
	struct vehicle_gps_position_s	_report_gps_pos;				///< uORB topic for gps position
	orb_advert_t			_report_gps_pos_pub;				///< uORB pub for gps position
	struct satellite_info_s		*_p_report_sat_info;				///< pointer to uORB topic for satellite info
	orb_advert_t			_report_sat_info_pub;				///< uORB pub for satellite info
	SyncObj				_sync;
	int _fix_type;
	int _num_sat;
	int _noise_multiplier;

	std::default_random_engine _gen;

	/**
	 * Try to configure the GPS, handle outgoing communication to the GPS
	 */
	void			 	config();

	/**
	 * Trampoline to the worker task
	 */
	static int			task_main_trampoline(int argc, char *argv[]);


	/**
	 * Worker task: main GPS thread that configures the GPS and parses incoming data, always running
	 */
	void				task_main();

	/**
	 * Set the baudrate of the UART to the GPS
	 */
	int				set_baudrate(unsigned baud);

	/**
	 * Send a reset command to the GPS
	 */
	void				cmd_reset();

	int 				receive(int timeout);
};


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int gpssim_main(int argc, char *argv[]);

namespace
{

GPSSIM	*g_dev = nullptr;

}


GPSSIM::GPSSIM(bool fake_gps, bool enable_sat_info,
	       int fix_type, int num_sat, int noise_multiplier) :
	VirtDevObj("gps", GPSSIM_DEVICE_PATH, nullptr, 1e6 / 10),
	_task_should_exit(false),
	_Sat_Info(nullptr),
	_report_gps_pos{},
	_report_gps_pos_pub(nullptr),
	_p_report_sat_info(nullptr),
	_report_sat_info_pub(nullptr),
	_fix_type(fix_type),
	_num_sat(num_sat),
	_noise_multiplier(noise_multiplier)
{
	/* we need this potentially before it could be set in task_main */
	g_dev = this;
	_report_gps_pos.heading = NAN;
	_report_gps_pos.heading_offset = NAN;

	/* create satellite info data object if requested */
	if (enable_sat_info) {
		_Sat_Info = new (GPS_Sat_Info);
		_p_report_sat_info = &_Sat_Info->_data;
		memset(_p_report_sat_info, 0, sizeof(*_p_report_sat_info));
	}
}

GPSSIM::~GPSSIM()
{
	delete _Sat_Info;

	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		px4_usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1) {
		px4_task_delete(_task);
	}

	g_dev = nullptr;
}

int
GPSSIM::init()
{
	int ret = PX4_ERROR;

	/* do regular cdev init */
	if (VirtDevObj::init() != OK) {
		goto out;
	}

	/* start the GPS driver worker task */
	_task = px4_task_spawn_cmd("gpssim", SCHED_DEFAULT,
				   SCHED_PRIORITY_DEFAULT, 1500, (px4_main_t)&GPSSIM::task_main_trampoline, nullptr);

	if (_task < 0) {
		PX4_ERR("task start failed: %d", errno);
		return -errno;
	}

	ret = OK;
out:
	return ret;
}

int
GPSSIM::devIOCTL(unsigned long cmd, unsigned long arg)
{
	_sync.lock();

	int ret = OK;

	switch (cmd) {
	case SENSORIOCRESET:
		cmd_reset();
		break;

	default:
		/* give it to parent if no one wants it */
		ret = VirtDevObj::devIOCTL(cmd, arg);
		break;
	}

	_sync.unlock();

	return ret;
}

int
GPSSIM::task_main_trampoline(int argc, char *argv[])
{
	g_dev->task_main();
	return 0;
}

int
GPSSIM::receive(int timeout)
{
	Simulator *sim = Simulator::getInstance();
	simulator::RawGPSData gps;

	static uint64_t timestamp_last = 0;

	if (sim->getGPSSample((uint8_t *)&gps, sizeof(gps)) &&
	    (gps.timestamp != timestamp_last || timestamp_last == 0)) {
		_report_gps_pos.timestamp = hrt_absolute_time();
		_report_gps_pos.lat = gps.lat;
		_report_gps_pos.lon = gps.lon;
		_report_gps_pos.alt = gps.alt;
		_report_gps_pos.eph = (float)gps.eph * 1e-2f;
		_report_gps_pos.epv = (float)gps.epv * 1e-2f;
		_report_gps_pos.vel_m_s = (float)(gps.vel) / 100.0f;
		_report_gps_pos.vel_n_m_s = (float)(gps.vn) / 100.0f;
		_report_gps_pos.vel_e_m_s = (float)(gps.ve) / 100.0f;
		_report_gps_pos.vel_d_m_s = (float)(gps.vd) / 100.0f;
		_report_gps_pos.cog_rad = (float)(gps.cog) * 3.1415f / (100.0f * 180.0f);
		_report_gps_pos.fix_type = gps.fix_type;
		_report_gps_pos.satellites_used = gps.satellites_visible;
		_report_gps_pos.s_variance_m_s = 0.25f;

		timestamp_last = gps.timestamp;

		// check for data set by the user
		_report_gps_pos.fix_type = (_fix_type < 0) ? _report_gps_pos.fix_type : _fix_type;
		_report_gps_pos.satellites_used = (_num_sat < 0) ? _report_gps_pos.satellites_used : _num_sat;
		// use normal distribution for noise
		std::normal_distribution<float> normal_distribution(0.0f, 1.0f);
		_report_gps_pos.lat += (int32_t)(_noise_multiplier * normal_distribution(_gen));
		_report_gps_pos.lon += (int32_t)(_noise_multiplier * normal_distribution(_gen));

		return 1;

	} else {

		px4_usleep(timeout);
		return 0;
	}
}

void
GPSSIM::task_main()
{
	/* loop handling received serial bytes and also configuring in between */
	while (!_task_should_exit) {

		int recv_ret = receive(TIMEOUT_100MS);

		if (recv_ret > 0) {

			/* opportunistic publishing - else invalid data would end up on the bus */
			if (_report_gps_pos_pub != nullptr) {
				orb_publish(ORB_ID(vehicle_gps_position), _report_gps_pos_pub, &_report_gps_pos);

			} else {
				_report_gps_pos_pub = orb_advertise(ORB_ID(vehicle_gps_position), &_report_gps_pos);
			}

			if (_p_report_sat_info) {
				if (_report_sat_info_pub != nullptr) {
					orb_publish(ORB_ID(satellite_info), _report_sat_info_pub, _p_report_sat_info);

				} else {
					_report_sat_info_pub = orb_advertise(ORB_ID(satellite_info), _p_report_sat_info);
				}
			}
		}
	}

	if (_report_gps_pos_pub) {
		orb_unadvertise(_report_gps_pos_pub);
	}

	if (_report_sat_info_pub) {
		orb_unadvertise(_report_sat_info_pub);
	}

	PX4_INFO("exiting");

	/* tell the dtor that we are exiting */
	_task = -1;
}



void
GPSSIM::cmd_reset()
{
}

void
GPSSIM::print_info()
{
	//GPS Mode
	PX4_INFO("protocol: SIM");

	PX4_INFO("sat info: %s, noise: %d, jamming detected: %s",
		 (_p_report_sat_info != nullptr) ? "enabled" : "disabled",
		 _report_gps_pos.noise_per_ms,
		 _report_gps_pos.jamming_indicator == 255 ? "YES" : "NO");

	if (_report_gps_pos.timestamp != 0) {
		print_message(_report_gps_pos);
	}

	px4_usleep(100000);
}

void
GPSSIM::set(int fix_type, int num_sat, int noise_multiplier)
{
	_fix_type = fix_type;
	_num_sat = num_sat;
	_noise_multiplier = noise_multiplier;
	PX4_INFO("Parameters set");
}

/**
 * Local functions in support of the shell command.
 */
namespace gpssim
{

GPSSIM	*g_dev = nullptr;

void	start(bool fake_gps, bool enable_sat_info,
	      int fix_type, int num_sat, int noise_multiplier);
void	stop();
void	test();
void	reset();
void	info();
void	usage(const char *reason);

/**
 * Start the driver.
 */
void
start(bool fake_gps, bool enable_sat_info, int fix_type, int num_sat, int noise_multiplier)
{
	DevHandle h;

	/* create the driver */
	g_dev = new GPSSIM(fake_gps, enable_sat_info, fix_type, num_sat, noise_multiplier);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	DevMgr::getHandle(GPSSIM_DEVICE_PATH, h);

	if (!h.isValid()) {
		PX4_ERR("getHandle failed: %s", GPSSIM_DEVICE_PATH);
		goto fail;
	}

	return;

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	PX4_ERR("start failed");
}

/**
 * Stop the driver.
 */
void
stop()
{
	delete g_dev;
	g_dev = nullptr;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	PX4_INFO("PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	DevHandle h;
	DevMgr::getHandle(GPSSIM_DEVICE_PATH, h);

	if (!h.isValid()) {
		PX4_ERR("failed ");
	}

	if (h.ioctl(SENSORIOCRESET, 0) < 0) {
		PX4_ERR("reset failed");
	}
}

/**
 * Print the status of the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("gpssim not running");
		return;
	}

	g_dev->print_info();
}

void
usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PX4_INFO("usage:");
	PX4_INFO("gpssim {start|stop|test|reset|status|set}");
	PX4_INFO("       [-d /dev/ttyS0-n][-f (for enabling fake)][-s (to enable sat info)]");
	PX4_INFO("       [-t # (for setting fix_type)][-n # (for setting # satellites)][-m # (for setting noise multiplier [scalar] for normal distribution)]");
}

} // namespace



int
gpssim_main(int argc, char *argv[])
{
	// set to default
	bool fake_gps = false;
	bool enable_sat_info = false;
	int fix_type = -1;
	int num_sat = -1;
	int noise_multiplier = 0;

	// check for optional arguments
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "fst:n:m:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'f':
			fake_gps = true;
			PX4_INFO("Using fake GPS");
			break;

		case 's':
			enable_sat_info = true;
			PX4_INFO("Satellite info enabled");
			break;

		case 't':
			fix_type = atoi(myoptarg);
			PX4_INFO("Setting fix_type to %d", fix_type);
			break;

		case 'n':
			num_sat = atoi(myoptarg);
			PX4_INFO("Setting number of satellites to %d", num_sat);
			break;

		case 'm':
			noise_multiplier = atoi(myoptarg);
			PX4_INFO("Setting noise multiplier to %d", noise_multiplier);
			break;

		default:
			PX4_WARN("Unknown option!");
		}
	}

	if (myoptind >= argc) {
		gpssim::usage("not enough arguments supplied");
		return 1;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		if (g_dev != nullptr) {
			PX4_WARN("already started");
			return 0;
		}

		gpssim::start(fake_gps, enable_sat_info, fix_type, num_sat, noise_multiplier);
		return 0;
	}

	/* The following need gpssim running. */
	if (g_dev == nullptr) {
		PX4_WARN("not running");
		return 1;
	}

	if (!strcmp(argv[myoptind], "stop")) {
		gpssim::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[myoptind], "test")) {
		gpssim::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[myoptind], "reset")) {
		gpssim::reset();
	}

	/*
	 * Print driver status.
	 */
	if (!strcmp(argv[myoptind], "status")) {
		gpssim::info();
	}

	/*
	 * Set parameters
	 */
	if (!strcmp(argv[myoptind], "set")) {
		g_dev->set(fix_type, num_sat, noise_multiplier);
	}

	return 0;
}
