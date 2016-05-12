/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file gps.cpp
 * Driver for the GPS on a serial port
 */

#ifdef __PX4_NUTTX
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#endif


#ifndef __PX4_QURT
#include <termios.h>
#include <poll.h>
#else
#include <sys/ioctl.h>
#include <dev_fs_lib_serial.h>
#endif


#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <px4_config.h>
#include <px4_time.h>
#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <systemlib/systemlib.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <drivers/drv_gps.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/satellite_info.h>
#include <uORB/topics/gps_inject_data.h>

#include <board_config.h>

#include "devices/src/ubx.h"
#include "devices/src/mtk.h"
#include "devices/src/ashtech.h"


#define TIMEOUT_5HZ 500
#define RATE_MEASUREMENT_PERIOD 5000000
#define GPS_WAIT_BEFORE_READ	20		// ms, wait before reading to save read() calls


/* class for dynamic allocation of satellite info data */
class GPS_Sat_Info
{
public:
	struct satellite_info_s 	_data;
};


class GPS
{
public:
	GPS(const char *uart_path, bool fake_gps, bool enable_sat_info, int gps_num);
	virtual ~GPS();

	virtual int			init();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void				print_info();

private:

	bool				_task_should_exit;				///< flag to make the main worker task exit
	int				_serial_fd;					///< serial interface to GPS
	unsigned			_baudrate;					///< current baudrate
	char				_port[20];					///< device / serial port path
	volatile int			_task;						///< worker task
	bool				_healthy;					///< flag to signal if the GPS is ok
	bool				_baudrate_changed;				///< flag to signal that the baudrate with the GPS has changed
	bool				_mode_changed;					///< flag that the GPS mode has changed
	gps_driver_mode_t		_mode;						///< current mode
	GPSHelper			*_helper;					///< instance of GPS parser
	GPS_Sat_Info			*_sat_info;					///< instance of GPS sat info data object
	struct vehicle_gps_position_s	_report_gps_pos;				///< uORB topic for gps position
	orb_advert_t			_report_gps_pos_pub;				///< uORB pub for gps position
	int					_gps_orb_instance;				///< uORB multi-topic instance
	struct satellite_info_s		*_p_report_sat_info;				///< pointer to uORB topic for satellite info
	int					_gps_sat_orb_instance;				///< uORB multi-topic instance for satellite info
	orb_advert_t			_report_sat_info_pub;				///< uORB pub for satellite info
	float				_rate;						///< position update rate
	float				_rate_rtcm_injection;				///< RTCM message injection rate
	unsigned			_last_rate_rtcm_injection_count; 		///< counter for number of RTCM messages
	bool				_fake_gps;					///< fake gps output
	int 				_gps_num;					///< number of GPS connected

	static const int _orb_inject_data_fd_count = 4;
	int _orb_inject_data_fd[_orb_inject_data_fd_count];
	int _orb_inject_data_next = 0;

	// dump communication to file
#ifdef __PX4_POSIX_EAGLE
	static constexpr const char	*DUMP_ROOT = PX4_ROOTFSDIR;
#else
	static constexpr const char 	*DUMP_ROOT = PX4_ROOTFSDIR"/fs/microsd";
#endif
	int _dump_to_gps_device_fd = -1;              ///< file descriptors, to safe all device communication to a file
	int _dump_from_gps_device_fd = -1;
	int _vehicle_status_sub = -1;
	bool _is_armed = false;                       ///< current arming state (only updated if communication dump is enabled)

	/**
	 * Try to configure the GPS, handle outgoing communication to the GPS
	 */
	void config();

	/**
	 * Trampoline to the worker task
	 */
	static void task_main_trampoline(int argc, char *argv[]);

	/**
	 * Worker task: main GPS thread that configures the GPS and parses incoming data, always running
	 */
	void task_main(void);

	/**
	 * Set the baudrate of the UART to the GPS
	 */
	int set_baudrate(unsigned baud);

	/**
	 * Send a reset command to the GPS
	 */
	void cmd_reset();

	/**
	 * Publish the gps struct
	 */
	void 				publish();

	/**
	 * Publish the satellite info
	 */
	void 				publishSatelliteInfo();

	/**
	 * This is an abstraction for the poll on serial used.
	 *
	 * @param buf: pointer to read buffer
	 * @param buf_length: size of read buffer
	 * @param timeout: timeout in ms
	 * @return: 0 for nothing read, or poll timed out
	 *	    < 0 for error
	 *	    > 0 number of bytes read
	 */
	int pollOrRead(uint8_t *buf, size_t buf_length, int timeout);

	/**
	 * check for new messages on the inject data topic & handle them
	 */
	void handleInjectDataTopic();

	/**
	 * send data to the device, such as an RTCM stream
	 * @param data
	 * @param len
	 */
	inline bool injectData(uint8_t *data, size_t len);

	/**
	 * set the Baudrate
	 * @param baud
	 * @return 0 on success, <0 on error
	 */
	int setBaudrate(unsigned baud);

	/**
	 * callback from the driver for the platform specific stuff
	 */
	static int callback(GPSCallbackType type, void *data1, int data2, void *user);

	/**
	 * Setup dumping of the GPS device input and output stream to a file.
	 */
	void initializeCommunicationDump();

	int getDumpFileName(char *file_name, size_t file_name_size, const char *suffix);

	static bool fileExist(const char *filename);
};


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int gps_main(int argc, char *argv[]);

namespace
{

GPS	*g_dev[2] = {nullptr, nullptr};
volatile bool is_gps1_advertised = false; ///< for the second gps we want to make sure that it gets instance 1
/// and thus we wait until the first one publishes at least one message.
}


GPS::GPS(const char *uart_path, bool fake_gps, bool enable_sat_info, int gps_num) :
	_task_should_exit(false),
	_healthy(false),
	_mode_changed(false),
	_mode(GPS_DRIVER_MODE_UBX),
	_helper(nullptr),
	_sat_info(nullptr),
	_report_gps_pos_pub{nullptr},
	_gps_orb_instance(-1),
	_p_report_sat_info(nullptr),
	_report_sat_info_pub(nullptr),
	_rate(0.0f),
	_rate_rtcm_injection(0.0f),
	_last_rate_rtcm_injection_count(0),
	_fake_gps(fake_gps),
	_gps_num(gps_num)
{
	/* store port name */
	strncpy(_port, uart_path, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	memset(&_report_gps_pos, 0, sizeof(_report_gps_pos));

	/* create satellite info data object if requested */
	if (enable_sat_info) {
		_sat_info = new GPS_Sat_Info();
		_p_report_sat_info = &_sat_info->_data;
		memset(_p_report_sat_info, 0, sizeof(*_p_report_sat_info));
	}

	for (int i = 0; i < _orb_inject_data_fd_count; ++i) {
		_orb_inject_data_fd[i] = -1;
	}
}

GPS::~GPS()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1) {
		px4_task_delete(_task);
	}

	if (_sat_info) {
		delete(_sat_info);
	}


}

int GPS::init()
{

	char gps_num[2] = {(char)('0' + _gps_num), 0};
	char *const args[2] = { gps_num, NULL };

	/* start the GPS driver worker task */
	_task = px4_task_spawn_cmd("gps", SCHED_DEFAULT,
				   SCHED_PRIORITY_SLOW_DRIVER, 1200, (px4_main_t)&GPS::task_main_trampoline, args);

	if (_task < 0) {
		PX4_WARN("task start failed: %d", errno);
		_task = -1;
		return -errno;
	}

	return OK;
}

void GPS::task_main_trampoline(int argc, char *argv[])
{
	g_dev[argv[argc - 1][0] - '1']->task_main();
}

int GPS::callback(GPSCallbackType type, void *data1, int data2, void *user)
{
	GPS *gps = (GPS *)user;

	switch (type) {
	case GPSCallbackType::readDeviceData: {
			int num_read = gps->pollOrRead((uint8_t *)data1, data2, *((int *)data1));

			if (num_read > 0 && gps->_dump_from_gps_device_fd >= 0) {
				if (write(gps->_dump_from_gps_device_fd, data1, (size_t)num_read) != (size_t)num_read) {
					PX4_WARN("gps dump failed");
				}
			}

			return num_read;
		}

	case GPSCallbackType::writeDeviceData:
		if (gps->_dump_to_gps_device_fd >= 0) {
			if (write(gps->_dump_to_gps_device_fd, data1, (size_t)data2) != (size_t)data2) {
				PX4_WARN("gps dump failed");
			}
		}

		return write(gps->_serial_fd, data1, (size_t)data2);

	case GPSCallbackType::setBaudrate:
		return gps->setBaudrate(data2);

	case GPSCallbackType::gotRTCMMessage:
		/* not used */
		break;

	case GPSCallbackType::surveyInStatus:
		/* not used */
		break;

	case GPSCallbackType::setClock:
		px4_clock_settime(CLOCK_REALTIME, (timespec *)data1);
		break;
	}

	return 0;
}

int GPS::pollOrRead(uint8_t *buf, size_t buf_length, int timeout)
{
	handleInjectDataTopic();

#ifndef __PX4_QURT

	/* For non QURT, use the usual polling. */

	//Poll only for the serial data. In the same thread we also need to handle orb messages,
	//so ideally we would poll on both, the serial fd and orb subscription. Unfortunately the
	//two pollings use different underlying mechanisms (at least under posix), which makes this
	//impossible. Instead we limit the maximum polling interval and regularly check for new orb
	//messages.
	//FIXME: add a unified poll() API
	const int max_timeout = 50;

	pollfd fds[1];
	fds[0].fd = _serial_fd;
	fds[0].events = POLLIN;

	int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), math::min(max_timeout, timeout));

	if (ret > 0) {
		/* if we have new data from GPS, go handle it */
		if (fds[0].revents & POLLIN) {
			/*
			 * We are here because poll says there is some data, so this
			 * won't block even on a blocking device. But don't read immediately
			 * by 1-2 bytes, wait for some more data to save expensive read() calls.
			 * If more bytes are available, we'll go back to poll() again.
			 */
			usleep(GPS_WAIT_BEFORE_READ * 1000);
			ret = ::read(_serial_fd, buf, buf_length);

		} else {
			ret = -1;
		}
	}

	return ret;

#else
	/* For QURT, just use read for now, since this doesn't block, we need to slow it down
	 * just a bit. */
	usleep(10000);
	return ::read(_serial_fd, buf, buf_length);
#endif
}

void GPS::handleInjectDataTopic()
{
	if (_orb_inject_data_fd[0] == -1) {
		return;
	}

	bool updated = false;

	do {
		int orb_inject_data_cur_fd = _orb_inject_data_fd[_orb_inject_data_next];
		orb_check(orb_inject_data_cur_fd, &updated);

		if (updated) {
			struct gps_inject_data_s msg;
			orb_copy(ORB_ID(gps_inject_data), orb_inject_data_cur_fd, &msg);
			injectData(msg.data, msg.len);

			_orb_inject_data_next = (_orb_inject_data_next + 1) % _orb_inject_data_fd_count;

			++_last_rate_rtcm_injection_count;
		}
	} while (updated);
}

bool GPS::injectData(uint8_t *data, size_t len)
{
	return ::write(_serial_fd, data, len) == len;
}

int GPS::setBaudrate(unsigned baud)
{

#if __PX4_QURT
	// TODO: currently QURT does not support configuration with termios.
	dspal_serial_ioctl_data_rate data_rate;

	switch (baud) {
	case 9600: data_rate.bit_rate = DSPAL_SIO_BITRATE_9600; break;

	case 19200: data_rate.bit_rate = DSPAL_SIO_BITRATE_19200; break;

	case 38400: data_rate.bit_rate = DSPAL_SIO_BITRATE_38400; break;

	case 57600: data_rate.bit_rate = DSPAL_SIO_BITRATE_57600; break;

	case 115200: data_rate.bit_rate = DSPAL_SIO_BITRATE_115200; break;

	default:
		PX4_ERR("ERR: unknown baudrate: %d", baud);
		return -EINVAL;
	}

	int ret = ::ioctl(_serial_fd, SERIAL_IOCTL_SET_DATA_RATE, (void *)&data_rate);

	if (ret != 0) {

		return ret;
	}

#else
	/* process baud rate */
	int speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	default:
		PX4_ERR("ERR: unknown baudrate: %d", baud);
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_serial_fd, &uart_config);

	/* properly configure the terminal (see also https://en.wikibooks.org/wiki/Serial_Programming/termios ) */

	//
	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
				 INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	uart_config.c_oflag = 0;

	//
	// No line processing
	//
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		GPS_ERR("ERR: %d (cfsetispeed)", termios_state);
		return -1;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		GPS_ERR("ERR: %d (cfsetospeed)", termios_state);
		return -1;
	}

	if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
		GPS_ERR("ERR: %d (tcsetattr)", termios_state);
		return -1;
	}

#endif
	return 0;
}

bool GPS::fileExist(const char *filename)
{
	struct stat buffer;
	return stat(filename, &buffer) == 0;
}

void GPS::initializeCommunicationDump()
{
	param_t gps_dump_comm_ph = param_find("GPS_DUMP_COMM");
	int32_t param_dump_comm;

	if (gps_dump_comm_ph == PARAM_INVALID || param_get(gps_dump_comm_ph, &param_dump_comm) != 0) {
		return;
	}

	if (param_dump_comm != 1) {
		return; //dumping disabled
	}

	char to_device_file_name[128] = "";
	char from_device_file_name[128] = "";

	if (getDumpFileName(to_device_file_name, sizeof(to_device_file_name), "to")
	    || getDumpFileName(from_device_file_name, sizeof(from_device_file_name), "from")) {
		PX4_ERR("Failed to get GPS dump file name");
		return;
	}

	//open files
	if ((_dump_to_gps_device_fd = open(to_device_file_name, O_CREAT | O_WRONLY, PX4_O_MODE_666)) < 0) {
		return;
	}

	if ((_dump_from_gps_device_fd = open(from_device_file_name, O_CREAT | O_WRONLY, PX4_O_MODE_666)) < 0) {
		close(_dump_to_gps_device_fd);
		_dump_to_gps_device_fd = -1;
		return;
	}

	PX4_INFO("Dumping GPS comm to files %s, %s", to_device_file_name, from_device_file_name);

	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
}

int GPS::getDumpFileName(char *file_name, size_t file_name_size, const char *suffix)
{
	uint16_t file_number = 1; // start with file gps_dump_<num>_<suffix>_dev_001.dat

	const uint16_t max_num_files = 999;

	while (file_number <= max_num_files) {
		snprintf(file_name, file_name_size, "%s/gps_dump_%u_%s_dev_%03u.dat",
			 DUMP_ROOT, _gps_num, suffix, file_number);

		if (!fileExist(file_name)) {
			break;
		}

		file_number++;
	}

	if (file_number > max_num_files) {
		return -1;
	}

	return 0;
}

void
GPS::task_main()
{
	/* open the serial port */
	_serial_fd = ::open(_port, O_RDWR | O_NOCTTY);

	if (_serial_fd < 0) {
		PX4_ERR("GPS: failed to open serial port: %s err: %d", _port, errno);

		/* tell the dtor that we are exiting, set error code */
		_task = -1;
		px4_task_exit(1);
	}

#ifndef __PX4_QURT
	// TODO: this call is not supported on Snapdragon just yet.
	// However it seems to be nonblocking anyway and working.
	int flags = fcntl(_serial_fd, F_GETFL, 0);
	fcntl(_serial_fd, F_SETFL, flags | O_NONBLOCK);
#endif

	for (int i = 0; i < _orb_inject_data_fd_count; ++i) {
		_orb_inject_data_fd[i] = orb_subscribe_multi(ORB_ID(gps_inject_data), i);
	}

	initializeCommunicationDump();

	uint64_t last_rate_measurement = hrt_absolute_time();
	unsigned last_rate_count = 0;

	/* loop handling received serial bytes and also configuring in between */
	while (!_task_should_exit) {

		if (_fake_gps) {
			_report_gps_pos.timestamp_position = hrt_absolute_time();
			_report_gps_pos.lat = (int32_t)47.378301e7f;
			_report_gps_pos.lon = (int32_t)8.538777e7f;
			_report_gps_pos.alt = (int32_t)1200e3f;
			_report_gps_pos.timestamp_variance = hrt_absolute_time();
			_report_gps_pos.s_variance_m_s = 10.0f;
			_report_gps_pos.c_variance_rad = 0.1f;
			_report_gps_pos.fix_type = 3;
			_report_gps_pos.eph = 0.9f;
			_report_gps_pos.epv = 1.8f;
			_report_gps_pos.timestamp_velocity = hrt_absolute_time();
			_report_gps_pos.vel_n_m_s = 0.0f;
			_report_gps_pos.vel_e_m_s = 0.0f;
			_report_gps_pos.vel_d_m_s = 0.0f;
			_report_gps_pos.vel_m_s = sqrtf(_report_gps_pos.vel_n_m_s * _report_gps_pos.vel_n_m_s + _report_gps_pos.vel_e_m_s *
							_report_gps_pos.vel_e_m_s + _report_gps_pos.vel_d_m_s * _report_gps_pos.vel_d_m_s);
			_report_gps_pos.cog_rad = 0.0f;
			_report_gps_pos.vel_ned_valid = true;

			/* no time and satellite information simulated */


			publish();

			usleep(2e5);

		} else {

			if (_helper != nullptr) {
				delete(_helper);
				/* set to zero to ensure parser is not used while not instantiated */
				_helper = nullptr;
			}

			switch (_mode) {
			case GPS_DRIVER_MODE_UBX:
				_helper = new GPSDriverUBX(&GPS::callback, this, &_report_gps_pos, _p_report_sat_info);
				break;

			case GPS_DRIVER_MODE_MTK:
				_helper = new GPSDriverMTK(&GPS::callback, this, &_report_gps_pos);
				break;

			case GPS_DRIVER_MODE_ASHTECH:
				_helper = new GPSDriverAshtech(&GPS::callback, this, &_report_gps_pos, _p_report_sat_info);
				break;

			default:
				break;
			}


			/* the Ashtech driver lies about successful configuration and the
			 * MTK driver is not well tested, so we really only trust the UBX
			 * driver for an advance publication
			 */
			if (_helper->configure(_baudrate, GPSHelper::OutputMode::GPS) == 0) {

				/* reset report */
				memset(&_report_gps_pos, 0, sizeof(_report_gps_pos));

				if (_mode == GPS_DRIVER_MODE_UBX) {
					/* Publish initial report that we have access to a GPS,
					 * but set all critical state fields to indicate we have
					 * no valid position lock
					 */

					_report_gps_pos.timestamp_time = hrt_absolute_time();

					/* reset the timestamps for data, because we have no data yet */
					_report_gps_pos.timestamp_position = 0;
					_report_gps_pos.timestamp_variance = 0;
					_report_gps_pos.timestamp_velocity = 0;

					/* set a massive variance */
					_report_gps_pos.eph = 10000.0f;
					_report_gps_pos.epv = 10000.0f;
					_report_gps_pos.fix_type = 0;

					publish();

					/* GPS is obviously detected successfully, reset statistics */
					_helper->resetUpdateRates();
				}

				int helper_ret;

				while ((helper_ret = _helper->receive(TIMEOUT_5HZ)) > 0 && !_task_should_exit) {

					if (helper_ret & 1) {
						publish();

						last_rate_count++;
					}

					if (_p_report_sat_info && (helper_ret & 2)) {
						publishSatelliteInfo();
					}

					/* measure update rate every 5 seconds */
					if (hrt_absolute_time() - last_rate_measurement > RATE_MEASUREMENT_PERIOD) {
						float dt = (float)((hrt_absolute_time() - last_rate_measurement)) / 1000000.0f;
						_rate = last_rate_count / dt;
						_rate_rtcm_injection = _last_rate_rtcm_injection_count / dt;
						last_rate_measurement = hrt_absolute_time();
						last_rate_count = 0;
						_last_rate_rtcm_injection_count = 0;
						_helper->storeUpdateRates();
						_helper->resetUpdateRates();
					}

					if (!_healthy) {
						// Helpful for debugging, but too verbose for normal ops
//						const char *mode_str = "unknown";
//
//						switch (_mode) {
//						case GPS_DRIVER_MODE_UBX:
//							mode_str = "UBX";
//							break;
//
//						case GPS_DRIVER_MODE_MTK:
//							mode_str = "MTK";
//							break;
//
//						case GPS_DRIVER_MODE_ASHTECH:
//							mode_str = "ASHTECH";
//							break;
//
//						default:
//							break;
//						}
//
//						PX4_WARN("module found: %s", mode_str);
						_healthy = true;
					}

					//check for disarming
					if (_vehicle_status_sub != -1 && _dump_from_gps_device_fd != -1) {
						bool updated;
						orb_check(_vehicle_status_sub, &updated);

						if (updated) {
							vehicle_status_s vehicle_status;
							orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &vehicle_status);
							bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ||
								     (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR);

							if (armed) {
								_is_armed = true;

							} else if (_is_armed) {
								//disable communication dump when disarming
								close(_dump_from_gps_device_fd);
								_dump_from_gps_device_fd = -1;
								close(_dump_to_gps_device_fd);
								_dump_to_gps_device_fd = -1;
								_is_armed = false;
							}

						}
					}
				}

				if (_healthy) {
					PX4_WARN("GPS module lost");
					_healthy = false;
					_rate = 0.0f;
					_rate_rtcm_injection = 0.0f;
				}
			}

			/* select next mode */
			switch (_mode) {
			case GPS_DRIVER_MODE_UBX:
				_mode = GPS_DRIVER_MODE_MTK;
				break;

			case GPS_DRIVER_MODE_MTK:
				_mode = GPS_DRIVER_MODE_ASHTECH;
				break;

			case GPS_DRIVER_MODE_ASHTECH:
				_mode = GPS_DRIVER_MODE_UBX;
				break;

			default:
				break;
			}
		}

	}

	PX4_WARN("exiting");

	for (size_t i = 0; i < _orb_inject_data_fd_count; ++i) {
		orb_unsubscribe(_orb_inject_data_fd[i]);
		_orb_inject_data_fd[i] = -1;
	}

	if (_dump_to_gps_device_fd != -1) {
		close(_dump_to_gps_device_fd);
		_dump_to_gps_device_fd = -1;
	}

	if (_dump_from_gps_device_fd != -1) {
		close(_dump_from_gps_device_fd);
		_dump_from_gps_device_fd = -1;
	}

	if (_vehicle_status_sub != -1) {
		orb_unsubscribe(_vehicle_status_sub);
		_vehicle_status_sub = -1;
	}

	::close(_serial_fd);

	orb_unadvertise(_report_gps_pos_pub);

	/* tell the dtor that we are exiting */
	_task = -1;
	px4_task_exit(0);
}



void
GPS::cmd_reset()
{
#ifdef GPIO_GPS_NRESET
	PX4_WARN("Toggling GPS reset pin");
	stm32_configgpio(GPIO_GPS_NRESET);
	stm32_gpiowrite(GPIO_GPS_NRESET, 0);
	usleep(100);
	stm32_gpiowrite(GPIO_GPS_NRESET, 1);
	PX4_WARN("Toggled GPS reset pin");
#endif
}

void
GPS::print_info()
{
	PX4_WARN("GPS %i:", _gps_num);

	//GPS Mode
	if (_fake_gps) {
		PX4_WARN("protocol: SIMULATED");
	}

	else {
		switch (_mode) {
		case GPS_DRIVER_MODE_UBX:
			PX4_WARN("protocol: UBX");
			break;

		case GPS_DRIVER_MODE_MTK:
			PX4_WARN("protocol: MTK");
			break;

		case GPS_DRIVER_MODE_ASHTECH:
			PX4_WARN("protocol: ASHTECH");
			break;

		default:
			break;
		}
	}

	PX4_WARN("port: %s, baudrate: %d, status: %s", _port, _baudrate, (_healthy) ? "OK" : "NOT OK");
	PX4_WARN("sat info: %s, noise: %d, jamming detected: %s",
		 (_p_report_sat_info != nullptr) ? "enabled" : "disabled",
		 _report_gps_pos.noise_per_ms,
		 _report_gps_pos.jamming_indicator == 255 ? "YES" : "NO");

	if (_report_gps_pos.timestamp_position != 0) {
		PX4_WARN("position lock: %d, satellites: %d, last update: %8.4fms ago", (int)_report_gps_pos.fix_type,
			 _report_gps_pos.satellites_used, (double)(hrt_absolute_time() - _report_gps_pos.timestamp_position) / 1000.0);
		PX4_WARN("lat: %d, lon: %d, alt: %d", _report_gps_pos.lat, _report_gps_pos.lon, _report_gps_pos.alt);
		PX4_WARN("vel: %.2fm/s, %.2fm/s, %.2fm/s", (double)_report_gps_pos.vel_n_m_s,
			 (double)_report_gps_pos.vel_e_m_s, (double)_report_gps_pos.vel_d_m_s);
		PX4_WARN("hdop: %.2f, vdop: %.2f", (double)_report_gps_pos.hdop, (double)_report_gps_pos.vdop);
		PX4_WARN("eph: %.2fm, epv: %.2fm", (double)_report_gps_pos.eph, (double)_report_gps_pos.epv);
		PX4_WARN("rate position: \t\t%6.2f Hz", (double)_helper->getPositionUpdateRate());
		PX4_WARN("rate velocity: \t\t%6.2f Hz", (double)_helper->getVelocityUpdateRate());
		PX4_WARN("rate publication:\t\t%6.2f Hz", (double)_rate);
		PX4_WARN("rate RTCM injection:\t%6.2f Hz", (double)_rate_rtcm_injection);

	}

	usleep(100000);
}

void
GPS::publish()
{
	if (_gps_num == 1) {
		orb_publish_auto(ORB_ID(vehicle_gps_position), &_report_gps_pos_pub, &_report_gps_pos, &_gps_orb_instance,
				 ORB_PRIO_DEFAULT);
		is_gps1_advertised = true;

	} else if (is_gps1_advertised) {
		orb_publish_auto(ORB_ID(vehicle_gps_position), &_report_gps_pos_pub, &_report_gps_pos, &_gps_orb_instance,
				 ORB_PRIO_DEFAULT);
	}

}
void
GPS::publishSatelliteInfo()
{
	if (_gps_num == 1) {
		orb_publish_auto(ORB_ID(satellite_info), &_report_sat_info_pub, _p_report_sat_info, &_gps_sat_orb_instance,
				 ORB_PRIO_DEFAULT);

	} else {
		//we don't publish satellite info for the secondary gps
	}

}

/**
 * Local functions in support of the shell command.
 */
namespace gps
{


void	start(const char *uart_path, bool fake_gps, bool enable_sat_info, int gps_num);
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start(const char *uart_path, bool fake_gps, bool enable_sat_info, int gps_num)
{
	if (g_dev[gps_num - 1] != nullptr) {
		PX4_WARN("GPS %i already started", gps_num);
		return;
	}

	/* create the driver */
	g_dev[gps_num - 1] = new GPS(uart_path, fake_gps, enable_sat_info, gps_num);

	if (!g_dev[gps_num - 1] || OK != g_dev[gps_num - 1]->init()) {
		if (g_dev[gps_num - 1] != nullptr) {
			delete g_dev[gps_num - 1];
			g_dev[gps_num - 1] = nullptr;
		}

		PX4_ERR("start of GPS %i failed", gps_num);
	}
}

/**
 * Stop the driver.
 */
void
stop()
{
	delete g_dev[0];
	g_dev[0] = nullptr;

	if (g_dev[1] != nullptr) {
		delete g_dev[1];
	}

	g_dev[1] = nullptr;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	PX4_ERR("GPS reset not supported");
	return;
}

/**
 * Print the status of the driver.
 */
void
info()
{
	if (g_dev[0] == nullptr) {
		PX4_ERR("GPS Not running");
		return;
	}

	g_dev[0]->print_info();

	if (g_dev[1] != nullptr) {
		g_dev[1]->print_info();
	}

	return;
}

} // namespace


int
gps_main(int argc, char *argv[])
{
	/* set to default */
	const char *device_name = GPS_DEFAULT_UART_PORT;
	const char *device_name2 = nullptr;
	bool fake_gps = false;
	bool enable_sat_info = false;

	if (argc < 2) {
		goto out;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		/* work around getopt unreliability */
		if (argc > 3) {
			if (!strcmp(argv[2], "-d")) {
				device_name = argv[3];

			} else {
				PX4_ERR("DID NOT GET -d");
				goto out;
			}
		}

		/* Detect fake gps option */
		for (int i = 2; i < argc; i++) {
			if (!strcmp(argv[i], "-f")) {
				fake_gps = true;
			}
		}

		/* Detect sat info option */
		for (int i = 2; i < argc; i++) {
			if (!strcmp(argv[i], "-s")) {
				enable_sat_info = true;
			}
		}

		/* Allow to use a second gps device */
		for (int i = 2; i < argc; i++) {
			if (!strcmp(argv[i], "-dualgps")) {
				if (argc > i + 1) {
					device_name2 = argv[i + 1];

				} else {
					PX4_ERR("Did not get second device address");
				}
			}
		}

		gps::start(device_name, fake_gps, enable_sat_info, 1);

		if (device_name2) {
			gps::start(device_name2, fake_gps, enable_sat_info, 2);
		}

	}

	if (!strcmp(argv[1], "stop")) {
		gps::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		gps::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		gps::reset();
	}

	/*
	 * Print driver status.
	 */
	if (!strcmp(argv[1], "status")) {
		gps::info();
	}

	return 0;

out:
	PX4_ERR("unrecognized command, try 'start', 'stop', 'test', 'reset' or 'status'\n [-d /dev/ttyS0-n][-f (for enabling fake)][-s (to enable sat info)]");
	return 1;
}
