/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * Driver for the GPS on a serial/spi port
 */

#ifdef __PX4_NUTTX
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#endif

#ifndef __PX4_QURT
#include <poll.h>
#endif

#include <termios.h>
#include <cstring>

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/gps_dump.h>
#include <uORB/topics/gps_inject_data.h>
#include <uORB/topics/sensor_gps.h>

#ifndef CONSTRAINED_FLASH
# include "devices/src/ashtech.h"
# include "devices/src/emlid_reach.h"
# include "devices/src/mtk.h"
# include "devices/src/femtomes.h"
# include "devices/src/nmea.h"
#endif // CONSTRAINED_FLASH
#include "devices/src/ubx.h"

#ifdef __PX4_LINUX
#include <linux/spi/spidev.h>
#endif /* __PX4_LINUX */

#define TIMEOUT_1HZ		1300	//!< Timeout time in mS, 1000 mS (1Hz) + 300 mS delta for error
#define TIMEOUT_5HZ		500		//!< Timeout time in mS,  200 mS (5Hz) + 300 mS delta for error
#define RATE_MEASUREMENT_PERIOD 5000000

enum class gps_driver_mode_t {
	None = 0,
	UBX,
	MTK,
	ASHTECH,
	EMLIDREACH,
	FEMTOMES,
	NMEA
};

enum class gps_dump_comm_mode_t : int32_t {
	Disabled = 0,
	Full, ///< dump full RX and TX data for all devices
	RTCM ///< dump received RTCM from Main GPS
};

/* struct for dynamic allocation of satellite info data */
struct GPS_Sat_Info {
	satellite_info_s _data;
};

static constexpr int TASK_STACK_SIZE = PX4_STACK_ADJUSTED(1760);


class GPS : public ModuleBase<GPS>, public device::Device
{
public:

	/** The GPS allows to run multiple instances */
	enum class Instance : uint8_t {
		Main = 0,
		Secondary,

		Count
	};

	GPS(const char *path, gps_driver_mode_t mode, GPSHelper::Interface interface, bool enable_sat_info, Instance instance,
	    unsigned configured_baudrate);
	~GPS() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** spawn task and select the instance */
	static int task_spawn(int argc, char *argv[], Instance instance);

	/** @see ModuleBase */
	static GPS *instantiate(int argc, char *argv[]);

	static GPS *instantiate(int argc, char *argv[], Instance instance);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/**
	 * task spawn trampoline for the secondary GPS
	 */
	static int run_trampoline_secondary(int argc, char *argv[]);

	/** @see ModuleBase::run() */
	void run() override;

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	int print_status() override;

	/**
	 * Schedule reset of the GPS device
	 */
	void schedule_reset(GPSRestartType restart_type);

	/**
	 * Reset device if reset was scheduled
	 */
	void reset_if_scheduled();

private:
	int				_serial_fd{-1};					///< serial interface to GPS
	unsigned			_baudrate{0};					///< current baudrate
	const unsigned			_configured_baudrate{0};			///< configured baudrate (0=auto-detect)
	char				_port[20] {};					///< device / serial port path

	bool				_healthy{false};				///< flag to signal if the GPS is ok
	bool				_mode_auto;					///< if true, auto-detect which GPS is attached

	gps_driver_mode_t		_mode;						///< current mode

	GPSHelper::Interface		_interface;   					///< interface
	GPSHelper			*_helper{nullptr};				///< instance of GPS parser

	GPS_Sat_Info			*_sat_info{nullptr};				///< instance of GPS sat info data object

	sensor_gps_s			_report_gps_pos{};				///< uORB topic for gps position
	satellite_info_s		*_p_report_sat_info{nullptr};			///< pointer to uORB topic for satellite info

	uORB::PublicationMulti<sensor_gps_s>	_report_gps_pos_pub{ORB_ID(sensor_gps)};	///< uORB pub for gps position
	uORB::PublicationMulti<satellite_info_s>	_report_sat_info_pub{ORB_ID(satellite_info)};		///< uORB pub for satellite info

	float				_rate{0.0f};					///< position update rate
	float				_rate_rtcm_injection{0.0f};			///< RTCM message injection rate
	unsigned			_last_rate_rtcm_injection_count{0};		///< counter for number of RTCM messages
	unsigned			_num_bytes_read{0}; 				///< counter for number of read bytes from the UART (within update interval)
	unsigned			_rate_reading{0}; 				///< reading rate in B/s

	const Instance 			_instance;

	uORB::Subscription		_orb_inject_data_sub{ORB_ID(gps_inject_data)};
	uORB::Publication<gps_dump_s>	_dump_communication_pub{ORB_ID(gps_dump)};
	gps_dump_s			*_dump_to_device{nullptr};
	gps_dump_s			*_dump_from_device{nullptr};
	gps_dump_comm_mode_t _dump_communication_mode{gps_dump_comm_mode_t::Disabled};

	static px4::atomic_bool _is_gps_main_advertised; ///< for the second gps we want to make sure that it gets instance 1
	/// and thus we wait until the first one publishes at least one message.

	static px4::atomic<GPS *> _secondary_instance;

	px4::atomic<int> _scheduled_reset{(int)GPSRestartType::None};

	perf_counter_t _gps_set_clock_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": set clock interval")};

	bool _update_system_time{true};

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
	 * Dump gps communication.
	 * @param data message
	 * @param len length of the message
	 * @param mode calling source
	 * @param msg_to_gps_device if true, this is a message sent to the gps device, otherwise it's from the device
	 */
	void dumpGpsData(uint8_t *data, size_t len, gps_dump_comm_mode_t mode, bool msg_to_gps_device);

	void setClock(timespec *rtc_gps_time);

	void initializeCommunicationDump();

	static constexpr int SET_CLOCK_DRIFT_TIME_NS{500 * 1000 * 1000};	///< RTC drift time when time synchronization is needed (in nanoseconds)
};

px4::atomic_bool GPS::_is_gps_main_advertised{false};
px4::atomic<GPS *> GPS::_secondary_instance{nullptr};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int gps_main(int argc, char *argv[]);


GPS::GPS(const char *path, gps_driver_mode_t mode, GPSHelper::Interface interface, bool enable_sat_info,
	 Instance instance, unsigned configured_baudrate) :
	Device(MODULE_NAME),
	_configured_baudrate(configured_baudrate),
	_mode(mode),
	_interface(interface),
	_instance(instance)
{
	/* store port name */
	strncpy(_port, path, sizeof(_port) - 1);
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	_report_gps_pos.heading = NAN;
	_report_gps_pos.heading_offset = NAN;

	/* create satellite info data object if requested */
	if (enable_sat_info) {
		_sat_info = new GPS_Sat_Info();
		_p_report_sat_info = &_sat_info->_data;
		memset(_p_report_sat_info, 0, sizeof(*_p_report_sat_info));
	}

	if (_interface == GPSHelper::Interface::UART) {
		set_device_bus_type(device::Device::DeviceBusType::DeviceBusType_SERIAL);

		char c = _port[strlen(_port) - 1]; // last digit of path (eg /dev/ttyS2)
		set_device_bus(atoi(&c));

	} else if (_interface == GPSHelper::Interface::SPI) {
		set_device_bus_type(device::Device::DeviceBusType::DeviceBusType_SPI);
	}

	if (_mode == gps_driver_mode_t::None) {
		// use parameter to select mode if not provided via CLI
		char protocol_param_name[16];
		snprintf(protocol_param_name, sizeof(protocol_param_name), "GPS_%i_PROTOCOL", (int)_instance + 1);
		int32_t protocol = 0;
		param_get(param_find(protocol_param_name), &protocol);

		switch (protocol) {
		case 1: _mode = gps_driver_mode_t::UBX; break;
#ifndef CONSTRAINED_FLASH

		case 2: _mode = gps_driver_mode_t::MTK; break;

		case 3: _mode = gps_driver_mode_t::ASHTECH; break;

		case 4: _mode = gps_driver_mode_t::EMLIDREACH; break;

		case 5: _mode = gps_driver_mode_t::FEMTOMES; break;

		case 6: _mode = gps_driver_mode_t::NMEA; break;
#endif // CONSTRAINED_FLASH
		}
	}

	_mode_auto = _mode == gps_driver_mode_t::None;
}

GPS::~GPS()
{
	GPS *secondary_instance = _secondary_instance.load();

	if (_instance == Instance::Main && secondary_instance) {
		secondary_instance->request_stop();

		// wait for it to exit
		unsigned int i = 0;

		do {
			px4_usleep(20000); // 20 ms
			++i;
		} while (_secondary_instance.load() && i < 100);
	}

	delete _sat_info;
	delete _dump_to_device;
	delete _dump_from_device;
	delete _helper;

	perf_free(_gps_set_clock_perf);
}

int GPS::callback(GPSCallbackType type, void *data1, int data2, void *user)
{
	GPS *gps = (GPS *)user;

	switch (type) {
	case GPSCallbackType::readDeviceData: {
			int timeout;
			memcpy(&timeout, data1, sizeof(timeout));
			int num_read = gps->pollOrRead((uint8_t *)data1, data2, timeout);

			if (num_read > 0) {
				gps->dumpGpsData((uint8_t *)data1, (size_t)num_read, gps_dump_comm_mode_t::Full, false);
			}

			return num_read;
		}

	case GPSCallbackType::writeDeviceData:
		gps->dumpGpsData((uint8_t *)data1, (size_t)data2, gps_dump_comm_mode_t::Full, true);

		return ::write(gps->_serial_fd, data1, (size_t)data2);

	case GPSCallbackType::setBaudrate:
		return gps->setBaudrate(data2);

	case GPSCallbackType::gotRTCMMessage:
		gps->dumpGpsData((uint8_t *)data1, (size_t)data2, gps_dump_comm_mode_t::RTCM, false);
		break;

	case GPSCallbackType::surveyInStatus:
		/* not used */
		break;

	case GPSCallbackType::setClock:
		gps->setClock(static_cast<timespec *>(data1));
		break;
	}

	return 0;
}

int GPS::pollOrRead(uint8_t *buf, size_t buf_length, int timeout)
{
	handleInjectDataTopic();

#if !defined(__PX4_QURT)

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
			 * If we have all requested data available, read it without waiting.
			 * If more bytes are available, we'll go back to poll() again.
			 */
			const unsigned character_count = 32; // minimum bytes that we want to read
			unsigned baudrate = _baudrate == 0 ? 115200 : _baudrate;
			const unsigned sleeptime = character_count * 1000000 / (baudrate / 10);

#ifdef __PX4_NUTTX
			int err = 0;
			int bytes_available = 0;
			err = ::ioctl(_serial_fd, FIONREAD, (unsigned long)&bytes_available);

			if (err != 0 || bytes_available < (int)character_count) {
				px4_usleep(sleeptime);
			}

#else
			px4_usleep(sleeptime);
#endif

			ret = ::read(_serial_fd, buf, buf_length);

			if (ret > 0) {
				_num_bytes_read += ret;
			}

		} else {
			ret = -1;
		}
	}

	return ret;

#else
	/* For QURT, just use read for now, since this doesn't block, we need to slow it down
	 * just a bit. */
	px4_usleep(10000);
	return ::read(_serial_fd, buf, buf_length);
#endif
}

void GPS::handleInjectDataTopic()
{
	if (!_helper->shouldInjectRTCM()) {
		return;
	}

	bool updated = false;

	// Limit maximum number of GPS injections to 6 since usually
	// GPS injections should consist of 1-4 packets (GPS, Glonass, BeiDou, Galileo).
	// Looking at 6 packets thus guarantees, that at least a full injection
	// data set is evaluated.
	const size_t max_num_injections = 6;
	size_t num_injections = 0;

	do {
		num_injections++;
		updated = _orb_inject_data_sub.updated();

		if (updated) {
			gps_inject_data_s msg;

			if (_orb_inject_data_sub.copy(&msg)) {

				/* Write the message to the gps device. Note that the message could be fragmented.
				 * But as we don't write anywhere else to the device during operation, we don't
				 * need to assemble the message first.
				 */
				injectData(msg.data, msg.len);

				++_last_rate_rtcm_injection_count;
			}
		}
	} while (updated && num_injections < max_num_injections);
}

bool GPS::injectData(uint8_t *data, size_t len)
{
	dumpGpsData(data, len, gps_dump_comm_mode_t::Full, true);

	size_t written = ::write(_serial_fd, data, len);
	::fsync(_serial_fd);
	return written == len;
}

int GPS::setBaudrate(unsigned baud)
{
	/* process baud rate */
	int speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

#ifndef B460800
#define B460800 460800
#endif

	case 460800: speed = B460800; break;

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

	/* no parity, one stop bit, disable flow control */
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

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

	return 0;
}

void GPS::initializeCommunicationDump()
{
	param_t gps_dump_comm_ph = param_find("GPS_DUMP_COMM");
	int32_t param_dump_comm;

	if (gps_dump_comm_ph == PARAM_INVALID || param_get(gps_dump_comm_ph, &param_dump_comm) != 0) {
		return;
	}

	if (param_dump_comm < 1 || param_dump_comm > 2) {
		return; //dumping disabled
	}

	_dump_from_device = new gps_dump_s();
	_dump_to_device = new gps_dump_s();

	if (!_dump_from_device || !_dump_to_device) {
		PX4_ERR("failed to allocated dump data");
		return;
	}

	memset(_dump_to_device, 0, sizeof(gps_dump_s));
	memset(_dump_from_device, 0, sizeof(gps_dump_s));

	//make sure to use a large enough queue size, so that we don't lose messages. You may also want
	//to increase the logger rate for that.
	_dump_communication_pub.advertise();

	_dump_communication_mode = (gps_dump_comm_mode_t)param_dump_comm;
}

void GPS::dumpGpsData(uint8_t *data, size_t len, gps_dump_comm_mode_t mode, bool msg_to_gps_device)
{
	gps_dump_s *dump_data  = msg_to_gps_device ? _dump_to_device : _dump_from_device;

	if (_dump_communication_mode != mode || !dump_data) {
		return;
	}

	dump_data->instance = (uint8_t)_instance;

	while (len > 0) {
		size_t write_len = len;

		if (write_len > sizeof(dump_data->data) - dump_data->len) {
			write_len = sizeof(dump_data->data) - dump_data->len;
		}

		memcpy(dump_data->data + dump_data->len, data, write_len);
		data += write_len;
		dump_data->len += write_len;
		len -= write_len;

		if (dump_data->len >= sizeof(dump_data->data)) {
			if (msg_to_gps_device) {
				dump_data->len |= 1 << 7;
			}

			dump_data->timestamp = hrt_absolute_time();
			_dump_communication_pub.publish(*dump_data);
			dump_data->len = 0;
		}
	}
}

void GPS::setClock(timespec *rtc_gps_time)
{
	if (_update_system_time) {
		px4_clock_settime(CLOCK_REALTIME, rtc_gps_time);
		perf_count(_gps_set_clock_perf);
		_update_system_time = false;

	} else {
		timespec rtc_system_time;
		px4_clock_gettime(CLOCK_REALTIME, &rtc_system_time);

		const int64_t system_time = rtc_system_time.tv_sec * 1e9 + rtc_system_time.tv_nsec;
		const int64_t gps_time = rtc_gps_time->tv_sec * 1e9 + rtc_gps_time->tv_nsec;

		int64_t drift_time = abs(system_time - gps_time);

		if (drift_time > SET_CLOCK_DRIFT_TIME_NS) {
			// update clock immediately on next callback
			_update_system_time = true;
		}
	}
}

void
GPS::run()
{
	param_t handle = param_find("GPS_YAW_OFFSET");
	float heading_offset = 0.f;

	if (handle != PARAM_INVALID) {
		param_get(handle, &heading_offset);
		heading_offset = matrix::wrap_pi(math::radians(heading_offset));
	}

	int32_t gps_ubx_dynmodel = 7; // default to 7: airborne with <2g acceleration
	handle = param_find("GPS_UBX_DYNMODEL");

	if (handle != PARAM_INVALID) {
		param_get(handle, &gps_ubx_dynmodel);
	}

	handle = param_find("GPS_UBX_MODE");

	GPSDriverUBX::UBXMode ubx_mode{GPSDriverUBX::UBXMode::Normal};

	if (handle != PARAM_INVALID) {
		int32_t gps_ubx_mode = 0;
		param_get(handle, &gps_ubx_mode);

		if (gps_ubx_mode == 1) { // heading
			if (_instance == Instance::Main) {
				ubx_mode = GPSDriverUBX::UBXMode::RoverWithMovingBase;

			} else {
				ubx_mode = GPSDriverUBX::UBXMode::MovingBase;
			}
		}
	}

	int32_t gnssSystemsParam = static_cast<int32_t>(GPSHelper::GNSSSystemsMask::RECEIVER_DEFAULTS);

	if (_instance == Instance::Main) {
		handle = param_find("GPS_1_GNSS");
		param_get(handle, &gnssSystemsParam);

	} else if (_instance == Instance::Secondary) {
		handle = param_find("GPS_2_GNSS");
		param_get(handle, &gnssSystemsParam);
	}

	initializeCommunicationDump();

	uint64_t last_rate_measurement = hrt_absolute_time();
	unsigned last_rate_count = 0;

	/* loop handling received serial bytes and also configuring in between */
	while (!should_exit()) {
		if (_helper != nullptr) {
			delete (_helper);
			_helper = nullptr;
		}

		if (_serial_fd < 0) {
			/* open the serial port */
			_serial_fd = ::open(_port, O_RDWR | O_NOCTTY);

			if (_serial_fd < 0) {
				PX4_ERR("failed to open %s err: %d", _port, errno);
				px4_sleep(1);
				continue;
			}

#ifdef __PX4_LINUX

			if (_interface == GPSHelper::Interface::SPI) {
				int spi_speed = 1000000; // make sure the bus speed is not too high (required on RPi)
				int status_value = ::ioctl(_serial_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);

				if (status_value < 0) {
					PX4_ERR("SPI_IOC_WR_MAX_SPEED_HZ failed for %s (%d)", _port, errno);
				}

				status_value = ::ioctl(_serial_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);

				if (status_value < 0) {
					PX4_ERR("SPI_IOC_RD_MAX_SPEED_HZ failed for %s (%d)", _port, errno);
				}
			}

#endif /* __PX4_LINUX */
		}

		switch (_mode) {
		case gps_driver_mode_t::None:
			_mode = gps_driver_mode_t::UBX;

		/* FALLTHROUGH */
		case gps_driver_mode_t::UBX:
			_helper = new GPSDriverUBX(_interface, &GPS::callback, this, &_report_gps_pos, _p_report_sat_info,
						   gps_ubx_dynmodel, heading_offset, ubx_mode);
			set_device_type(DRV_GPS_DEVTYPE_UBX);
			break;
#ifndef CONSTRAINED_FLASH

		case gps_driver_mode_t::MTK:
			_helper = new GPSDriverMTK(&GPS::callback, this, &_report_gps_pos);
			set_device_type(DRV_GPS_DEVTYPE_MTK);
			break;

		case gps_driver_mode_t::ASHTECH:
			_helper = new GPSDriverAshtech(&GPS::callback, this, &_report_gps_pos, _p_report_sat_info, heading_offset);
			set_device_type(DRV_GPS_DEVTYPE_ASHTECH);
			break;

		case gps_driver_mode_t::EMLIDREACH:
			_helper = new GPSDriverEmlidReach(&GPS::callback, this, &_report_gps_pos, _p_report_sat_info);
			set_device_type(DRV_GPS_DEVTYPE_EMLID_REACH);
			break;

		case gps_driver_mode_t::FEMTOMES:
			_helper = new GPSDriverFemto(&GPS::callback, this, &_report_gps_pos, _p_report_sat_info, heading_offset);
			set_device_type(DRV_GPS_DEVTYPE_FEMTOMES);
			break;

		case gps_driver_mode_t::NMEA:
			_helper = new GPSDriverNMEA(&GPS::callback, this, &_report_gps_pos, _p_report_sat_info, heading_offset);
			set_device_type(DRV_GPS_DEVTYPE_NMEA);
			break;
#endif // CONSTRAINED_FLASH

		default:
			break;
		}

		_baudrate = _configured_baudrate;
		GPSHelper::GPSConfig gpsConfig{};
		gpsConfig.gnss_systems = static_cast<GPSHelper::GNSSSystemsMask>(gnssSystemsParam);

		if (_instance == Instance::Main && _dump_communication_mode == gps_dump_comm_mode_t::RTCM) {
			gpsConfig.output_mode = GPSHelper::OutputMode::GPSAndRTCM;

		} else {
			gpsConfig.output_mode = GPSHelper::OutputMode::GPS;
		}

		if (_helper && _helper->configure(_baudrate, gpsConfig) == 0) {

			/* reset report */
			memset(&_report_gps_pos, 0, sizeof(_report_gps_pos));
			_report_gps_pos.heading = NAN;
			_report_gps_pos.heading_offset = heading_offset;

			if (_mode == gps_driver_mode_t::UBX) {

				/* GPS is obviously detected successfully, reset statistics */
				_helper->resetUpdateRates();

				// populate specific ublox model
				if (get_device_type() == DRV_GPS_DEVTYPE_UBX) {
					GPSDriverUBX *driver_ubx = (GPSDriverUBX *)_helper;

					switch (driver_ubx->board()) {
					case GPSDriverUBX::Board::u_blox6:
						set_device_type(DRV_GPS_DEVTYPE_UBX_6);
						break;

					case GPSDriverUBX::Board::u_blox7:
						set_device_type(DRV_GPS_DEVTYPE_UBX_7);
						break;

					case GPSDriverUBX::Board::u_blox8:
						set_device_type(DRV_GPS_DEVTYPE_UBX_8);
						break;

					case GPSDriverUBX::Board::u_blox9:
						set_device_type(DRV_GPS_DEVTYPE_UBX_9);
						break;

					case GPSDriverUBX::Board::u_blox9_F9P:
						set_device_type(DRV_GPS_DEVTYPE_UBX_F9P);
						break;

					default:
						set_device_type(DRV_GPS_DEVTYPE_UBX);
						break;
					}
				}
			}

			int helper_ret;
			unsigned receive_timeout = TIMEOUT_5HZ;

			if (ubx_mode == GPSDriverUBX::UBXMode::RoverWithMovingBase) {
				/* The MB rover will wait as long as possible to compute a navigation solution,
				 * possibly lowering the navigation rate all the way to 1 Hz while doing so. */
				receive_timeout = TIMEOUT_1HZ;
			}

			while ((helper_ret = _helper->receive(receive_timeout)) > 0 && !should_exit()) {

				if (helper_ret & 1) {
					publish();

					last_rate_count++;
				}

				if (_p_report_sat_info && (helper_ret & 2)) {
					publishSatelliteInfo();
				}

				reset_if_scheduled();

				/* measure update rate every 5 seconds */
				if (hrt_absolute_time() - last_rate_measurement > RATE_MEASUREMENT_PERIOD) {
					float dt = (float)((hrt_absolute_time() - last_rate_measurement)) / 1000000.0f;
					_rate = last_rate_count / dt;
					_rate_rtcm_injection = _last_rate_rtcm_injection_count / dt;
					_rate_reading = _num_bytes_read / dt;
					last_rate_measurement = hrt_absolute_time();
					last_rate_count = 0;
					_last_rate_rtcm_injection_count = 0;
					_num_bytes_read = 0;
					_helper->storeUpdateRates();
					_helper->resetUpdateRates();
				}

				if (!_healthy) {
					// Helpful for debugging, but too verbose for normal ops
//						const char *mode_str = "unknown";
//
//						switch (_mode) {
//						case gps_driver_mode_t::UBX:
//							mode_str = "UBX";
//							break;
//
//						case gps_driver_mode_t::MTK:
//							mode_str = "MTK";
//							break;
//
//						case gps_driver_mode_t::ASHTECH:
//							mode_str = "ASHTECH";
//							break;
//
//						case gps_driver_mode_t::EMLIDREACH:
//							mode_str = "EMLID REACH";
//							break;
//
//						default:
//							break;
//						}
//
//						PX4_WARN("module found: %s", mode_str);
					_healthy = true;
				}
			}

			if (_healthy) {
				_healthy = false;
				_rate = 0.0f;
				_rate_rtcm_injection = 0.0f;
			}
		}

		if (_serial_fd >= 0) {
			::close(_serial_fd);
			_serial_fd = -1;
		}

		if (_mode_auto) {
			switch (_mode) {
			case gps_driver_mode_t::UBX:
#ifndef CONSTRAINED_FLASH
				_mode = gps_driver_mode_t::MTK;
				break;

			case gps_driver_mode_t::MTK:
				_mode = gps_driver_mode_t::ASHTECH;
				break;

			case gps_driver_mode_t::ASHTECH:
				_mode = gps_driver_mode_t::EMLIDREACH;
				break;

			case gps_driver_mode_t::EMLIDREACH:
				_mode = gps_driver_mode_t::FEMTOMES;
				break;

			case gps_driver_mode_t::FEMTOMES:
			case gps_driver_mode_t::NMEA: // skip NMEA for auto-detection to avoid false positive matching
#endif // CONSTRAINED_FLASH
				_mode = gps_driver_mode_t::UBX;
				px4_usleep(500000); // tried all possible drivers. Wait a bit before next round
				break;

			default:
				break;
			}

		} else {
			px4_usleep(500000);
		}
	}

	PX4_INFO("exiting");
}

int
GPS::print_status()
{
	switch (_instance) {
	case Instance::Main:
		PX4_INFO("Main GPS");
		break;

	case Instance::Secondary:
		PX4_INFO("");
		PX4_INFO("Secondary GPS");
		break;

	default:
		break;
	}

	// GPS Mode
	switch (_mode) {
	case gps_driver_mode_t::UBX:
		PX4_INFO("protocol: UBX");
		break;
#ifndef CONSTRAINED_FLASH

	case gps_driver_mode_t::MTK:
		PX4_INFO("protocol: MTK");
		break;

	case gps_driver_mode_t::ASHTECH:
		PX4_INFO("protocol: ASHTECH");
		break;

	case gps_driver_mode_t::EMLIDREACH:
		PX4_INFO("protocol: EMLIDREACH");
		break;

	case gps_driver_mode_t::FEMTOMES:
		PX4_INFO("protocol: FEMTOMES");
		break;

	case gps_driver_mode_t::NMEA:
		PX4_INFO("protocol: NMEA");
		break;
#endif // CONSTRAINED_FLASH

	default:
		break;
	}

	PX4_INFO("status: %s, port: %s, baudrate: %d", _healthy ? "OK" : "NOT OK", _port, _baudrate);
	PX4_INFO("sat info: %s", (_p_report_sat_info != nullptr) ? "enabled" : "disabled");
	PX4_INFO("rate reading: \t\t%6i B/s", _rate_reading);

	perf_print_counter(_gps_set_clock_perf);

	if (_report_gps_pos.timestamp != 0) {
		if (_helper) {
			PX4_INFO("rate position: \t\t%6.2f Hz", (double)_helper->getPositionUpdateRate());
			PX4_INFO("rate velocity: \t\t%6.2f Hz", (double)_helper->getVelocityUpdateRate());
		}

		PX4_INFO("rate publication:\t\t%6.2f Hz", (double)_rate);
		PX4_INFO("rate RTCM injection:\t%6.2f Hz", (double)_rate_rtcm_injection);

		print_message(_report_gps_pos);
	}

	if (_instance == Instance::Main && _secondary_instance.load()) {
		GPS *secondary_instance = _secondary_instance.load();
		secondary_instance->print_status();
	}

	return 0;
}

void
GPS::schedule_reset(GPSRestartType restart_type)
{
	_scheduled_reset.store((int)restart_type);

	if (_instance == Instance::Main && _secondary_instance.load()) {
		GPS *secondary_instance = _secondary_instance.load();
		secondary_instance->schedule_reset(restart_type);
	}
}

void
GPS::reset_if_scheduled()
{
	GPSRestartType restart_type = (GPSRestartType)_scheduled_reset.load();

	if (restart_type != GPSRestartType::None) {
		_scheduled_reset.store((int)GPSRestartType::None);
		int res = _helper->reset(restart_type);

		if (res == -1) {
			PX4_INFO("Reset is not supported on this device.");

		} else if (res < 0) {
			PX4_INFO("Reset failed.");

		} else {
			PX4_INFO("Reset succeeded.");
		}
	}
}

void
GPS::publish()
{
	if (_instance == Instance::Main || _is_gps_main_advertised.load()) {
		_report_gps_pos.device_id = get_device_id();

		_report_gps_pos_pub.publish(_report_gps_pos);
		// Heading/yaw data can be updated at a lower rate than the other navigation data.
		// The uORB message definition requires this data to be set to a NAN if no new valid data is available.
		_report_gps_pos.heading = NAN;
		_is_gps_main_advertised.store(true);
	}
}

void
GPS::publishSatelliteInfo()
{
	if (_instance == Instance::Main) {
		if (_p_report_sat_info != nullptr) {
			_report_sat_info_pub.publish(*_p_report_sat_info);
		}

	} else {
		//we don't publish satellite info for the secondary gps
	}
}

int
GPS::custom_command(int argc, char *argv[])
{
	// Check if the driver is running.
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	GPS *_instance = get_instance();

	bool res = false;

	if (argc == 2 && !strcmp(argv[0], "reset")) {

		if (!strcmp(argv[1], "hot")) {
			res = true;
			_instance->schedule_reset(GPSRestartType::Hot);

		} else if (!strcmp(argv[1], "cold")) {
			res = true;
			_instance->schedule_reset(GPSRestartType::Cold);

		} else if (!strcmp(argv[1], "warm")) {
			res = true;
			_instance->schedule_reset(GPSRestartType::Warm);
		}
	}

	if (res) {
		PX4_INFO("Resetting GPS - %s", argv[1]);
		return 0;
	}

	return (res) ? 0 : print_usage("unknown command");
}

int GPS::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
GPS driver module that handles the communication with the device and publishes the position via uORB.
It supports multiple protocols (device vendors) and by default automatically selects the correct one.

The module supports a secondary GPS device, specified via `-e` parameter. The position will be published
on the second uORB topic instance, but it's currently not used by the rest of the system (however the
data will be logged, so that it can be used for comparisons).

### Implementation
There is a thread for each device polling for data. The GPS protocol classes are implemented with callbacks
so that they can be used in other projects as well (eg. QGroundControl uses them too).

### Examples

Starting 2 GPS devices (the main GPS on /dev/ttyS3 and the secondary on /dev/ttyS4):
$ gps start -d /dev/ttyS3 -e /dev/ttyS4

Initiate warm restart of GPS device
$ gps reset warm
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gps", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "GPS device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 0, 0, 3000000, "Baudrate (can also be p:<param_name>)", true);
	PRINT_MODULE_USAGE_PARAM_STRING('e', nullptr, "<file:dev>", "Optional secondary GPS device", true);
	PRINT_MODULE_USAGE_PARAM_INT('g', 0, 0, 3000000, "Baudrate (secondary GPS, can also be p:<param_name>)", true);

	PRINT_MODULE_USAGE_PARAM_FLAG('s', "Enable publication of satellite info", true);

	PRINT_MODULE_USAGE_PARAM_STRING('i', "uart", "spi|uart", "GPS interface", true);
	PRINT_MODULE_USAGE_PARAM_STRING('j', "uart", "spi|uart", "secondary GPS interface", true);
	PRINT_MODULE_USAGE_PARAM_STRING('p', nullptr, "ubx|mtk|ash|eml|fem|nmea", "GPS Protocol (default=auto select)", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset GPS device");
	PRINT_MODULE_USAGE_ARG("cold|warm|hot", "Specify reset type", false);

	return 0;
}

int GPS::task_spawn(int argc, char *argv[])
{
	return task_spawn(argc, argv, Instance::Main);
}

int GPS::task_spawn(int argc, char *argv[], Instance instance)
{
	px4_main_t entry_point;
	if (instance == Instance::Main) {
		entry_point = (px4_main_t)&run_trampoline;
	} else {
		entry_point = (px4_main_t)&run_trampoline_secondary;
	}

	int task_id = px4_task_spawn_cmd("gps", SCHED_DEFAULT,
				   SCHED_PRIORITY_SLOW_DRIVER, TASK_STACK_SIZE,
				   entry_point, (char *const *)argv);

	if (task_id < 0) {
		task_id = -1;
		return -errno;
	}

	if (instance == Instance::Main) {
		_task_id = task_id;
	}

	return 0;
}

int GPS::run_trampoline_secondary(int argc, char *argv[])
{
	// the task name is the first argument
	argc -= 1;
	argv += 1;

	GPS *gps = instantiate(argc, argv, Instance::Secondary);
	if (gps) {
		_secondary_instance.store(gps);
		gps->run();

		_secondary_instance.store(nullptr);
		delete gps;
	}
	return 0;
}
GPS *GPS::instantiate(int argc, char *argv[])
{
	return instantiate(argc, argv, Instance::Main);
}

GPS *GPS::instantiate(int argc, char *argv[], Instance instance)
{
	const char *device_name = "/dev/ttyS3";
	const char *device_name_secondary = nullptr;
	int baudrate_main = 0;
	int baudrate_secondary = 0;
	bool enable_sat_info = false;
	GPSHelper::Interface interface = GPSHelper::Interface::UART;
	GPSHelper::Interface interface_secondary = GPSHelper::Interface::UART;
	gps_driver_mode_t mode = gps_driver_mode_t::None;

	bool error_flag = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "b:d:e:g:si:j:p:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			if (px4_get_parameter_value(myoptarg, baudrate_main) != 0) {
				PX4_ERR("baudrate parsing failed");
				error_flag = true;
			}
			break;
		case 'g':
			if (px4_get_parameter_value(myoptarg, baudrate_secondary) != 0) {
				PX4_ERR("baudrate parsing failed");
				error_flag = true;
			}
			break;

		case 'd':
			device_name = myoptarg;
			break;

		case 'e':
			device_name_secondary = myoptarg;
			break;

		case 's':
			enable_sat_info = true;
			break;

		case 'i':
			if (!strcmp(myoptarg, "spi")) {
				interface = GPSHelper::Interface::SPI;

			} else if (!strcmp(myoptarg, "uart")) {
				interface = GPSHelper::Interface::UART;

			} else {
				PX4_ERR("unknown interface: %s", myoptarg);
				error_flag = true;
			}
			break;

		case 'j':
			if (!strcmp(myoptarg, "spi")) {
				interface_secondary = GPSHelper::Interface::SPI;

			} else if (!strcmp(myoptarg, "uart")) {
				interface_secondary = GPSHelper::Interface::UART;

			} else {
				PX4_ERR("unknown interface for secondary: %s", myoptarg);
				error_flag = true;
			}
			break;

		case 'p':
			if (!strcmp(myoptarg, "ubx")) {
				mode = gps_driver_mode_t::UBX;
#ifndef CONSTRAINED_FLASH
			} else if (!strcmp(myoptarg, "mtk")) {
				mode = gps_driver_mode_t::MTK;

			} else if (!strcmp(myoptarg, "ash")) {
				mode = gps_driver_mode_t::ASHTECH;

			} else if (!strcmp(myoptarg, "eml")) {
				mode = gps_driver_mode_t::EMLIDREACH;

			} else if (!strcmp(myoptarg, "fem")) {
				mode = gps_driver_mode_t::FEMTOMES;

			} else if (!strcmp(myoptarg, "nmea")) {
				mode = gps_driver_mode_t::NMEA;
#endif // CONSTRAINED_FLASH
			} else {
				PX4_ERR("unknown protocol: %s", myoptarg);
				error_flag = true;
			}
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	GPS *gps;
	if (instance == Instance::Main) {
		gps = new GPS(device_name, mode, interface, enable_sat_info, instance, baudrate_main);

		if (gps && device_name_secondary) {
			task_spawn(argc, argv, Instance::Secondary);
			// wait until running
			int i = 0;

			do {
				/* wait up to 1s */
				px4_usleep(2500);

			} while (!_secondary_instance.load() && ++i < 400);

			if (i == 400) {
				PX4_ERR("Timed out while waiting for thread to start");
			}
		}
	} else { // secondary instance
		gps = new GPS(device_name_secondary, mode, interface_secondary, enable_sat_info, instance, baudrate_secondary);
	}

	return gps;
}

int
gps_main(int argc, char *argv[])
{
	return GPS::main(argc, argv);
}
