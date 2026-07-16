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

#include <cstring>

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/Serial.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/gps_dump.h>
#include <uORB/topics/rtcm_data.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_gnss_relative.h>

#include <lib/failure_injection/FailureInjection.hpp>
#include <lib/gnss/correction_framer.h>

#include "devices/src/gps_helper.h"

#if defined(CONFIG_GPS_UBX)
# include "devices/src/ubx.h"
#endif
#if defined(CONFIG_GPS_MTK)
# include "devices/src/mtk.h"
#endif
#if defined(CONFIG_GPS_ASHTECH)
# include "devices/src/ashtech.h"
#endif
#if defined(CONFIG_GPS_EMLIDREACH)
# include "devices/src/emlid_reach.h"
#endif
#if defined(CONFIG_GPS_FEMTOMES)
# include "devices/src/femtomes.h"
#endif
#if defined(CONFIG_GPS_NMEA)
# include "devices/src/nmea.h"
#endif

#ifdef __PX4_LINUX
#include <linux/spi/spidev.h>
#endif /* __PX4_LINUX */

using namespace device;
using namespace time_literals;

#define TIMEOUT_1HZ		1300 //!< Timeout time in mS, 1000 mS (1Hz) + 300 mS delta for error
#define TIMEOUT_5HZ		500 //!< Timeout time in mS,  200 mS (5Hz) + 300 mS delta for error
#define TIMEOUT_INIT_1HZ	(3 * TIMEOUT_1HZ) //!< Timeout time in mS, used until GPS is healthy
#define TIMEOUT_INIT_5HZ	(3 * TIMEOUT_5HZ) //!< Timeout time in mS, used until GPS is healthy
#define TIMEOUT_DUMP_ADD	450 //!< Additional time in mS to account for RTCM3 parsing and dumping

enum class gps_driver_mode_t {
	None = 0,
	UBX,
	MTK,
	ASHTECH,
	EMLIDREACH,
	FEMTOMES,
	NMEA,
};

// NMEA is excluded because it can produce false-positive detections.
// The trailing None also supports builds without an auto-detectable protocol.
static constexpr gps_driver_mode_t kAutoDetectModes[] = {
#if defined(CONFIG_GPS_UBX)
	gps_driver_mode_t::UBX,
#endif
#if defined(CONFIG_GPS_MTK)
	gps_driver_mode_t::MTK,
#endif
#if defined(CONFIG_GPS_ASHTECH)
	gps_driver_mode_t::ASHTECH,
#endif
#if defined(CONFIG_GPS_EMLIDREACH)
	gps_driver_mode_t::EMLIDREACH,
#endif
#if defined(CONFIG_GPS_FEMTOMES)
	gps_driver_mode_t::FEMTOMES,
#endif
	gps_driver_mode_t::None,
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

static constexpr int TASK_STACK_SIZE = PX4_STACK_ADJUSTED(2040);


class GPS : public ModuleBase, public device::Device
{
public:

	/** The GPS allows to run multiple instances */
	enum class Instance : uint8_t {
		Main = 0,
		Secondary,

		Count
	};

	GPS(const char *path, gps_driver_mode_t mode, GPSHelper::Interface interface, Instance instance,
	    unsigned configured_baudrate);
	~GPS() override;

	static Descriptor desc;

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
	 * task spawn trampoline for the primary GPS
	 */
	static int run_trampoline(int argc, char *argv[]);

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
#ifdef __PX4_LINUX
	int				_spi_fd {-1};					///< SPI interface to GPS
#endif
	Serial 			_uart {};				///< UART interface to GPS
	unsigned			_baudrate{0};					///< current baudrate
	const unsigned			_configured_baudrate{0};			///< configured baudrate (0=auto-detect)
	char				_port[20] {};					///< device / serial port path

	bool				_healthy{false};				///< flag to signal if the GPS is ok
#if defined(CONFIG_GPS_UBX)
	bool				_cfg_wiped {false};				///< flag to signal if the config was already wiped
#endif
	bool				_mode_auto;					///< if true, auto-detect which GPS is attached

	gps_driver_mode_t		_mode;						///< current mode

	GPSHelper::Interface		_interface;   					///< interface
	GPSHelper			*_helper{nullptr};				///< instance of GPS parser

	GPS_Sat_Info			*_sat_info{nullptr};				///< instance of GPS sat info data object

	sensor_gps_s			_sensor_gps{};				///< uORB topic for gps position
	satellite_info_s		*_p_report_sat_info{nullptr};			///< pointer to uORB topic for satellite info

	uORB::PublicationMulti<sensor_gps_s>	_sensor_gps_pub{ORB_ID(sensor_gps)};	///< uORB pub for gps position
	uORB::PublicationMulti<sensor_gnss_relative_s> _sensor_gnss_relative_pub{ORB_ID(sensor_gnss_relative)};

	uORB::PublicationMulti<satellite_info_s>	_report_sat_info_pub{ORB_ID(satellite_info)};		///< uORB pub for satellite info

	failure_injection::Config _failure_config;
	failure_injection::Stuck<sensor_gps_s> _stuck;

	float				_rate{0.0f};					///< position update rate
	float				_rtcm_injection_rate{0.0f};			///< corrections injection rate (Hz, RTCM3 and SPARTN)
	uint64_t			_last_rtcm_corrections_injection_count{0};	///< corrections-injection perf snapshot for rate calc
	unsigned			_rtcm_frames_in_rate_window{0};			///< corrections-stream RTCM3 frames in rate window
	unsigned			_spartn_frames_in_rate_window {0};		///< corrections-stream SPARTN frames in rate window
	bool				_injecting_rtcm {false};				///< latched: RTCM3 corrections injected in last rate window
	bool				_injecting_spartn {false};			///< latched: SPARTN injected in last rate window
	unsigned			_num_bytes_read{0}; 				///< counter for number of read bytes from the UART (within update interval)
	unsigned			_rate_reading{0}; 				///< reading rate in B/s
	hrt_abstime			_last_rtcm_injection_time{0};			///< time of last corrections injection
	uint8_t				_selected_rtcm_instance{0};			///< uorb instance that is being used for corrections

	const Instance 			_instance;

	uORB::SubscriptionMultiArray<rtcm_data_s, rtcm_data_s::MAX_INSTANCES> _rtcm_corrections_sub{ORB_ID::rtcm_corrections};
	uORB::Subscription _rtcm_moving_baseline_sub{ORB_ID(rtcm_moving_baseline)};
	uORB::PublicationMulti<rtcm_data_s> _rtcm_corrections_pub{ORB_ID(rtcm_corrections)};
	uORB::Publication<rtcm_data_s> _rtcm_moving_baseline_pub{ORB_ID(rtcm_moving_baseline)};
	uORB::Publication<gps_dump_s>	     _dump_communication_pub{ORB_ID(gps_dump)};
	gps_dump_s			     *_dump_to_device{nullptr};
	gps_dump_s			     *_dump_from_device{nullptr};
	gps_dump_comm_mode_t                 _dump_communication_mode{gps_dump_comm_mode_t::Disabled};

	// Each stream reassembles in its own framer: a fragmented fixed-base frame must not be
	// corrupted by moving-baseline bytes appended mid-frame (e.g. fixed-base + moving-base +
	// rover setups, where the rover injects both streams). Within a framer, RTCM3 and SPARTN
	// share one buffer so neither protocol can resync inside the other's payloads (see
	// correction_framer.h); SPARTN only ever arrives on the corrections stream.
	gnss::CorrectionFramer		     _rtcm_corrections_framer{};
	gnss::CorrectionFramer		     _rtcm_moving_baseline_framer{};

	perf_counter_t _uart_tx_buffer_full_perf{perf_alloc(PC_COUNT, MODULE_NAME": tx buf full")};
	perf_counter_t _correction_buffer_full_perf{perf_alloc(PC_COUNT, MODULE_NAME": corrections buf full")};
	perf_counter_t _rtcm_corrections_injection_perf{perf_alloc(PC_COUNT, MODULE_NAME": rtcm corrections injected")};
	perf_counter_t _rtcm_moving_baseline_injection_perf{perf_alloc(PC_COUNT, MODULE_NAME": rtcm moving baseline injected")};

	static px4::atomic_bool _is_gps_main_advertised; ///< for the second gps we want to make sure that it gets instance 1
	/// and thus we wait until the first one publishes at least one message.

	static px4::atomic<GPS *> _secondary_instance;

	px4::atomic<int> _scheduled_reset{(int)GPSRestartType::None};

	/**
	 * Publish the gps struct
	 */
	void 				publish();

	/**
	 * Publish the satellite info
	 */
	void 				publishSatelliteInfo();

	/**
	 * Publish RTCM corrections
	 */
	void 				publishRTCMCorrections(uint8_t *data, size_t len);

	/**
	 * Publish RTCM corrections
	 */
	void 				publishRelativePosition(sensor_gnss_relative_s &gnss_relative);

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
	 * Drain the multi-instance rtcm_corrections subscription into its RTCM parser, selecting an
	 * active instance if the current one goes stale.
	 */
	void drainRtcmCorrections();

	/**
	 * Drain the single-publisher rtcm_moving_baseline subscription into its RTCM parser.
	 */
	void drainMovingBaseline();

	/**
	 * Inject all complete frames reassembled in a framer (RTCM3 and/or SPARTN) into the receiver,
	 * counting each injected frame on the given perf counter and optional per-protocol rate-window
	 * counters.
	 */
	void injectRtcmFrames(gnss::CorrectionFramer &framer, perf_counter_t injection_perf,
			      unsigned *rtcm_frames_in_window = nullptr, unsigned *spartn_frames_in_window = nullptr);

	/**
	 * send data to the device, such as an RTCM stream
	 * @param data
	 * @param len
	 */
	inline bool injectData(const uint8_t *data, size_t len);

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
	void dumpGpsData(const uint8_t *data, size_t len, gps_dump_comm_mode_t mode, bool msg_to_gps_device);

	void initializeCommunicationDump();

	static constexpr int SET_CLOCK_DRIFT_TIME_S{5};			///< RTC drift time when time synchronization is needed (in seconds)
};

px4::atomic_bool GPS::_is_gps_main_advertised{false};
px4::atomic<GPS *> GPS::_secondary_instance{nullptr};
ModuleBase::Descriptor GPS::desc{task_spawn, custom_command, print_usage};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int gps_main(int argc, char *argv[]);


GPS::GPS(const char *path, gps_driver_mode_t mode, GPSHelper::Interface interface, Instance instance,
	 unsigned configured_baudrate) :
	Device(MODULE_NAME),
	_configured_baudrate(configured_baudrate),
	_mode(mode),
	_interface(interface),
	_instance(instance)
{
	/* store port name */
	if (path != nullptr) {
		strncpy(_port, path, sizeof(_port) - 1);
		_port[sizeof(_port) - 1] = '\0';

	} else {
		_port[0] = '\0';
	}

	_sensor_gps.heading = NAN;
	_sensor_gps.heading_offset = NAN;

	int32_t enable_sat_info = 0;
	param_get(param_find("GPS_SAT_INFO"), &enable_sat_info);

	/* create satellite info data object if requested */
	if (enable_sat_info) {
		_sat_info = new GPS_Sat_Info();
		_p_report_sat_info = &_sat_info->_data;
		memset(_p_report_sat_info, 0, sizeof(*_p_report_sat_info));
	}

	if (_interface == GPSHelper::Interface::UART) {
		set_device_bus_type(device::Device::DeviceBusType::DeviceBusType_SERIAL);

		char c = _port[strlen(_port) - 1]; // last digit of path (eg /dev/ttyS2)
		set_device_bus(c - 48); // sub 48 to convert char to integer

#ifdef __PX4_LINUX

	} else if (_interface == GPSHelper::Interface::SPI) {
		set_device_bus_type(device::Device::DeviceBusType::DeviceBusType_SPI);
#endif
	}

	if (_mode == gps_driver_mode_t::None) {
		// use parameter to select mode if not provided via CLI
		char protocol_param_name[17];
		snprintf(protocol_param_name, sizeof(protocol_param_name), "GPS_%i_PROTOCOL", (int)_instance + 1);
		int32_t protocol = 0;
		param_get(param_find(protocol_param_name), &protocol);

		switch (protocol) {
#if defined(CONFIG_GPS_UBX)

		case 1: _mode = gps_driver_mode_t::UBX; break;
#endif // CONFIG_GPS_UBX
#if defined(CONFIG_GPS_MTK)

		case 2: _mode = gps_driver_mode_t::MTK; break;
#endif // CONFIG_GPS_MTK
#if defined(CONFIG_GPS_ASHTECH)

		case 3: _mode = gps_driver_mode_t::ASHTECH; break;
#endif // CONFIG_GPS_ASHTECH
#if defined(CONFIG_GPS_EMLIDREACH)

		case 4: _mode = gps_driver_mode_t::EMLIDREACH; break;
#endif // CONFIG_GPS_EMLIDREACH
#if defined(CONFIG_GPS_FEMTOMES)

		case 5: _mode = gps_driver_mode_t::FEMTOMES; break;
#endif // CONFIG_GPS_FEMTOMES
#if defined(CONFIG_GPS_NMEA)

		case 6: _mode = gps_driver_mode_t::NMEA; break;
#endif // CONFIG_GPS_NMEA
		}

		if (protocol != 0 && _mode == gps_driver_mode_t::None) {
			PX4_WARN("%s=%" PRId32 " unsupported by this build, using auto-detection",
				 protocol_param_name, protocol);
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

	perf_free(_uart_tx_buffer_full_perf);
	perf_free(_correction_buffer_full_perf);
	perf_free(_rtcm_corrections_injection_perf);
	perf_free(_rtcm_moving_baseline_injection_perf);

	delete _sat_info;
	delete _dump_to_device;
	delete _dump_from_device;
	delete _helper;
}

int GPS::callback(GPSCallbackType type, void *data1, int data2, void *user)
{
	GPS *gps = (GPS *)user;

	timespec rtc_system_time;

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

	case GPSCallbackType::writeDeviceData: {
			gps->dumpGpsData((uint8_t *)data1, (size_t)data2, gps_dump_comm_mode_t::Full, true);

			int ret = 0;

			if (gps->_interface == GPSHelper::Interface::UART) {
				ret = gps->_uart.write((void *) data1, (size_t) data2);

#ifdef __PX4_LINUX

			} else if (gps->_spi_fd >= 0) {
				ret = ::write(gps->_spi_fd, data1, (size_t)data2);
#endif
			}

			return ret;
		}

	case GPSCallbackType::setBaudrate:
		return gps->setBaudrate(data2);

	case GPSCallbackType::gotRTCMMessage:
		gps->publishRTCMCorrections((uint8_t *)data1, (size_t)data2);
		gps->dumpGpsData((uint8_t *)data1, (size_t)data2, gps_dump_comm_mode_t::RTCM, false);
		break;

	case GPSCallbackType::gotRelativePositionMessage:
		if (data1 && data2 == sizeof(sensor_gnss_relative_s)) {
			gps->publishRelativePosition(*static_cast<sensor_gnss_relative_s *>(data1));
		}

		break;

	case GPSCallbackType::surveyInStatus:
		/* not used */
		break;

	case GPSCallbackType::setClock:

		px4_clock_gettime(CLOCK_REALTIME, &rtc_system_time);
		timespec rtc_gps_time = *(timespec *)data1;
		int drift_time = abs(static_cast<long>(rtc_system_time.tv_sec - rtc_gps_time.tv_sec));

		if (drift_time >= SET_CLOCK_DRIFT_TIME_S) {
			// as of 2021 setting the time on Nuttx temporarily pauses interrupts
			// so only set the time if it is very wrong.
			// TODO: clock slewing of the RTC for small time differences
			px4_clock_settime(CLOCK_REALTIME, &rtc_gps_time);
		}


		break;
	}

	return 0;
}

int GPS::pollOrRead(uint8_t *buf, size_t buf_length, int timeout)
{
	int ret = 0;
	const size_t character_count = 32; // minimum bytes that we want to read
	const int max_timeout = 50;
	int timeout_adjusted = math::min(max_timeout, timeout);

	handleInjectDataTopic();

	if (_interface == GPSHelper::Interface::UART) {
		ret = _uart.readAtLeast(buf, buf_length, math::min(character_count, buf_length), timeout_adjusted);

		if (ret > 0) {
			_num_bytes_read += ret;
		}

// SPI is only supported on LInux
#if defined(__PX4_LINUX)

	} else if ((_interface == GPSHelper::Interface::SPI) && (_spi_fd >= 0)) {

		//Poll only for the SPI data. In the same thread we also need to handle orb messages,
		//so ideally we would poll on both, the SPI fd and orb subscription. Unfortunately the
		//two pollings use different underlying mechanisms (at least under posix), which makes this
		//impossible. Instead we limit the maximum polling interval and regularly check for new orb
		//messages.
		//FIXME: add a unified poll() API

		pollfd fds[1];
		fds[0].fd = _spi_fd;
		fds[0].events = POLLIN;

		ret = poll(fds, sizeof(fds) / sizeof(fds[0]), timeout_adjusted);

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
				unsigned baudrate = _baudrate == 0 ? 115200 : _baudrate;
				const unsigned sleeptime = character_count * 1000000 / (baudrate / 10);

				px4_usleep(sleeptime);

				ret = ::read(_spi_fd, buf, buf_length);

				if (ret > 0) {
					_num_bytes_read += ret;
				}

			} else {
				ret = -1;
			}
		}

#endif
	}

	return ret;
}

void GPS::drainRtcmCorrections()
{
	// rtcm_corrections may have several sources (MAVLink plus CAN nodes), one uORB instance each.
	rtcm_data_s msg;
	bool already_copied = false;

	const hrt_abstime now = hrt_absolute_time();

	// If there has not been a valid RTCM message for a while, try to switch to a different RTCM link
	if (now > _last_rtcm_injection_time + 5_s) {
		for (int instance = 0; instance < _rtcm_corrections_sub.size(); instance++) {
			if (_rtcm_corrections_sub[instance].advertised() && _rtcm_corrections_sub[instance].copy(&msg)) {
				/* Don't select the own RTCM instance. In case it has a lower
				 * instance number, it will be selected and will be rejected
				 * later in the code, resulting in no RTCM injection at all.
				 */
				if (msg.device_id != get_device_id() && now < msg.timestamp + 5_s) {
					already_copied = true;
					_selected_rtcm_instance = instance;
					break;
				}
			}
		}
	}

	bool updated = already_copied;
	size_t num_injections = 0;

	// Limit maximum number of injections per call so a burst can't starve the driver loop.
	do {
		if (updated) {
			num_injections++;

			// Prevent injection of data from self
			if (msg.device_id != get_device_id()) {
				// Add data to the framer buffer for frame reassembly
				if (_rtcm_corrections_framer.addData(msg.data, msg.len) < msg.len) {
					perf_count(_correction_buffer_full_perf);
				}

				_last_rtcm_injection_time = hrt_absolute_time();
			}
		}

		auto &sub = _rtcm_corrections_sub[_selected_rtcm_instance];
		const unsigned last_generation = sub.get_last_generation();

		updated = sub.update(&msg);

		if (updated && sub.get_last_generation() != last_generation + 1) {
			PX4_WARN("%s lost, generation %u -> %u", sub.get_topic()->o_name,
				 last_generation, sub.get_last_generation());
		}
	} while (updated && num_injections < rtcm_data_s::ORB_QUEUE_LENGTH);
}

void GPS::drainMovingBaseline()
{
	// rtcm_moving_baseline has a single publisher (instance 0), so there is no stale-link instance
	// selection - just drain the queue into the parser.
	rtcm_data_s msg;
	size_t num_injections = 0;

	while (num_injections < rtcm_data_s::ORB_QUEUE_LENGTH) {
		const unsigned last_generation = _rtcm_moving_baseline_sub.get_last_generation();

		if (!_rtcm_moving_baseline_sub.update(&msg)) {
			break;
		}

		num_injections++;

		if (_rtcm_moving_baseline_sub.get_last_generation() != last_generation + 1) {
			PX4_WARN("%s lost, generation %u -> %u", _rtcm_moving_baseline_sub.get_topic()->o_name,
				 last_generation, _rtcm_moving_baseline_sub.get_last_generation());
		}

		// Prevent injection of data from self
		if (msg.device_id != get_device_id()) {
			if (_rtcm_moving_baseline_framer.addData(msg.data, msg.len) < msg.len) {
				perf_count(_correction_buffer_full_perf);
			}
		}
	}
}

void GPS::handleInjectDataTopic()
{
	// receiverReady() is false until the receiver is configured, which keeps us from writing to the
	// device mid-configuration.
	if (!_helper->receiverReady()) {
		return;
	}

	// Fixed-base corrections (MAVLink GPS_RTCM_DATA, UAVCAN RTCMStream): RTCM3 and, when enabled,
	// SPARTN framed from one buffer in arrival order. Every configured receiver can use these -
	// including a UART2 moving-base rover, whose baseline arrives in hardware but which still wants
	// fixed-base corrections over its main link.
	drainRtcmCorrections();
	injectRtcmFrames(_rtcm_corrections_framer, _rtcm_corrections_injection_perf,
			 &_rtcm_frames_in_rate_window, &_spartn_frames_in_rate_window);

	// Moving-baseline RTCM (RTCM 4072 etc.) from a peer moving-base GPS. Only a heading rover that
	// receives the baseline through the flight controller (UART1 / CAN) injects it here; a UART2
	// rover gets it in hardware and a moving base produces it. Single publisher, so no instance
	// selection - and a separate framer keeps the two byte streams from interleaving into corrupt
	// frames.
	if (_helper->shouldInjectMovingBaseline()) {
		drainMovingBaseline();
		injectRtcmFrames(_rtcm_moving_baseline_framer, _rtcm_moving_baseline_injection_perf);
	}
}

void GPS::injectRtcmFrames(gnss::CorrectionFramer &framer, perf_counter_t injection_perf,
			   unsigned *rtcm_frames_in_window, unsigned *spartn_frames_in_window)
{
	// Inject all complete frames reassembled in this framer, in arrival order
	size_t frame_len = {};
	const uint8_t *frame_ptr = {};
	gnss::CorrectionProtocol protocol = gnss::CorrectionProtocol::Rtcm3;

	while ((frame_ptr = framer.getNextMessage(&frame_len, &protocol)) != nullptr) {
		// Check TX buffer space before writing
		if (_interface == GPSHelper::Interface::UART) {
			const ssize_t tx_available = _uart.txSpaceAvailable();

			if ((ssize_t)frame_len > tx_available) {
				// TX buffer full, stop and let it drain - frames stay in the framer buffer
				perf_count(_uart_tx_buffer_full_perf);
				break;
			}
		}

		injectData(frame_ptr, frame_len);
		framer.consumeMessage(frame_len);
		perf_count(injection_perf);

		if (protocol == gnss::CorrectionProtocol::Rtcm3) {
			if (rtcm_frames_in_window != nullptr) {
				(*rtcm_frames_in_window)++;
			}

		} else if (spartn_frames_in_window != nullptr) {
			(*spartn_frames_in_window)++;
		}
	}
}

bool GPS::injectData(const uint8_t *data, size_t len)
{
	dumpGpsData(data, len, gps_dump_comm_mode_t::Full, true);

	size_t written = 0;

	if (_interface == GPSHelper::Interface::UART) {
		written = _uart.write((const void *) data, len);

#ifdef __PX4_LINUX

	} else if (_interface == GPSHelper::Interface::SPI) {
		written = ::write(_spi_fd, data, len);
		::fsync(_spi_fd);
#endif
	}

	return written == len;
}

int GPS::setBaudrate(unsigned baud)
{
	if (_interface == GPSHelper::Interface::UART) {
		if (_uart.setBaudrate(baud)) {
			return 0;
		}

#ifdef __PX4_LINUX

	} else if (_interface == GPSHelper::Interface::SPI) {
		// Can't set the baudrate on a SPI port but just return a success
		return 0;
#endif
	}

	return -1;
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

void GPS::dumpGpsData(const uint8_t *data, size_t len, gps_dump_comm_mode_t mode, bool msg_to_gps_device)
{
	gps_dump_s *dump_data  = msg_to_gps_device ? _dump_to_device : _dump_from_device;

	if (_dump_communication_mode != mode || !dump_data) {
		return;
	}

	dump_data->instance = (uint8_t)_instance;
	dump_data->device_id = get_device_id();

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

void
GPS::run()
{
	param_t handle = param_find("GPS_YAW_OFFSET");
	float heading_offset = 0.f;

	if (handle != PARAM_INVALID) {
		param_get(handle, &heading_offset);
		heading_offset = matrix::wrap_pi(math::radians(heading_offset));
	}

#if defined(CONFIG_GPS_UBX)

	int32_t gps_ubx_dynmodel = 7; // default to 7: airborne with <2g acceleration
	handle = param_find("GPS_UBX_DYNMODEL");

	if (handle != PARAM_INVALID) {
		param_get(handle, &gps_ubx_dynmodel);
	}

	int32_t gps_ubx_dgnss_to = 0;
	handle = param_find("GPS_UBX_DGNSS_TO");

	if (handle != PARAM_INVALID) {
		param_get(handle, &gps_ubx_dgnss_to);
	}

	int32_t gps_ubx_min_cno = 0;
	handle = param_find("GPS_UBX_MIN_CNO");

	if (handle != PARAM_INVALID) {
		param_get(handle, &gps_ubx_min_cno);
	}

	int32_t gps_ubx_min_elev = 0;
	handle = param_find("GPS_UBX_MIN_ELEV");

	if (handle != PARAM_INVALID) {
		param_get(handle, &gps_ubx_min_elev);
	}

	int32_t gps_ubx_rate = 0;
	handle = param_find("GPS_UBX_RATE");

	if (handle != PARAM_INVALID) {
		param_get(handle, &gps_ubx_rate);
	}

	handle = param_find("GPS_UBX_MODE");

	GPSDriverUBX::UBXMode ubx_mode{GPSDriverUBX::UBXMode::Normal};

	if (handle != PARAM_INVALID) {
		int32_t gps_ubx_mode = 0;
		param_get(handle, &gps_ubx_mode);

		switch (gps_ubx_mode) {
		case 1:  // heading
			if (_instance == Instance::Main) {
				ubx_mode = GPSDriverUBX::UBXMode::RoverWithMovingBaseUART2;

			} else {
				ubx_mode = GPSDriverUBX::UBXMode::MovingBaseUART2;
			}

			break;

		case 2:
			ubx_mode = GPSDriverUBX::UBXMode::MovingBaseUART2;
			break;

		case 3:
			if (_instance == Instance::Main) {
				ubx_mode = GPSDriverUBX::UBXMode::RoverWithMovingBaseUART1;

			} else {
				ubx_mode = GPSDriverUBX::UBXMode::MovingBaseUART1;
			}

			break;

		case 4:
			ubx_mode = GPSDriverUBX::UBXMode::MovingBaseUART1;
			break;

		case 5:  // rover with static base on Uart2
			ubx_mode = GPSDriverUBX::UBXMode::RoverWithStaticBaseUART2;
			break;

		case 6:
			ubx_mode = GPSDriverUBX::UBXMode::GroundControlStation;
			break;

		default:
			break;

		}
	}

	handle = param_find("GPS_UBX_BAUD2");
	int32_t f9p_uart2_baudrate = 57600;

	if (handle != PARAM_INVALID) {
		param_get(handle, &f9p_uart2_baudrate);
	}

	handle = param_find("GPS_UBX_PPK");
	int32_t ppk_output = 0;

	if (handle != PARAM_INVALID) {
		param_get(handle, &ppk_output);
	}

	handle = param_find("GPS_UBX_JAM_DET");
	int32_t jam_det_sensitivity_hi = 1;

	if (handle != PARAM_INVALID) {
		param_get(handle, &jam_det_sensitivity_hi);
	}

#endif // CONFIG_GPS_UBX

	int32_t gnssSystemsParam = static_cast<int32_t>(GPSHelper::GNSSSystemsMask::RECEIVER_DEFAULTS);

	if (_instance == Instance::Main) {
		handle = param_find("GPS_1_GNSS");
		param_get(handle, &gnssSystemsParam);

	} else if (_instance == Instance::Secondary) {
		handle = param_find("GPS_2_GNSS");
		param_get(handle, &gnssSystemsParam);
	}

	initializeCommunicationDump();

	if (_mode_auto && kAutoDetectModes[0] == gps_driver_mode_t::None) {
#if defined(CONFIG_GPS_NMEA)
		PX4_WARN("none of the enabled GPS protocols supports auto-detection, set GPS_%i_PROTOCOL", (int)_instance + 1);
#else
		PX4_ERR("no GPS protocols enabled");
		return;
#endif
	}

	uint64_t last_rate_measurement = hrt_absolute_time();
	unsigned last_rate_count = 0;

	/* loop handling received serial bytes and also configuring in between */
	while (!should_exit()) {
		if (_helper != nullptr) {
			delete (_helper);
			_helper = nullptr;
		}

		if ((_interface == GPSHelper::Interface::UART) && (! _uart.isOpen())) {

			// Configure UART port
			if (!_uart.setPort(_port)) {
				PX4_ERR("Error configuring serial device on port %s", _port);
				px4_sleep(1);
				continue;
			}

			// Configure the desired baudrate if one was specified by the user.
			// Otherwise the default baudrate will be used.
			if (_configured_baudrate) {
				if (! _uart.setBaudrate(_configured_baudrate)) {
					PX4_ERR("Error setting baudrate to %u on %s", _configured_baudrate, _port);
					px4_sleep(1);
					continue;
				}
			}

			// Open the UART. If this is successful then the UART is ready to use.
			if (! _uart.open()) {
				PX4_ERR("Error opening serial device  %s", _port);
				px4_sleep(1);
				continue;
			}

#ifdef __PX4_LINUX

		} else if ((_interface == GPSHelper::Interface::SPI) && (_spi_fd < 0)) {
			_spi_fd = ::open(_port, O_RDWR | O_NOCTTY);

			if (_spi_fd < 0) {
				PX4_ERR("failed to open SPI port %s err: %d", _port, errno);
				px4_sleep(1);
				continue;
			}

			int spi_speed = 1000000; // make sure the bus speed is not too high (required on RPi)
			int status_value = ::ioctl(_spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);

			if (status_value < 0) {
				PX4_ERR("SPI_IOC_WR_MAX_SPEED_HZ failed for %s (%d)", _port, errno);
			}

			status_value = ::ioctl(_spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);

			if (status_value < 0) {
				PX4_ERR("SPI_IOC_RD_MAX_SPEED_HZ failed for %s (%d)", _port, errno);
			}

#endif /* __PX4_LINUX */
		}

		if (_mode == gps_driver_mode_t::None) {
			_mode = kAutoDetectModes[0];
		}

		switch (_mode) {
#if defined(CONFIG_GPS_UBX)

		case gps_driver_mode_t::UBX: {
				GPSDriverUBX::Settings settings = {
					.dynamic_model = (uint8_t)gps_ubx_dynmodel,
					.dgnss_timeout = (uint8_t)gps_ubx_dgnss_to,
					.min_cno = (uint8_t)gps_ubx_min_cno,
					.min_elev = (int8_t)gps_ubx_min_elev,
					.output_rate = (uint8_t)gps_ubx_rate,
					.heading_offset = heading_offset,
					.uart2_baudrate = f9p_uart2_baudrate,
					.ppk_output = ppk_output > 0,
					.jam_det_sensitivity_hi = jam_det_sensitivity_hi > 0,
					.mode = ubx_mode,
				};

				_helper = new GPSDriverUBX(_interface, &GPS::callback, this, &_sensor_gps, _p_report_sat_info, settings);

				set_device_type(DRV_GPS_DEVTYPE_UBX);
				break;
			}

#endif // CONFIG_GPS_UBX
#if defined(CONFIG_GPS_MTK)

		case gps_driver_mode_t::MTK:
			_helper = new GPSDriverMTK(&GPS::callback, this, &_sensor_gps);
			set_device_type(DRV_GPS_DEVTYPE_MTK);
			break;
#endif // CONFIG_GPS_MTK
#if defined(CONFIG_GPS_ASHTECH)

		case gps_driver_mode_t::ASHTECH:
			_helper = new GPSDriverAshtech(&GPS::callback, this, &_sensor_gps, _p_report_sat_info, heading_offset);
			set_device_type(DRV_GPS_DEVTYPE_ASHTECH);
			break;
#endif // CONFIG_GPS_ASHTECH
#if defined(CONFIG_GPS_EMLIDREACH)

		case gps_driver_mode_t::EMLIDREACH:
			_helper = new GPSDriverEmlidReach(&GPS::callback, this, &_sensor_gps, _p_report_sat_info);
			set_device_type(DRV_GPS_DEVTYPE_EMLID_REACH);
			break;
#endif // CONFIG_GPS_EMLIDREACH
#if defined(CONFIG_GPS_FEMTOMES)

		case gps_driver_mode_t::FEMTOMES:
			_helper = new GPSDriverFemto(&GPS::callback, this, &_sensor_gps, _p_report_sat_info, heading_offset);
			set_device_type(DRV_GPS_DEVTYPE_FEMTOMES);
			break;
#endif // CONFIG_GPS_FEMTOMES
#if defined(CONFIG_GPS_NMEA)

		case gps_driver_mode_t::NMEA:
			_helper = new GPSDriverNMEA(&GPS::callback, this, &_sensor_gps, _p_report_sat_info, heading_offset);
			set_device_type(DRV_GPS_DEVTYPE_NMEA);
			break;
#endif // CONFIG_GPS_NMEA

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

#if defined(CONFIG_GPS_UBX)

		int32_t gps_ubx_cfg_intf = static_cast<int32_t>(GPSHelper::InterfaceProtocolsMask::ALL_DISABLED);
		handle = param_find("GPS_UBX_CFG_INTF");

		if (handle != PARAM_INVALID) {
			param_get(handle, &gps_ubx_cfg_intf);
		}

		gpsConfig.interface_protocols = static_cast<GPSHelper::InterfaceProtocolsMask>(gps_ubx_cfg_intf);

		int32_t gps_cfg_wipe = 0;
		handle = param_find("GPS_CFG_WIPE");

		if (handle != PARAM_INVALID) {
			param_get(handle, &gps_cfg_wipe);
		}

		gpsConfig.cfg_wipe = static_cast<bool>(gps_cfg_wipe) && !_cfg_wiped;

#endif // CONFIG_GPS_UBX

		if (_helper && _helper->configure(_baudrate, gpsConfig) == 0) {

			/* reset report */
			memset(&_sensor_gps, 0, sizeof(_sensor_gps));
			_sensor_gps.heading = NAN;
			_sensor_gps.heading_offset = heading_offset;

#if defined(CONFIG_GPS_UBX)

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

					case GPSDriverUBX::Board::u_blox9_F9P_L1L2:
					case GPSDriverUBX::Board::u_blox9_F9P_L1L5:
						set_device_type(DRV_GPS_DEVTYPE_UBX_F9P);
						break;

					case GPSDriverUBX::Board::u_blox10_L1L5:
						set_device_type(DRV_GPS_DEVTYPE_UBX_10);
						break;

					case GPSDriverUBX::Board::u_blox_X20:
						set_device_type(DRV_GPS_DEVTYPE_UBX_20);
						break;

					default:
						set_device_type(DRV_GPS_DEVTYPE_UBX);
						break;
					}
				}
			}

#endif // CONFIG_GPS_UBX

			int helper_ret;

			/* After being configured (especially in combination with FLASH wipes) the GPS may require
			 * additional time before outputting the first navigation data. To account for this, there is
			 * an init timeout. As soon as the GPS is healthy, the timeout is decreased. This allows for
			 * a quick reaction to a connection loss. */
			unsigned receive_timeout = TIMEOUT_INIT_5HZ;
			unsigned healthy_timeout = TIMEOUT_5HZ;

#if defined(CONFIG_GPS_UBX)

			if ((ubx_mode == GPSDriverUBX::UBXMode::RoverWithMovingBaseUART2)
			    || (ubx_mode == GPSDriverUBX::UBXMode::RoverWithMovingBaseUART1)) {
				/* The MB rover will wait as long as possible to compute a navigation solution,
				 * possibly lowering the navigation rate all the way to 1 Hz while doing so. */
				receive_timeout = TIMEOUT_INIT_1HZ;
				healthy_timeout = TIMEOUT_1HZ;
			}

#endif // CONFIG_GPS_UBX

			if (_dump_communication_mode != gps_dump_comm_mode_t::Disabled) {
				/* Dumping the RTCM3/UBX data requires additional parsing and storing of data via uORB.
				 * Without additional time this can lead to timeouts. */
				healthy_timeout += TIMEOUT_DUMP_ADD;
			}

			PX4_INFO("GPS device configured @ %u baud", _baudrate);

			while ((helper_ret = _helper->receive(receive_timeout)) > 0 && !should_exit()) {

				if (helper_ret & 1) {
					publish();

					last_rate_count++;
				}

				if (_p_report_sat_info && (helper_ret & 2)) {
					publishSatelliteInfo();
				}

				reset_if_scheduled();

				const hrt_abstime now = hrt_absolute_time();

				/* measure update rate every 5 seconds */
				if (now > last_rate_measurement + 5_s) {
					float dt = (float)((now - last_rate_measurement)) / 1e6f;
					_rate = last_rate_count / dt;
					// Report the fixed-base corrections injection rate; moving-baseline injection is
					// tracked separately on its own perf counter.
					const uint64_t corrections_count = perf_event_count(_rtcm_corrections_injection_perf);
					_rtcm_injection_rate = (corrections_count - _last_rtcm_corrections_injection_count) / dt;
					_last_rtcm_corrections_injection_count = corrections_count;
					_injecting_rtcm = _rtcm_frames_in_rate_window > 0;
					_injecting_spartn = _spartn_frames_in_rate_window > 0;
					_rate_reading = _num_bytes_read / dt;
					last_rate_measurement = now;
					last_rate_count = 0;
					_rtcm_frames_in_rate_window = 0;
					_spartn_frames_in_rate_window = 0;
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
					receive_timeout = healthy_timeout;
				}

				/* Do not wipe the FLASH config multiple times. */
#if defined(CONFIG_GPS_UBX)

				if (!_cfg_wiped) {
					_cfg_wiped = true;
				}

#endif
			}

			if (_healthy) {
				_healthy = false;
				_rate = 0.0f;
				_rtcm_injection_rate = 0.0f;
				_injecting_rtcm = false;
				_injecting_spartn = false;
			}
		}

		if (_interface == GPSHelper::Interface::UART) {
			(void) _uart.close();

#ifdef __PX4_LINUX

		} else if ((_interface == GPSHelper::Interface::SPI) && (_spi_fd >= 0)) {
			::close(_spi_fd);
			_spi_fd = -1;
#endif
		}

		if (_mode_auto) {
			size_t i = 0;

			while (kAutoDetectModes[i] != _mode && kAutoDetectModes[i] != gps_driver_mode_t::None) {
				++i;
			}

			if (kAutoDetectModes[i] == gps_driver_mode_t::None
			    || kAutoDetectModes[i + 1] == gps_driver_mode_t::None) {
				_mode = kAutoDetectModes[0];
				px4_usleep(500000); // tried all possible drivers. Wait a bit before next round

			} else {
				_mode = kAutoDetectModes[i + 1];
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
#if defined(CONFIG_GPS_UBX)

	case gps_driver_mode_t::UBX:
		PX4_INFO("protocol: UBX");
		break;
#endif // CONFIG_GPS_UBX
#if defined(CONFIG_GPS_MTK)

	case gps_driver_mode_t::MTK:
		PX4_INFO("protocol: MTK");
		break;
#endif // CONFIG_GPS_MTK
#if defined(CONFIG_GPS_ASHTECH)

	case gps_driver_mode_t::ASHTECH:
		PX4_INFO("protocol: ASHTECH");
		break;
#endif // CONFIG_GPS_ASHTECH
#if defined(CONFIG_GPS_EMLIDREACH)

	case gps_driver_mode_t::EMLIDREACH:
		PX4_INFO("protocol: EMLIDREACH");
		break;
#endif // CONFIG_GPS_EMLIDREACH
#if defined(CONFIG_GPS_FEMTOMES)

	case gps_driver_mode_t::FEMTOMES:
		PX4_INFO("protocol: FEMTOMES");
		break;
#endif // CONFIG_GPS_FEMTOMES
#if defined(CONFIG_GPS_NMEA)

	case gps_driver_mode_t::NMEA:
		PX4_INFO("protocol: NMEA");
#endif // CONFIG_GPS_NMEA

	default:
		break;
	}

	PX4_INFO("status: %s, port: %s, baudrate: %d", _healthy ? "OK" : "NOT OK", _port, _baudrate);
	PX4_INFO("sat info: %s", (_p_report_sat_info != nullptr) ? "enabled" : "disabled");
	// Fixed-width labels so values stay aligned (longest: "rate RTCM injection")
	PX4_INFO("rate reading:        %6i B/s", _rate_reading);

	if (_sensor_gps.timestamp != 0) {
		if (_helper) {
			PX4_INFO("rate position:       %6.2f Hz", (double)_helper->getPositionUpdateRate());
			PX4_INFO("rate velocity:       %6.2f Hz", (double)_helper->getVelocityUpdateRate());
		}

		PX4_INFO("rate publication:    %6.2f Hz", (double)_rate);
		PX4_INFO("rate RTCM injection: %6.2f Hz", (double)_rtcm_injection_rate);

		// _injecting_spartn stays false when CONFIG_GPS_SPARTN is disabled
		const char *corrections = "none";

		if (_injecting_rtcm && _injecting_spartn) {
			corrections = "RTCM + SPARTN";

		} else if (_injecting_rtcm) {
			corrections = "RTCM";

		} else if (_injecting_spartn) {
			corrections = "SPARTN";
		}

		PX4_INFO("corrections:         %s", corrections);

		print_message(ORB_ID(sensor_gps), _sensor_gps);
	}

	perf_print_counter(_uart_tx_buffer_full_perf);
	perf_print_counter(_correction_buffer_full_perf);
	perf_print_counter(_rtcm_corrections_injection_perf);
	perf_print_counter(_rtcm_moving_baseline_injection_perf);

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
		_sensor_gps.device_id = get_device_id();

		_sensor_gps.selected_rtcm_instance = _selected_rtcm_instance;
		_sensor_gps.rtcm_injection_rate = _rtcm_injection_rate;

		_failure_config.update();

		if (!failure_injection::process(_failure_config, failure_injection_s::FAILURE_UNIT_SENSOR_GPS,
						_sensor_gps_pub.get_instance(), _sensor_gps, _stuck)) {
			return;
		}

		_sensor_gps_pub.publish(_sensor_gps);
		// Heading/yaw data can be updated at a lower rate than the other navigation data.
		// The uORB message definition requires this data to be set to a NAN if no new valid data is available.
		_sensor_gps.heading = NAN;
		_is_gps_main_advertised.store(true);
	}
}

void
GPS::publishSatelliteInfo()
{
	if (_instance == Instance::Main || _is_gps_main_advertised.load()) {
		if (_p_report_sat_info != nullptr) {
			_report_sat_info_pub.publish(*_p_report_sat_info);
		}

		_is_gps_main_advertised.store(true);

	} else {
		//we don't publish satellite info for the secondary gps
	}
}

// Chunk an RTCM byte stream into a uORB message and publish it. RTCM frames larger than the
// message payload are split across consecutive publications (flags LSB = fragmented). Templated
// on the publication so the same code serves both the corrections and moving-baseline topics.
template <typename PubT>
static void publish_rtcm_chunks(PubT &pub, const uint8_t *data, size_t len, hrt_abstime timestamp,
				uint32_t device_id)
{
	rtcm_data_s msg{};
	msg.timestamp = timestamp;
	msg.device_id = device_id;

	const size_t capacity = sizeof(msg.data);
	msg.flags = (len > capacity) ? 1 : 0; // LSB: 1=fragmented

	size_t written = 0;

	while (written < len) {
		const size_t chunk = math::min(len - written, capacity);
		msg.len = chunk;
		memcpy(msg.data, &data[written], chunk);
		pub.publish(msg);
		written += chunk;
	}
}

void
GPS::publishRTCMCorrections(uint8_t *data, size_t len)
{
	const hrt_abstime timestamp = hrt_absolute_time();
	const uint32_t device_id = get_device_id();

	// If this GPS is a moving base, its RTCM output is moving-baseline data for a rover (not
	// external corrections). Route it to a dedicated topic so downstream consumers can tell it
	// apart from fixed-base RTCM.
	if (_helper && _helper->isMovingBase()) {
		publish_rtcm_chunks(_rtcm_moving_baseline_pub, data, len, timestamp, device_id);

	} else {
		publish_rtcm_chunks(_rtcm_corrections_pub, data, len, timestamp, device_id);
	}
}

void
GPS::publishRelativePosition(sensor_gnss_relative_s &gnss_relative)
{
	gnss_relative.device_id = get_device_id();
	gnss_relative.timestamp = hrt_absolute_time();
	_sensor_gnss_relative_pub.publish(gnss_relative);
}

int
GPS::custom_command(int argc, char *argv[])
{
	// Check if the driver is running.
	if (!is_running(desc)) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	GPS *_instance = get_instance<GPS>(desc);

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
The available device protocols are selected at build time.

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

	PRINT_MODULE_USAGE_PARAM_STRING('i', "uart", "spi|uart", "GPS interface", true);
	PRINT_MODULE_USAGE_PARAM_STRING('j', "uart", "spi|uart", "secondary GPS interface", true);
	PRINT_MODULE_USAGE_PARAM_STRING('p', nullptr, "ubx|mtk|ash|eml|fem|nmea",
					"GPS protocol (availability depends on build; default from GPS_x_PROTOCOL)", true);

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
		desc.task_id = -1;
		return -errno;
	}

	if (instance == Instance::Main) {
		desc.task_id = task_id;
	}

	return 0;
}

int GPS::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return GPS::instantiate(ac, av);
	}, argc, argv);
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
	const char *device_name = nullptr;
	const char *device_name_secondary = nullptr;
	int baudrate_main = 0;
	int baudrate_secondary = 0;
	GPSHelper::Interface interface = GPSHelper::Interface::UART;
	GPSHelper::Interface interface_secondary = GPSHelper::Interface::UART;
	gps_driver_mode_t mode = gps_driver_mode_t::None;

	bool error_flag = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "b:d:e:g:i:j:p:", &myoptind, &myoptarg)) != EOF) {
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

		case 'i':
			if (!strcmp(myoptarg, "uart")) {
				interface = GPSHelper::Interface::UART;
#ifdef __PX4_LINUX
			} else if (!strcmp(myoptarg, "spi")) {
				interface = GPSHelper::Interface::SPI;
#endif
			} else {
				PX4_ERR("unknown interface: %s", myoptarg);
				error_flag = true;
			}
			break;

		case 'j':
			if (!strcmp(myoptarg, "uart")) {
				interface_secondary = GPSHelper::Interface::UART;
#ifdef __PX4_LINUX
			} else if (!strcmp(myoptarg, "spi")) {
				interface_secondary = GPSHelper::Interface::SPI;
#endif
			} else {
				PX4_ERR("unknown interface for secondary: %s", myoptarg);
				error_flag = true;
			}
			break;

		case 'p': {
			gps_driver_mode_t requested_mode = gps_driver_mode_t::None;
#if defined(CONFIG_GPS_UBX)
			if (!strcmp(myoptarg, "ubx")) {
				requested_mode = gps_driver_mode_t::UBX;
			}
#endif // CONFIG_GPS_UBX
#if defined(CONFIG_GPS_MTK)
			if (!strcmp(myoptarg, "mtk")) {
				requested_mode = gps_driver_mode_t::MTK;
			}
#endif // CONFIG_GPS_MTK
#if defined(CONFIG_GPS_ASHTECH)
			if (!strcmp(myoptarg, "ash")) {
				requested_mode = gps_driver_mode_t::ASHTECH;
			}
#endif // CONFIG_GPS_ASHTECH
#if defined(CONFIG_GPS_EMLIDREACH)
			if (!strcmp(myoptarg, "eml")) {
				requested_mode = gps_driver_mode_t::EMLIDREACH;
			}
#endif // CONFIG_GPS_EMLIDREACH
#if defined(CONFIG_GPS_FEMTOMES)
			if (!strcmp(myoptarg, "fem")) {
				requested_mode = gps_driver_mode_t::FEMTOMES;
			}
#endif // CONFIG_GPS_FEMTOMES
#if defined(CONFIG_GPS_NMEA)
			if (!strcmp(myoptarg, "nmea")) {
				requested_mode = gps_driver_mode_t::NMEA;
			}
#endif // CONFIG_GPS_NMEA

			if (requested_mode == gps_driver_mode_t::None) {
				PX4_ERR("unknown protocol: %s", myoptarg);
				error_flag = true;

			} else {
				mode = requested_mode;
			}

			break;
		}

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

	GPS *gps = nullptr;
	if (instance == Instance::Main) {
		if (Serial::validatePort(device_name)) {
			gps = new GPS(device_name, mode, interface, instance, baudrate_main);

		} else {
			PX4_ERR("invalid device (-d) %s", device_name ? device_name  : "");
		}

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
		if (Serial::validatePort(device_name_secondary)) {
			gps = new GPS(device_name_secondary, mode, interface_secondary, instance, baudrate_secondary);

		} else {
			PX4_ERR("invalid secondary device (-g) %s", device_name_secondary ? device_name_secondary : "");
		}
	}

	return gps;
}

int
gps_main(int argc, char *argv[])
{
	return ModuleBase::main(GPS::desc, argc, argv);
}
