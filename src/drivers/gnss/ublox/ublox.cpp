/****************************************************************************
 *
 *   Copyright (c) 2013-2025 PX4 Development Team. All rights reserved.
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
 * @file ublox.cpp
 * Dedicated, self-contained driver for u-blox GNSS receivers on a serial port.
 *
 * Derived from the generic `gps` driver, pruned to u-blox/UBX only and a single
 * instance. The UBX protocol parser is vendored alongside this shell.
 */

#ifdef __PX4_NUTTX
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#endif

#include <cstdlib>
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
#include <px4_platform_common/time.h>
#include <px4_platform_common/Serial.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/gps_dump.h>
#include <uORB/topics/gps_inject_data.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_gnss_relative.h>

#include <lib/gnss/rtcm.h>

#include "ubx.h"
#include "UbloxFirmwareUpdater.hpp"

using namespace device;
using namespace time_literals;

#define TIMEOUT_1HZ		1300 //!< Timeout time in mS, 1000 mS (1Hz) + 300 mS delta for error
#define TIMEOUT_5HZ		500 //!< Timeout time in mS,  200 mS (5Hz) + 300 mS delta for error
#define TIMEOUT_INIT_1HZ	(3 * TIMEOUT_1HZ) //!< Timeout time in mS, used until GPS is healthy
#define TIMEOUT_INIT_5HZ	(3 * TIMEOUT_5HZ) //!< Timeout time in mS, used until GPS is healthy
#define TIMEOUT_DUMP_ADD	450 //!< Additional time in mS to account for RTCM3 parsing and dumping

enum class gps_dump_comm_mode_t : int32_t {
	Disabled = 0,
	Full, ///< dump full RX and TX data for all devices
	RTCM ///< dump received RTCM from GPS
};

/* struct for dynamic allocation of satellite info data */
struct GPS_Sat_Info {
	satellite_info_s _data;
};

static constexpr int TASK_STACK_SIZE = PX4_STACK_ADJUSTED(2040);


class Ublox : public ModuleBase, public device::Device
{
public:

	Ublox(const char *path, unsigned configured_baudrate);
	~Ublox() override;

	static Descriptor desc;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Ublox *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** task spawn trampoline */
	static int run_trampoline(int argc, char *argv[]);

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

	/**
	 * Request a firmware update from the FC's DroneCAN file server. Latches the
	 * request; the run loop leaves the nav loop and performs it on this thread.
	 * @param server_node_id file server node id (1..127)
	 * @param path version-gated query path (e.g. "module/ublox-f9p@1.32")
	 */
	int request_firmware_update(uint8_t server_node_id, const char *path);

private:
	Serial				_uart {};					///< UART interface to GPS
	unsigned			_baudrate{0};					///< current baudrate
	const unsigned			_configured_baudrate{0};			///< configured baudrate (0=auto-detect)
	char				_port[20] {};					///< device / serial port path

	bool				_healthy{false};				///< flag to signal if the GPS is ok
	bool				_cfg_wiped{false};				///< flag to signal if the config was already wiped

	GPSHelper			*_helper{nullptr};				///< instance of GPS parser

	GPS_Sat_Info			*_sat_info{nullptr};				///< instance of GPS sat info data object

	sensor_gps_s			_sensor_gps{};				///< uORB topic for gps position
	satellite_info_s		*_p_report_sat_info{nullptr};			///< pointer to uORB topic for satellite info

	uORB::PublicationMulti<sensor_gps_s>	_sensor_gps_pub{ORB_ID(sensor_gps)};	///< uORB pub for gps position
	uORB::PublicationMulti<sensor_gnss_relative_s> _sensor_gnss_relative_pub{ORB_ID(sensor_gnss_relative)};

	uORB::PublicationMulti<satellite_info_s>	_report_sat_info_pub{ORB_ID(satellite_info)};		///< uORB pub for satellite info

	float				_rate{0.0f};					///< position update rate
	float				_rtcm_injection_rate{0.0f};			///< RTCM message injection rate
	unsigned			_rtcm_injection_rate_message_count{0};		///< counter for number of RTCM messages
	unsigned			_num_bytes_read{0}; 				///< counter for number of read bytes from the UART (within update interval)
	unsigned			_rate_reading{0}; 				///< reading rate in B/s
	hrt_abstime			_last_rtcm_injection_time{0};			///< time of last rtcm injection
	uint8_t				_selected_rtcm_instance{0};			///< uorb instance that is being used for RTCM corrections

	uORB::SubscriptionMultiArray<gps_inject_data_s, gps_inject_data_s::MAX_INSTANCES> _orb_inject_data_sub{ORB_ID::gps_inject_data};
	uORB::Publication<gps_inject_data_s> _gps_inject_data_pub{ORB_ID(gps_inject_data)};
	uORB::Publication<gps_dump_s>	     _dump_communication_pub{ORB_ID(gps_dump)};
	gps_dump_s			     *_dump_to_device{nullptr};
	gps_dump_s			     *_dump_from_device{nullptr};
	gps_dump_comm_mode_t                 _dump_communication_mode{gps_dump_comm_mode_t::Disabled};

	gnss::Rtcm3Parser		     _rtcm_parser{};

	perf_counter_t _uart_tx_buffer_full_perf{perf_alloc(PC_COUNT, MODULE_NAME": tx buf full")};
	perf_counter_t _rtcm_buffer_full_perf{perf_alloc(PC_COUNT, MODULE_NAME": rtcm buf full")};

	px4::atomic<int> _scheduled_reset{(int)GPSRestartType::None};

	// Firmware-update request, latched by request_firmware_update() and consumed
	// by the run loop. _fw_update_path / _fw_update_node_id are written before
	// the flag is set and read after it is observed, so the atomic orders them.
	px4::atomic_bool _fw_update_requested{false};
	char             _fw_update_path[128] {};
	uint8_t          _fw_update_node_id{0};

	/**
	 * Run a firmware update on this thread (blocks nav until done). Drives the
	 * shared helper with the device-specific UbloxFirmwareUpdater.
	 */
	void run_firmware_update();

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
	 * Publish relative position
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

ModuleBase::Descriptor Ublox::desc{task_spawn, custom_command, print_usage};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ublox_main(int argc, char *argv[]);


Ublox::Ublox(const char *path, unsigned configured_baudrate) :
	Device(MODULE_NAME),
	_configured_baudrate(configured_baudrate)
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

	set_device_bus_type(device::Device::DeviceBusType::DeviceBusType_SERIAL);

	char c = _port[strlen(_port) - 1]; // last digit of path (eg /dev/ttyS2)
	set_device_bus(c - 48); // sub 48 to convert char to integer
}

Ublox::~Ublox()
{
	perf_free(_uart_tx_buffer_full_perf);
	perf_free(_rtcm_buffer_full_perf);

	delete _sat_info;
	delete _dump_to_device;
	delete _dump_from_device;
	delete _helper;
}

int Ublox::callback(GPSCallbackType type, void *data1, int data2, void *user)
{
	Ublox *gps = (Ublox *)user;

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

	case GPSCallbackType::writeDeviceData:
		gps->dumpGpsData((uint8_t *)data1, (size_t)data2, gps_dump_comm_mode_t::Full, true);

		return gps->_uart.write((void *) data1, (size_t) data2);

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

int Ublox::pollOrRead(uint8_t *buf, size_t buf_length, int timeout)
{
	const size_t character_count = 32; // minimum bytes that we want to read
	const int max_timeout = 50;
	int timeout_adjusted = math::min(max_timeout, timeout);

	handleInjectDataTopic();

	int ret = _uart.readAtLeast(buf, buf_length, math::min(character_count, buf_length), timeout_adjusted);

	if (ret > 0) {
		_num_bytes_read += ret;
	}

	return ret;
}

void Ublox::handleInjectDataTopic()
{
	if (!_helper->shouldInjectRTCM()) {
		return;
	}

	// We don't want to call copy again further down if we have already done a
	// copy in the selection process.
	bool already_copied = false;
	gps_inject_data_s msg;

	const hrt_abstime now = hrt_absolute_time();

	// If there has not been a valid RTCM message for a while, try to switch to a different RTCM link
	if (now > _last_rtcm_injection_time + 5_s) {

		for (int instance = 0; instance < _orb_inject_data_sub.size(); instance++) {
			const bool exists = _orb_inject_data_sub[instance].advertised();

			if (exists && _orb_inject_data_sub[instance].copy(&msg)) {
				/* Don't select the own RTCM instance. In case it has a lower
				 * instance number, it will be selected and will be rejected
				 * later in the code, resulting in no RTCM injection at all.
				 */
				if (msg.device_id != get_device_id()) {
					// Only use the message if it is up to date
					if (now < msg.timestamp + 5_s) {
						// Remember that we already did a copy on this instance.
						already_copied = true;
						_selected_rtcm_instance = instance;
						break;
					}
				}
			}
		}
	}

	bool updated = already_copied;

	// Limit maximum number of GPS injections to 8 since usually
	// GPS injections should consist of 1-4 packets (GPS, Glonass, BeiDou, Galileo).
	// Looking at 8 packets thus guarantees, that at least a full injection
	// data set is evaluated.
	// Moving Base reuires a higher rate, so we allow up to 8 packets.
	// Drain uORB messages into RTCM parser and inject full messages after draining the queue.
	const size_t max_num_injections = gps_inject_data_s::ORB_QUEUE_LENGTH;
	size_t num_injections = 0;

	do {
		if (updated) {
			num_injections++;

			// Prevent injection of data from self
			if (msg.device_id != get_device_id()) {
				// Add data to the RTCM parser buffer for frame reassembly
				size_t added = _rtcm_parser.addData(msg.data, msg.len);

				if (added < msg.len) {
					perf_count(_rtcm_buffer_full_perf);
				}

				_last_rtcm_injection_time = hrt_absolute_time();
			}
		}

		auto &gps_inject_data_sub = _orb_inject_data_sub[_selected_rtcm_instance];

		const unsigned last_generation = gps_inject_data_sub.get_last_generation();

		updated = gps_inject_data_sub.update(&msg);

		if (updated) {
			if (gps_inject_data_sub.get_last_generation() != last_generation + 1) {
				PX4_WARN("gps_inject_data lost, generation %u -> %u", last_generation, gps_inject_data_sub.get_last_generation());
			}
		}

	} while (updated && num_injections < max_num_injections);

	// Now inject all complete RTCM frames from the parser buffer
	size_t frame_len = {};
	const uint8_t *frame_ptr = {};

	while ((frame_ptr = _rtcm_parser.getNextMessage(&frame_len)) != nullptr) {
		// Check TX buffer space before writing
		ssize_t tx_available = _uart.txSpaceAvailable();

		if ((ssize_t)frame_len > tx_available) {
			// TX buffer full, stop and let it drain - frames stay in parser buffer
			perf_count(_uart_tx_buffer_full_perf);
			break;
		}

		injectData(frame_ptr, frame_len);
		_rtcm_parser.consumeMessage(frame_len);
		_rtcm_injection_rate_message_count++;
	}
}

bool Ublox::injectData(const uint8_t *data, size_t len)
{
	dumpGpsData(data, len, gps_dump_comm_mode_t::Full, true);

	size_t written = _uart.write((const void *) data, len);

	return written == len;
}

int Ublox::setBaudrate(unsigned baud)
{
	if (_uart.setBaudrate(baud)) {
		return 0;
	}

	return -1;
}

void Ublox::initializeCommunicationDump()
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

void Ublox::dumpGpsData(const uint8_t *data, size_t len, gps_dump_comm_mode_t mode, bool msg_to_gps_device)
{
	gps_dump_s *dump_data  = msg_to_gps_device ? _dump_to_device : _dump_from_device;

	if (_dump_communication_mode != mode || !dump_data) {
		return;
	}

	dump_data->instance = 0;
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
Ublox::run()
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
			ubx_mode = GPSDriverUBX::UBXMode::RoverWithMovingBase;
			break;

		case 2:
			ubx_mode = GPSDriverUBX::UBXMode::MovingBase;
			break;

		case 3:
			ubx_mode = GPSDriverUBX::UBXMode::RoverWithMovingBaseUART1;
			break;

		case 4:
			ubx_mode = GPSDriverUBX::UBXMode::MovingBaseUART1;
			break;

		case 5:  // rover with static base on Uart2
			ubx_mode = GPSDriverUBX::UBXMode::RoverWithStaticBaseUart2;
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

	int32_t gnssSystemsParam = static_cast<int32_t>(GPSHelper::GNSSSystemsMask::RECEIVER_DEFAULTS);

	handle = param_find("GPS_1_GNSS");
	param_get(handle, &gnssSystemsParam);

	initializeCommunicationDump();

	uint64_t last_rate_measurement = hrt_absolute_time();
	unsigned last_rate_count = 0;

	/* loop handling received serial bytes and also configuring in between */
	while (!should_exit()) {
		if (_helper != nullptr) {
			delete (_helper);
			_helper = nullptr;
		}

		if (!_uart.isOpen()) {

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
		}

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

		_helper = new GPSDriverUBX(GPSHelper::Interface::UART, &Ublox::callback, this, &_sensor_gps, _p_report_sat_info,
					   settings);

		set_device_type(DRV_GPS_DEVTYPE_UBX);

		_baudrate = _configured_baudrate;
		GPSHelper::GPSConfig gpsConfig{};
		gpsConfig.gnss_systems = static_cast<GPSHelper::GNSSSystemsMask>(gnssSystemsParam);

		if (_dump_communication_mode == gps_dump_comm_mode_t::RTCM) {
			gpsConfig.output_mode = GPSHelper::OutputMode::GPSAndRTCM;

		} else {
			gpsConfig.output_mode = GPSHelper::OutputMode::GPS;
		}

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

		if (_helper && _helper->configure(_baudrate, gpsConfig) == 0) {

			/* reset report */
			memset(&_sensor_gps, 0, sizeof(_sensor_gps));
			_sensor_gps.heading = NAN;
			_sensor_gps.heading_offset = heading_offset;

			/* GPS is obviously detected successfully, reset statistics */
			_helper->resetUpdateRates();

			// populate specific ublox model
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

			int helper_ret;

			/* After being configured (especially in combination with FLASH wipes) the GPS may require
			 * additional time before outputting the first navigation data. To account for this, there is
			 * an init timeout. As soon as the GPS is healthy, the timeout is decreased. This allows for
			 * a quick reaction to a connection loss. */
			unsigned receive_timeout = TIMEOUT_INIT_5HZ;
			unsigned healthy_timeout = TIMEOUT_5HZ;

			if ((ubx_mode == GPSDriverUBX::UBXMode::RoverWithMovingBase)
			    || (ubx_mode == GPSDriverUBX::UBXMode::RoverWithMovingBaseUART1)) {
				/* The MB rover will wait as long as possible to compute a navigation solution,
				 * possibly lowering the navigation rate all the way to 1 Hz while doing so. */
				receive_timeout = TIMEOUT_INIT_1HZ;
				healthy_timeout = TIMEOUT_1HZ;
			}

			if (_dump_communication_mode != gps_dump_comm_mode_t::Disabled) {
				/* Dumping the RTCM3/UBX data requires additional parsing and storing of data via uORB.
				 * Without additional time this can lead to timeouts. */
				healthy_timeout += TIMEOUT_DUMP_ADD;
			}

			PX4_INFO("u-blox GPS device configured @ %u baud", _baudrate);

			while ((helper_ret = _helper->receive(receive_timeout)) > 0 && !should_exit()) {

				if (helper_ret & 1) {
					publish();

					last_rate_count++;
				}

				if (_p_report_sat_info && (helper_ret & 2)) {
					publishSatelliteInfo();
				}

				reset_if_scheduled();

				// Leave the nav loop to service a firmware-update request.
				if (_fw_update_requested.load()) {
					break;
				}

				const hrt_abstime now = hrt_absolute_time();

				/* measure update rate every 5 seconds */
				if (now > last_rate_measurement + 5_s) {
					float dt = (float)((now - last_rate_measurement)) / 1e6f;
					_rate = last_rate_count / dt;
					_rtcm_injection_rate = _rtcm_injection_rate_message_count / dt;
					_rate_reading = _num_bytes_read / dt;
					last_rate_measurement = now;
					last_rate_count = 0;
					_rtcm_injection_rate_message_count = 0;
					_num_bytes_read = 0;
					_helper->storeUpdateRates();
					_helper->resetUpdateRates();
				}

				if (!_healthy) {
					_healthy = true;
					receive_timeout = healthy_timeout;
				}

				/* Do not wipe the FLASH config multiple times. */
				if (!_cfg_wiped) {
					_cfg_wiped = true;
				}
			}

			if (_healthy) {
				_healthy = false;
				_rate = 0.0f;
				_rtcm_injection_rate = 0.0f;
			}
		}

		// Perform a pending firmware update before tearing down the link. Runs
		// even if configure() failed above, to allow recovery of a bricked module.
		if (_fw_update_requested.load()) {
			run_firmware_update();
			_fw_update_requested.store(false);
		}

		(void) _uart.close();

		px4_usleep(500000);
	}

	PX4_INFO("exiting");
}

void
Ublox::run_firmware_update()
{
	UbloxFirmwareUpdater updater;
	module_fw_update::ModuleFirmwareUpdater helper(updater);

	PX4_INFO("ublox: starting firmware update from node %u, path '%s'", _fw_update_node_id, _fw_update_path);

	const module_fw_update::ModuleFirmwareUpdater::Result res = helper.update(_fw_update_path, _fw_update_node_id);

	switch (res) {
	case module_fw_update::ModuleFirmwareUpdater::Result::Success:
		PX4_INFO("ublox: firmware update complete (%lu B)", (unsigned long)helper.bytesWritten());
		break;

	case module_fw_update::ModuleFirmwareUpdater::Result::UpToDate:
		PX4_INFO("ublox: firmware already up to date");
		break;

	case module_fw_update::ModuleFirmwareUpdater::Result::Failed:
		PX4_ERR("ublox: firmware update failed");
		break;
	}
}

int
Ublox::request_firmware_update(uint8_t server_node_id, const char *path)
{
	if (server_node_id == 0 || server_node_id > 127) {
		return -EINVAL;
	}

	if (_fw_update_requested.load()) {
		return -EBUSY;
	}

	strncpy(_fw_update_path, path, sizeof(_fw_update_path) - 1);
	_fw_update_path[sizeof(_fw_update_path) - 1] = '\0';
	_fw_update_node_id = server_node_id;
	_fw_update_requested.store(true);

	return PX4_OK;
}

int
Ublox::print_status()
{
	PX4_INFO("protocol: UBX");

	PX4_INFO("status: %s, port: %s, baudrate: %d", _healthy ? "OK" : "NOT OK", _port, _baudrate);
	PX4_INFO("sat info: %s", (_p_report_sat_info != nullptr) ? "enabled" : "disabled");
	PX4_INFO("rate reading: \t\t%6i B/s", _rate_reading);

	if (_sensor_gps.timestamp != 0) {
		if (_helper) {
			PX4_INFO("rate position: \t\t%6.2f Hz", (double)_helper->getPositionUpdateRate());
			PX4_INFO("rate velocity: \t\t%6.2f Hz", (double)_helper->getVelocityUpdateRate());
		}

		PX4_INFO("rate publication:\t\t%6.2f Hz", (double)_rate);
		PX4_INFO("rate RTCM injection:\t%6.2f Hz", (double)_rtcm_injection_rate);

		print_message(ORB_ID(sensor_gps), _sensor_gps);
	}

	perf_print_counter(_uart_tx_buffer_full_perf);
	perf_print_counter(_rtcm_buffer_full_perf);

	return 0;
}

void
Ublox::schedule_reset(GPSRestartType restart_type)
{
	_scheduled_reset.store((int)restart_type);
}

void
Ublox::reset_if_scheduled()
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
Ublox::publish()
{
	_sensor_gps.device_id = get_device_id();

	_sensor_gps.selected_rtcm_instance = _selected_rtcm_instance;
	_sensor_gps.rtcm_injection_rate = _rtcm_injection_rate;

	_sensor_gps_pub.publish(_sensor_gps);
	// Heading/yaw data can be updated at a lower rate than the other navigation data.
	// The uORB message definition requires this data to be set to a NAN if no new valid data is available.
	_sensor_gps.heading = NAN;
}

void
Ublox::publishSatelliteInfo()
{
	if (_p_report_sat_info != nullptr) {
		_report_sat_info_pub.publish(*_p_report_sat_info);
	}
}

void
Ublox::publishRTCMCorrections(uint8_t *data, size_t len)
{
	gps_inject_data_s gps_inject_data{};

	gps_inject_data.timestamp = hrt_absolute_time();
	gps_inject_data.device_id = get_device_id();

	size_t capacity = (sizeof(gps_inject_data.data) / sizeof(gps_inject_data.data[0]));

	if (len > capacity) {
		gps_inject_data.flags = 1; //LSB: 1=fragmented

	} else {
		gps_inject_data.flags = 0;
	}

	size_t written = 0;

	while (written < len) {

		gps_inject_data.len = len - written;

		if (gps_inject_data.len > capacity) {
			gps_inject_data.len = capacity;
		}

		memcpy(gps_inject_data.data, &data[written], gps_inject_data.len);

		_gps_inject_data_pub.publish(gps_inject_data);

		written = written + gps_inject_data.len;
	}
}

void
Ublox::publishRelativePosition(sensor_gnss_relative_s &gnss_relative)
{
	gnss_relative.device_id = get_device_id();
	gnss_relative.timestamp = hrt_absolute_time();
	_sensor_gnss_relative_pub.publish(gnss_relative);
}

int
Ublox::custom_command(int argc, char *argv[])
{
	// Check if the driver is running.
	if (!is_running(desc)) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	Ublox *_instance = get_instance<Ublox>(desc);

	if (argc >= 2 && !strcmp(argv[0], "fw_update")) {
		const int node_id = atoi(argv[1]);
		const char *path = (argc >= 3) ? argv[2] : "ublox.bin";
		const int ret = _instance->request_firmware_update((uint8_t)node_id, path);

		if (ret == PX4_OK) {
			PX4_INFO("ublox: firmware update requested (node %d, path '%s')", node_id, path);
			return 0;
		}

		PX4_ERR("ublox: firmware update request failed (%d)", ret);
		return ret;
	}

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

int Ublox::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Dedicated driver for u-blox GNSS receivers. Handles the communication with the
device and publishes the position via uORB. The UBX protocol parser is vendored
in-tree (no PX4-GPSDrivers submodule).

### Implementation
There is a dedicated thread polling for data. The u-blox protocol class is
implemented with callbacks so it can be reused in other projects as well.

### Examples

Starting the driver on /dev/ttyS0:
$ ublox start -d /dev/ttyS0

Initiate warm restart of GPS device
$ ublox reset warm

Pull a firmware image from the FC's DroneCAN file server (node 1) and flash it
$ ublox fw_update 1 module/ublox-f9p@1.32
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ublox", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS0", "<file:dev>", "GPS device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 0, 0, 3000000, "Baudrate (can also be p:<param_name>)", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset GPS device");
	PRINT_MODULE_USAGE_ARG("cold|warm|hot", "Specify reset type", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("fw_update", "Pull & flash firmware over DroneCAN");
	PRINT_MODULE_USAGE_ARG("<server_node_id> [<path>]", "FC file server node id and (optional) image path", false);

	return 0;
}

int Ublox::task_spawn(int argc, char *argv[])
{
	int task_id = px4_task_spawn_cmd("ublox", SCHED_DEFAULT,
					 SCHED_PRIORITY_SLOW_DRIVER, TASK_STACK_SIZE,
					 (px4_main_t)&run_trampoline, (char *const *)argv);

	if (task_id < 0) {
		desc.task_id = -1;
		return -errno;
	}

	desc.task_id = task_id;

	return 0;
}

int Ublox::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return Ublox::instantiate(ac, av);
	}, argc, argv);
}

Ublox *Ublox::instantiate(int argc, char *argv[])
{
	const char *device_name = nullptr;
	int baudrate_main = 0;

	bool error_flag = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "b:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			if (px4_get_parameter_value(myoptarg, baudrate_main) != 0) {
				PX4_ERR("baudrate parsing failed");
				error_flag = true;
			}

			break;

		case 'd':
			device_name = myoptarg;
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

	Ublox *gps = nullptr;

	if (Serial::validatePort(device_name)) {
		gps = new Ublox(device_name, baudrate_main);

	} else {
		PX4_ERR("invalid device (-d) %s", device_name ? device_name  : "");
	}

	return gps;
}

int
ublox_main(int argc, char *argv[])
{
	return ModuleBase::main(Ublox::desc, argc, argv);
}
