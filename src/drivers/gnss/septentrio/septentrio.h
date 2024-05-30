/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file septentrio.h
 *
 * Septentrio GNSS receiver driver
 *
 * @author Matej Franceskin <Matej.Franceskin@gmail.com>
 * @author <a href="https://github.com/SeppeG">Seppe Geuens</a>
 * @author Thomas Frans
*/

#pragma once

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/Serial.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/satellite_info.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/gps_dump.h>
#include <uORB/topics/gps_inject_data.h>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/parameters/param.h>

#include "module.h"
#include "sbf/decoder.h"
#include "rtcm.h"

using namespace time_literals;

namespace septentrio
{

/**
 * The parsed command line arguments for this module.
 */
struct ModuleArguments {
	int baud_rate_main {0};
	int baud_rate_secondary {0};
	const char *device_path_main {nullptr};
	const char *device_path_secondary {nullptr};
};

/**
 * Which raw communication data to dump to the log file.
*/
enum class DumpMode : int32_t {
	Disabled = 0,
	FromReceiver = 1,
	ToReceiver = 2,
	Both = 3,
};

/**
 * Instance of the driver.
*/
enum class Instance : uint8_t {
	Main = 0,
	Secondary,
};

/**
 * Hardware setup and use of the connected receivers.
*/
enum class ReceiverSetup {
	/// Two receivers with the same purpose.
	Default    = 0,

	/// One rover and one moving base receiver, used for heading with two single-antenna receivers.
	MovingBase = 1,
};

/**
 * Type of reset to perform on the receiver.
*/
enum class ReceiverResetType {
	/**
	 * There is no pending GPS reset.
	 */
	None,

	/**
	 * In hot start mode, the receiver was powered down only for a short time (4 hours or less),
	 * so that its ephemeris is still valid. Since the receiver doesn't need to download ephemeris
	 * again, this is the fastest startup method.
	 */
	Hot,

	/**
	 * In warm start mode, the receiver has approximate information for time, position, and coarse
	 * satellite position data (Almanac). In this mode, after power-up, the receiver normally needs
	 * to download ephemeris before it can calculate position and velocity data.
	 */
	Warm,

	/**
	 * In cold start mode, the receiver has no information from the last position at startup.
	 * Therefore, the receiver must search the full time and frequency space, and all possible
	 * satellite numbers. If a satellite signal is found, it is tracked to decode the ephemeris,
	 * whereas the other channels continue to search satellites. Once there is a sufficient number
	 * of satellites with valid ephemeris, the receiver can calculate position and velocity data.
	 */
	Cold
};

/**
 * Direction of raw data.
*/
enum class DataDirection {
	/// Data sent by the flight controller to the receiver.
	ToReceiver,

	/// Data sent by the receiver to the flight controller.
	FromReceiver,
};

/**
 * Which satellites the receiver should use for PVT computation.
*/
enum class SatelliteUsage : int32_t {
	Default = 0,
	GPS     = 1 << 0,
	GLONASS = 1 << 1,
	Galileo = 1 << 2,
	SBAS    = 1 << 3,
	BeiDou  = 1 << 4,
};

/**
 * General logging level of the receiver that determines the blocks to log.
*/
enum class ReceiverLogLevel : int32_t {
	Lite    = 0,
	Basic   = 1,
	Default = 2,
	Full    = 3,
};

/**
 * Logging frequency of the receiver that determines SBF output frequency.
*/
enum class ReceiverLogFrequency : int32_t {
	Disabled = 0,
	Hz0_1    = 1,
	Hz0_2    = 2,
	Hz0_5    = 3,
	Hz1_0    = 4,
	Hz2_0    = 5,
	Hz5_0    = 6,
	Hz10_0   = 7,
	Hz20_0   = 8,
	Hz25_0   = 9,
	Hz50_0   = 10,
};

/**
 * Output frequency for the main SBF blocks required for PVT computation.
*/
enum class SBFOutputFrequency : int32_t {
	Hz5_0    = 0,
	Hz10_0   = 1,
	Hz20_0   = 2,
	Hz25_0   = 3,
};

/**
 * Tracker for messages received by the driver.
*/
struct MessageTracker {
	bool dop {false};
	bool pvt_geodetic {false};
	bool vel_cov_geodetic {false};
	bool att_euler {false};
	bool att_cov_euler {false};
};

/**
 * Used for a bitmask to keep track of received messages to know when we need to broadcast them and to monitor receiver health.
*/
enum class ReceiverOutputTracker {
	None                     = 0,
	DOP                      = 1 << 0,
	PVTGeodetic              = 1 << 1,
	VelCovGeodetic           = 1 << 2,
	AttEuler                 = 1 << 3,
	AttCovEuler              = 1 << 4,
	HeadingMessages          = AttEuler + AttCovEuler,
	RequiredPositionMessages = DOP + PVTGeodetic + VelCovGeodetic + AttCovEuler,
	PositioningMessages      = DOP + PVTGeodetic + VelCovGeodetic + AttEuler + AttCovEuler,
};

class SeptentrioDriver : public ModuleBase<SeptentrioDriver>, public device::Device
{
public:
	SeptentrioDriver(const char *device_path, Instance instance, uint32_t baud_rate);
	~SeptentrioDriver() override;

	/** @see ModuleBase */
	int print_status() override;

	/** @see ModuleBase */
	void run() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	static int task_spawn(int argc, char *argv[], Instance instance);

	/**
	 * @brief Secondary run trampoline to support two driver instances.
	 */
	static int run_trampoline_secondary(int argc, char *argv[]);

	/** @see ModuleBase */
	static SeptentrioDriver *instantiate(int argc, char *argv[]);

	static SeptentrioDriver *instantiate(int argc, char *argv[], Instance instance);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/**
	 * @brief Reset the connected GPS receiver.
	 *
	 * @return `PX4_OK` on success, `PX4_ERROR` on otherwise
	 */
	int reset(ReceiverResetType type);

	/**
	 * @brief Force command input on the currently used port on the receiver.
	 *
	 * @return `PX4_OK` on success, `PX4_ERROR` otherwise
	 */
	int force_input();
private:
	enum class State {
		OpeningSerialPort,
		ConfiguringDevice,
		ReceivingData,
	};

	/**
	 * The current decoder that data has to be fed to.
	*/
	enum class DecodingStatus {
		Searching,
		SBF,
		RTCMv3,
	};

	enum class ReceiveResult {
		ReadError,
		Timeout,
		Receiving,
		MessageAvailable,
	};

	/**
	 * Maximum timeout to wait for command acknowledgement by the receiver.
	*/
	static constexpr uint16_t k_receiver_ack_timeout = 200;

	/**
	 * Duration of one update monitoring interval in us.
	 * This should be longer than the time it takes for all *positioning* SBF messages to be sent once by the receiver!
	 * Otherwise the driver will assume the receiver configuration isn't healthy because it doesn't see all blocks in time.
	*/
	static constexpr hrt_abstime k_update_monitoring_interval_duration = 5_s;

	/**
	 * The default stream for output of PVT data.
	*/
	static constexpr uint8_t k_default_main_stream = 1;

	/**
	 * The default stream for output of logging data.
	*/
	static constexpr uint8_t k_default_log_stream = 2;

	/**
	 * @brief Parse command line arguments for this module.
	 *
	 * @param argc Number of arguments.
	 * @param argv The arguments.
	 * @param arguments The parsed arguments.
	 *
	 * @return `PX4_OK` on success, `PX4_ERROR` on a parsing error.
	*/
	static int parse_cli_arguments(int argc, char *argv[], ModuleArguments &arguments);

	/**
	 * @brief Wait for the second instance to properly start up.
	 *
	 * @return `PX4_OK` once the second instance has started, or `PX4_ERROR` if timed out waiting.
	*/
	static int await_second_instance_startup();

	/**
	 * @brief Wait for the second instance to properly shut down.
	 *
	 * @return `PX4_OK` once the second instance has shut down, or `PX4_ERROR` if timed out waiting.
	*/
	int await_second_instance_shutdown();

	/**
	 * @brief Schedule a reset of the connected receiver.
	 */
	void schedule_reset(ReceiverResetType type);

	/**
	 * @brief Detect the current baud rate used by the receiver on the connected port.
	 *
	 * @param force_input Choose whether the receiver forces input on the port
	 *
	 * @return The detected baud rate on success, or `0` on error
	 */
	uint32_t detect_receiver_baud_rate(bool forced_input);

	/**
	 * @brief Try to detect the serial port used on the receiver side.
	 *
	 * @param port_name A string with a length of 5 to store the result
	 *
	 * @return `PX4_OK` on success, `PX4_ERROR` on error
	*/
	int detect_serial_port(char *const port_name);

	/**
	 * @brief Configure the attached receiver based on the user's parameters.
	 *
	 * If the user has disabled automatic configuration, only execute the steps that do not touch the receiver (e.g., baud rate detection, port detection...).
	 *
	 * @return `PX4_OK` on success, `PX4_ERROR` otherwise.
	 */
	int configure();

	/**
	 * @brief Parse the next byte of a received message from the receiver.
	 *
	 * @return 0 = decoding, 1 = message handled, 2 = sat info message handled
	 */
	int parse_char(const uint8_t byte);

	/**
	 * @brief Process a fully received message from the receiver.
	 *
	 * @return `PX4_OK` on message handled, `PX4_ERROR` on error.
	*/
	int process_message();

	/**
	 * @brief Add payload rx byte.
	 *
	 * @return -1 = error, 0 = ok, 1 = payload received completely
	 */
	int payload_rx_add(const uint8_t byte);

	/**
	 * @brief Parses incoming SBF blocks.
	 *
	 * @return bitfield: all 0 = no message handled, 1 = position handled, 2 = satellite info handled
	 */
	int payload_rx_done();

	/**
	 * @brief Send a message.
	 *
	 * @return true on success, false on write error (errno set)
	 */
	[[nodiscard]] bool send_message(const char *msg);

	/**
	 * @brief Send a message and waits for acknowledge.
	 *
	 * @param msg The message to send to the receiver
	 * @param timeout The time before sending the message times out
	 *
	 * @return true on success, false on write error (errno set) or ack wait timeout
	 */
	[[nodiscard]] bool send_message_and_wait_for_ack(const char *msg, const int timeout);

	/**
	 * @brief Receive incoming messages.
	 *
	 * @param timeout Maximum time to wait for new data in ms, after which we return.
	 *
	 * @return -1 = error, 0 = no message handled, 1 = message handled, 2 = sat info message handled
	 */
	int receive(unsigned timeout);

	/**
	 * @brief Read from the receiver.
	 *
	 * @param buf        Data that is read
	 * @param buf_length Size of the buffer
	 * @param timeout    Reading timeout
	 *
	 * @return 0 on nothing read or poll timeout, <0 on error and >0 on bytes read (nr of bytes)
	*/
	int read(uint8_t *buf, size_t buf_length, int timeout);

	/**
	 * @brief This is an abstraction for the poll on serial used.
	 *
	 * @param buf        The read buffer
	 * @param buf_length Size of the read buffer
	 * @param timeout    Read timeout in ms
	 *
	 * @return 0 on nothing read or poll timeout, <0 on error and >0 on bytes read (nr of bytes)
	 */
	int poll_or_read(uint8_t *buf, size_t buf_length, int timeout);

	/**
	 * @brief Write to the receiver.
	 *
	 * @param buf Data to be written
	 * @param buf_length Amount of bytes to be written
	 *
	 * @return the number of bytes written on success, or -1 otherwise
	 */
	int write(const uint8_t *buf, size_t buf_length);

	/**
	 * @brief Initialize uORB GPS logging and advertise the topic.
	 *
	 * @return `PX4_OK` on success, `PX4_ERROR` otherwise
	 */
	int initialize_communication_dump(DumpMode mode);

	/**
	 * @brief Reset the receiver if it was requested by the user.
	 */
	void reset_if_scheduled();

	/**
	 * @brief Set the baudrate of the serial connection.
	 *
	 * @param baud The baud rate of the connection
	 *
	 * @return `PX4_OK` on success, `PX4_ERROR` on otherwise
	 */
	int set_baudrate(uint32_t baud);

	/**
	 * @brief Handle incoming messages on the "inject data" uORB topic and send them to the receiver.
	 */
	void handle_inject_data_topic();

	/**
	 * @brief Send data to the receiver, such as RTCM injections.
	 *
	 * @param data The raw data to send to the device
	 * @param len The size of `data`
	 *
	 * @return `true` if all the data was written correctly, `false` otherwise
	 */
	inline bool inject_data(uint8_t *data, size_t len);

	/**
	 * @brief Publish new GPS data with uORB.
	 */
	void publish();

	/**
	 * @brief Publish new GPS satellite data with uORB.
	 */
	void publish_satellite_info();

	/**
	 * @brief Publish RTCM corrections.
	 *
	 * @param data: The raw data to publish
	 * @param len: The size of `data`
	 */
	void publish_rtcm_corrections(uint8_t *data, size_t len);

	/**
	 * @brief Dump gps communication.
	 *
	 * @param data The raw data of the message.
	 * @param len The size of `data`.
	 * @param data_direction The type of data, either incoming or outgoing.
	 */
	void dump_gps_data(const uint8_t *data, size_t len, DataDirection data_direction);

	/**
	 * @brief Check whether we should dump incoming data.
	 *
	 * @return `true` when we should dump incoming data, `false` otherwise.
	*/
	bool should_dump_incoming() const;

	/**
	 * @brief Check whether we should dump outgoing data.
	 *
	 * @return `true` when we should dump outgoing data, `false` otherwise.
	*/
	bool should_dump_outgoing() const;

	/**
	 * @brief Start a new update frequency monitoring interval.
	*/
	void start_update_monitoring_interval();

	/**
	 * @brief Check whether the current update monitoring interval should end.
	 *
	 * @return `true` if a new interval should be started, or `false` if the current interval is still valid.
	*/
	bool update_monitoring_interval_ended() const;

	/**
	 * @brief Get the duration of the current update frequency monitoring interval.
	 *
	 * @return The duration of the current interval in us.
	*/
	hrt_abstime current_monitoring_interval_duration() const;

	/**
	 * @brief Calculate RTCM message injection frequency for the current measurement interval.
	 *
	 * @return The RTCM message injection frequency for the current measurement interval in Hz.
	*/
	float rtcm_injection_frequency() const;

	/**
	 * @brief Calculate output data rate to the receiver for the current measurement interval.
	 *
	 * @return The output data rate for the current measurement interval in B/s.
	*/
	uint32_t output_data_rate() const;

	/**
	 * @brief Calculate input data rate from the receiver for the current measurement interval.
	 *
	 * @return The input data rate for the current measurement interval in B/s.
	*/
	uint32_t input_data_rate() const;

	/**
	 * @brief Check whether the current receiver configuration is likely healthy.
	 *
	 * @return `true` if the receiver is operating correctly, `false` if it needs to be reconfigured.
	*/
	bool receiver_configuration_healthy() const;

	/**
	 * @brief Convert from microseconds to seconds.
	 *
	 * @return `us` converted into seconds.
	*/
	static float us_to_s(uint64_t us);

	/**
	 * @brief Check if the system clock needs to be updated with new time obtained from the receiver.
	 *
	 * Setting the clock on Nuttx temporarily pauses interrupts. Therefore it should only be set if it is absolutely necessary.
	 *
	 * @return `true` if the clock should be update, `false` if the clock is still accurate enough.
	*/
	static bool clock_needs_update(timespec real_time);

	/**
	 * @brief Used to set the system clock accurately.
	 *
	 * This does not guarantee that the clock will be set.
	 *
	 * @param time The current time.
	 */
	static void set_clock(timespec rtc_gps_time);

	/**
	 * @brief Check whether the driver is operating correctly.
	 *
	 * @return `true` if the driver is working as expected, `false` otherwise.
	*/
	bool is_healthy() const;

	/**
	 * @brief Reset the GPS state uORB message for reuse.
	*/
	void reset_gps_state_message();

	/**
	 * @brief Get the parameter with the given name into `value`.
	 *
	 * @param name The name of the parameter.
	 * @param value The value in which to store the parameter.
	 *
	 * @return `PX4_OK` on success, or `PX4_ERROR` if the parameter wasn't found.
	*/
	static uint32_t get_parameter(const char *name, int32_t *value);

	/**
	 * @brief Get the parameter with the given name into `value`.
	 *
	 * @param name The name of the parameter.
	 * @param value The value in which to store the parameter.
	 *
	 * @return `PX4_OK` on success, or `PX4_ERROR` if the parameter wasn't found.
	*/
	static uint32_t get_parameter(const char *name, float *value);

	/**
	 * @brief Don't use this, use the other parameter functions instead!
	 */
	template<typename T>
	static uint32_t _get_parameter(const char *name, T *value)
	{
		param_t handle = param_find(name);

		if (handle == PARAM_INVALID || param_get(handle, value) == PX4_ERROR) {
			SEP_ERR("Failed to get parameter %s", name);
			return PX4_ERROR;
		}

		return PX4_OK;
	}

	State                                  _state {State::OpeningSerialPort};                            ///< Driver state which allows for single run loop
	px4::atomic<int>                       _scheduled_reset {static_cast<int>(ReceiverResetType::None)}; ///< The type of receiver reset that is scheduled
	DumpMode                               _dump_communication_mode {DumpMode::Disabled};                ///< GPS communication dump mode
	device::Serial                         _uart {};                                                     ///< Serial UART port for communication with the receiver
	char                                   _port[20] {};                                                 ///< The path of the used serial device
	hrt_abstime                            _last_rtcm_injection_time {0};                                ///< Time of last RTCM injection
	uint8_t                                _selected_rtcm_instance {0};                                  ///< uORB instance that is being used for RTCM corrections
	uint8_t                                _spoofing_state {0};                                          ///< Receiver spoofing state
	uint8_t                                _jamming_state {0};                                           ///< Receiver jamming state
	const Instance                         _instance {Instance::Main};                                   ///< The receiver that this instance of the driver controls
	uint32_t                               _baud_rate {0};
	static px4::atomic<SeptentrioDriver *> _secondary_instance;
	hrt_abstime                            _sleep_end {0};                                               ///< End time for sleeping
	State                                  _resume_state {State::OpeningSerialPort};                     ///< Resume state after sleep

	// Module configuration
	float                                  _heading_offset {0.0f};                                       ///< The heading offset given by the `SEP_YAW_OFFS` parameter
	float                                  _pitch_offset {0.0f};                                         ///< The pitch offset given by the `SEP_PITCH_OFFS` parameter
	uint32_t                               _receiver_stream_main {k_default_main_stream};                ///< The main output stream for the receiver given by the `SEP_STREAM_MAIN` parameter
	uint32_t                               _receiver_stream_log {k_default_log_stream};                  ///< The log output stream for the receiver given by the `SEP_STREAM_LOG` parameter
	SBFOutputFrequency                     _sbf_output_frequency {SBFOutputFrequency::Hz5_0};            ///< Output frequency of the main SBF blocks given by the `SEP_OUTP_HZ` parameter
	ReceiverLogFrequency                   _receiver_logging_frequency {ReceiverLogFrequency::Hz1_0};    ///< Logging frequency of the receiver given by the `SEP_LOG_HZ` parameter
	ReceiverLogLevel                       _receiver_logging_level {ReceiverLogLevel::Default};          ///< Logging level of the receiver given by the `SEP_LOG_LEVEL` parameter
	bool                                   _receiver_logging_overwrite {0};                              ///< Logging overwrite behavior, given by the `SEP_LOG_FORCE` parameter
	bool                                   _automatic_configuration {true};                              ///< Automatic configuration of the receiver given by the `SEP_AUTO_CONFIG` parameter
	ReceiverSetup                          _receiver_setup {ReceiverSetup::Default};                     ///< Purpose of the receivers, given by the `SEP_HARDW_SETUP` parameter
	int32_t                                _receiver_constellation_usage {0};                            ///< Constellation usage in PVT computation given by the `SEP_CONST_USAGE` parameter

	// Decoding and parsing
	DecodingStatus                                 _active_decoder {DecodingStatus::Searching}; ///< Currently active decoder that new data has to be fed into
	sbf::Decoder                                   _sbf_decoder {};                             ///< SBF message decoder
	rtcm::Decoder                                  *_rtcm_decoder {nullptr};                    ///< RTCM message decoder

	// uORB topics and subscriptions
	sensor_gps_s                                   _message_gps_state {};                          ///< uORB topic for position
	gps_dump_s                                     *_message_data_to_receiver {nullptr};           ///< uORB topic for dumping data to the receiver
	gps_dump_s                                     *_message_data_from_receiver {nullptr};         ///< uORB topic for dumping data from the receiver
	satellite_info_s                               *_message_satellite_info {nullptr};             ///< uORB topic for satellite info
	uORB::PublicationMulti<sensor_gps_s>           _sensor_gps_pub {ORB_ID(sensor_gps)};           ///< uORB publication for gps position
	uORB::Publication<gps_dump_s>                  _gps_dump_pub {ORB_ID(gps_dump)};               ///< uORB publication for dump GPS data
	uORB::Publication<gps_inject_data_s>           _gps_inject_data_pub {ORB_ID(gps_inject_data)}; ///< uORB publication for injected data to the receiver
	uORB::PublicationMulti<satellite_info_s>       _satellite_info_pub {ORB_ID(satellite_info)};   ///< uORB publication for satellite info
	uORB::SubscriptionMultiArray<gps_inject_data_s, gps_inject_data_s::MAX_INSTANCES> _gps_inject_data_sub {ORB_ID::gps_inject_data}; ///< uORB subscription about data to inject to the receiver

	// Data about update frequencies of various bits of information like RTCM message injection frequency, received data rate...
	hrt_abstime _current_interval_start_time {0};      ///< Start time of the current update measurement interval in us
	uint16_t    _last_interval_rtcm_injections {0};    ///< Nr of RTCM message injections in the last measurement interval
	uint16_t    _current_interval_rtcm_injections {0}; ///< Nr of RTCM message injections in the current measurement interval
	uint32_t    _last_interval_bytes_written {0};      ///< Nr of bytes written to the receiver in the last measurement interval
	uint32_t    _current_interval_bytes_written {0};   ///< Nr of bytes written to the receiver in the current measurement interval
	uint32_t    _last_interval_bytes_read {0};         ///< Nr of bytes read from the receiver in the last measurement interval
	uint32_t    _current_interval_bytes_read {0};      ///< Nr of bytes read from the receiver in the current measurement interval
	MessageTracker _last_interval_messages {};         ///< Messages encountered in the last measurement interval
	MessageTracker _current_interval_messages {};      ///< Messages encountered in the current measurement interval
};

} // namespace septentrio
