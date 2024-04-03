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
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/satellite_info.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/gps_dump.h>
#include <uORB/topics/gps_inject_data.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/parameters/param.h>

#include "sbf.h"
#include "rtcm.h"

/* Message decoder state */
typedef enum {
	SBF_DECODE_SYNC1 = 0,
	SBF_DECODE_SYNC2,
	SBF_DECODE_PAYLOAD,
	SBF_DECODE_RTCM3,
} sbf_decode_state_t;

/**
 *  Struct for dynamic allocation of satellite info data.
 */
struct GPSSatelliteInfo {
	satellite_info_s _data;
};

enum class SeptentrioDumpCommMode : int32_t {
	Disabled = 0,
	Full, ///< dump full RX and TX data for all devices
	RTCM, ///< dump received RTCM from Main GPS
};

/**
 * The type of serial connection to the receiver.
 */
enum class SeptentrioSerialInterface : uint8_t {
	UART = 0,
	SPI,
};

enum class SeptentrioGPSResetType {
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
 * The type of data the receiver should output.
 */
enum class SeptentrioGPSOutputMode {
	GPS,        ///< Only GPS output
	GPSAndRTCM, ///< GPS and RTCM output
};

class SeptentrioGPS : public ModuleBase<SeptentrioGPS>, public device::Device
{
public:
	SeptentrioGPS(const char *device_path);
	~SeptentrioGPS() override;

	int print_status() override;

	void run() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static SeptentrioGPS *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/**
	 * @brief Reset the connected GPS receiver.
	 *
	 * @return 0 on success, -1 on not implemented, >0 on error
	 */
	int reset(SeptentrioGPSResetType type);

	/**
	 * @brief Get the update rate for position information from the receiver.
	 *
	 * @return the position update rate of the receiver
	 */
	float get_position_update_rate();

	/**
	 * @brief Get the update rate for velocity information from the receiver.
	 *
	 * @return the velocity update rate of the receiver
	 */
	float get_velocity_update_rate();
private:
	/**
	 * @brief Schedule a reset of the connected receiver.
	 */
	void schedule_reset(SeptentrioGPSResetType type);

	/**
	 * Configure the receiver.
	 *
	 * @return `PX4_OK` on success, `PX4_ERROR` otherwise
	 */
	int configure(float heading_offset);

	/**
	 * Open the serial connection to the receiver.
	 * On error, wait a second and return.
	 * Does nothing if the connection is already open.
	 *
	 * @return `PX4_OK` on success, `PX4_ERROR` on error
	 */
	int serial_open();

	/**
	 * @brief Close the serial connection to the receiver.
	 *
	 * Does nothing if the connection is already closed.
	 * If an error occurs while closing the file,`_serial_fd` is still set to -1.
	 *
	 * @return `PX4_OK` on success or closed connection, `PX4_ERROR` on error
	 */
	int serial_close();

	/**
	 * @brief Parse the next byte of a received message from the receiver.
	 *
	 * @return 0 = decoding, 1 = message handled, 2 = sat info message handled
	 */
	int parse_char(const uint8_t byte);

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
	 * @brief Reset the parse state machine for a fresh start.
	 */
	void decode_init();

	/**
	 * @brief Send a message.
	 *
	 * @return true on success, false on write error (errno set)
	 */
	bool send_message(const char *msg);

	/**
	 * @brief Send a message and waits for acknowledge.
	 *
	 * @param msg The message to send to the receiver
	 * @param timeout The time before sending the message times out
	 *
	 * @return true on success, false on write error (errno set) or ack wait timeout
	 */
	bool send_message_and_wait_for_ack(const char *msg, const int timeout);

	/**
	 * @brief Receive incoming messages.
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
	 * This is an abstraction for the poll on serial used.
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
	int initialize_communication_dump();

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
	int set_baudrate(unsigned baud);

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
	 * Publish the gps struct.
	 */
	void publish();

	/**
	 * Publish the satellite info.
	 */
	void publish_satellite_info();

	/**
	 * Publish RTCM corrections.
	 *
	 * @param data: The raw data to publish
	 * @param len: The size of `data`
	 */
	void publish_rtcm_corrections(uint8_t *data, size_t len);

	/**
	 * Dump gps communication.
	 *
	 * @param data The raw data of the message
	 * @param len The size of `data`
	 * @param mode The calling source
	 * @param msg_to_gps_device `true` if the message should be sent to the device, `false` otherwise
	 */
	void dump_gps_data(const uint8_t *data, size_t len, SeptentrioDumpCommMode mode, bool msg_to_gps_device);

	/**
	 * Handle an RTCM message.
	 */
	void got_rtcm_message(uint8_t *data, size_t len);

	/**
	 * @brief Store the currently recorded update rates.
	 */
	void store_update_rates();

	/**
	 * @brief Reset the update rate measurement state.
	 */
	void reset_update_rates();

	/**
	 * Used to set the system clock accurately.
	 *
	 * @param time The current time.
	 */
	void set_clock(timespec rtc_gps_time);

	px4::atomic<int>                               _scheduled_reset{(int)SeptentrioGPSResetType::None};                                   ///< The type of receiver reset that is scheduled
	SeptentrioGPSOutputMode                        _output_mode{SeptentrioGPSOutputMode::GPS};                                       ///< The type of data the receiver should output
	SeptentrioDumpCommMode                         _dump_communication_mode{SeptentrioDumpCommMode::Disabled};                       ///< GPS communication dump mode
	sbf_decode_state_t                             _decode_state{SBF_DECODE_SYNC1};                                                  ///< State of the SBF parser
	int                                            _serial_fd{-1};                                                                   ///< The file descriptor used for communication with the receiver
	char                                           _port[20] {};                                                                     ///< The path of the used serial device
	bool                                           _configured{false};                                                               ///< Configuration status of the connected receiver
	uint64_t                                       _last_timestamp_time{0};                                                          ///< The last time a timestamp was added to a position uORB message (us)
	hrt_abstime                                    _last_rtcm_injection_time{0};                                                     ///< Time of last RTCM injection
	uint8_t                                        _msg_status{0};
	uint16_t                                       _rx_payload_index{0};                                                             ///< State for the message parser
	sbf_buf_t
	_buf;                                                                             ///< The complete received message
	RTCMParsing                                    *_rtcm_parsing{nullptr};                                                          ///< RTCM message parser
	uint8_t                                        _selected_rtcm_instance{0};                                                       ///< uORB instance that is being used for RTCM corrections
	bool                                           _healthy{false};                                                                  ///< Flag to signal if the GPS is OK
	uint8_t                                        _spoofing_state{0};                                                               ///< Receiver spoofing state
	uint8_t                                        _jamming_state{0};                                                                ///< Receiver jamming state

	// uORB topics and subscriptions
	gps_dump_s                                     *_dump_to_device{nullptr};                                                        ///< uORB GPS dump data (to the receiver)
	gps_dump_s                                     *_dump_from_device{nullptr};                                                      ///< uORB GPS dump data (from the receiver)
	sensor_gps_s                                   _report_gps_pos{};                                                                ///< uORB topic for gps position
	satellite_info_s                               *_p_report_sat_info{nullptr};                                                     ///< Pointer to uORB topic for satellite info
	GPSSatelliteInfo                               *_sat_info{nullptr};                                                              ///< Instance of GPS sat info data object
	uORB::Publication<gps_dump_s>                  _dump_communication_pub{ORB_ID(gps_dump)};                                        ///< uORB topic for dump GPS data
	uORB::Publication<gps_inject_data_s>           _gps_inject_data_pub{ORB_ID(gps_inject_data)};                                    ///< uORB topic for injected data to the receiver
	uORB::PublicationMulti<sensor_gps_s>           _report_gps_pos_pub{ORB_ID(sensor_gps)};                                          ///< uORB topic for gps position
	uORB::PublicationMulti<satellite_info_s>       _report_sat_info_pub{ORB_ID(satellite_info)};                                     ///< uORB topic for satellite info
	uORB::SubscriptionMultiArray<gps_inject_data_s, gps_inject_data_s::MAX_INSTANCES> _orb_inject_data_sub{ORB_ID::gps_inject_data}; ///< uORB topic about data to inject to the receiver

	// Reading and update rates
	// NOTE: Both `_rate_vel` and `_rate_lat_lon` seem to be used in exactly the same way.
	uint64_t _interval_rate_start{0};            ///< Start moment of measurement interval for position and velocity messages from the receiver
	float    _rate{0.0f};                        ///< uORB position data publish rate (Hz)
	uint8_t  _rate_count_vel;                    ///< Velocity messages from receiver in current measurement interval
	uint8_t  _rate_count_lat_lon{};              ///< Position messages from receiver in current measurement interval
	unsigned _last_rate_rtcm_injection_count{0}; ///< Counter for number of RTCM messages
	float    _rate_vel{0.0f};                    ///< Velocity message rate from receiver in the last measurement interval (Hz)
	float    _rate_lat_lon{0.0f};                ///< Position message rate from receiver in the last measurement interval (Hz)
	float    _rate_rtcm_injection{0.0f};         ///< RTCM message injection rate (Hz)
	unsigned _rate_reading{0};                   ///< Byte reading rate (Hz)
	unsigned _num_bytes_read{0};                 ///< Number of bytes read in last measurement interval
};