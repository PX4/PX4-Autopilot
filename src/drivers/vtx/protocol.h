/****************************************************************************
 *
 *	Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#pragma once
#include <board_config.h>
#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/log.h>
#include <uORB/topics/vtx.h>
#include "../vtxtable/VtxTable.hpp"

namespace vtx
{

/**
 * Parser class for working with Tramp protocol.
 * @author Niklas Hauser <niklas@auterion.com>
 */
class Protocol
{
public:
	/// @brief Constructor
	/// @param baudrate The baud rate for the serial connection
	constexpr Protocol(uint16_t baudrate) :
		_baudrate(baudrate)
	{}
	virtual inline ~Protocol()
	{
		if (_serial) {
			_serial->close();
			delete _serial;
		}
	}

	inline int init(const char *device)
	{
		if (_serial == nullptr) {
			_serial = new device::Serial(device, _baudrate);

		} else {
			_serial->close();
		}

		if (_serial == nullptr) {
			PX4_ERR("Serial alloc failed");
			return PX4_ERROR;
		}

#ifdef CONFIG_BOARD_SERIAL_RC

		// All RC serial ports are RX by default, so we need to swap RX and TX
		// to get the TX pin on the RX pin.
		if (strcmp(device, CONFIG_BOARD_SERIAL_RC) == 0 && !board_rc_swap_rxtx(device)) {
			_serial->setSwapRxTxMode();
		}

#endif
		_serial->setStopbits(device::SerialConfig::StopBits::Two);
		_serial->setSingleWireMode();

		if (!_serial->open()) {
			PX4_ERR("Serial open failed");
			delete _serial;
			_serial = nullptr;
			return PX4_ERROR;
		}

		if (reset() != 0) {
			return PX4_ERROR;
		}

		return PX4_OK;
	}

	/// @brief Reset the VTX
	/// @return 0 on success, negative error code on failure
	virtual int reset() { return 0; }

	/// @brief Get the current settings from the VTX
	/// @return 0 on success, negative error code on failure
	virtual int get_settings() = 0;

	/// @brief Set the power
	/// @param power_level The power level to set, negative values set dBm or mW depending on the implementation
	/// @return 0 on success, negative error code on failure
	virtual int set_power(int16_t power_level) = 0;

	/// @brief Set the frequency
	/// @param frequency_MHz The frequency to set, negative values set pit frequency
	/// @return 0 on success, negative error code on failure
	virtual int set_frequency(int16_t frequency_MHz) = 0;

	/// @brief Set the frequency based on band and channel
	/// @param band The band (0-7)
	/// @param channel The channel (0-7)
	/// @return 0 on success, negative error code on failure
	virtual int set_channel(uint8_t band, uint8_t channel)
	{
		return set_frequency(vtxtable().frequency(band, channel));
	}

	/// @brief Set the pit mode
	/// @param onoff true to enable pit mode, false to disable
	/// @return 0 on success, negative error code on failure
	virtual int set_pit_mode(bool onoff) = 0;

	/// @brief Print the current settings
	/// @return true if device is connected and settings are valid
	virtual bool print_settings() = 0;

	/// @brief Copy the current settings to a uORB message
	/// @param msg The message to copy the settings to
	/// @return true if data is valid and has been copied
	virtual bool copy_to(vtx_s *msg) = 0;

protected:
	/// @brief Parse incoming bytes
	/// @param value The incoming byte
	/// @return Number of remaining bytes to read, 0 if a full message has been received, negative on error
	virtual int rx_parser(uint8_t value) = 0;

	inline int rx_msg()
	{
		using namespace time_literals;
		memset(_rx_buf, 0, sizeof(_rx_buf));
		_read_state = 0;

		const hrt_abstime start_time_us = hrt_absolute_time();
		int remaining{3};

		while (true) {
			const int new_bytes = _serial->readAtLeast(_serial_buf, sizeof(_serial_buf), remaining, 20);

			for (int i = 0; i < new_bytes; i++) {
				remaining = rx_parser(_serial_buf[i]);

				if (remaining <= 0) { return remaining; }
			}

			if (hrt_elapsed_time(&start_time_us) > 200_ms) {
				return -ETIMEDOUT;
			}
		}

		return -1;
	}

	const uint16_t _baudrate{};
	device::Serial *_serial{};
	uint8_t _tx_buf[32] {};
	uint8_t _rx_buf[32] {};
	uint8_t _serial_buf[32] {};
	uint8_t _read_state{};
};

} // namespace vtx
