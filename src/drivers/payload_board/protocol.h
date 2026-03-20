/****************************************************************************
 *
 *	Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
#include <px4_platform_common/log.h>
#include <uORB/topics/payload_response.h>

#include <px4_platform_common/Serial.hpp>
#include <drivers/drv_hrt.h>

#include "impl/util/msp_message.hpp"

using namespace time_literals;

namespace payload_board
{

class Protocol
{
public:
	virtual ~Protocol()
	{
		if (_serial) {
			_serial->close();
			delete _serial;
		}
	}

	int init(const char *device)
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

		if (_singlewire) { _serial->setSingleWireMode(); }

		if (_swap_rx_tx) { _serial->setSwapRxTxMode(); }

		if (!_serial->open()) {
			PX4_ERR("Serial open failed");
			delete _serial;
			_serial = nullptr;
			return PX4_ERROR;
		}

		return PX4_OK;
	}

	void setSwapRxTx(bool swap) { _swap_rx_tx = swap; }

	int copy_to(payload_response_s *data)
	{
		data->timestamp = hrt_absolute_time();
		data->state = _last_state;
		data->countdown = _countdown;
		memset(data->custom_message, 0, sizeof(data->custom_message));
		return get_custom_message(data->custom_message, sizeof(data->custom_message));
	}

	int update()
	{
		const auto res = rx_msg();

		if (res != PX4_OK) {
			return res;
		}

		if (should_reply()) {
			create_reply();
			return reply();
		}

		return PX4_OK;
	}

	bool serial_valid() const
	{
		return _serial != nullptr && _serial->isOpen();
	}

protected:
	virtual int update_state() = 0;
	virtual int rx_parser(uint8_t c) = 0;

	/**
	 * For bidirectional communication
	 * @return true if received message is a request for data, otherwise false
	 */
	virtual bool should_reply() { return false; }

	/**
	 * For bidirectional communication
	 * Generates message reply message for last received message if it is a request
	 */
	virtual void create_reply() {}
	virtual int get_custom_message(uint8_t *message, size_t size) { return PX4_OK; }
	virtual void begin_read()
	{
		memset(_rx_buf, 0, sizeof(_rx_buf));
		_read_state = 0;
		_read_data_length = 0;
	}

	int rx_msg()
	{
		begin_read();

		const hrt_abstime start_time_us = hrt_absolute_time();
		int remaining{3};

		while (_serial != nullptr) {
			if (remaining < 0 || static_cast<size_t>(remaining) > sizeof(_serial_buf)) {
				PX4_INFO("Unable to read remaining bytes - buffer too short. Requested [%d]",
					 remaining);
				return -1;
			}

			const int new_bytes = _serial->readAtLeast(_serial_buf, sizeof(_serial_buf), remaining, 20);

			for (int i = 0; i < new_bytes; i++) {
				remaining = rx_parser(_serial_buf[i]);

				if (remaining < 0) {
					/* Return parse error */
					return remaining;

				} else if (remaining == 0) {
					/* Parsing finished, try to update state */
					if (update_state() != PX4_OK) {
						return PX4_ERROR;
					}

					return PX4_OK;
				}
			}

			if (hrt_elapsed_time(&start_time_us) > 300_ms) {
				return -ETIMEDOUT;
			}
		}

		return PX4_ERROR;
	}

	int reply() const
	{
		if (_serial->write(_tx_buf, _transmit_length) < ssize_t(_transmit_length)) {
			return -errno;
		}

		_serial->flush();
		return PX4_OK;
	}

	uint32_t _baudrate{115200};
	bool _singlewire{false};
	bool _swap_rx_tx{false};
	device::Serial *_serial{};
	uint8_t _rx_buf[MSPMessage::kMaxMessageLength] {};
	uint8_t _tx_buf[MSPMessage::kMaxMessageLength] {};
	uint8_t _serial_buf[MSPMessage::kMaxMessageLength] {};
	uint8_t _read_state{};
	uint8_t _read_data_length{};
	uint8_t _read_length{};
	uint8_t _transmit_length{};
	uint8_t _countdown{};
	uint8_t _last_state{payload_response_s::STATE_INACTIVE};
};

}  // namespace payload_board
