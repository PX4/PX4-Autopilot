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

#include <cstdint>
#include <cstring>

/**
 * Represents MSP message structure used in communication with payload boards
 */
class MSPMessage
{
public:
	static constexpr const uint8_t kMaxMessageLength{64};
	static constexpr const uint8_t kNonPayloadLength{6};  // 2 sync bytes, direction, command, crc
	static constexpr const uint8_t kMaxPayloadLength{kMaxMessageLength - kNonPayloadLength};

	enum Direction : uint8_t { IN = '<', OUT = '>', ERROR = '!' };

	enum Command : uint8_t {
		MSP_RAW_RC = 105,
		MSP_STATUS = 101,
		MSP_SET_NAME = 11,
	};

	MSPMessage();

	static MSPMessage response(const Command command, const uint8_t *data, const uint8_t length);

	/**
	 * Copies message to destination byte array
	 * @return Total length of copied bytes
	 */
	template <size_t N>
	int copyTo(uint8_t (&data)[N]) const
	{
		if (N < getTotalLength()) {
			return -1;
		}

		data[0] = _sync1;
		data[1] = _sync2;
		data[2] = _direction;
		data[3] = _length;
		data[4] = _command;
		memcpy(data + 5, _data, _length);
		data[5 + _length] = _crc;
		return getTotalLength();
	}

	/**
	 * Copies message data to destination byte array
	 * @return Total length of copied data bytes
	 */
	int copyDataTo(uint8_t *data, const size_t size) const
	{
		if (size < _length) {
			return -1;
		}

		memcpy(data, _data, _length);
		return _length;
	}

	/**
	 * Reads byte to message structure
	 * @return Calculated remaining bytes to complete message. -1 in case of error
	 */
	int readByte(uint8_t byte);
	bool isCrcValid() const;
	Direction getDirection() const;
	Command getCommand() const;
	uint8_t getPayloadLength() const;
	uint8_t getTotalLength() const;
	bool isReplyNeeded() const;
	void clearData();

private:
	enum ReadState : uint8_t {
		SYNC1 = 0,
		SYNC2 = 1,
		DIRECTION = 2,
		LENGTH = 3,
		COMMAND = 4,
		DATA = 5,
		CRC = 6,
	};

	MSPMessage(Direction direction, Command command, const uint8_t *data, uint8_t length);

	void updateCrc();
	uint8_t crc8() const;
	static uint8_t crc8(const uint8_t *data, uint8_t size);

	ReadState _read_state{};
	uint8_t _read_data_length{};

	const uint8_t _sync1{'$'};
	const uint8_t _sync2{'M'};
	Direction _direction{};
	uint8_t _length{};
	Command _command{};
	uint8_t _data[kMaxPayloadLength] {};
	uint8_t _crc{};
};
