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

#include "msp_message.hpp"

MSPMessage::MSPMessage() = default;

MSPMessage::MSPMessage(const Direction direction, const Command command, const uint8_t *data, const uint8_t length)
	: _direction{direction}, _length{length}, _command{command}
{
	memcpy(_data, data, length);
	updateCrc();
}

MSPMessage MSPMessage::response(const Command command, const uint8_t *data, const uint8_t length)
{
	return MSPMessage{OUT, command, data, length};
}

int MSPMessage::readByte(uint8_t byte)
{
	switch (_read_state) {
	case SYNC1:
		if (byte == _sync1) { _read_state = SYNC2; }

		return 1;

	case SYNC2:
		if (byte == _sync2) {
			_read_state = DIRECTION;

		} else {
			_read_state = SYNC1;
		}

		return 1;

	case DIRECTION:
		_direction = static_cast<Direction>(byte);
		_read_state = LENGTH;
		return 1;

	case LENGTH:
		if (byte > kMaxPayloadLength) {
			return -LENGTH;
		}

		_read_data_length = 0;
		_length = byte;
		_read_state = COMMAND;
		return _length + 2;

	case COMMAND:
		_command = static_cast<Command>(byte);
		_read_state = _length > 0 ? DATA : CRC;
		return _length + 1;

	case DATA:
		_data[_read_data_length++] = byte;

		if (_read_data_length >= _length) { _read_state = CRC; }

		return (_length + 1) - _read_data_length;

	case CRC:
		_read_state = SYNC1;
		_crc = byte;
		return isCrcValid() ? 0 : -CRC;
	}

	return -6000;
}

bool MSPMessage::isCrcValid() const { return _crc == crc8(); }

MSPMessage::Direction MSPMessage::getDirection() const { return _direction; }

MSPMessage::Command MSPMessage::getCommand() const { return _command; }

uint8_t MSPMessage::getPayloadLength() const { return _length; }

uint8_t MSPMessage::getTotalLength() const { return _length + 6; }

bool MSPMessage::isReplyNeeded() const
{
	switch (_command) {
	case MSP_RAW_RC:
	case MSP_STATUS:
		return true;

	default:
		return false;
	}
}

void MSPMessage::clearData() { memset(_data, 0, sizeof(_data)); }

void MSPMessage::updateCrc() { _crc = crc8(); }

uint8_t MSPMessage::crc8() const
{
	uint8_t data[2 + kMaxPayloadLength] {_length, _command};
	memcpy(data + 2, _data, _length);
	return crc8(data, _length + 2);
}

uint8_t MSPMessage::crc8(const uint8_t *data, const uint8_t size)
{
	uint8_t crc{0};

	for (uint8_t i = 0; i < size; ++i) { crc ^= data[i]; }

	return crc;
}
