/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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


#include "uORBMessageFields.hpp"

#include <px4_platform_common/log.h>

namespace uORB
{

MessageFormatReader::State MessageFormatReader::readMore()
{
	if (_state == State::Complete || _state == State::Failure) {
		return _state;
	}

	if (_buffer_length == _buffer_capacity) {
		_state = State::Failure;
		PX4_ERR("buffer too small");
		return _state;
	}

	const uint8_t *compressed_formats = orb_compressed_message_formats();
	const unsigned compressed_formats_size = orb_compressed_message_formats_size();

	if (_buffer_length == 0 && _compressed_formats_idx == compressed_formats_size) {
		_state = State::Complete;
		return _state;
	}

	const unsigned max_num_iterations = 5; // Safeguard, we're not expected to do more than a few iterations

	for (unsigned iteration = 0; iteration < max_num_iterations; ++iteration) {
		switch (_state) {
		case State::ReadOrbIDs: {
				int num_orb_ids = _buffer[0];
				const unsigned orb_ids_size = 1 + num_orb_ids * sizeof(orb_id_size_t);

				if (_buffer_length > orb_ids_size) {
					int num_dependent_orb_ids = _buffer[orb_ids_size];
					const unsigned orb_ids_dependent_size = 1 + num_dependent_orb_ids * sizeof(orb_id_size_t);

					if (_buffer_length >= orb_ids_size + orb_ids_dependent_size) {

						orb_id_size_t orb_id;
						_state = State::ReadingFormat;
						_format_length = 0;

						_orb_ids.clear();

						for (int i = 0; i < num_orb_ids; ++i) {
							memcpy(&orb_id, &_buffer[1 + sizeof(orb_id_size_t) * i], sizeof(orb_id_size_t));
							_orb_ids.push_back(orb_id);
						}

						_orb_ids_dependencies.clear();

						for (int i = 0; i < num_dependent_orb_ids; ++i) {
							memcpy(&orb_id, &_buffer[orb_ids_size + 1 + sizeof(orb_id_size_t) * i],
							       sizeof(orb_id_size_t));
							_orb_ids_dependencies.push_back(orb_id);
						}

						memmove(_buffer, _buffer + orb_ids_size + orb_ids_dependent_size,
							_buffer_length - orb_ids_size - orb_ids_dependent_size);
						_buffer_length -= orb_ids_size + orb_ids_dependent_size;

						return State::ReadOrbIDs;
					}
				}

				if (_buffer_length == _buffer_capacity) {
					_state = State::Failure;
					PX4_ERR("buffer too small");
					return _state;
				}

				break;
			}

		case State::ReadingFormat: {
				const bool got_new_data = _format_length < _buffer_length;

				for (; _format_length < _buffer_length; ++_format_length) {
					if (_buffer[_format_length] == '\0') {
						_state = State::FormatComplete;
						return _state;
					}
				}

				if (got_new_data) {
					return _state;
				}
			}
			break;

		case State::FormatComplete:
			if (_format_length != 0) {
				PX4_ERR("Invalid API calls"); // Missing call to clearFormatFromBuffer or clearFormatAndRestoreLeftover
				_state = State::Failure;
				return _state;
			}

			_state = State::ReadOrbIDs;
			break;

		case State::Failure:
		case State::Complete:
			return _state;
		}

		// Decompress more data
		size_t count = 0;

		if (heatshrink_decoder_sink(&_hsd, &compressed_formats[_compressed_formats_idx],
					    compressed_formats_size - _compressed_formats_idx, &count) < 0) {
			_state = State::Failure;
			return _state;
		}

		_compressed_formats_idx += count;

		if (_compressed_formats_idx == compressed_formats_size) {
			const HSD_finish_res fres = heatshrink_decoder_finish(&_hsd);

			if (fres != HSDR_FINISH_MORE && fres != HSDR_FINISH_DONE) {
				_state = State::Failure;
				return _state;
			}
		}

		const HSD_poll_res pres = heatshrink_decoder_poll(&_hsd, reinterpret_cast<uint8_t *>(&_buffer[_buffer_length]),
					  _buffer_capacity - _buffer_length, &count);
		_buffer_length += count;

		if (HSDR_POLL_EMPTY != pres && HSDR_POLL_MORE != pres) {
			_state = State::Failure;
			return _state;
		}

		if (_compressed_formats_idx == compressed_formats_size) {
			const HSD_finish_res fres = heatshrink_decoder_finish(&_hsd);

			if (HSDR_FINISH_DONE != fres && HSDR_FINISH_MORE != fres) {
				_state = State::Failure;
				return _state;
			}
		}

	}

	// Not expected to get here
	PX4_ERR("logic error");
	_state = State::Failure;
	return _state;
}

void MessageFormatReader::clearFormatFromBuffer()
{
	if (_state == State::FormatComplete) {
		++_format_length; // Include null char
		memmove(_buffer, _buffer + _format_length, _buffer_length - _format_length);
		_buffer_length -= _format_length;

	} else {
		// Full buffer is occupied with format
		_buffer_length = 0;
	}

	_format_length = 0;
}

int MessageFormatReader::expandMessageFormat(char *format, unsigned len, unsigned buf_len)
{
	++len; // Include null char

	int format_idx = 0;

	while (format[format_idx] != 0) {

		const char *c_type = orb_get_c_type(format[format_idx]);

		if (c_type) {
			// Replace 1 char type with expanded c_type
			const int c_type_len = (int)strlen(c_type);

			if (len + c_type_len - 1 > buf_len) {
				return -1;
			}

			memmove(format + format_idx + c_type_len, format + format_idx + 1, len - format_idx - 1);
			memcpy(format + format_idx, c_type, c_type_len);
			format_idx += c_type_len - 1;
			len += c_type_len - 1;
		}

		// Go to next field
		const char *end_field = strchr(format + format_idx, ';');

		if (!end_field) {
			PX4_ERR("Format error in %s", format);
			return -1;
		}

		format_idx = (int)(end_field - format + 1);
	}

	if (format_idx + 1 != (int)len) {
		PX4_ERR("logic error");
		return -1;
	}

	return format_idx;
}

bool MessageFormatReader::readUntilFormat(orb_id_size_t orb_id)
{
	bool done = false;
	bool found_format = false;

	while (!done && !found_format) {
		switch (readMore()) {
		case State::ReadOrbIDs:
			for (const orb_id_size_t current_orb_id : orbIDs()) {
				if (current_orb_id == orb_id) {
					found_format = true;
				}
			}

			break;

		case State::ReadingFormat:
		case State::FormatComplete:
			clearFormatFromBuffer();
			break;

		case State::Complete:
		case State::Failure:
			done = true;
			break;

		default:
			break;
		}
	}

	return found_format;
}

bool MessageFormatReader::readNextField(int &field_length)
{
	if (field_length > 0) {
		// Move left-over part to beginning
		++field_length; // include null
		memmove(_buffer, _buffer + field_length, _buffer_length - field_length);
		_buffer_length -= field_length;
		_format_length -= field_length;
	}

	auto findFieldEnd = [&]() {
		// Find ';'
		bool found = false;

		for (field_length = 0; field_length < (int)_format_length; ++field_length) {
			if (_buffer[field_length] == ';') {
				_buffer[field_length] = '\0';
				found = true;
				break;
			}
		}

		return found;
	};

	// We might still have a field in the buffer
	if (findFieldEnd()) {
		return true;
	}

	bool done = false;
	bool ret = false;

	while (!done) {
		switch (readMore()) {
		case State::ReadingFormat:
			if (findFieldEnd()) {
				ret = true;
				done = true;
			}

			break;

		case State::FormatComplete: {
				ret = findFieldEnd(); // Expected to return true here
				done = true;
				break;
			}

		case State::ReadOrbIDs: // Arrived at the next format -> we're done
		case State::Complete:
		case State::Failure:
			done = true;
			break;
		}
	}

	return ret;
}


} // namespace uORB
