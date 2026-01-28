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

#pragma once

#include <uORB/topics/uORBMessageFieldsGenerated.hpp>
#include <uORB/uORB.h>
#include <containers/Array.hpp>
#include <cstring>

#define HEATSHRINK_DYNAMIC_ALLOC 0
#include <lib/heatshrink/heatshrink/heatshrink_decoder.h>

namespace uORB
{

class MessageFormatReader
{
public:
	enum class State {
		ReadOrbIDs,
		ReadingFormat,
		FormatComplete,
		Failure,
		Complete
	};

	MessageFormatReader(char *buffer, unsigned buffer_capacity)
		: _buffer(buffer), _buffer_capacity(buffer_capacity)
	{
		heatshrink_decoder_reset(&_hsd);
		static_assert(orb_compressed_heatshrink_window_length == HEATSHRINK_STATIC_WINDOW_BITS, "window length mismatch");
		static_assert(orb_compressed_heatshrink_lookahead_length == HEATSHRINK_STATIC_LOOKAHEAD_BITS,
			      "lookahead length mismatch");
		_buffer[0] = 0;
	}

	/**
	 * Read and decompress more data into the given buffer (from the constructor).
	 * Call iteratively until completed or a failure happens.
	 * @return current state
	 */
	State readMore();

	/**
	 * Read until the start of a format given an ORB ID
	 * @return true on success
	 */
	bool readUntilFormat(orb_id_size_t orb_id);

	/**
	 * Iteratively read fields for the current format
	 * @param field_length [in,out] field length, set to 0 initially
	 * @return true while there is a field
	 */
	bool readNextField(int &field_length);

	/**
	 * Current length of the buffer
	 */
	uint32_t bufferLength() const { return _buffer_length; }
	/**
	 * Clear the buffer during ReadingFormat (if it does not need to be accumulated) or FormatComplete.
	 * After FormatComplete either this or clearFormatAndRestoreLeftover must be called.
	 */
	void clearFormatFromBuffer();

	/**
	 * When FormatComplete, this can be called to move the remaining part after the format to the end of the buffer,
	 * allowing the buffer to be modified.
	 * @return length of the left-over part.
	 */
	unsigned moveLeftoverToBufferEnd()
	{
		_buffer_length -= _format_length + 1;
		memmove(_buffer + _buffer_capacity - _buffer_length, _buffer + _format_length + 1, _buffer_length);
		return _buffer_length;
	}
	/**
	 * After calling moveLeftoverToBufferEnd(), this must be called.
	 */
	void clearFormatAndRestoreLeftover()
	{
		memmove(_buffer, _buffer + _buffer_capacity - _buffer_length, _buffer_length);
		_format_length = 0;
	}

	/**
	 * Get the (partial if ReadingFormat or complete if FormatComplete) format length in the buffer
	 */
	unsigned formatLength() const { return _format_length; }

	/**
	 * In ReadOrbIDs, ReadingFormat or FormatComplete states, this returns the orb ID's accociated with the format.
	 */
	const px4::Array<orb_id_size_t, orb_compressed_max_num_orb_ids> &orbIDs() const { return _orb_ids; }
	/**
	 * In ReadOrbIDs, ReadingFormat or FormatComplete states, this returns the dependent orb ID's accociated with the
	 * format (for nested format definitions).
	 */
	const px4::Array<orb_id_size_t, orb_compressed_max_num_orb_id_dependencies> &orbIDsDependencies() const { return _orb_ids_dependencies; }

	/**
	 * Expand a tokenized format (after decompressing it)
	 * @param format tokenized format, expanded in-place
	 * @param len Length of the format, format[len] == '\0' must hold
	 * @param buf_len total length of format. Must be long enough for expanded format.
	 * @return expanded format length, or <0 on error
	 */
	static int expandMessageFormat(char *format, unsigned len, unsigned buf_len);

private:
	State _state{State::ReadOrbIDs};
	px4::Array<orb_id_size_t, orb_compressed_max_num_orb_ids> _orb_ids;
	px4::Array<orb_id_size_t, orb_compressed_max_num_orb_id_dependencies> _orb_ids_dependencies;

	unsigned _compressed_formats_idx{0};
	char *_buffer{nullptr};
	const unsigned _buffer_capacity;
	uint32_t _buffer_length{0};
	unsigned _format_length{0};

	heatshrink_decoder _hsd;
};


} // namespace uORB
