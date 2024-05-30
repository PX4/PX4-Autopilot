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
 * @file decoder.h
 *
 * Decoding logic for the Septentrio Binary Format (SBF).
 *
 * @author Thomas Frans
*/

#pragma once

#include <cstdint>

#include "messages.h"

namespace septentrio
{

namespace sbf
{

#pragma pack(push, 1)

/// A complete SBF message with parsed header and unparsed body.
typedef struct {
	Header header;
	uint8_t payload[k_max_message_size];
} message_t;

#pragma pack(pop)

/**
 * @brief A decoder and parser for Septentrio Binary Format (SBF) data.
 */
class Decoder
{
public:
	/**
	 * @brief The current decoding state of the decoder.
	 */
	enum class State {
		/// Searching for the first sync byte of an SBF message.
		SearchingSync1,

		/// Searching for the second sync byte of an SBF message.
		SearchingSync2,

		/// In the process of receiving an SBF message.
		Busy,

		/// Done receiving an SBF message and ready to parse.
		Done,
	};

	/**
	 * @brief Add one byte to the decoder.
	 *
	 * @param byte The byte to add.
	 *
	 * @return The decoding state after adding the byte.
	 */
	State add_byte(uint8_t byte);

	/**
	 * @brief Get the id of the current message.
	 *
	 * @return The SBF id of the current message.
	*/
	BlockID id() const;

	/**
	 * @brief Try to parse the current header.
	 *
	 * @param header The header data structure to parse into.
	 *
	 * @return `PX4_OK` if success, or `PX4_ERROR` when an error occurs.
	 */
	int parse(Header *header) const;

	/**
	 * @brief Parse a DOP SBF message.
	 *
	 * @param message The DOP data structure to parse into.
	 *
	 * @return `PX4_OK` if success, or `PX4_ERROR` when an error occurs.
	 */
	int parse(DOP *message) const;

	/**
	 * @brief Parse a PVTGeodetic SBF message.
	 *
	 * @param message The PVTGeodetic data structure to parse into.
	 *
	 * @return `PX4_OK` if success, or `PX4_ERROR` when an error occurs.
	*/
	int parse(PVTGeodetic *message) const;

	/**
	 * @brief Parse a ReceiverStatus SBF message.
	 *
	 * @param message The ReceiverStatus data structure to parse into.
	 *
	 * @return `PX4_OK` if success, or `PX4_ERROR` when an error occurs.
	*/
	int parse(ReceiverStatus *message) const;

	/**
	 * @brief Parse a QualityInd SBF message.
	 *
	 * @param message The QualityInd data structure to parse into.
	 *
	 * @return `PX4_OK` if success, or `PX4_ERROR` when an error occurs.
	*/
	int parse(QualityInd *message) const;

	/**
	 * @brief Parse an RFSTatus SBF message.
	 *
	 * @param message The RFStatus data structure to parse into.
	 *
	 * @return `PX4_OK` if success, or `PX4_ERROR` when an error occurs.
	*/
	int parse(RFStatus *message) const;

	/**
	 * @brief Parse a GALAuthStatus SBF message.
	 *
	 * @param message The GALAuthStatus data structure to parse into.
	 *
	 * @return `PX4_OK` if success, or `PX4_ERROR` when an error occurs.
	*/
	int parse(GALAuthStatus *message) const;

	/**
	 * @brief Parse a VelCovGeodetic SBF message.
	 *
	 * @param message The VelCovGeodetic data structure to parse into.
	 *
	 * @return `PX4_OK` if success, or `PX4_ERROR` when an error occurs.
	*/
	int parse(VelCovGeodetic *message) const;

	/**
	 * @brief Parse a GEOIonoDelay SBF message.
	 *
	 * @param message The GEOIonoDelay data structure to parse into.
	 *
	 * @return `PX4_OK` if success, or `PX4_ERROR` when an error occurs.
	 */
	int parse(GEOIonoDelay *message) const; // NOTE: This serves as an example of how to parse sub-blocks.

	/**
	 * @brief Parse an AttEuler SBF message.
	 *
	 * @param message The AttEuler data structure to parse into.
	 *
	 * @return `PX4_OK` if success, or `PX4_ERROR` when an error occurs.
	*/
	int parse(AttEuler *message) const;

	/**
	 * @brief Parse an AttCovEuler SBF message.
	 *
	 * @param message The AttCovEuler data structure to parse into.
	 *
	 * @return `PX4_OK` if success, or `PX4_ERROR` when an error occurs.
	*/
	int parse(AttCovEuler *message) const;

	/**
	 * @brief Reset the decoder to a clean state.
	 *
	 * If the decoder is in the process of decoding a message or contains a complete message, it will discard it and
	 * become ready for the next message.
	 */
	void reset();
private:
	/**
	 * @brief Check whether a full message has been received.
	 *
	 * Does not guarantee validity of the message, only that a complete message should be available.
	 *
	 * @return `true` if a message is ready, `false` if still decoding.
	*/
	bool done() const;

	/**
	 * @brief Check whether a full message has been received and is valid.
	 *
	 * @return `true` if the data can be parsed, `false` if the message is incomplete or not valid.
	*/
	bool can_parse() const;

	State _state{State::SearchingSync1};
	uint16_t _current_index;
	message_t _message;
};

} // namespace sbf

} // namespace septentrio
