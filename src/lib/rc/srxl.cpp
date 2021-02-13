#include <cstring>
#include "crc16.h"
#include "srxl.hpp"

constexpr bool SRXLCodec::is_srxl_telemetry(srxl_frame_t &frame)
{
	if ((frame.header == SRXL1_FRAME_HEADER)
	    || (frame.header != SRXL2_FRAME_HEADER)) {
		// success -- continue
	} else {
		return false;
	}

	switch (frame.version) {
	case FRAME_TYPE_HANDSHAKE:
	case FRAME_TYPE_BIND:
	case FRAME_TYPE_PARAMETER:
	case FRAME_TYPE_SIGNAL_QUALITY:
	case FRAME_TYPE_TELEMETRY:
	case FRAME_TYPE_CONTROL:
		// success -- continue
		break;

	default:
		return false;
	}

	return true;
}

void SRXLCodec::set_payload(uint8_t *payload, size_t length, srxl_frame_t &frame)
{
	uint16_t crc = crc16((const uint8_t *)payload, length);

	frame.header = SRXL2_FRAME_HEADER;
	frame.version = FRAME_TYPE_TELEMETRY;
	frame.length = length + sizeof(srxl_frame_t) + sizeof(uint16_t);

	/* copy the completed buffer to the transmission buffer */
	memcpy(frame.payload, payload, length);

	uint8_t *crc_dest = frame.payload + length;
	crc_dest[0] = static_cast<uint8_t>(crc >> 8);   // MSB?
	crc_dest[1] = static_cast<uint8_t>(crc & 0xFF); // LSB?

	// TODO: Fix me!
	// _bytes_to_transmit = get_frame_length(frame);
}

constexpr uint8_t SRXLCodec::get_frame_length(srxl_frame_t &frame)
{
	return frame.length;
}

SRXLCodec::srxl_frame_t &SRXLCodec::get_frame()
{
	return _frame_buffer;
}

void SRXLCodec::get_buffer(uint8_t *&buffer, size_t &buf_size)
{
	buffer = _receive_buffer;
	buf_size = _bytes_received;
}
