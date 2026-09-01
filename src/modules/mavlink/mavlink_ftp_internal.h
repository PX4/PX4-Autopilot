#pragma once

#include <cstddef>
#include <cstring>

#include "mavlink_ftp.h"

static constexpr size_t kMavlinkFTPCachedReplyLength =
	MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN - MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN
	+ sizeof(MavlinkFTP::PayloadHeader) + sizeof(uint32_t);

static constexpr size_t kMavlinkFTPPayloadDataLength =
	MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN - sizeof(MavlinkFTP::PayloadHeader);

static_assert(kMavlinkFTPCachedReplyLength <= sizeof(mavlink_file_transfer_protocol_t),
	      "cached MAVLink FTP reply must fit in a full MAVLink FTP message");

inline void mavlink_ftp_trim_reply_payload(mavlink_file_transfer_protocol_t &ftp_reply)
{
	auto *payload = reinterpret_cast<MavlinkFTP::PayloadHeader *>(&ftp_reply.payload[0]);

	memset(&payload->data[payload->size], 0, kMavlinkFTPPayloadDataLength - payload->size);
}

template<size_t N>
inline void mavlink_ftp_expand_cached_reply(const uint8_t (&cached_reply)[N],
		mavlink_file_transfer_protocol_t &ftp_reply)
{
	static_assert(N <= sizeof(mavlink_file_transfer_protocol_t),
		      "cached MAVLink FTP reply must fit in the destination message");

	ftp_reply = {};
	memcpy(&ftp_reply, cached_reply, N);
}
