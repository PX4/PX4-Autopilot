/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#ifndef GPS_RTCM_DATA_HPP
#define GPS_RTCM_DATA_HPP

#include <cstring>
#include <lib/gnss/rtcm.h>
#include <uORB/topics/gps_inject_data.h>

class MavlinkStreamGPSRTCMData : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamGPSRTCMData(mavlink); }

	static constexpr const char *get_name_static() { return "GPS_RTCM_DATA"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_GPS_RTCM_DATA; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return (_gps_inject_data_sub.updated() || pendingChunkAvailable() || _rtcm_parser.bufferedBytes() > 0
			|| _rtcm_fragmenter.active()) ?
		       (MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamGPSRTCMData(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _gps_inject_data_sub{ORB_ID(gps_inject_data), 0};
	gnss::Rtcm3Parser _rtcm_parser{};
	gnss::GpsRtcmMessageFragmenter _rtcm_fragmenter{};

	bool send() override
	{
		bool sent = false;
		gps_inject_data_s gps_inject_data{};

		while (_mavlink->get_free_tx_buf() >= get_size()) {
			if (_rtcm_fragmenter.active()) {
				// Continue with the MAVLink packet sequence already in progress.
			} else if (loadNextFrameForTransmission()) {
				// A complete RTCM frame is now staged in the fragmenter.
			} else if (pendingChunkAvailable()) {
				if (flushPendingChunkToParser() == 0) {
					break;
				}

				continue;

			} else {
				// Pull one fresh uORB chunk only after any staged MAVLink packets
				// and pending parser tail have been handled.
				if (!_gps_inject_data_sub.update(&gps_inject_data)) {
					break;
				}

				// Feed the uORB chunk into the RTCM stream parser. A zero return
				// means the bytes were kept pending for a later retry.
				if (addChunkToParser(gps_inject_data.data, gps_inject_data.len) == 0) {
					break;
				}

				continue;
			}

			mavlink_gps_rtcm_data_t msg{};
			uint8_t flags = 0;
			const uint8_t *packet_data = nullptr;
			size_t packet_len = 0;

			if (!_rtcm_fragmenter.nextPacket(flags, packet_data, packet_len)) {
				break;
			}

			if (packet_len > sizeof(msg.data)) {
				PX4_ERR("RTCM fragment too large: %u > %u", (unsigned)packet_len, (unsigned)sizeof(msg.data));
				break;
			}

			msg.flags = flags;
			msg.len = static_cast<uint8_t>(packet_len);

			if (packet_len > 0) {
				memcpy(msg.data, packet_data, packet_len);
			}

			mavlink_msg_gps_rtcm_data_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
		}

		return sent;
	}

	bool loadNextFrameForTransmission()
	{
		size_t frame_len = 0;
		const uint8_t *frame = _rtcm_parser.getNextMessage(&frame_len);

		while (frame != nullptr) {
			// MAVLink GPS_RTCM_DATA can transport at most 4 x 180 bytes.
			if (frame_len > gnss::GPS_RTCM_MAX_MESSAGE_LEN) {
				if (!_warned_oversized_frame_once) {
					PX4_WARN("dropping RTCM frame (%u bytes > %u byte MAVLink limit)",
						 (unsigned)frame_len, (unsigned)gnss::GPS_RTCM_MAX_MESSAGE_LEN);
					_warned_oversized_frame_once = true;
				}

				_rtcm_parser.consumeMessage(frame_len);
				frame = _rtcm_parser.getNextMessage(&frame_len);
				continue;
			}

			// startMessage() copies the frame bytes, so it must happen before
			// consumeMessage() shifts the parser buffer.
			const bool started = _rtcm_fragmenter.startMessage(frame, frame_len);
			_rtcm_parser.consumeMessage(frame_len);

			if (started) {
				return true;
			}

			frame = _rtcm_parser.getNextMessage(&frame_len);
		}

		return false;
	}

	size_t addChunkToParser(const uint8_t *data, size_t len)
	{
		const size_t added = _rtcm_parser.addData(data, len);

		if (added < len) {
			const size_t remaining = len - added;
			memcpy(_pending_chunk, &data[added], remaining);
			_pending_chunk_len = static_cast<uint16_t>(remaining);
			_pending_chunk_offset = 0;
		}

		return added;
	}

	size_t flushPendingChunkToParser()
	{
		if (!pendingChunkAvailable()) {
			return 0;
		}

		const size_t pending_len = _pending_chunk_len - _pending_chunk_offset;
		const size_t added = _rtcm_parser.addData(&_pending_chunk[_pending_chunk_offset], pending_len);
		_pending_chunk_offset += static_cast<uint16_t>(added);

		if (_pending_chunk_offset >= _pending_chunk_len) {
			clearPendingChunk();
		}

		return added;
	}

	bool pendingChunkAvailable() const
	{
		return _pending_chunk_offset < _pending_chunk_len;
	}

	void clearPendingChunk()
	{
		_pending_chunk_len = 0;
		_pending_chunk_offset = 0;
	}

	uint8_t _pending_chunk[sizeof(gps_inject_data_s::data)] {};
	uint16_t _pending_chunk_len {0};
	uint16_t _pending_chunk_offset {0};
	bool _warned_oversized_frame_once {false};
};

#endif // GPS_RTCM_DATA_HPP
