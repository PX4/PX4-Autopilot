/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file ModuleFirmwareUpdater.hpp
 *
 * Shared, device-agnostic firmware-update choreography for cannode peripheral
 * drivers. Owns the GetInfo -> Read-loop -> write sequence (session, offset,
 * timeout, retry) over the generic DroneCAN file-read uORB seam
 * (DronecanFileReadRequest / DronecanFileReadResponse), so each driver only has
 * to implement the device-specific write via the FirmwareWriter interface.
 *
 * Designed to run synchronously on the driver's own task thread (never the
 * uavcan work queue), so FirmwareWriter::writeChunk() may block freely while
 * clocking bytes out to the peripheral.
 */

#pragma once

#include <stdint.h>

#include <uORB/Publication.hpp>
#include <uORB/SubscriptionBlocking.hpp>
#include <uORB/topics/dronecan_file_read_request.h>
#include <uORB/topics/dronecan_file_read_response.h>

namespace module_fw_update
{

/**
 * Device-specific firmware writer. The only per-device code in the update path.
 * All methods run on the driver thread and may block.
 *
 * Return convention: >= 0 on success, < 0 on error (aborts the transfer).
 */
class FirmwareWriter
{
public:
	virtual ~FirmwareWriter() = default;

	/** Begin a transfer of @p total_size bytes (e.g. enter bootloader). */
	virtual int begin(uint64_t total_size) = 0;

	/** Write one in-order chunk at @p offset. */
	virtual int writeChunk(uint32_t offset, const uint8_t *data, uint16_t len) = 0;

	/** Finalize the transfer (e.g. verify image, leave bootloader). */
	virtual int finish() = 0;

	/** Abort a transfer in progress (called on any error). */
	virtual void abort() = 0;
};

class ModuleFirmwareUpdater
{
public:
	enum class Result {
		Success,    ///< a newer image was fetched and written
		UpToDate,   ///< the server reports no newer image (no flash performed)
		Failed,     ///< transport, server, or writer error
	};

	explicit ModuleFirmwareUpdater(FirmwareWriter &writer) : _writer(writer) {}

	/**
	 * Run a complete update synchronously. Blocks the calling thread until the
	 * image is written, the server reports up-to-date, or the transfer fails.
	 *
	 * @param path           version-gated query path (e.g. "module/ublox-f9p@1.32")
	 * @param server_node_id DroneCAN file server node id (1..127)
	 */
	Result update(const char *path, uint8_t server_node_id);

	uint32_t bytesWritten() const { return _bytes_written; }
	uint64_t totalSize() const { return _total_size; }

private:
	// One ServiceClient call resolves at the uavcannode within ~500 ms, so an
	// attempt window comfortably larger than that lets us see the (error)
	// response rather than racing it; retries only cover a lost request.
	static constexpr uint32_t kAttemptTimeoutUs = 1500000;
	static constexpr int      kMaxRetries       = 3;

	bool requestAndWait(uint8_t cmd, const char *path, uint8_t server_node_id,
			    uint32_t offset, dronecan_file_read_response_s &out);

	FirmwareWriter &_writer;

	uORB::Publication<dronecan_file_read_request_s> _request_pub{ORB_ID(dronecan_file_read_request)};
	uORB::SubscriptionBlocking<dronecan_file_read_response_s> _response_sub{ORB_ID(dronecan_file_read_response)};

	uint32_t _session_id{0};
	uint32_t _bytes_written{0};
	uint64_t _total_size{0};
};

} // namespace module_fw_update
