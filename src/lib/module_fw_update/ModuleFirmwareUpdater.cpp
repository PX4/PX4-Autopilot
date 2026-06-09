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

#include "ModuleFirmwareUpdater.hpp"

#include <cstring>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>

namespace module_fw_update
{

bool ModuleFirmwareUpdater::requestAndWait(uint8_t cmd, const char *path, uint8_t server_node_id,
		uint32_t offset, dronecan_file_read_response_s &out)
{
	for (int attempt = 0; attempt <= kMaxRetries; attempt++) {
		// Publish the (idempotent) request. Retries simply re-publish.
		dronecan_file_read_request_s req{};
		req.timestamp      = hrt_absolute_time();
		req.session_id     = _session_id;
		req.cmd            = cmd;
		req.offset         = offset;
		req.server_node_id = server_node_id;
		strncpy(req.path, path, sizeof(req.path) - 1);
		req.path[sizeof(req.path) - 1] = '\0';
		_request_pub.publish(req);

		const hrt_abstime deadline = hrt_absolute_time() + kAttemptTimeoutUs;

		while (hrt_absolute_time() < deadline) {
			const uint32_t remaining = (uint32_t)(deadline - hrt_absolute_time());

			if (_response_sub.updatedBlocking(remaining)) {
				// Drain the queue, advancing the generation, and accept the
				// first reply that matches this request.
				dronecan_file_read_response_s resp;

				while (_response_sub.update(&resp)) {
					if (resp.session_id != _session_id || resp.cmd != cmd) {
						continue; // stale or from a different command
					}

					if (cmd == dronecan_file_read_request_s::CMD_READ && resp.offset != offset) {
						continue; // duplicate/stale chunk from a retry
					}

					out = resp;
					return true;
				}
			}
		}

		PX4_WARN("module_fw_update: timeout (cmd %u offset %lu), retry %d/%d",
			 cmd, (unsigned long)offset, attempt + 1, kMaxRetries);
	}

	return false;
}

ModuleFirmwareUpdater::Result ModuleFirmwareUpdater::update(const char *path, uint8_t server_node_id)
{
	_session_id++;
	_bytes_written = 0;
	_total_size    = 0;

	dronecan_file_read_response_s resp;

	// 1) GetInfo: resolve total size, or learn that we are already up to date.
	if (!requestAndWait(dronecan_file_read_request_s::CMD_GETINFO, path, server_node_id, 0, resp)) {
		PX4_ERR("module_fw_update: GetInfo got no response");
		return Result::Failed;
	}

	if (resp.error == dronecan_file_read_response_s::ERROR_NO_UPDATE) {
		PX4_INFO("module_fw_update: already up to date");
		return Result::UpToDate;
	}

	if (resp.error != dronecan_file_read_response_s::ERROR_OK) {
		PX4_ERR("module_fw_update: GetInfo error %d", resp.error);
		return Result::Failed;
	}

	_total_size = resp.file_size;

	if (_writer.begin(_total_size) < 0) {
		PX4_ERR("module_fw_update: begin() failed");
		return Result::Failed;
	}

	// 2) Read loop: strict lockstep, one chunk in flight.
	uint32_t offset = 0;

	while (true) {
		if (!requestAndWait(dronecan_file_read_request_s::CMD_READ, path, server_node_id, offset, resp)) {
			PX4_ERR("module_fw_update: Read got no response at offset %lu", (unsigned long)offset);
			_writer.abort();
			return Result::Failed;
		}

		if (resp.error != dronecan_file_read_response_s::ERROR_OK) {
			PX4_ERR("module_fw_update: Read error %d at offset %lu", resp.error, (unsigned long)offset);
			_writer.abort();
			return Result::Failed;
		}

		if (resp.len > 0) {
			if (_writer.writeChunk(offset, resp.data, resp.len) < 0) {
				PX4_ERR("module_fw_update: writeChunk failed at offset %lu", (unsigned long)offset);
				_writer.abort();
				return Result::Failed;
			}

			offset         += resp.len;
			_bytes_written += resp.len;
		}

		if (resp.eof) {
			break;
		}
	}

	// 3) Finalize.
	if (_writer.finish() < 0) {
		PX4_ERR("module_fw_update: finish() failed");
		return Result::Failed;
	}

	PX4_INFO("module_fw_update: wrote %lu B (file size %llu)",
		 (unsigned long)_bytes_written, (unsigned long long)_total_size);
	return Result::Success;
}

} // namespace module_fw_update
