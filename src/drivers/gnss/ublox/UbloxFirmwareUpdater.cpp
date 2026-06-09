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

#include "UbloxFirmwareUpdater.hpp"

#include <px4_platform_common/log.h>

static constexpr uint32_t kProgressStepBytes = 64 * 1024;

int UbloxFirmwareUpdater::begin(uint64_t total_size)
{
	_total_size    = total_size;
	_written       = 0;
	_checksum      = 0;
	_next_progress = kProgressStepBytes;
	PX4_INFO("ublox fw update: begin, image %lu B", (unsigned long)total_size);
	return 0;
}

int UbloxFirmwareUpdater::writeChunk(uint32_t offset, const uint8_t *data, uint16_t len)
{
	// STUB: the real writer clocks these bytes to the u-blox bootloader and
	// waits for the per-block ACK. Here we just account for and checksum them.
	(void)offset;

	for (uint16_t i = 0; i < len; i++) {
		_checksum += data[i];
	}

	_written += len;

	if (_written >= _next_progress) {
		PX4_INFO("ublox fw update: %lu / %lu B", (unsigned long)_written, (unsigned long)_total_size);
		_next_progress += kProgressStepBytes;
	}

	return 0;
}

int UbloxFirmwareUpdater::finish()
{
	PX4_INFO("ublox fw update: finish, wrote %lu / %lu B, checksum 0x%08lx",
		 (unsigned long)_written, (unsigned long)_total_size, (unsigned long)_checksum);
	return 0;
}

void UbloxFirmwareUpdater::abort()
{
	PX4_WARN("ublox fw update: aborted after %lu B", (unsigned long)_written);
}
