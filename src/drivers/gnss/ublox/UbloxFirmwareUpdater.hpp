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
 * @file UbloxFirmwareUpdater.hpp
 *
 * Device-specific firmware writer for u-blox receivers.
 *
 * STUB: for now this only accounts for and checksums the streamed image (like
 * the Teseo proxy's mock_xloader_write). The real implementation (Stage 3) will
 * drive the u-blox bootloader: safeboot entry, baud handshake, per-block flash
 * write/ACK, and image verification, clocking bytes out over the driver's UART.
 */

#pragma once

#include <lib/module_fw_update/ModuleFirmwareUpdater.hpp>

class UbloxFirmwareUpdater : public module_fw_update::FirmwareWriter
{
public:
	UbloxFirmwareUpdater() = default;

	int begin(uint64_t total_size) override;
	int writeChunk(uint32_t offset, const uint8_t *data, uint16_t len) override;
	int finish() override;
	void abort() override;

private:
	uint64_t _total_size{0};
	uint32_t _written{0};
	uint32_t _checksum{0};
	uint32_t _next_progress{0};
};
