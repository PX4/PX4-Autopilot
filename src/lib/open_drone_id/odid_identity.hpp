/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "odid_bid_storage.hpp"

#include <stddef.h>

namespace open_drone_id
{

static constexpr size_t UID_MAX_SIZE = 18;
static constexpr uint32_t UAS_ID_RECORD_MAGIC = 0x4449444f; // "ODID"
static constexpr uint8_t UAS_ID_RECORD_VERSION = 2;

// Version 2 binds the identity to the MCU. Version 1 records have no UID and
// must be reprovisioned explicitly; never bind an existing record on upgrade.
struct __attribute__((packed)) UasIdRecord {
	uint32_t magic;
	uint8_t version;
	uint8_t id_type;
	uint8_t ua_type;
	uint8_t uid_size;
	uint8_t uas_id[UAS_ID_SIZE];
	uint8_t uid[UID_MAX_SIZE];
	uint32_t crc;
};

// CTA-2063 serial layout: 4 manufacturer characters, a hexadecimal suffix
// length (1-F), then 1-15 serial characters. Unused MAVLink bytes must be zero.
// This checks syntax, not manufacturer registration or authenticity.
bool serialNumberValid(const uint8_t *serial, size_t size);
// GB/T 41300-2022: exactly 4 manufacturer + 4 model + 12 serial characters.
// The fifth character is part of the model code, never a CTA length code.
bool cnProductIdValid(const uint8_t *product_id, size_t size);
bool uasIdValid(uint8_t id_type, const uint8_t *uas_id, size_t size);
bool uidValid(const uint8_t *uid, size_t size);
bool recordCreate(UasIdRecord &record, uint8_t id_type, uint8_t ua_type, const uint8_t *serial, size_t serial_size,
		  const uint8_t *uid, size_t uid_size);
bool recordValid(const UasIdRecord &record, const uint8_t *uid, size_t uid_size);
bool recordBlank(const UasIdRecord &record);

} // namespace open_drone_id
