/****************************************************************************
 *
 *   Copyright (c) 2017-2024 PX4 Development Team. All rights reserved.
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

#include "AUAV.hpp"

/* AUAV EEPROM addresses for differential channel */
static constexpr uint8_t EEPROM_DIFF_AHW 	= 0x2B;
static constexpr uint8_t EEPROM_DIFF_ALW 	= 0x2C;
static constexpr uint8_t EEPROM_DIFF_BHW 	= 0x2D;
static constexpr uint8_t EEPROM_DIFF_BLW 	= 0x2E;
static constexpr uint8_t EEPROM_DIFF_CHW 	= 0x2F;
static constexpr uint8_t EEPROM_DIFF_CLW 	= 0x30;
static constexpr uint8_t EEPROM_DIFF_DHW 	= 0x31;
static constexpr uint8_t EEPROM_DIFF_DLW 	= 0x32;
static constexpr uint8_t EEPROM_DIFF_TC50 	= 0x33;
static constexpr uint8_t EEPROM_DIFF_ES		= 0x34;

class AUAV_Differential : public AUAV
{
public:
	AUAV_Differential(const I2CSPIDriverConfig &config);
	~AUAV_Differential() override;

	void RunImpl() override;
	void print_status() override;
};
