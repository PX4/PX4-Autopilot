/****************************************************************************

 * Copyright (c) 2026, PX4 Development Team.
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ****************************************************************************/

#pragma once
#include <cstdint>

namespace AF9838
{

static constexpr uint8_t I2C_ADDR = 0x0C;

// --- Register Map ---
static constexpr uint8_t REG_PCODE   = 0x00;
static constexpr uint8_t REG_CH0_LSB = 0x01;
static constexpr uint8_t REG_CH0_MSB = 0x02;
static constexpr uint8_t REG_CH1_LSB = 0x03;
static constexpr uint8_t REG_CH1_MSB = 0x04;
static constexpr uint8_t REG_CH2_LSB = 0x05;
static constexpr uint8_t REG_CH2_MSB = 0x06;
static constexpr uint8_t REG_CH3_LSB = 0x07;
static constexpr uint8_t REG_CH3_MSB = 0x08;
static constexpr uint8_t REG_STATUS  = 0x09;
static constexpr uint8_t REG_STATE   = 0x0A;
static constexpr uint8_t REG_SAMPR   = 0x0C;
static constexpr uint8_t REG_SWR     = 0x11;
static constexpr uint8_t REG_ENABLE  = 0x13;
static constexpr uint8_t REG_TCODE   = 0x20;

// --- STATUS bits ---
static constexpr uint8_t STATUS_ACQ    = 1u << 0;
static constexpr uint8_t STATUS_ST_ERR = 1u << 1;
static constexpr uint8_t STATUS_HOFL   = 1u << 2;

// --- STATE values ---
static constexpr uint8_t STATE_STANDBY   = 0x00;
static constexpr uint8_t STATE_SINGLE    = 0x01;
static constexpr uint8_t STATE_CONT      = 0x02;
static constexpr uint8_t STATE_SELF_TEST = 0x09;

// --- conversion ---
static constexpr float LSB_TO_T = 0.1e-6f; // 0.1 µT/LSB

} // namespace AF9838
