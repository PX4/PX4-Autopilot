/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <cstdint>
#include <cstddef>

namespace NXP_PCT2075TP
{

// Constants
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

static constexpr uint8_t DEFAULT_I2C_ADDRESS = 0x48;

static constexpr uint32_t SAMPLING_INTERVAL_USEC = 100000;

// Register addresses
namespace Register
{
static constexpr uint8_t TEMPERATURE         = 0x00;
static constexpr uint8_t CONFIGURATION       = 0x01;
static constexpr uint8_t HYSTERESIS          = 0x02;
static constexpr uint8_t OVER_TEMPERATURE    = 0x03;
static constexpr uint8_t IDLE_REGISTER       = 0x04;
}

// Configuration register bits
namespace ConfigBits
{
static constexpr uint8_t SHUTDOWN            = Bit0;  // 0 = normal operation, 1 = shutdown
static constexpr uint8_t OS_INTERRUPT_MODE   = Bit1;  // 0 = comparator, 1 = interrupt
static constexpr uint8_t OS_POLARITY_HIGH    = Bit2;  // 0 = active LOW, 1 = active HIGH
static constexpr uint8_t OS_FAULT_QUEUE_1    = (0 << 3);
static constexpr uint8_t OS_FAULT_QUEUE_2    = (1 << 3);
static constexpr uint8_t OS_FAULT_QUEUE_4    = (2 << 3);
static constexpr uint8_t OS_FAULT_QUEUE_6    = (3 << 3);
static constexpr uint8_t OS_FAULT_QUEUE_MASK = (3 << 3);
}

// Temperature conversion factor
// Temperature is 11-bit in 16-bit register, LSB = 0.125°C
// Shifted right by 5 bits (divide by 32) to get 11 bits, then multiply by 0.125
static constexpr float TEMP_CONVERSION_FACTOR = 0.125f / 32.0f;

} // namespace NXP_PCT2075TP
