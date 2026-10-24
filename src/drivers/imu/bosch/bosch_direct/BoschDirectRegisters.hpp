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

#include <cstdint>

namespace bosch_direct_registers
{
constexpr uint8_t Read      { 0x80 };
constexpr uint8_t ChipId    { 0x00 };
constexpr uint8_t SoftReset { 0xb6 };
constexpr uint8_t FifoFlush { 0xb0 };

namespace fixed
{
constexpr uint8_t Temperature     { 0x08 };
constexpr uint8_t FifoStatus      { 0x0e };
constexpr uint8_t Range           { 0x0f };
constexpr uint8_t Bandwidth       { 0x10 };
constexpr uint8_t HighBandwidth   { 0x13 };
constexpr uint8_t Reset           { 0x14 };
constexpr uint8_t GyroIntEnable   { 0x15 };
constexpr uint8_t GyroIntIo       { 0x16 };
constexpr uint8_t AccelIntEnable  { 0x17 };
constexpr uint8_t GyroIntMap      { 0x18 };
constexpr uint8_t AccelIntMap     { 0x1a };
constexpr uint8_t WatermarkEnable { 0x1e };
constexpr uint8_t AccelIntIo      { 0x20 };
constexpr uint8_t AccelWatermark  { 0x30 };
constexpr uint8_t GyroWatermark   { 0x3d };
constexpr uint8_t FifoConfig      { 0x3e };
constexpr uint8_t FifoData        { 0x3f };
constexpr uint8_t Overrun         { 0x80 };
constexpr uint8_t CountMask       { 0x7f };
constexpr uint8_t FifoMode        { 0x40 };

} // namespace fixed

namespace tagged
{
constexpr uint8_t Error          { 0x02 };
constexpr uint8_t InternalStatus { 0x21 };
constexpr uint8_t Temperature    { 0x22 };
constexpr uint8_t FifoLength     { 0x24 };
constexpr uint8_t FifoData       { 0x26 };
constexpr uint8_t AccelConfig    { 0x40 };
constexpr uint8_t AccelRange     { 0x41 };
constexpr uint8_t GyroConfig     { 0x42 };
constexpr uint8_t GyroRange      { 0x43 };
constexpr uint8_t WatermarkLow   { 0x46 };
constexpr uint8_t WatermarkHigh  { 0x47 };
constexpr uint8_t FifoConfig0    { 0x48 };
constexpr uint8_t FifoConfig1    { 0x49 };
constexpr uint8_t Int1Io         { 0x53 };
constexpr uint8_t Int2Io         { 0x54 };
constexpr uint8_t IntMap         { 0x58 };
constexpr uint8_t InitControl    { 0x59 };
constexpr uint8_t PowerConfig    { 0x7c };
constexpr uint8_t PowerControl   { 0x7d };
constexpr uint8_t Command        { 0x7e };
constexpr uint8_t FifoAccel      { 0x40 };
constexpr uint8_t FifoGyro       { 0x80 };
constexpr uint8_t FifoHeader     { 0x10 };
constexpr uint8_t FifoError      { 0x40 };
constexpr uint8_t ConfigLoaded   { 0x01 };

} // namespace tagged

} // namespace bosch_direct_registers
