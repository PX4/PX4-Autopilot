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

#include <stdint.h>

/**
 * Physical routing of a comms bus (I2C, SPI, and conceptually UART/CAN).
 *
 * This is a property of the wires, not of any sensor on them:
 *
 * - Internal: onboard chips only, no connector.
 * - External: a connector only; anything attached is unknown until probed.
 * - Shared: onboard chips and a connector on the same wires.
 *
 * Why firmware cares:
 *
 * 1. Probe for unknown devices on External and Shared buses (-X / -S,
 *    SENS_EXT_I2C_PRB, i2c_launcher).
 * 2. An internal (onboard) sensor is hard-mounted: ASIC orientation (-R) plus
 *    board rotation (SENS_BOARD_ROT).
 * 3. An external sensor defaults to higher calibration priority (75 vs 50).
 *
 * (2) and (3) cannot be inferred from a Shared bus. They follow the start
 * flag: -I/-s classifies that instance internal, -X/-S classifies it external.
 * On a Shared bus, pin the onboard chip with -I -b <n> (or -s -b <n>) and
 * still probe the connector with -X (or -S).
 *
 * UART, CAN and MAVLink have no board bus table; sensors on those transports
 * are external unless the driver sets otherwise.
 */
enum class BusTopology : uint8_t {
	Internal = 0,
	External = 1,
	Shared   = 2,
};

static inline constexpr bool bus_topology_has_external(BusTopology topology)
{
	return topology != BusTopology::Internal;
}

static inline constexpr bool bus_topology_has_internal(BusTopology topology)
{
	return topology != BusTopology::External;
}
