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
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
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
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <cstdint>

enum class CrsfUartSetupResult {
	AlreadyOpen,
	Opened,
	BaudrateError,
	OpenError,
};

/**
 * @brief Configure and open the CRSF UART when needed.
 *
 * Keeping setup independent of the scheduled worker makes each outcome
 * testable without running the work queue.
 *
 * @tparam SerialType Serial implementation providing the UART setup operations.
 * @param[in,out] uart Serial device to prepare.
 * @param[in] baudrate Baud rate to apply [bit/s].
 * @param[in] swap_rxtx Enable RX/TX pin swapping after the port opens.
 * @param[in] singlewire Enable single-wire mode after the port opens.
 * @return Setup outcome; an already-open UART is left unchanged.
 */
template<typename SerialType>
CrsfUartSetupResult CrsfUartSetup(SerialType &uart, uint32_t baudrate, bool swap_rxtx, bool singlewire)
{
	if (uart.isOpen()) {
		return CrsfUartSetupResult::AlreadyOpen;
	}

	if (!uart.setBaudrate(baudrate)) {
		return CrsfUartSetupResult::BaudrateError;
	}

	if (!uart.open()) {
		return CrsfUartSetupResult::OpenError;
	}

	if (swap_rxtx) {
		uart.setSwapRxTxMode();
	}

	if (singlewire) {
		uart.setSingleWireMode();
	}

	// Start CRSF processing with empty buffers under the finalized port configuration.
	uart.flush();
	return CrsfUartSetupResult::Opened;
}
