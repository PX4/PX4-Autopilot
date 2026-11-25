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

#include <gtest/gtest.h>

#include "ADISBurstDecoder.hpp"

TEST(ADISBurstDecoderTest, DecimationDiagnosticAndDecode)
{
	using adis_burst_decoder::Format;
	using adis_burst_decoder::decimationDelayUs;
	using adis_burst_decoder::kDiagOverrun;
	using adis_burst_decoder::kDiagSpiError;
	using adis_burst_decoder::kTransientDiagnosticMask;
	using adis_burst_decoder::shouldRecoverOnDiagnostic;

	EXPECT_EQ(decimationDelayUs(Format::kBurst16, 0), 0u);
	EXPECT_EQ(decimationDelayUs(Format::kBurst16, 1), 0u);

	EXPECT_EQ(decimationDelayUs(Format::kBurst32, 0), 0u);
	EXPECT_EQ(decimationDelayUs(Format::kBurst32, 1), 500u);
	EXPECT_EQ(decimationDelayUs(Format::kBurst32, 3), 1000u);

	EXPECT_EQ(decimationDelayUs(Format::kTimestamp32, 0), 0u);
	EXPECT_EQ(decimationDelayUs(Format::kTimestamp32, 1), 250u);
	EXPECT_EQ(decimationDelayUs(Format::kTimestamp32, 199), 49'750u);

	EXPECT_FALSE(shouldRecoverOnDiagnostic(0, 0));
	EXPECT_FALSE(shouldRecoverOnDiagnostic(0, 1));
	EXPECT_FALSE(shouldRecoverOnDiagnostic(0, 4));

	for (uint8_t streak = 1; streak <= 3; ++streak) {
		EXPECT_FALSE(shouldRecoverOnDiagnostic(kDiagOverrun, streak));
		EXPECT_FALSE(shouldRecoverOnDiagnostic(kDiagSpiError, streak));
		EXPECT_FALSE(shouldRecoverOnDiagnostic(kTransientDiagnosticMask, streak));
	}

	EXPECT_TRUE(shouldRecoverOnDiagnostic(kDiagOverrun, 4));
	EXPECT_TRUE(shouldRecoverOnDiagnostic(kDiagSpiError, 4));
	EXPECT_TRUE(shouldRecoverOnDiagnostic(kTransientDiagnosticMask, 4));

	// Non-transient fault bit (e.g. self-test failure bit 5) triggers immediate recovery on streak 1.
	EXPECT_TRUE(shouldRecoverOnDiagnostic(uint16_t(1u << 5), 1));
	EXPECT_TRUE(shouldRecoverOnDiagnostic(uint16_t((1u << 5) | kDiagOverrun), 1));

	// ADIS1657x (kTimestamp32) excludes DIAG_STAT from SPI_CHKSUM:
	// an all-zero dead bus stays invalid without updating diagnostic, while a safe-mode fault frame
	// (DIAG_STAT == 0x0001, zeroed sensor/counter words, SPI_CHKSUM == 0) reports diagnostic == 0x0001.
	adis_burst_decoder::Sample sample {};
	uint16_t diagnostic = 0xffffu;
	uint8_t  dead_bus[adis_burst_decoder::kSizeTimestamp32] {};
	EXPECT_FALSE(adis_burst_decoder::decode(dead_bus, sizeof(dead_bus), Format::kTimestamp32, sample, &diagnostic));
	EXPECT_EQ(diagnostic, 0xffffu);

	uint8_t safe_mode[adis_burst_decoder::kSizeTimestamp32] {};
	safe_mode[3] = 0x01; // DIAG_STAT = 0x0001 (big-endian)
	diagnostic   = 0;
	EXPECT_FALSE(adis_burst_decoder::decode(safe_mode, sizeof(safe_mode), Format::kTimestamp32, sample, &diagnostic));
	EXPECT_EQ(diagnostic, 0x0001u);
}
