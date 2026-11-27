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

/**
 * @file TunesTest.cpp
 *
 * Every Tunes::Status::Continue must carry a defined frequency, duration and silence, because the
 * consumers publish them as they are and schedule the next note from them.
 */

#include <gtest/gtest.h>

#include "tunes.h"

static constexpr unsigned kSentinel = 0xDEADBEEF;

TEST(TunesTest, RestWithN0HasDefinedNoteFields)
{
	Tunes tunes;
	tunes.set_string("N0", tune_control_s::VOLUME_LEVEL_DEFAULT);

	unsigned frequency = kSentinel;
	unsigned duration = kSentinel;
	unsigned silence = kSentinel;
	uint8_t volume = 0xAA;

	EXPECT_EQ(tunes.get_next_note(frequency, duration, silence, volume), Tunes::Status::Continue);
	EXPECT_EQ(frequency, 0u);
	EXPECT_EQ(duration, 0u);
	EXPECT_GT(silence, 0u);
	EXPECT_EQ(volume, 0);
}

TEST(TunesTest, RepeatingTuneRestartsWithDefinedNoteFields)
{
	Tunes tunes;
	tunes.set_string("MBC", tune_control_s::VOLUME_LEVEL_DEFAULT);

	unsigned frequency = kSentinel;
	unsigned duration = kSentinel;
	unsigned silence = kSentinel;
	uint8_t volume = 0xAA;

	// The note C
	EXPECT_EQ(tunes.get_next_note(frequency, duration, silence, volume), Tunes::Status::Continue);
	EXPECT_GT(frequency, 0u);
	EXPECT_GT(duration, 0u);

	// End of the string: the tune repeats, which is reported as a zero-length rest
	frequency = kSentinel;
	duration = kSentinel;
	silence = kSentinel;
	volume = 0xAA;
	EXPECT_EQ(tunes.get_next_note(frequency, duration, silence, volume), Tunes::Status::Continue);
	EXPECT_EQ(frequency, 0u);
	EXPECT_EQ(duration, 0u);
	EXPECT_EQ(silence, 0u);
	EXPECT_EQ(volume, 0);

	// The note C again
	EXPECT_EQ(tunes.get_next_note(frequency, duration, silence, volume), Tunes::Status::Continue);
	EXPECT_GT(frequency, 0u);
	EXPECT_GT(duration, 0u);
}

TEST(TunesTest, NonRepeatingTuneStops)
{
	Tunes tunes;
	tunes.set_string("C", tune_control_s::VOLUME_LEVEL_DEFAULT);

	unsigned frequency = 0;
	unsigned duration = 0;
	unsigned silence = 0;
	uint8_t volume = 0;

	EXPECT_EQ(tunes.get_next_note(frequency, duration, silence, volume), Tunes::Status::Continue);
	EXPECT_EQ(tunes.get_next_note(frequency, duration, silence, volume), Tunes::Status::Stop);
}
