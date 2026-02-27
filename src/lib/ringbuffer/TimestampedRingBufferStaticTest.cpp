/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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
#include <math.h>
#include "TimestampedRingBuffer.hpp"

struct Sample {
	uint64_t time_us;
	float data[3];
};


class TimestampedRingBufferStaticTest : public ::testing::Test
{
public:
	Sample _x, _y, _z, _w;
	TimestampedRingBuffer<Sample, 3> *_buffer{nullptr};

	void SetUp() override
	{
		_buffer = new TimestampedRingBuffer<Sample, 3>();
		_x.time_us = 1000000;
		_x.data[0] = _x.data[1] = _x.data[2] = 1.5f;

		_y.time_us = 2000000;
		_y.data[0] = _y.data[1] = _y.data[2] = 3.0f;

		_z.time_us = 3000000;
		_z.data[0] = _z.data[1] = _z.data[2] = 4.0f;

		_w.time_us = 4000000;
		_w.data[0] = _w.data[1] = _w.data[2] = 5.0f;
	}

	void TearDown() override
	{
		delete _buffer;
	}
};

TEST_F(TimestampedRingBufferStaticTest, goodInitialisation)
{
	// WHEN: buffer was allocated
	// THEN: allocation should have succeed
	EXPECT_TRUE(_buffer->valid());
	EXPECT_FALSE(_buffer->allocate(2));
	EXPECT_EQ(_buffer->get_length(), 3);
}

TEST_F(TimestampedRingBufferStaticTest, NextPrevHelpers)
{
	EXPECT_EQ(_buffer->next(0), 1);
	EXPECT_EQ(_buffer->next(1), 2);
	EXPECT_EQ(_buffer->next(2), 0);

	EXPECT_EQ(_buffer->prev(0), 2);
	EXPECT_EQ(_buffer->prev(1), 0);
	EXPECT_EQ(_buffer->prev(2), 1);
}

TEST_F(TimestampedRingBufferStaticTest, PushWrapAndIndices)
{
	_buffer->reset();
	ASSERT_TRUE(_buffer->empty());

	_buffer->push(_x);
	_buffer->push(_y);
	EXPECT_EQ(_buffer->entries(), 2);
	_buffer->push(_z);

	EXPECT_EQ(_buffer->entries(), 3);
	EXPECT_EQ(_buffer->get_oldest().time_us, _x.time_us);
	EXPECT_EQ(_buffer->get_newest().time_us, _z.time_us);

	// Overwrite once (ring wrap-around).
	_buffer->push(_w);

	EXPECT_EQ(_buffer->entries(), 3);
	EXPECT_EQ(_buffer->get_oldest().time_us, _y.time_us);
	EXPECT_EQ(_buffer->get_newest().time_us, _w.time_us);
}

TEST_F(TimestampedRingBufferStaticTest, PopFirstOlderThanDiscardsOlderHistory)
{
	_buffer->reset();
	_buffer->push(_x);
	_buffer->push(_y);
	_buffer->push(_z);

	Sample pop = {};

	// Query timestamp older than any in the buffer: no match.
	EXPECT_FALSE(_buffer->pop_first_older_than(0, &pop));

	// Pop y (also discards x), leaving z as the only remaining Sample.
	EXPECT_TRUE(_buffer->pop_first_older_than(_y.time_us + 10, &pop));
	EXPECT_EQ(pop.time_us, _y.time_us);

	EXPECT_EQ(_buffer->entries(), 1);
	EXPECT_EQ(_buffer->get_oldest().time_us, _z.time_us);
	EXPECT_EQ(_buffer->get_newest().time_us, _z.time_us);

	// Pop z (buffer becomes empty).
	EXPECT_TRUE(_buffer->pop_first_older_than(_z.time_us + 1, &pop));
	EXPECT_EQ(pop.time_us, _z.time_us);
	EXPECT_TRUE(_buffer->empty());
}

TEST_F(TimestampedRingBufferStaticTest, ResetClearsTimestamps)
{
	_buffer->push(_x);
	_buffer->push(_y);
	EXPECT_FALSE(_buffer->empty());

	_buffer->reset();
	EXPECT_TRUE(_buffer->empty());

	for (uint8_t i = 0; i < _buffer->get_length(); i++) {
		EXPECT_EQ((*_buffer)[i].time_us, 0u);
	}
}
