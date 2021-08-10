/****************************************************************************
 *
 *   Copyright (c) 2019 ECL Development Team. All rights reserved.
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
#include "EKF/ekf.h"

struct sample {
	uint64_t time_us;
	float data[3];
};


class EkfRingBufferTest : public ::testing::Test
{
public:

	sample _x, _y, _z;
	RingBuffer<sample> *_buffer;

	void SetUp() override
	{
		_buffer = new RingBuffer<sample>();
		_x.time_us = 1000000;
		_x.data[0] = _x.data[1] = _x.data[2] = 1.5f;

		_y.time_us = 2000000;
		_y.data[0] = _y.data[1] = _y.data[2] = 3.0f;

		_z.time_us = 3000000;
		_z.data[0] = _z.data[1] = _z.data[2] = 4.0f;
	}

	void TearDown() override
	{
		delete _buffer;
	}
};

TEST_F(EkfRingBufferTest, goodInitialisation)
{
	// WHEN: buffer was allocated
	// THEN: allocation should have succeed
	ASSERT_EQ(true, _buffer->allocate(3));

}

TEST_F(EkfRingBufferTest, badInitialisation)
{
	// WHEN: buffer allocation input is bad
	// THEN: allocation should fail

	// TODO: Change buffer implementation to pass this test
	// ASSERT_EQ(false, _buffer->allocate(-1));
	// ASSERT_EQ(false, _buffer->allocate(0));
}

TEST_F(EkfRingBufferTest, orderOfSamples)
{
	ASSERT_EQ(true, _buffer->allocate(3));
	// GIVEN: allocated buffer
	// WHEN: adding multiple samples
	// THEN: they should be added in order
	_buffer->push(_x);

	EXPECT_EQ(_x.time_us, _buffer->get_newest().time_us);
	EXPECT_EQ(_x.time_us, _buffer->get_oldest().time_us);

	_buffer->push(_z);
	_buffer->push(_y);

	EXPECT_EQ(_x.time_us, _buffer->get_oldest().time_us);
	EXPECT_EQ(_y.time_us, _buffer->get_newest().time_us);
}

TEST_F(EkfRingBufferTest, popSample)
{
	ASSERT_EQ(true, _buffer->allocate(3));
	_buffer->push(_x);
	_buffer->push(_y);
	_buffer->push(_z);

	// GIVEN: allocated and filled buffer

	sample pop = {};
	// WHEN: we want to retrieve a sample that is older than any in the buffer
	// THEN: we should not get any sample
	EXPECT_EQ(false, _buffer->pop_first_older_than(0, &pop));

	// WHEN: when calling "pop_first_older_than"
	// THEN: we should get the first sample from the head that is older
	EXPECT_EQ(true, _buffer->pop_first_older_than(_x.time_us + 1, &pop));
	EXPECT_EQ(_x.time_us, pop.time_us);
	EXPECT_EQ(true, _buffer->pop_first_older_than(_y.time_us + 10, &pop));
	EXPECT_EQ(_y.time_us, pop.time_us);
	EXPECT_EQ(true, _buffer->pop_first_older_than(_z.time_us + 100, &pop));
	EXPECT_EQ(_z.time_us, pop.time_us);
	// TODO: When changing the order of popping sample it does not behave as expected, fix this
}

TEST_F(EkfRingBufferTest, askingForTooNewSample)
{
	ASSERT_EQ(true, _buffer->allocate(3));
	_buffer->push(_x);
	_buffer->push(_y);
	_buffer->push(_z);

	sample pop = {};
	// WHEN: all buffered samples are older by 0.1s than your query timestamp
	// THEN: should get no sample returned
	EXPECT_EQ(true, _buffer->pop_first_older_than(_z.time_us + 99000, &pop));
	EXPECT_EQ(false, _buffer->pop_first_older_than(_y.time_us + 100000, &pop));
}

TEST_F(EkfRingBufferTest, reallocateBuffer)
{
	ASSERT_EQ(true, _buffer->allocate(5));
	_buffer->push(_x);
	_buffer->push(_y);
	_buffer->push(_y);
	_buffer->push(_y);
	_buffer->push(_z);

	// GIVEN: allocated and filled buffer
	// WHEN: do another allocate call
	_buffer->allocate(3);
	// THEN: its length should update
	EXPECT_EQ(3, _buffer->get_length());

}
