/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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
#include <stdint.h>
#include <string.h>


#include "VariableLengthRingbuffer.hpp"

class TempData
{
public:
	TempData(size_t len)
	{
		_size = len;
		_buf = new uint8_t[_size];
	}

	~TempData()
	{
		delete[] _buf;
		_buf = nullptr;
	}

	uint8_t *buf() const
	{
		return _buf;
	}

	size_t size() const
	{
		return _size;
	}

	void paint(unsigned offset = 0)
	{
		for (size_t i = 0; i < _size; ++i) {
			_buf[i] = (uint8_t)((i + offset) % UINT8_MAX);
		}
	}

private:
	uint8_t *_buf {nullptr};
	size_t _size{0};

};

bool operator==(const TempData &lhs, const TempData &rhs)
{
	if (lhs.size() != rhs.size()) {
		return false;
	}

	return memcmp(lhs.buf(), rhs.buf(), lhs.size()) == 0;
}


TEST(VariableLengthRingbuffer, AllocateAndDeallocate)
{
	VariableLengthRingbuffer buf;
	ASSERT_TRUE(buf.allocate(100));
	buf.deallocate();

	ASSERT_TRUE(buf.allocate(1000));
	// The second time we forget to clean up, but we expect no leak.
}

TEST(VariableLengthRingbuffer, PushATooBigMessage)
{
	VariableLengthRingbuffer buf;
	ASSERT_TRUE(buf.allocate(100));

	TempData data{200};

	// A message that doesn't fit should get rejected.
	EXPECT_FALSE(buf.push_back(data.buf(), data.size()));
}

TEST(VariableLengthRingbuffer, PushAndPopOne)
{
	VariableLengthRingbuffer buf;
	ASSERT_TRUE(buf.allocate(100));

	TempData data{20};
	data.paint();

	EXPECT_TRUE(buf.push_back(data.buf(), data.size()));


	// Out buffer is the same size
	TempData out{20};
	EXPECT_EQ(buf.pop_front(out.buf(), out.size()), 20);
	EXPECT_EQ(data, out);

	EXPECT_TRUE(buf.push_back(data.buf(), data.size()));
	// Out buffer is supposedly bigger
	TempData out2{20};
	EXPECT_EQ(buf.pop_front(out2.buf(), 21), 20);
	EXPECT_EQ(data, out2);

	EXPECT_TRUE(buf.push_back(data.buf(), data.size()));

	// Disabled because it doesn't work reliably.
	// For some reason the abort works when tests are filtered using TESTFILTER
	// but not when all tests are run.
	//
	// Out buffer is too small
	// Asserts are disabled in release build
	//TempData out3{19};
	//EXPECT_DEATH(buf.pop_front(out3.buf(), out3.size()), ".*");
}

TEST(VariableLengthRingbuffer, PushAndPopSeveral)
{
	VariableLengthRingbuffer buf;
	ASSERT_TRUE(buf.allocate(100));

	TempData data{20};
	data.paint();

	// 4 should fit
	for (unsigned i = 0; i < 4; ++i) {
		EXPECT_TRUE(buf.push_back(data.buf(), data.size()));
	}

	// 5 won't because of overhead
	EXPECT_FALSE(buf.push_back(data.buf(), data.size()));

	// Take 4 back out
	for (unsigned i = 0; i < 4; ++i) {
		TempData out{20};
		EXPECT_EQ(buf.pop_front(out.buf(), out.size()), data.size());
		EXPECT_EQ(data, out);
	}

	TempData out{20};
	EXPECT_EQ(buf.pop_front(out.buf(), out.size()), 0);
}

TEST(VariableLengthRingbuffer, PushAndPopSeveralVariableSize)
{
	VariableLengthRingbuffer buf;
	ASSERT_TRUE(buf.allocate(100));

	TempData data1{50};
	data1.paint();
	EXPECT_TRUE(buf.push_back(data1.buf(), data1.size()));

	TempData data2{30};
	data2.paint(42);
	EXPECT_TRUE(buf.push_back(data2.buf(), data2.size()));

	// Supposedly more space
	TempData out1{50};
	EXPECT_EQ(buf.pop_front(out1.buf(), 100), data1.size());
	EXPECT_EQ(data1, out1);

	TempData data3{50};
	data3.paint(33);
	EXPECT_TRUE(buf.push_back(data3.buf(), data3.size()));

	// Supposedly more space
	TempData out2{30};
	EXPECT_EQ(buf.pop_front(out2.buf(), 100), data2.size());
	EXPECT_EQ(data2, out2);

	// Supposedly more space
	TempData out3{50};
	EXPECT_EQ(buf.pop_front(out3.buf(), 100), data3.size());
	EXPECT_EQ(data3, out3);

	TempData out4{100};
	EXPECT_EQ(buf.pop_front(out4.buf(), out4.size()), 0);
}

TEST(VariableLengthRingbuffer, PushEmpty)
{
	VariableLengthRingbuffer buf;
	ASSERT_TRUE(buf.allocate(100));

	EXPECT_FALSE(buf.push_back(nullptr, 0));
}

TEST(VariableLengthRingbuffer, PopWithoutBuffer)
{
	VariableLengthRingbuffer buf;
	ASSERT_TRUE(buf.allocate(100));

	EXPECT_FALSE(buf.push_back(nullptr, 0));

	TempData data{50};
	data.paint();

	EXPECT_TRUE(buf.push_back(data.buf(), data.size()));


	EXPECT_EQ(buf.pop_front(nullptr, 50), 0);
}

TEST(VariableLengthRingbuffer, EmptyAndNoSpaceForHeader)
{
	// Addressing a corner case where start and end are at the end
	// and the same.

	VariableLengthRingbuffer buf;
	// Allocate 4+1 bytes more than the packet, 4 for the header, 1 for the start/end logic.
	ASSERT_TRUE(buf.allocate(25));

	{
		TempData data{20};
		data.paint();
		EXPECT_TRUE(buf.push_back(data.buf(), data.size()));
		TempData out{20};
		EXPECT_EQ(buf.pop_front(out.buf(), out.size()), out.size());
		EXPECT_EQ(data, out);
	}

	{
		TempData data{10};
		data.paint();
		EXPECT_TRUE(buf.push_back(data.buf(), data.size()));
		TempData out{10};
		EXPECT_EQ(buf.pop_front(out.buf(), out.size()), out.size());
		EXPECT_EQ(data, out);
	}
}
