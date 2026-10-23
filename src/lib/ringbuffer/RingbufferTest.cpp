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


#include "Ringbuffer.hpp"

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


TEST(Ringbuffer, AllocateAndDeallocate)
{
	Ringbuffer buf;
	ASSERT_TRUE(buf.allocate(100));
	buf.deallocate();

	ASSERT_TRUE(buf.allocate(1000));
	// The second time we forget to clean up, but we expect no leak.
}

class RingbufferReallocation : public ::testing::TestWithParam<size_t>
{
protected:
	void check_reallocated(Ringbuffer &buf)
	{
		const size_t capacity = GetParam();
		buf.deallocate();
		ASSERT_TRUE(buf.allocate(capacity));
		// Keep these non-fatal so stale indices still reach the sanitizer-backed
		// write/read below, which catches out-of-bounds reuse after shrinking.
		EXPECT_EQ(buf.space_used(), 0u);
		EXPECT_EQ(buf.space_available(), capacity - 1);

		uint8_t byte = 0;
		EXPECT_EQ(buf.pop_front(&byte, sizeof(byte)), 0u);

		TempData data{capacity - 1};
		data.paint(41);
		ASSERT_TRUE(buf.push_back(data.buf(), data.size()));
		EXPECT_EQ(buf.space_used(), data.size());
		EXPECT_EQ(buf.space_available(), 0u);
		EXPECT_FALSE(buf.push_back(&byte, sizeof(byte)));

		TempData out{data.size()};
		ASSERT_EQ(buf.pop_front(out.buf(), out.size()), out.size());
		EXPECT_EQ(data, out);
		EXPECT_EQ(buf.space_used(), 0u);
		EXPECT_EQ(buf.space_available(), capacity - 1);
		EXPECT_EQ(buf.pop_front(&byte, sizeof(byte)), 0u);
	}
};

TEST_P(RingbufferReallocation, PartiallyUsed)
{
	Ringbuffer buf;
	ASSERT_TRUE(buf.allocate(16));
	TempData data{4};
	data.paint();
	ASSERT_TRUE(buf.push_back(data.buf(), data.size()));
	TempData out{2};
	ASSERT_EQ(buf.pop_front(out.buf(), out.size()), out.size());
	ASSERT_EQ(buf.space_used(), 2u);

	check_reallocated(buf);
}

TEST_P(RingbufferReallocation, Drained)
{
	Ringbuffer buf;
	ASSERT_TRUE(buf.allocate(16));
	TempData data{12};
	data.paint();
	ASSERT_TRUE(buf.push_back(data.buf(), data.size()));
	TempData out{data.size()};
	ASSERT_EQ(buf.pop_front(out.buf(), out.size()), out.size());
	ASSERT_EQ(data, out);
	ASSERT_EQ(buf.space_used(), 0u);

	check_reallocated(buf);
}

TEST_P(RingbufferReallocation, Wrapped)
{
	Ringbuffer buf;
	ASSERT_TRUE(buf.allocate(16));
	TempData first{12};
	first.paint();
	ASSERT_TRUE(buf.push_back(first.buf(), first.size()));
	TempData out{10};
	ASSERT_EQ(buf.pop_front(out.buf(), out.size()), out.size());

	// Wrap the tail while the head is still near the end of the old allocation.
	TempData second{8};
	second.paint(12);
	ASSERT_TRUE(buf.push_back(second.buf(), second.size()));
	ASSERT_EQ(buf.space_used(), 10u);

	check_reallocated(buf);
}

INSTANTIATE_TEST_SUITE_P(NewCapacity, RingbufferReallocation, ::testing::Values(size_t{8}, size_t{32}));

TEST(Ringbuffer, PushATooBigMessage)
{
	Ringbuffer buf;
	ASSERT_TRUE(buf.allocate(100));

	TempData data{200};

	// A message that doesn't fit should get rejected.
	EXPECT_FALSE(buf.push_back(data.buf(), data.size()));
}

TEST(Ringbuffer, PushAndPopOne)
{
	Ringbuffer buf;
	ASSERT_TRUE(buf.allocate(100));

	TempData data{20};
	data.paint();

	EXPECT_TRUE(buf.push_back(data.buf(), data.size()));

	EXPECT_EQ(buf.space_used(), 20);
	EXPECT_EQ(buf.space_available(), 79);

	// Get everything
	TempData out{20};
	EXPECT_EQ(buf.pop_front(out.buf(), out.size()), 20);
	EXPECT_EQ(data, out);

	// Nothing remaining
	EXPECT_EQ(buf.pop_front(out.buf(), out.size()), 0);
}

TEST(Ringbuffer, PushAndPopSeveral)
{
	Ringbuffer buf;
	ASSERT_TRUE(buf.allocate(100));

	TempData data{90};
	data.paint();

	// 9 little chunks in
	for (unsigned i = 0; i < 9; ++i) {
		EXPECT_TRUE(buf.push_back(data.buf() + i * 10, 10));
	}

	// 10 won't because of overhead inside the buffer
	EXPECT_FALSE(buf.push_back(data.buf(), 10));

	TempData out{90};
	// Take it back out in 2 big steps
	EXPECT_EQ(buf.pop_front(out.buf(), 50), 50);
	EXPECT_EQ(buf.pop_front(out.buf() + 50, 40), 40);
	EXPECT_EQ(data, out);
}

TEST(Ringbuffer, PushAndTryToPopMore)
{
	Ringbuffer buf;
	ASSERT_TRUE(buf.allocate(100));

	TempData data1{50};
	data1.paint();
	EXPECT_TRUE(buf.push_back(data1.buf(), data1.size()));

	TempData out1{80};
	EXPECT_EQ(buf.pop_front(out1.buf(), out1.size()), data1.size());
}

TEST(Ringbuffer, PushAndPopSeveralInterleaved)
{
	Ringbuffer buf;
	ASSERT_TRUE(buf.allocate(100));

	TempData data1{50};
	data1.paint();
	EXPECT_TRUE(buf.push_back(data1.buf(), data1.size()));

	TempData data2{30};
	data2.paint(50);
	EXPECT_TRUE(buf.push_back(data2.buf(), data2.size()));

	TempData out12{80};
	EXPECT_EQ(buf.pop_front(out12.buf(), out12.size()), out12.size());

	TempData out12_ref{80};
	out12_ref.paint();
	EXPECT_EQ(out12_ref, out12);

	TempData data3{50};
	data3.paint(33);
	EXPECT_TRUE(buf.push_back(data3.buf(), data3.size()));

	TempData out3{50};
	EXPECT_EQ(buf.pop_front(out3.buf(), out3.size()), data3.size());
	EXPECT_EQ(data3, out3);
}

TEST(Ringbuffer, PushEmpty)
{
	Ringbuffer buf;
	ASSERT_TRUE(buf.allocate(100));

	EXPECT_FALSE(buf.push_back(nullptr, 0));
}

TEST(Ringbuffer, PopWithoutBuffer)
{
	Ringbuffer buf;
	ASSERT_TRUE(buf.allocate(100));

	EXPECT_FALSE(buf.push_back(nullptr, 0));

	TempData data{50};
	data.paint();

	EXPECT_TRUE(buf.push_back(data.buf(), data.size()));


	EXPECT_EQ(buf.pop_front(nullptr, 50), 0);
}

TEST(Ringbuffer, EmptyAndNoSpaceForHeader)
{
	// Addressing a corner case where start and end are at the end
	// and the same.

	Ringbuffer buf;
	// Allocate 1 bytes more than the packet, 1 for the start/end logic.
	ASSERT_TRUE(buf.allocate(21));

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
