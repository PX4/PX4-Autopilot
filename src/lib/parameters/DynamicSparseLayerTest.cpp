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
 * @file DynamicSparseLayerTest.cpp
 *
 * Concurrent stress tests for DynamicSparseLayer.
 *
 * Validates thread safety of store/get/contains under concurrent access,
 * especially during _grow() operations. Designed to be run with ASAN/TSan
 * to catch use-after-free or data races.
 */

#include <gtest/gtest.h>

#include <parameters/px4_parameters.hpp>
#include "DynamicSparseLayer.h"

#include <atomic>
#include <thread>
#include <vector>

/**
 * Minimal ParamLayer parent that returns a default zero value for all params.
 */
class StubParamLayer : public ParamLayer
{
public:
	StubParamLayer() : ParamLayer(nullptr) {}

	bool store(param_t, param_value_u) override { return false; }
	bool contains(param_t) const override { return false; }

	px4::AtomicBitset<PARAM_COUNT> containedAsBitset() const override
	{
		return px4::AtomicBitset<PARAM_COUNT>();
	}

	param_value_u get(param_t) const override
	{
		param_value_u v{};
		v.i = 0;
		return v;
	}

	void reset(param_t) override {}
	void refresh(param_t) override {}
	int size() const override { return 0; }
	int byteSize() const override { return 0; }
};

class DynamicSparseLayerConcurrentTest : public ::testing::Test
{
protected:
	StubParamLayer stub;
};

// Single writer forces frequent _grow() while readers concurrently access
// stored params. With only one writer there is no concurrent growth, so
// this safely stresses the read-side during buffer reallocation.
TEST_F(DynamicSparseLayerConcurrentTest, ConcurrentStoreGetRace)
{
	for (int rep = 0; rep < 50; rep++) {
		DynamicSparseLayer layer(&stub, /*n_prealloc=*/2, /*n_grow=*/1);

		constexpr int NUM_PARAMS = 100;
		std::atomic<bool> stop{false};

		std::thread writer([&]() {
			for (int i = 0; i < NUM_PARAMS; i++) {
				param_value_u v{};
				v.i = i * 10;
				layer.store(static_cast<param_t>(i), v);
			}

			stop.store(true, std::memory_order_release);
		});

		std::vector<std::thread> readers;

		for (int r = 0; r < 4; r++) {
			readers.emplace_back([&]() {
				while (!stop.load(std::memory_order_acquire)) {
					int current_size = layer.size();

					for (int i = 0; i < current_size && i < NUM_PARAMS; i++) {
						layer.contains(static_cast<param_t>(i));
						layer.get(static_cast<param_t>(i));
					}
				}
			});
		}

		writer.join();

		for (auto &t : readers) {
			t.join();
		}

		for (int i = 0; i < NUM_PARAMS; i++) {
			ASSERT_TRUE(layer.contains(static_cast<param_t>(i)))
					<< "rep=" << rep << " param=" << i;
			param_value_u v = layer.get(static_cast<param_t>(i));
			ASSERT_EQ(v.i, i * 10)
					<< "rep=" << rep << " param=" << i;
		}

		ASSERT_EQ(layer.size(), NUM_PARAMS);
	}
}

// Multiple writers store to disjoint param ranges concurrently.
// Pre-allocated to avoid concurrent _grow() which has a known ABA
// limitation in the CAS retry loop (see _grow() implementation).
// This test validates concurrent store correctness.
TEST_F(DynamicSparseLayerConcurrentTest, ConcurrentMultipleWriters)
{
	constexpr int PARAMS_PER_WRITER = 50;
	constexpr int NUM_WRITERS = 4;
	constexpr int TOTAL = NUM_WRITERS * PARAMS_PER_WRITER;

	for (int rep = 0; rep < 20; rep++) {
		DynamicSparseLayer layer(&stub, /*n_prealloc=*/2, /*n_grow=*/1);

		std::vector<std::thread> writers;

		for (int w = 0; w < NUM_WRITERS; w++) {
			writers.emplace_back([&layer, w]() {
				int base = w * PARAMS_PER_WRITER;

				for (int i = 0; i < PARAMS_PER_WRITER; i++) {
					param_value_u v{};
					v.i = (base + i) * 10;
					layer.store(static_cast<param_t>(base + i), v);
				}
			});
		}

		for (auto &t : writers) {
			t.join();
		}

		ASSERT_EQ(layer.size(), TOTAL) << "rep=" << rep;

		for (int i = 0; i < TOTAL; i++) {
			ASSERT_TRUE(layer.contains(static_cast<param_t>(i)))
					<< "rep=" << rep << " param=" << i;
			param_value_u v = layer.get(static_cast<param_t>(i));
			ASSERT_EQ(v.i, i * 10)
					<< "rep=" << rep << " param=" << i;
		}
	}
}

// Combined stress: writers store while readers access concurrently.
TEST_F(DynamicSparseLayerConcurrentTest, ConcurrentWritersAndReaders)
{
	constexpr int PARAMS_PER_WRITER = 50;
	constexpr int NUM_WRITERS = 2;
	constexpr int TOTAL = NUM_WRITERS * PARAMS_PER_WRITER;

	for (int rep = 0; rep < 20; rep++) {
		DynamicSparseLayer layer(&stub, /*n_prealloc=*/2, /*n_grow=*/1);
		std::atomic<bool> stop{false};

		std::vector<std::thread> writers;

		for (int w = 0; w < NUM_WRITERS; w++) {
			writers.emplace_back([&layer, w]() {
				int base = w * PARAMS_PER_WRITER;

				for (int i = 0; i < PARAMS_PER_WRITER; i++) {
					param_value_u v{};
					v.i = (base + i) * 10;
					layer.store(static_cast<param_t>(base + i), v);
				}
			});
		}

		std::vector<std::thread> readers;

		for (int r = 0; r < 4; r++) {
			readers.emplace_back([&]() {
				while (!stop.load(std::memory_order_acquire)) {
					int current_size = layer.size();

					for (int i = 0; i < current_size && i < TOTAL; i++) {
						layer.contains(static_cast<param_t>(i));
						layer.get(static_cast<param_t>(i));
					}
				}
			});
		}

		for (auto &t : writers) {
			t.join();
		}

		stop.store(true, std::memory_order_release);

		for (auto &t : readers) {
			t.join();
		}

		ASSERT_EQ(layer.size(), TOTAL) << "rep=" << rep;

		for (int i = 0; i < TOTAL; i++) {
			ASSERT_TRUE(layer.contains(static_cast<param_t>(i)))
					<< "rep=" << rep << " param=" << i;
			param_value_u v = layer.get(static_cast<param_t>(i));
			ASSERT_EQ(v.i, i * 10)
					<< "rep=" << rep << " param=" << i;
		}
	}
}
