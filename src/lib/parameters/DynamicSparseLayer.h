/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "ParamLayer.h"

#include <px4_platform_common/atomic.h>

class DynamicSparseLayer : public ParamLayer
{
public:
	DynamicSparseLayer(ParamLayer *parent, int n_prealloc = 32, int n_grow = 4) : ParamLayer(parent),
		_n_slots(n_prealloc), _n_grow(n_grow)
	{
		Slot *slots = (Slot *)malloc(sizeof(Slot) * n_prealloc);

		if (slots == nullptr) {
			PX4_ERR("Failed to allocate memory for dynamic sparse layer");
			_n_slots = 0;
			return;
		}

		for (int i = 0; i < _n_slots; i++) {
			slots[i] = {UINT16_MAX, param_value_u{}};
		}

		_slots.store(slots);
	}

	virtual ~DynamicSparseLayer()
	{
		if (_slots.load()) {
			free(_slots.load());
		}
	}

	bool store(param_t param, param_value_u value) override
	{
		AtomicTransaction transaction;
		Slot *slots = _slots.load();

		const int index = _getIndex(param);

		if (index < _next_slot) { // already exists
			slots[index].value = value;

		} else if (_next_slot < _n_slots) {
			slots[_next_slot++] = {param, value};
			_sort();

		} else {
			if (!_grow(transaction)) {
				return false;
			}

			_slots.load()[_next_slot++] = {param, value};
			_sort();
		}

		return true;
	}

	bool contains(param_t param) const override
	{
		const AtomicTransaction transaction;
		return _getIndex(param) < _next_slot;
	}

	px4::AtomicBitset<PARAM_COUNT> containedAsBitset() const override
	{
		px4::AtomicBitset<PARAM_COUNT> set;
		const AtomicTransaction transaction;
		Slot *slots = _slots.load();

		for (int i = 0; i < _next_slot; i++) {
			set.set(slots[i].param);
		}

		return set;
	}

	param_value_u get(param_t param) const override
	{
		const AtomicTransaction transaction;
		Slot *slots = _slots.load();

		const int index = _getIndex(param);

		if (index < _next_slot) { // exists in our data structure
			return slots[index].value;
		}

		return _parent->get(param);
	}

	void reset(param_t param) override
	{
		const AtomicTransaction transaction;
		int index = _getIndex(param);
		Slot *slots = _slots.load();

		if (index < _next_slot) {
			slots[index] = {UINT16_MAX, param_value_u{}};
			_sort();
			_next_slot--;
		}
	}

	void refresh(param_t param) override
	{
		_parent->refresh(param);
	}

	int size() const override
	{
		return _next_slot;
	}

	int byteSize() const override
	{
		return _n_slots * sizeof(Slot);
	}

private:
	struct Slot {
		param_t param;
		param_value_u value;
	};

	static int _slotCompare(const void *a, const void *b)
	{
		return ((int)((Slot *)a)->param) - ((int)((Slot *)b)->param);
	}

	void _sort()
	{
		qsort(_slots.load(), _n_slots, sizeof(Slot), _slotCompare);
	}

	int _getIndex(param_t param) const
	{
		int left = 0;
		int right = _next_slot - 1;
		Slot *slots = _slots.load();

		while (left <= right) {
			int mid = (left + right) / 2;

			if (slots[mid].param == param) {
				return mid;

			} else if (slots[mid].param < param) {
				left = mid + 1;

			} else {
				right = mid - 1;
			}
		}

		return _next_slot;
	}

	bool _grow(AtomicTransaction &transaction)
	{
		if (_n_slots == 0) {
			return false;
		}

		int max_retries = 5;

		// As malloc uses locking, so we need to re-enable IRQ's during malloc/free and
		// then atomically exchange the buffer
		while (_next_slot >= _n_slots && max_retries-- > 0) {
			Slot *previous_slots = nullptr;
			Slot *new_slots = nullptr;

			do {
				previous_slots = _slots.load();
				transaction.unlock();

				if (new_slots) {
					free(new_slots);
				}

				new_slots = (Slot *) malloc(sizeof(Slot) * (_n_slots + _n_grow));
				transaction.lock();

				if (new_slots == nullptr) {
					return false;
				}

			} while (!_slots.compare_exchange(&previous_slots, new_slots));

			memcpy(new_slots, previous_slots, sizeof(Slot) * _n_slots);

			for (int i = _n_slots; i < _n_slots + _n_grow; i++) {
				new_slots[i] = {UINT16_MAX, param_value_u{}};
			}

			_n_slots += _n_grow;

			transaction.unlock();
			free(previous_slots);
			transaction.lock();
		}

		return _next_slot < _n_slots;
	}

	int _next_slot = 0;
	int _n_slots = 0;
	const int _n_grow;
	px4::atomic<Slot *> _slots{nullptr};
};
