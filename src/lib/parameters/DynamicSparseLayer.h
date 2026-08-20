/****************************************************************************
 *
 *   Copyright (c) 2023-2026 PX4 Development Team. All rights reserved.
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

#include <cassert>

/**
 * DynamicSparseLayer - a parameter layer that stores only the parameters that
 * have actually been set, in a heap-allocated array that grows on demand.
 *
 * ## Data layout
 *
 * `_slots` points to a heap array of `_n_slots` Slot entries. The first
 * `_next_slot` entries are "valid" (a param->value override) and are kept
 * sorted by `param` ascending so lookups can binary-search them (_getIndex()).
 * The remaining `_n_slots - _next_slot` entries are unused, filled with the
 * sentinel {UINT16_MAX, {}} (UINT16_MAX sorts to the end). When the valid
 * entries fill the array (`_next_slot == _n_slots`), _grow() reallocates a
 * bigger one.
 *
 * ## Concurrency model
 *
 * A single recursive mutex - taken by constructing an AtomicTransaction - guards
 * ALL three members (`_slots`, `_next_slot`, `_n_slots`). Every public method
 * takes it for its whole duration, so readers and writers never see a partial
 * update and never race. (On NuttX AtomicTransaction disables interrupts; on
 * POSIX it is a recursive pthread mutex.)
 *
 * There is exactly ONE place the lock is dropped mid-operation: _grow() has to
 * call malloc()/free(), which must not run with the lock held (on NuttX the lock
 * disables interrupts, and the allocator takes its own locks). So _grow()
 * *unlocks around the allocation only*. That single window is the reason store()
 * and _grow() are written as re-checking loops rather than straight-line code:
 * while the lock is dropped another writer may grow the buffer or take a slot, so
 * any value read before the window must be re-validated after it. Nothing else in
 * this class ever observes a concurrent modification, because nothing else
 * releases the lock.
 *
 * `_slots` is a px4::atomic only so that this unlock-around-malloc window is
 * well-defined; given every access is under the mutex it could equally be a plain
 * pointer.
 *
 * ## Why the lock is dropped on POSIX too (do not "optimize" this away)
 *
 * Only NuttX *needs* _grow() to drop the lock: there the lock disables interrupts
 * and malloc()/free() can't run in that state. On POSIX the lock is an ordinary
 * recursive mutex, so _grow() could just hold it across the allocation and the
 * unlock/lock pair would be unnecessary for correctness. We drop it on BOTH
 * platforms deliberately. ThreadSanitizer and the concurrent stress tests only run
 * on POSIX/SITL - if the unlock window existed solely on NuttX, the entire
 * concurrent-grow path (the re-check loops, the count-based ABA guard) would run
 * only on the platform we cannot instrument, and a bug in it would surface only on
 * hardware. Keeping the code path identical lets SITL+TSan exercise the exact same
 * window that runs on NuttX. So: do not #ifdef the unlock away on POSIX "because
 * it isn't needed there" - that would hide the embedded concurrency from the only
 * tooling that can catch bugs in it.
 */
class DynamicSparseLayer : public ParamLayer
{
public:
	DynamicSparseLayer(ParamLayer *parent, int n_prealloc = 32, int n_grow = 4) : ParamLayer(parent),
		_n_slots(n_prealloc),
		// _grow() only terminates if every grow adds at least one slot, so clamp
		// _n_grow to >= 1. The assert flags the misconfiguration in debug builds;
		// the clamp keeps the release build safe (asserts compile out under NDEBUG).
		_n_grow(n_grow > 0 ? n_grow : 1)
	{
		assert(n_grow > 0);

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
		// Take the lock so concurrent readers can't observe a half-freed
		// state (relevant on POSIX, where DynamicSparseLayer instances are
		// statics in parameters.cpp and other threads may still be calling
		// get()/contains() while the process is running atexit handlers).
		// Zero _next_slot and _slots so any reader that acquires the mutex
		// after us short-circuits on the empty layer instead of dereferencing
		// freed memory.
		AtomicTransaction transaction;
		Slot *slots = _slots.load();
		_slots.store(nullptr);
		_next_slot = 0;
		_n_slots = 0;
		free(slots);
	}

	bool store(param_t param, param_value_u value) override
	{
		AtomicTransaction transaction;

		// _grow() releases the lock while it allocates, so any decision made
		// before calling it can be stale afterwards (another writer may have
		// added our param or taken the slot we were about to use). So we never
		// carry a decision across a grow: we re-read _slots, re-search, and
		// re-check capacity from scratch on every iteration. Without a grow this
		// loop body runs exactly once.
		while (true) {
			Slot *slots = _slots.load();
			const int index = _getIndex(param);

			if (index < _next_slot) {
				// The param already has an override: update it in place.
				slots[index].value = value;
				return true;
			}

			if (_next_slot < _n_slots) {
				// New param and there is a free slot: append it and re-sort so
				// _getIndex() can keep binary-searching. This branch holds the
				// lock throughout (no _grow), so the read-then-append is atomic.
				slots[_next_slot++] = {param, value};
				_sort();
				return true;
			}

			// New param but the array is full: grow and loop to retry. _grow()
			// returns false only if allocation failed, in which case the store
			// fails.
			if (!_grow(transaction)) {
				return false;
			}
		}
	}

	bool contains(param_t param) const override
	{
		// Reader: hold the lock for the whole lookup so we never read _slots /
		// _next_slot while a writer (store/_grow/reset) is mutating them.
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
		// Reader: lock held for the whole lookup (see contains()).
		const AtomicTransaction transaction;
		Slot *slots = _slots.load();

		const int index = _getIndex(param);

		if (index < _next_slot) { // exists in our data structure
			return slots[index].value;
		}

		// Not overridden here: defer to the layer below us.
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
		const AtomicTransaction transaction;
		return _next_slot;
	}

	int byteSize() const override
	{
		const AtomicTransaction transaction;
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

	// Keep the array sorted by param ascending. Sentinels (param == UINT16_MAX)
	// sort to the end, so the valid entries stay packed in [0, _next_slot).
	// Caller must hold the lock.
	void _sort()
	{
		qsort(_slots.load(), _n_slots, sizeof(Slot), _slotCompare);
	}

	// Binary-search the sorted valid range for `param`. Returns its index if
	// found, or `_next_slot` (the "not found" sentinel, == past-the-end of the
	// valid range) otherwise. Caller must hold the lock.
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

	// Called by store() with the lock held when the array is full. Replaces
	// `_slots` with a buffer `_n_grow` entries larger and returns whether there
	// is now room. The caller (store) must re-check its state afterwards, because
	// this releases the lock around the allocation - see below.
	bool _grow(AtomicTransaction &transaction)
	{
		if (_n_slots == 0) {
			// Construction failed to allocate; there is no buffer to grow.
			return false;
		}

		// Loop until there is room. Each iteration either grows the buffer
		// ourselves, or detects that another writer grew it while we were
		// allocating. Both cases strictly increase _n_slots (by >= 1, since
		// _n_grow is clamped to >= 1 in the ctor). _next_slot is the count of
		// distinct stored params, so it is capped at PARAM_COUNT; once _n_slots
		// passes that there is permanent room. Hence this terminates in at most
		// ~PARAM_COUNT/_n_grow iterations even under contention; only a malloc
		// failure aborts early.
		while (_next_slot >= _n_slots) {
			// Snapshot the size we are allocating for. We must drop the lock
			// around malloc()/free(): on NuttX the lock disables interrupts and
			// the allocator can't run there; on POSIX the allocator takes its own
			// locks. This unlock..lock pair is the ONLY window in the whole class
			// where another thread can change our members underneath us.
			const int alloc_n_slots = _n_slots;

			transaction.unlock();
			Slot *new_slots = (Slot *) malloc(sizeof(Slot) * (alloc_n_slots + _n_grow));
			transaction.lock();

			if (new_slots == nullptr) {
				return false;
			}

			// Back under the lock: did another writer grow the buffer while we
			// were allocating? We check the slot COUNT, which only ever increases
			// under the lock. We deliberately do NOT compare the `_slots` pointer
			// (the previous approach): malloc can hand back the address of a
			// buffer that was just freed by another grow, so a pointer that looks
			// unchanged ("A") may actually be a different, current buffer ("ABA").
			// A pointer compare would then pass while _n_slots has moved on, and
			// the memcpy below would copy _n_slots entries into our buffer that was
			// only sized for alloc_n_slots - a heap overflow.
			if (_n_slots != alloc_n_slots) {
				// Someone else already grew it. Our buffer is now the wrong size;
				// throw it away. This is progress, not a failure, so the loop just
				// re-evaluates: there may already be room, or we grow again.
				free(new_slots);
				continue;
			}

			// _n_slots == alloc_n_slots, so the buffer is correctly sized and
			// `_slots` still points at the array we sized against. Copy the live
			// entries over, sentinel-fill the new tail, and publish.
			Slot *previous_slots = _slots.load();
			memcpy(new_slots, previous_slots, sizeof(Slot) * _n_slots);

			for (int i = _n_slots; i < _n_slots + _n_grow; i++) {
				new_slots[i] = {UINT16_MAX, param_value_u{}};
			}

			_slots.store(new_slots);
			_n_slots += _n_grow;

			// free() of the old buffer also can't run under the lock; drop it
			// again. Safe now because _slots already points at new_slots, so no
			// reader will touch previous_slots.
			transaction.unlock();
			free(previous_slots);
			transaction.lock();
		}

		return _next_slot < _n_slots;
	}

	// All three guarded by the AtomicTransaction mutex (see class comment).
	int _next_slot = 0;        // number of valid entries, all in _slots[0.._next_slot)
	int _n_slots = 0;          // capacity: total entries _slots points to
	const int _n_grow;         // how many slots to add per grow
	px4::atomic<Slot *> _slots{nullptr}; // the array; atomic only for the unlock-around-malloc window in _grow()
};
